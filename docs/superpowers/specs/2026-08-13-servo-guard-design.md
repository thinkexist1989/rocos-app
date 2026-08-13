# ServoGuard 伺服保护设计方案

> 日期: 2026-08-13  
> 范围: `Controller -> Hardware` 之间的每周期命令保护  
> 状态: 设计方案，尚未进入实现

## 1. 背景

当前控制周期主链路为:

```text
MotionInterface::GenerateRef()
  -> ControllerInterface::GenerateCmd()
  -> ControllerInterface::UpdateCmd()
       -> HardwareInterface::SetPosition() / SetTorque()
```

`Motion` 层已经有一部分前置校验，例如参数有限性、速度/加速度/jerk 正值检查，以及 `MoveLineOffline` / `MoveCircleOffline` 的整轨迹 IK 验证。但这些校验发生在运动启动前，不能覆盖每周期下发阶段的异常，例如:

- 控制器输出突变。
- UDP Servo 指令跳变或超时。
- 阻抗控制器力矩暴冲。
- 实际反馈接近软限位仍继续向外推。
- 当前反馈 NaN/Inf、速度异常、通信状态异常。

因此需要在控制器和底层硬件下发之间增加统一的伺服保护层。

## 2. 目标

- 所有控制器下发到硬件前都必须经过同一套 Guard 入口。
- Guard 内部统一使用 URDF/KDL 的 model joint order。
- 支持位置类控制器和力矩类控制器的不同保护策略。
- 支持 joint binding 映射，不允许保护逻辑按错轴检查。
- 保护失败后能把错误反馈给 `Executor` / `Robot FSM`，进入受控错误或停止流程。
- 第一阶段尽量少改现有控制器接口。

## 3. 非目标

- 第一阶段不重写所有 `ControllerInterface`。
- 第一阶段不改变 `MotionInterface` 的规划逻辑。
- 第一阶段不把 Guard 做成独立实时线程。
- 第一阶段不替代驱动器自身的硬件限位、急停、CiA 402 fault 机制。

Guard 是软件保护层，不是唯一安全边界。

## 4. 推荐接入方案

当前 `ControllerInterface::UpdateCmd()` 内部已经直接调用 `hardware_->SetPosition()` 或 `hardware_->SetTorque()`。如果立刻把 Controller 拆成“计算命令”和“下发命令”，改动会较大。

第一阶段推荐使用 `HardwareInterface` 包装器:

```text
Controller
  -> ServoGuard : HardwareInterface
       -> real Hardware : HardwareInterface
```

`ServoGuard` 继承 `HardwareInterface`，对读取类接口直接转发，对写入类接口先检查再转发。

```text
PositionController::UpdateCmd(q_cmd)
  -> servo_guard->SetPosition(q_cmd)
       -> CheckGlobalState()
       -> CheckPositionCommand(q_cmd)
       -> real_hardware->SetPosition(q_cmd)

JointImpedanceController::UpdateCmd(q_des)
  -> servo_guard->SetTorque(tau_cmd)
       -> CheckGlobalState()
       -> CheckTorqueCommand(tau_cmd)
       -> real_hardware->SetTorque(tau_cmd)
```

这样可以覆盖:

- `PositionController`
- `JointAdmittanceController`
- `JointImpedanceController`
- `CartesianImpedanceController`
- `MoveServo` 通过控制器下发的位置或笛卡尔 IK 结果

## 5. Robot 初始化流程

推荐初始化顺序:

```text
Robot::Robot()
  -> hardware = std::make_unique<Hardware>(...)
  -> model = std::make_unique<Model>(...)
  -> jointBinding()
       -> hardware->SetJointBinding(model_index_to_drive_id)
  -> buildServoGuardLimits()
       -> 生成 model order 的 limits_by_model_index
  -> servo_guard = std::make_unique<ServoGuard>(hardware.get(), limits)
  -> executor->SwitchHardware(hardware.get())
  -> SetWorkMode("position")
       -> controller->SetHardware(servo_guard.get())
```

关键原则:

- `Robot` 自己的上使能、下使能、状态读取可以继续使用真实 `hardware`。
- `Controller` 必须拿 `servo_guard`，保证所有 `SetPosition()` / `SetTorque()` 都必经保护。
- `ServoGuard` 内部的 `inner_` 指向真实 `HardwareInterface`，不拥有生命周期。
- `Executor` 第一阶段可以继续保存真实 `hardware`，Guard fault 由 `Robot::RunCycle()` 在 `executor->Update()` 后统一检查。

## 6. 保护生命周期

`Controller::SetReady()` 会写安全初值:

- 位置类控制器会 `SetPosition(current_position)` 并切 CSP。
- 力矩类控制器会 `SetTorque(gravity_compensation)` 并切 CST。

这些写入发生在工作模式切换或初始化阶段，不完全等同于运动中的每周期命令。但它们仍然会改变驱动的目标位置、目标力矩和控制模式。如果这些写入发生在未使能状态，驱动可能缓存目标值，并在后续上使能瞬间产生非预期运动。因此第一阶段采用更保守的规则: `Ready` 阶段也必须要求硬件已经 `ENABLED`。

因此 `ServoGuard` 需要区分保护阶段:

```cpp
enum class GuardPhase {
    Bypass,   // 内部恢复或明确旁路，默认不用于控制器写命令
    Ready,    // 已使能后的 SetReady 阶段，允许安全锁当前位置/重力补偿
    Active,   // 控制周期下发阶段，执行完整保护
    Faulted   // 已触发 fault，拒绝继续下发运动命令
};
```

推荐规则:

```text
Robot 构造:
  不调用 controller->SetReady()，或保持 GuardPhase::Bypass 且不写运动命令

ResetFault 内部恢复:
  清理 Guard fault，保持 Bypass/Ready，但不在未使能状态写目标命令

SetWorkMode 调用 controller->SetReady():
  必须先确认 hardware state == ENABLED
  GuardPhase::Ready

机器人进入 RUNNING / SERVOING / PAUSING / RESUMING / STOPPING:
  GuardPhase::Active

Guard 检查失败:
  GuardPhase::Faulted
```

`Ready` 阶段仍应做基础检查:

- 指针非空。
- 命令维度正确。
- 命令值有限。
- 位置准备命令应接近当前实际位置。
- 力矩准备命令应在 effort 限制内。
- `state == JntState::ENABLED`。

`Ready` 阶段要求 `state == ENABLED`，但不做跟随误差和速度/加速度命令连续性检查，因为此时还不是轨迹执行中的连续命令。

### 6.1 现有安全写入调用点

当前代码中，初始化、切模式和控制器析构阶段也会写硬件。这些调用不是普通轨迹命令，但仍应经过 Guard 的基础保护。

| 场景 | 调用函数 | 写入内容 | 建议 Guard 阶段 |
|------|----------|----------|-----------------|
| 位置控制器就绪 | `PositionController::SetReady()` | `SetPosition(GetPosition())`, `SetMode(CSP)` | `Ready` |
| 关节导纳控制器就绪 | `JointAdmittanceController::SetReady()` | `SetPosition(GetPosition())`, `SetMode(CSP)` | `Ready` |
| 关节阻抗控制器就绪 | `JointImpedanceController::SetReady()` | `SetTorque(tau_grav)`, `SetMode(CST)` | `Ready` |
| 笛卡尔阻抗控制器就绪 | `CartesianImpedanceController::SetReady()` | `SetTorque(tau_grav)`, `SetMode(CST)` | `Ready` |
| 控制器析构兜底 | 各 controller 析构函数 | `SetPosition(GetPosition())`, `SetMode(CSP)` | `Ready`，若未使能则只允许无运动副作用的清理 |
| 运动控制周期 | 各 controller `UpdateCmd()` | `SetPosition(q_cmd)` 或 `SetTorque(tau_cmd)` | `Active` |

这些调用的入口在 `Robot::SetWorkMode()`:

```text
new_controller->SetHardware(...)
new_controller->SetModel(...)
controller.reset()
new_controller->SetReady()
executor->SwitchController(controller.get())
```

因此 `Robot::SetWorkMode()` 应在调用 `SetReady()` 前确认机器人已经使能，并临时把 Guard 切到 `Ready`。`SetReady()` 成功后，如果只是 `STOPPED` 下切模式，可以保持 `Ready`；进入 `RUNNING` / `SERVOING` 前必须切到 `Active`。`IDLE` 未使能状态下不允许调用会写目标命令的 `SetReady()`。

### 6.2 Ready 阶段也需要保护

`Ready` 不是完全无保护。它应允许“安全准备写入”，但拒绝明显危险值。

位置类 `SetReady()` 的保护:

```text
q_ready = inner_->GetPosition()
SetPosition(q_ready)
```

Guard 应检查:

- `state == JntState::ENABLED`。
- `q_ready` 维度等于 model joint count。
- `q_ready` 全部有限。
- `q_ready` 在软限位内。
- 按本周期速度检查 `abs(q_ready[i] - q_actual[i]) / dt <= limits[i].vel`。

力矩类 `SetReady()` 的保护:

```text
tau_ready = gravity_compensation(q_actual)
SetTorque(tau_ready)
```

Guard 应检查:

- `state == JntState::ENABLED`。
- `tau_ready` 维度等于 model joint count。
- `tau_ready` 全部有限。
- `abs(tau_ready[i]) <= limits[i].effort`。
- 当前 `q_actual` / `q_dot_actual` 有限，并在软限位与速度限制内。

`SetMode()` 的保护:

- `Ready` 阶段允许切到当前控制器需要的 CSP/CST。
- `Active` 阶段原则上不应频繁切模式；若控制器首次 `UpdateCmd()` 因 `mode_set_ == false` 再次调用 `SetMode()`，Guard 可以允许同模式重复设置。
- 不允许未知 mode 或与当前控制器类别不匹配的 mode。第一阶段如果无法识别控制器类别，可以只记录告警并转发 CSP/CST，拒绝其他 mode。

控制器析构兜底写入是安全收尾动作，但也不应在未使能状态下写入可能被驱动缓存的目标。建议在 `Robot::SetWorkMode()` 销毁旧控制器前，如果硬件仍处于 `ENABLED`，临时进入 `Ready`，允许旧控制器把目标位置锁到当前位置并切回 CSP；如果硬件未使能，则跳过会写目标位置/力矩的兜底命令，只做对象状态清理。这个路径仍应检查 NaN/Inf 和维度，避免把坏反馈原样写回硬件。

## 7. 关节顺序与映射规则

这是本设计最重要的约束。

系统中存在两种顺序:

- model joint order: URDF/KDL/Controller/Motion 使用的逻辑关节顺序。
- hardware drive id/order: 真实 EtherCAT drive id 和 `hardware_talon_config.yaml` 中的驱动顺序。

已有 `Hardware::SetJointBinding()` 会保存:

```text
model_index -> drive_id
```

并且当前 `Hardware::GetPosition()` / `SetPosition()` 已经按 `DriveIdByModelIndex(i)` 做转换。因此:

```text
hardware->GetPosition()[i]
hardware->GetVelocity()[i]
hardware->GetTorque()[i]
hardware->SetPosition(q)[i]
hardware->SetTorque(tau)[i]
```

都应视为 model joint order。

`ServoGuard` 必须遵守:

```text
q_cmd[i]
q_actual[i]
q_dot_actual[i]
tau_cmd[i]
limits_by_model_index[i]
```

全部使用同一个 model index。

## 8. Limit 表构建

Guard 不能直接按 `config_.drives[i]` 取 limit，因为硬件配置顺序可能和 URDF 顺序不同。

推荐在 `Robot::jointBinding()` 完成后构建:

```cpp
struct JointGuardLimit {
    int32_t drive_id;
    std::string joint_name;
    double lower;
    double upper;
    double vel;
    double acc;
    double jerk;
    double effort;
};

std::vector<JointGuardLimit> limits_by_model_index;
```

构建规则:

```text
binding = hardware->GetJointBinding()

if binding 非空:
  for model_index i:
    drive_id = binding[i]
    drive = Hardware::findDriveById(drive_id)
    limits_by_model_index[i] = drive.limit

if binding 为空:
  认为 config_.drives[i] 对应 model_index i
```

更干净的长期接口是给 `HardwareInterface` 增加:

```cpp
virtual std::vector<JointGuardLimit> GetJointGuardLimits() const;
```

并要求返回值已经是 model order。第一阶段若不想扩展接口，可以在 `Robot` 内对真实 `Hardware*` 做一次配置读取，然后把整理好的 limit 表传给 `ServoGuard`。

## 9. ServoGuard 结构

推荐分层:

```text
ServoGuard
  -> GlobalStateGuard
  -> PositionCommandGuard
  -> TorqueCommandGuard
```

统一的是框架、配置、错误记录、日志和接入点。分开的是位置命令与力矩命令的语义。

### 9.1 GlobalStateGuard

所有控制器共享，读取真实硬件反馈:

```cpp
JntArray q_actual = inner_->GetPosition();
JntArray q_dot_actual = inner_->GetVelocity();
JntArray tau_actual = inner_->GetTorque();
JntState state = inner_->GetState();
uint32_t dt_us = inner_->GetDt();
```

检查项:

- `inner_ != nullptr`
- `state == JntState::ENABLED`
- `dt_us > 0`
- `q_actual.rows() == limits.size()`
- `q_dot_actual.rows() == limits.size()`
- `q_actual` / `q_dot_actual` 全部有限
- `q_actual` 未超过硬软限位
- `abs(q_dot_actual[i])` 未超过速度上限
- 可选: 反馈时间戳或通信质量未 stale

第一阶段如果底层没有反馈时间戳，可以先只检查 `dt`、状态、反馈有限性和速度/位置边界。通信 stale 后续再通过硬件层暴露时间戳或周期计数。

`GlobalStateGuard` 的完整检查在 `GuardPhase::Active` 下启用。`Ready` 阶段同样要求 `state == JntState::ENABLED`，但使用较弱的命令连续性检查，避免把安全锁当前位置和重力补偿初始化误判为轨迹跳变。

### 9.2 PositionCommandGuard

适用于最终调用 `SetPosition(q_cmd)` 的控制器:

- `PositionController`
- `JointAdmittanceController`
- joint position servo 模式

检查项:

- `q_cmd.rows() == limits.size()`
- `q_cmd` 全部有限
- `q_cmd[i]` 在 `[limits[i].lower, limits[i].upper]`
- 本周期要求速度 `abs(q_cmd[i] - q_actual[i]) / dt <= limits[i].vel`
- 命令序列速度 `abs(q_cmd[i] - q_last_cmd[i]) / dt <= limits[i].vel`
- 命令序列加速度 `abs(v_cmd[i] - v_last_cmd[i]) / dt <= limits[i].acc`

处理策略:

- NaN/Inf、维度不匹配、严重超限: fault。
- `abs(q_cmd[i] - q_actual[i]) / dt` 超过速度限制: fault，拒绝下发到底层硬件。
- 命令序列速度/加速度连续性超限: fault，拒绝下发到底层硬件。

位置模式第一阶段以“当前反馈到本次命令”的速度检查为主:

```text
dt = inner_->GetDt() / 1e6
required_velocity[i] = (q_cmd[i] - q_actual[i]) / dt

if abs(required_velocity[i]) > limits[i].vel:
  fault
  do not call inner_->SetPosition(q_cmd)
```

原因是 `q_cmd - q_actual` 描述的是“如果本周期接受这个目标，驱动需要从当前位置追到目标的瞬时要求”。它能直接拦截位置跳变、UDP Servo 突然给远点、控制器内部积分飘远等危险情况。`q_cmd - q_last_cmd` 和加速度检查作为第二层连续性保护，用于发现命令序列自身的突变。

### 9.3 TorqueCommandGuard

适用于最终调用 `SetTorque(tau_cmd)` 的控制器:

- `JointImpedanceController`
- `CartesianImpedanceController`

力矩控制不能直接复用位置控制的 `q_cmd - q_actual` 检查。阻抗控制允许期望位置和实际位置存在误差，这个误差本身就是控制器生成力矩的来源。如果 Guard 把它当作位置跟随误差直接拦截，会误杀正常阻抗控制。

检查项:

- `tau_cmd.rows() == limits.size()`
- `tau_cmd` 全部有限
- `abs(tau_cmd[i]) <= limits[i].effort`
- `q_actual[i]` 在 `[limits[i].lower, limits[i].upper]`
- `abs(q_dot_actual[i]) <= limits[i].vel`
- 靠近上限时禁止继续给正向外推力矩
- 靠近下限时禁止继续给负向外推力矩
- 高速接近限位时禁止继续加速

保护重点是:

```text
实际状态是否安全
力矩命令是否安全
力矩方向是否会把机器人推向危险区
```

处理策略:

- NaN/Inf、维度不匹配、严重状态异常: fault。
- 力矩绝对值超限: 第一阶段建议 fault，不建议静默饱和。
- 当前关节位置或速度已经超限: fault。
- 接近限位仍给外推力矩: fault。
- 高速接近限位仍给加速方向力矩: fault。

第一阶段不在 ServoGuard 中新增 `tau_rate_limit`、机械功率限制或力矩 clamp 策略，原因是这些阈值当前不是硬件配置里的现成字段。为了保持单一事实来源，ServoGuard 只使用 `Drive::Limit` 中已有的 `lower`、`upper`、`vel`、`acc`、`jerk`、`effort`。力矩变化率保护如果控制器内部已经存在，例如 `CartesianImpedanceController` 的局部力矩变化率限制，可以继续保留在控制器内部。Guard 第一阶段只做全局兜底，不把控制器局部策略搬进统一保护层。

### 9.4 第一阶段保护清单

第一阶段 ServoGuard 明确保护以下内容:

| 类别 | 保护项 | 依据 |
|------|--------|------|
| 通用状态 | hardware 指针、`ENABLED` 状态、`dt_us > 0`、反馈维度、反馈有限性 | 当前硬件反馈 |
| 当前关节状态 | `q_actual` 不越界、`q_dot_actual` 不超速 | `Drive::Limit.lower/upper/vel` |
| 位置命令 | `q_cmd` 维度、有限性、目标位置限位 | `Drive::Limit.lower/upper` |
| 位置跳变 | `(q_cmd - q_actual) / dt` 不超过速度限制 | `Drive::Limit.vel` |
| 位置连续性 | `(q_cmd - q_last_cmd) / dt` 和命令加速度不超限 | `Drive::Limit.vel/acc` |
| 力矩命令 | `tau_cmd` 维度、有限性、绝对力矩不超限 | `Drive::Limit.effort` |
| 力矩方向 | 靠近限位时禁止继续向外推 | `q_actual`、`q_dot_actual`、`tau_cmd` 符号 |
| Ready 阶段 | 已使能后才允许锁当前位置、重力补偿和 CSP/CST 切换 | `GuardPhase::Ready` |
| Fault 后 | 一旦 Guard fault，拒绝继续下发运动命令 | `GuardPhase::Faulted` |

第一阶段明确不做:

- 不新增 `config/servo_guard.yaml`。
- 不新增 `ServoGuardPolicy`。
- 不新增 `tau_rate_limit`。
- 不新增机械功率限制。
- 不在 ServoGuard 中做力矩 clamp 或位置 clamp。
- 不把阻抗控制的 `q_des - q_actual` 当作位置跟随误差检查。

## 10. 错误传播

`HardwareInterface::SetPosition()` / `SetTorque()` 当前返回 `void`，所以 Guard 不能直接把错误码返回给 Controller。

第一阶段推荐 Guard 内部记录 fault:

```cpp
class ServoGuard : public HardwareInterface {
public:
    Result GetLastGuardResult() const;
    void ClearGuardFault();

private:
    std::atomic<Result> last_result_{Result::NoError};
};
```

写命令流程:

```text
ServoGuard::SetPosition(q_cmd)
  -> if check failed:
       last_result_ = error
       不转发到底层 Hardware
       记录日志
       return
  -> inner_->SetPosition(q_cmd)
  -> last_result_ = NoError
```

第一阶段推荐由 `Robot::RunCycle()` 在 `executor->Update()` 之后检查 Guard:

```text
res = executor->Update()
guard_result = servo_guard->GetLastGuardResult()
if guard_result < 0:
  impl_->process_event(EventErrorOccurred)
```

这样无需立即修改 `Executor` 构造和接口。后续如果希望 `Executor::Update()` 直接返回 Guard 错误，可以给 `Executor` 增加可选的 `ServoGuard*` 或 `GuardStatusProvider*`。

`Robot::RunCycle()` 已经会在 `Executor::Update()` 返回负数时投递 `EventErrorOccurred`，进入 `ERROR_STATE`。Guard 错误也应复用同一条 FSM 错误路径。

## 11. 状态恢复

进入 `ERROR_STATE` 后:

- Guard 保留最近一次 fault 原因，便于日志和 HTTP 查询。
- `ResetFault()` 成功后应调用 `ServoGuard::ClearGuardFault()`。
- 切换工作模式时可清理上一控制器的 `last_cmd` 状态，但不应清除尚未复位的 fault。

建议新增状态查询:

```cpp
struct ServoGuardStatus {
    Result last_result;
    std::string last_message;
    int joint_index;
    std::string joint_name;
    double observed;
    double limit;
};
```

HTTP 可后续增加:

```text
GET /api/robot/servo_guard/status
```

第一阶段可以只写日志，不新增 HTTP API。

## 12. 与现有控制器的关系

### PositionController

原逻辑保持不变:

```text
GenerateCmd() 生成 q_cmd
UpdateCmd() 调用 SetPosition(q_cmd)
```

差异是 `hardware_` 指向 `ServoGuard`，所以 `SetPosition()` 被保护。

### JointAdmittanceController

最终仍是位置命令:

```text
q_out = q_des + q_adm
SetPosition(q_out)
```

走 `PositionCommandGuard`。这可以限制导纳积分漂移、外力导致的位置偏移过大、靠近软限位继续积分等风险。

### JointImpedanceController

最终是力矩命令:

```text
tau_cmd = Kp * (q_des - q_actual) - Kd * q_dot + tau_grav
SetTorque(tau_cmd)
```

走 `TorqueCommandGuard`。不要把 `q_des - q_actual` 作为普通位置跟随误差直接误杀，因为这是阻抗力矩来源。

### CartesianImpedanceController

内部已有力矩变化率和饱和保护，但仍要经过 `TorqueCommandGuard`。内部保护属于控制器局部策略，Guard 第一阶段只做来自硬件 limit 的全局兜底:

- `tau_cmd` 有限且不超过 `Drive::Limit.effort`。
- 当前 `q_actual` 不超过 `Drive::Limit.lower/upper`。
- 当前 `q_dot_actual` 不超过 `Drive::Limit.vel`。
- 靠近限位时不允许继续给外推方向力矩。

ServoGuard 不复制 `CartesianImpedanceController` 的力矩变化率策略，也不新增 Guard 自己的力矩变化率阈值。

## 13. 与 MoveServo 的关系

`MoveServo` 负责接收 UDP 指令并生成 `Reference`。它本身可以保留通信质量统计，但真正下发前仍应走 Guard。

推荐分工:

- `MoveServo`: 检查 UDP 包尺寸、message id、是否收到新命令、通信成功率统计。
- `ServoGuard`: 检查最终硬件命令和真实反馈状态。

对于 UDP 指令过期:

- 如果 `MoveServo` 能判断指令过期，应输出当前位置保持或 `PlanFinished`。
- 如果底层反馈/控制周期异常，`ServoGuard` 触发 fault。

## 14. 配置来源与简洁原则

第一阶段不新增 `config/servo_guard.yaml`。

原因:

- 现有 `hardware_talon_config.yaml` 已经包含每轴 `lower`、`upper`、`vel`、`acc`、`jerk`、`effort`。
- Guard 的核心保护应以这份硬件 limit 为单一事实来源。
- 额外 YAML 会引入第二套 limit/scale/margin 配置，增加不一致风险。
- 第一阶段目标是先把保护链路做短、做清楚，而不是增加运行时可调参数面。

第一阶段配置来源:

```text
per-joint hard/soft limits:
  hardware_talon_config.yaml -> Drive::Limit

model order limits:
  Robot::jointBinding() 后按 model_index -> drive_id 重排
```

第一阶段不引入 `ServoGuardPolicy` 这类策略结构体。Guard 直接使用硬件 limit:

```text
position lower/upper = Drive::Limit.lower/upper
velocity limit       = Drive::Limit.vel
acceleration limit   = Drive::Limit.acc
effort limit         = Drive::Limit.effort
```

实现中只允许使用不可配置的数值 epsilon 来处理浮点比较，例如 `1e-9` 级别的有限性和边界比较容差。不要引入 `velocity_scale`、`effort_scale`、`position_margin` 这类未被明确需求驱动的可调参数。

如果后续出现多机器人、多工况、现场调参需求，再根据真实需求把策略项加入已有硬件配置。第一阶段不做。

## 15. 第一阶段最小实现清单

1. 新增 `ServoGuard` 类，继承 `HardwareInterface`。
2. 读取类接口全部转发给真实 `HardwareInterface`。
3. `SetPosition()` 实现 Global + Position 检查。
4. `SetTorque()` 实现 Global + Torque 检查。
5. `Robot` 初始化时在 joint binding 后构建 model order limit 表。
6. `SetWorkMode()` 中把 `controller->SetHardware(hardware.get())` 改成 `controller->SetHardware(servo_guard.get())`。
7. `SetWorkMode()` 调用 `SetReady()` 前确认 `IsEnabled()`，设置 `GuardPhase::Ready`，进入运动控制状态后设置 `GuardPhase::Active`。
8. `Robot::RunCycle()` 在 `executor->Update()` 后检查 Guard fault。
9. `ResetFault()` 清理 Guard fault。
10. 添加单元测试覆盖:
   - binding 重排后 limit 与 q_cmd 对齐。
   - 位置命令超限触发 fault。
   - 位置跳变触发 fault。
   - 力矩超 effort 触发 fault。
   - 接近上限时正向力矩触发 fault。
   - 接近下限时负向力矩触发 fault。
   - `Ready` 阶段在已使能时允许当前位置锁定命令。
   - `Ready` 阶段未使能时拒绝 `SetPosition()` / `SetTorque()` / `SetMode()`。
   - `Active` 阶段硬件未 enabled 触发 fault。

## 16. 后续重构方向

长期更干净的架构是把 Controller 拆成:

```text
Controller::GenerateCommand()
CommandGuard::Check()
HardwareCommandSink::Write()
```

即 Controller 不再直接持有硬件指针，也不直接写 PDO。`Executor` 明确负责:

```text
Motion -> Controller -> Guard -> Hardware
```

但这会影响所有控制器接口，适合在第一阶段 Guard 跑稳定后再做。

## 17. 设计结论

第一阶段采用:

```text
Controller -> ServoGuard(HardwareInterface wrapper) -> Hardware
```

`ServoGuard` 统一接入所有控制器，但内部按命令类型分策略:

```text
GlobalStateGuard: 所有控制器共享
PositionCommandGuard: 位置/导纳/位置伺服
TorqueCommandGuard: 关节阻抗/笛卡尔阻抗
```

所有检查均使用 model joint order。当前关节状态通过 `inner_->GetPosition()` / `GetVelocity()` 读取，limit 表在 `Robot::jointBinding()` 后根据 `model_index -> drive_id` 映射构建，确保保护逻辑不会错轴。
