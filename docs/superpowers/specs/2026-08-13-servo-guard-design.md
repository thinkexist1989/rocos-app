# 控制器伺服命令保护设计方案

> 日期: 2026-08-13
> 范围: `Controller::UpdateCmd()` 下发前的每周期命令校验
> 状态: 设计方案，尚未进入实现

## 1. 背景

当前控制周期主链路为:

```text
MotionInterface::GenerateRef()
  -> ControllerInterface::GenerateCmd()
  -> ControllerInterface::UpdateCmd()
       -> HardwareInterface::SetPosition() / SetTorque()
```

`Motion` 层已经有启动前校验，例如目标点 IK、速度/加速度/jerk 参数合法性和离线轨迹检查。但这些校验发生在运动开始前，不能覆盖控制器每周期输出异常，例如:

- 位置命令突然跳变。
- UDP Servo 输入异常导致目标点突变。
- 导纳控制积分漂移。
- 阻抗控制输出 NaN/Inf 或力矩过大。
- 当前反馈速度异常。

因此第一阶段在每个 controller 的 `UpdateCmd()` 内增加下发前验证函数。验证失败时直接返回错误，不调用 `SetPosition()` / `SetTorque()`。

## 2. 第一阶段设计结论

第一阶段不新增 `ServoGuard : HardwareInterface` 包装层，改为轻量方案:

```text
Controller::UpdateCmd(q_cmd)
  -> ValidatePositionCommand(q_cmd) / ValidateTorqueCommand(tau_cmd)
  -> 校验通过后调用 hardware_->SetPosition() / SetTorque()
```

limit 来源采用:

```text
每周期 controller 校验: 使用 Model / URDF limit
底层硬件保护: 保留 hardware config / driver 自身 limit
启动一致性检查: Robot 初始化时比较 URDF limit 与 hardware limit
```

这样 controller 的校验只依赖 model order 的数据:

```text
q_cmd[i]
q_actual[i]
tau_cmd[i]
model_limit[i]
```

不需要在 controller 中处理 `model_index -> drive_id` 映射，也不需要读取真实 hardware config。

## 3. 为什么每周期校验使用 URDF / Model limit

`Controller`、`Motion`、`Model`、KDL/TRAC-IK 使用同一套 model joint order。`Hardware::GetPosition()` 当前也已经按 joint binding 返回 model order。因此 controller 中的命令和反馈天然对齐:

```text
q_cmd[i]      -> model joint i
q_actual[i]   -> model joint i
tau_cmd[i]    -> model joint i
URDF limit[i] -> model joint i
```

如果 controller 每周期直接使用 hardware limit，就必须处理:

```text
model_index -> drive_id
hardware drive order
hardware config drive order
```

这会把硬件映射细节带进控制器，增加错轴风险。第一阶段为了简洁和可读性，controller 只使用 `ModelInterface` 暴露的 URDF limit。

约束原则是: **URDF limit 必须是控制层认可的软件安全限制**。如果真实硬件限制比 URDF 更窄，应修改 URDF，使 URDF 更保守。

## 4. Model 需要暴露的 limit

当前 `Model` 已经从 URDF 解析位置上下限，并暴露:

```cpp
const JntArray& GetPosLowerLimit() const noexcept;
const JntArray& GetPosUpperLimit() const noexcept;
```

第一阶段需要补齐 URDF joint limit 的 `velocity` 和 `effort` 解析，并通过 `ModelInterface` 暴露:

```cpp
virtual const JntArray& GetPosLowerLimit() const = 0;
virtual const JntArray& GetPosUpperLimit() const = 0;
virtual const JntArray& GetVelocityLimit() const = 0;
virtual const JntArray& GetEffortLimit() const = 0;
```

`Model::ParseUrdf()` 按 KDL chain 中可动关节顺序解析:

```text
lower    -> q_min_[i]
upper    -> q_max_[i]
velocity -> q_vel_[i]
effort   -> q_effort_[i]
```

URDF 标准 joint limit 有 `lower`、`upper`、`velocity`、`effort`。第一阶段不从 URDF 解析 `acc` / `jerk`，controller 每周期保护也不使用命令加速度检查。

如果某个 URDF joint 缺少必要 limit，建议初始化失败或返回 `Result::IllegalParameter`。不要静默使用过宽默认值，否则保护意义会被削弱。

## 5. 位置类控制器校验

适用控制器:

- `PositionController`
- `JointAdmittanceController`
- 后续 joint position servo 控制器

调用位置:

```text
PositionController::UpdateCmd(q_cmd)
  -> ValidatePositionCommand(q_cmd)
  -> hardware_->SetPosition(q_cmd)

JointAdmittanceController::UpdateCmd(q_des)
  -> 计算 q_out
  -> ValidatePositionCommand(q_out)
  -> hardware_->SetPosition(q_out)
```

校验内容:

```text
hardware_ != nullptr
model_ != nullptr
dt > 0
q_cmd 维度 == joint_num
q_actual 维度 == joint_num
q_cmd 全部有限
q_actual 全部有限
q_cmd[i] 在 [lower[i], upper[i]] 内
abs(q_cmd[i] - q_actual[i]) / dt <= velocity_limit[i]
```

位置模式只保留一个速度约束:

```text
required_velocity[i] = (q_cmd[i] - q_actual[i]) / dt
```

如果:

```text
abs(required_velocity[i]) > velocity_limit[i]
```

则直接返回 `Result::SpeedLimit`，不调用 `hardware_->SetPosition(q_cmd)`。

第一阶段不检查:

```text
(q_cmd - q_last_cmd) / dt
命令加速度
命令 jerk
```

原因是 `q_cmd - q_actual` 已经直接描述“本周期如果接受这个目标，伺服从当前真实位置追到目标需要多快”。它能覆盖位置跳变、UDP 目标突变和导纳积分漂移这类核心风险。保留一个速度约束更容易理解、调试和验证。

## 6. 力矩类控制器校验

适用控制器:

- `JointImpedanceController`
- `CartesianImpedanceController`

调用位置:

```text
JointImpedanceController::UpdateCmd(q_des)
  -> 计算 tau_cmd
  -> ValidateTorqueCommand(tau_cmd)
  -> hardware_->SetTorque(tau_cmd)

CartesianImpedanceController::UpdateCmd(q_des)
  -> 计算 tau_cmd
  -> 控制器内部已有局部力矩变化率处理
  -> ValidateTorqueCommand(tau_cmd)
  -> hardware_->SetTorque(tau_cmd)
```

校验内容:

```text
hardware_ != nullptr
model_ != nullptr
tau_cmd 维度 == joint_num
q_actual 维度 == joint_num
q_dot_actual 维度 == joint_num
tau_cmd 全部有限
q_actual 全部有限
q_dot_actual 全部有限
q_actual[i] 在 [lower[i], upper[i]] 内
abs(q_dot_actual[i]) <= velocity_limit[i]
abs(tau_cmd[i]) <= effort_limit[i]
```

阻抗控制不能把 `q_des - q_actual` 当作普通位置跟随误差检查。这个误差本身是阻抗控制生成力矩的来源。力矩校验只检查当前状态安全、力矩大小安全。

第一阶段不在统一校验里新增:

```text
tau_rate_limit
机械功率限制
力矩 clamp
靠近限位 margin
```

如果某个 controller 内部已经有力矩变化率限制，例如 `CartesianImpedanceController`，继续保留在该 controller 内部。统一验证函数只做来自 URDF limit 的兜底检查。

## 7. Ready 阶段校验

`SetReady()` 也会准备硬件目标，因此仍需要走同一套下发前校验。第一阶段要求 Ready 阶段的目标写入也通过 `UpdateCmd()`，不要在 `SetReady()` 中直接调用 `SetPosition()` / `SetTorque()`。

位置类 controller:

```text
q_ready = GetPosition()
UpdateCmd(q_ready)
```

力矩类 controller:

```text
q_hold = GetPosition()
UpdateCmd(q_hold)
```

第一阶段规则:

```text
SetReady() 必须在硬件已经 ENABLED 后调用
Ready 目标写入必须经过 UpdateCmd()
UpdateCmd() 内部统一调用 ValidatePositionCommand() / ValidateTorqueCommand()
```

原因是如果在未使能状态写目标位置或目标力矩，驱动可能缓存目标值，并在后续上使能瞬间产生非预期运动。

Ready 阶段的位置写入通常是:

```text
q_ready = hardware_->GetPosition()
rc = UpdateCmd(q_ready)
if rc != Result::NoError:
  return rc
```

Ready 阶段的力矩写入通常是:

```text
q_hold = hardware_->GetPosition()
rc = UpdateCmd(q_hold)
if rc != Result::NoError:
  return rc
```

注意: 当前 `ControllerInterface::UpdateCmd(const JntArray&)` 对力矩类 controller 的参数语义仍是 `q_des`，不是 `tau_cmd`。因此 Ready 阶段不要把 `gravity_compensation` 当作参数传给 `UpdateCmd()`。力矩类 controller 应使用当前关节角作为保持目标，让 `UpdateCmd(q_hold)` 内部生成重力补偿或阻尼后的 `tau_cmd`，再执行 `ValidateTorqueCommand(tau_cmd)` 和 `SetTorque(tau_cmd)`。

这样 `SetReady()`、运动周期和伺服周期都复用同一个下发入口:

```text
UpdateCmd()
  -> Validate...
  -> SetMode(CSP/CST) if needed
  -> SetPosition() / SetTorque()
```

`SetReady()` 只负责准备 ready 命令，不再直接 `SetMode()`，也不再绕过 `UpdateCmd()` 写运动目标。

## 8. 析构函数收尾

当前多个 controller 析构函数中存在兜底写入:

```text
SetPosition(GetPosition())
SetMode(CSP)
```

这条路径也会绕过 `UpdateCmd()`，因此第一阶段需要同步处理。

推荐规则:

```text
controller 析构函数不直接调用 SetPosition() / SetTorque() / SetMode()
如果 hardware_ 为空，直接返回
如果硬件未 ENABLED，不写目标命令，避免驱动缓存目标
如果硬件已 ENABLED，读取 q_hold = hardware_->GetPosition()
调用本 controller 的 UpdateCmd(q_hold)
析构函数只记录 UpdateCmd() 的错误，不抛异常
```

伪代码:

```text
~Controller()
  if hardware_ == nullptr:
    return
  if hardware_->GetState() != ENABLED:
    return

  q_hold = hardware_->GetPosition()
  rc = UpdateCmd(q_hold)
  if rc != Result::NoError:
    log error
```

这使析构收尾、Ready 阶段和运动周期统一走:

```text
UpdateCmd()
  -> Validate...
  -> SetMode(CSP/CST) if needed
  -> SetPosition() / SetTorque()
```

注意: 析构函数不能返回 `Result`，也不适合承担主要状态机切换职责。更理想的长期做法是在 `Robot::SetWorkMode()` 销毁旧 controller 前显式调用一个可返回错误的收尾函数，例如 `PrepareForSwitch()`。第一阶段为了少改接口，先让析构函数走 `UpdateCmd(q_hold)` 并记录错误。

力矩类 controller 析构时同样传入 `q_hold`，不要直接写 `SetTorque(gravity_compensation)`。`UpdateCmd(q_hold)` 内部会按该 controller 的语义生成 `tau_cmd`，再经过 `ValidateTorqueCommand(tau_cmd)`。

## 9. Hardware limit 的角色

`hardware_talon_config.yaml` / `hardware_driver_config.yaml` 中的 `Drive::Limit` 继续保留，但第一阶段不作为 controller 每周期校验的直接来源。

它的角色是:

```text
底层硬件配置
驱动自身保护
启动时与 URDF limit 做一致性检查
```

推荐在 `Robot` 初始化、`jointBinding()` 完成后做一次一致性检查:

```text
urdf.lower >= hardware.lower
urdf.upper <= hardware.upper
urdf.velocity <= hardware.vel
urdf.effort <= hardware.effort
```

这一步需要使用已有的映射:

```text
model_index -> drive_id
```

但它只发生在启动阶段，不进入 controller 每周期控制路径。

如果不一致，推荐直接初始化失败或进入错误状态。至少要输出明确日志，包含:

```text
model_index
joint_name
drive_id
URDF limit
hardware limit
```

## 10. 错误处理

验证函数返回 `Result`。

推荐错误码:

```text
Result::ParameterPointerEqualsNullptr
Result::IllegalParameter
Result::PosLimit
Result::SpeedLimit
Result::ForceLimit
```

每个 `UpdateCmd()` 的处理方式:

```text
rc = ValidatePositionCommand(q_cmd) / ValidateTorqueCommand(tau_cmd)
if rc != Result::NoError:
  记录日志
  return rc

hardware_->SetPosition(q_cmd) / SetTorque(tau_cmd)
return Result::NoError
```

`Executor::Update()` 已经会接收 controller 的返回值。返回负值后，`Robot::RunCycle()` 复用现有 FSM 错误路径进入错误处理。

## 11. 第一阶段实现清单

1. `Model` 解析 URDF joint limit 的 `velocity` 和 `effort`。
2. `ModelInterface` / `Model` 暴露 `GetVelocityLimit()` 和 `GetEffortLimit()`。
3. `PositionController` 增加 `ValidatePositionCommand(q_cmd)`。
4. `JointAdmittanceController` 增加或复用 `ValidatePositionCommand(q_out)`。
5. `JointImpedanceController` 增加 `ValidateTorqueCommand(tau_cmd)`。
6. `CartesianImpedanceController` 增加或复用 `ValidateTorqueCommand(tau_cmd)`。
7. `Robot` 初始化阶段增加 URDF limit 与 hardware limit 的一致性检查。
8. `SetReady()` 要求硬件已 `ENABLED`，并通过 `UpdateCmd()` 完成 ready 目标下发。
9. controller 析构函数移除直接硬件写入，改为已使能时调用 `UpdateCmd(q_hold)`，未使能时不写目标。
10. 添加单元测试覆盖 Model limit 解析和 controller 下发前拒绝逻辑。

## 12. 测试重点

Model 测试:

- 能从 URDF 解析 `lower` / `upper` / `velocity` / `effort`。
- limit 数组维度等于 `GetJointNum()`。
- joint 顺序与 `GetJointNames()`、KDL `JntArray` 顺序一致。
- 缺少必要 limit 时返回错误或初始化失败。

位置控制器测试:

- `q_cmd` 超过位置上限，拒绝下发。
- `q_cmd` 低于位置下限，拒绝下发。
- `q_cmd` 有 NaN/Inf，拒绝下发。
- `(q_cmd - q_actual) / dt` 超过速度限制，拒绝下发。
- 正常命令通过并调用 `SetPosition()`。
- `SetReady()` 调用 `UpdateCmd(q_ready)`，不直接调用 `SetPosition()`。
- 析构函数在已使能时调用 `UpdateCmd(q_hold)`，不直接调用 `SetPosition()` / `SetMode()`。
- 析构函数在未使能时不写目标命令。

力矩控制器测试:

- `tau_cmd` 超过 effort，拒绝下发。
- `tau_cmd` 有 NaN/Inf，拒绝下发。
- 当前 `q_actual` 已越界，拒绝下发。
- 当前 `q_dot_actual` 超过 velocity，拒绝下发。
- 正常命令通过并调用 `SetTorque()`。
- `SetReady()` 调用 `UpdateCmd(q_hold)`，不直接调用 `SetTorque()`。
- 析构函数在已使能时调用 `UpdateCmd(q_hold)`，不直接调用 `SetTorque()` / `SetMode()`。
- 析构函数在未使能时不写目标命令。

启动一致性测试:

- URDF limit 比 hardware limit 更宽时，初始化失败或返回明确错误。
- joint binding 后，检查使用正确的 `model_index -> drive_id` 对应关系。

## 13. 后续演进

如果后续 controller 数量明显增加，或者出现 controller 之外直接写 hardware 的路径，可以再升级为统一的 `ServoGuard` 写入包装层。

第一阶段先保持简单:

```text
controller 内验证函数
Model / URDF 提供控制层 limit
hardware limit 只做底层配置和启动一致性检查
验证失败直接拒绝下发
```

这样能先把最关键的伺服保护落到每周期路径上，同时避免引入额外配置文件、策略结构体和复杂生命周期。
