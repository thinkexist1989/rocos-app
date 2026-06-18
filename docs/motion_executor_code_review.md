# Motion Executor 实现审查（对照设计文档）

本文档对照 `docs/motion_fsm_executor_design.md`（以下简称"设计文档"），审查当前 motion executor 架构的完整实现。审查覆盖 MoveJ/MoveL/MoveC 三个阶段的成果，重点标注与设计规格的偏差、已知缺陷和测试空白。

**审查基线**：`docs/motion_fsm_executor_design.md`（2226 行，含 DianaApi 错误码、17+ 场景、5 个实现阶段）

**实现范围**：MoveJCommand + MotionExecutor 基础（阶段 1）→ MoveL/MoveC + ModelProvider + CartesianGeometry（阶段 2）

**测试结果**：62/62 通过（排除已知 `unit_test` 既有崩溃），涵盖 12 个测试可执行文件。

---

## 1. 接口签名对照表

逐项对比设计文档建议的签名与当前实现签名。

### 1.1 MotionCommand

| 接口 | 设计文档签名 | 实现签名 | 偏差 |
|------|-------------|----------|------|
| `prepare` | `prepare(MotionContext& ctx, ModelProvider& model)` | `prepare(MotionContext& ctx, ModelProvider& model)` | **一致** |
| `start` | `start(MotionContext& ctx)` | `start(MotionContext& ctx)` | **一致** |
| `update` | `update(MotionContext& ctx, ModelProvider& model, ReferenceSpace required)` | `update(MotionContext& ctx, ModelProvider& model, bool required = true)` | **⚠ 偏差：第三参数类型从 `ReferenceSpace` 简化为 `bool`** |
| `pause` | `pause(MotionContext& ctx)` | `pause()` — 无参数 | **⚠ 偏差：缺少 `MotionContext&` 参数** |
| `resume` | `resume(MotionContext& ctx)` | `resume()` — 无参数 | **⚠ 偏差：同上** |
| `stop` | `stop(MotionContext& ctx)` | `stop()` — 无参数 | **⚠ 偏差：同上** |
| `producedReferenceSpace` | 默认返回 `ReferenceSpace::None` | 默认返回 `ReferenceSpace::None` | **一致** |

### 1.2 FiniteMotionCommand

| 接口 | 设计文档签名 | 实现签名 | 偏差 |
|------|-------------|----------|------|
| `profileLimits` | `profileLimits(MotionContext& ctx) const` | `profileLimits() const` | **⚠ 偏差：缺少 `MotionContext&` 参数** |
| `sample` | `sample(MotionContext& ctx, ModelProvider& model, double s, double s_dot, double s_ddot, ReferenceSpace required)` | `sample(double s, double s_dot, double s_ddot) const` | **⚠ 偏差：缺少 ctx/model/required 参数** |

### 1.3 MotionController

| 接口 | 设计文档签名 | 实现签名 | 偏差 |
|------|-------------|----------|------|
| `activate` | `activate(MotionContext& ctx, ModelProvider& model)` | `activate()` — 无参数 | **⚠ 偏差：缺少 ctx/model** |
| `update` | `update(MotionContext& ctx, ModelProvider& model, const MotionReference& ref, LowLevelCommand& out)` | `update(const MotionReference& ref, LowLevelCommand& out)` | **⚠ 偏差：缺少 ctx/model** |
| `deactivate` | `deactivate(MotionContext& ctx)` | `deactivate()` — 无参数 | **⚠ 偏差：缺少 ctx** |
| `safeStop` | `safeStop(MotionContext& ctx)` | `safeStop()` — 无参数 | **⚠ 偏差：缺少 ctx** |

### 1.4 MotionContext

| 设计文档建议方法 | 是否实现 | 说明 |
|----------------|---------|------|
| `jointCount()` | ❌ | 未实现 |
| `jointPositions()` | ❌ | 未实现，通过 `readStateSnapshot().q_actual` 间接访问 |
| `jointVelocities()` | ❌ | 同上 |
| `jointMode(int id)` | ❌ | 未实现 |
| `isEnabled()` | ❌ | 未实现，通过 `readStateSnapshot().enabled` 间接访问 |
| `flangePose()` | ❌ | 未实现 |
| `readStateSnapshot()` | ✅ | 已实现 |
| `maxJointVelocity()` | ❌ | 未实现 |
| `maxJointAcceleration()` | ❌ | 未实现 |
| `maxJointJerk()` | ❌ | 未实现 |
| `controlPeriod()` | ✅ | 已实现 |
| `setJointPosition(id, q)` | ❌ | 未实现（通过 `writeLowLevelCommand` 间接写入） |
| `setJointVelocity(id, dq)` | ❌ | 同上 |
| `setJointTargets(q)` | ❌ | 同上 |
| `writeLowLevelCommand(cmd)` | ✅ | 已实现，`GuardedMotionContext` 封装了安全检查 |
| `waitControlCycle()` | ❌ | 未在 `MotionContext` 基类声明（`RobotMotionContext` 内部实现） |
| `logger()` | ❌ | 未实现 |

**评价**：MotionContext 只实现了 3/16 个建议方法。当前 command 和 controller 无法在 `prepare()` 或 `update()` 中直接读取关节状态、模式、限位等信息，只能通过 `readStateSnapshot()` 获取一个结构体快照。这在初期可以接受，但后续需要收窄时逐个暴露。

### 1.5 ModelProvider

| 设计文档建议 | 实现 | 偏差 |
|-------------|------|------|
| `class ModelProvider` with `kinematics()` / `dynamics()` methods | `struct ModelProvider { KinematicsAdapter kinematics; }` — 裸 struct | **⚠ 简化：无方法封装，无 validation** |
| `KinematicsAdapter`: FK / IK / Jacobian / singularity check | FK / IK / getDof（三个 `std::function`） | **⚠ 简化：无 Jacobian、奇异性检查** |
| `DynamicsAdapter`: M(q) / C(q,q_dot) / G(q) / Lambda(q) / torque feasibility | 完全未实现 | **⚠ 缺失：动力学模型接口尚未落地** |

### 1.6 MotionStepResult / MotionResult

| 设计文档建议 | 实现 | 偏差 |
|-------------|------|------|
| `MotionStepResult { status, code, message, reference }` — 平铺 | `MotionStepResult { status, result: MotionResult, reference }` — 嵌套 | **⚠ 偏差：code/message 通过嵌套的 MotionResult 间接访问** |
| `MotionSampleResult { status, code, message, reference }` — 独立的 sample 结果类型 | 不存在此类型 | **⚠ 缺失：sample() 直接返回 MotionReference** |
| `MotionTaskError { result, api_error_code, message, source }` | 不存在 | **⚠ 缺失：错误来源字段未实现** |

### 1.7 MotionReference / LowLevelCommand

| 设计文档建议 | 实现 | 偏差 |
|-------------|------|------|
| `JointReference { KDL::JntArray q, q_dot, q_ddot }` | `JointReference { vector<double> position, velocity, acceleration }` | **⚠ 偏差：使用 `std::vector<double>` 而非 `KDL::JntArray`** |
| `CartesianReference { KDL::Frame pose, KDL::Twist twist, acceleration }` | 不存在 | **⚠ 缺失：笛卡尔参考类型未实现** |
| `MotionReference { space, optional<JointReference>, optional<CartesianReference> }` | `MotionReference { space, JointReference joint }` — joint 非 optional | **⚠ 偏差：Joint 直接嵌入，Cartesian 缺失** |
| `LowLevelCommand { optional<JntArray> target_position/velocity/torque }` | `LowLevelCommand { vector<double> target_position/velocity/torque }` + `hasTarget*()` | **⚠ 偏差：使用 vector + has 方法而非 optional** |

---

## 2. 关键偏差与缺陷

按严重性分级。Critical 必须在上线前修复，High 影响正确性或安全性，Medium 影响健壮性，Low 为代码质量问题。

### 2.1 Critical（上线前必修）

#### C-1: MoveLCommand::sample() 输出速度/加速度始终为零

**文件**：`include/rocos_app/motion/move_l_command.h:111-116`

```cpp
ref.joint.velocity.resize(dof_);      // resize 默认初始化为 0.0
ref.joint.acceleration.resize(dof_);   // 同上
```

`sample()` 接收 `s_dot` 和 `s_ddot` 参数但**完全忽略**，速度/加速度输出全零。这意味着下游 PositionController 透传的 `target_velocity` 全为零，MotionSafetyGuard 的命令速度检查只看到零速度（永远通过），硬件不会收到速度前馈。

**对比 MoveJCommand::sample()**（`move_j_command.h:108`）正确使用 `s_dot * delta`。

**影响**：MoveL 运动时硬件无速度前馈；位置控制精度和运动平滑性可能受损；安全检查被绕过。

**修复建议**：
```cpp
// 需要计算笛卡尔路径上的关节速度/加速度
// 简化方案：数值差分
//   q_dot[i] = (q_out[i] - q_prev[i]) / dt
// 或解析方案：利用雅可比 J(q) 映射笛卡尔速度
//   q_dot = J^{-1}(q) * v_cart * s_dot
```

#### C-2: IK lambda 捕获 `&robot` 引用，存在悬空引用风险

**文件**：`include/rocos_app/motion/move_l_submission.h:72`、`move_c_submission.h:76,166`

```cpp
MoveLCommand::IkCallback ik = [&robot](...) -> bool {
    ...
    robot.CartToJnt(q_init, target, q_result)
    ...
};
```

Submission 函数创建 IK lambda 时按引用捕获 `robot`。lambda 被 move 到 MoveLCommand 中，由 worker 线程在后续控制周期中调用。如果 `Robot` 对象在 command 生命周期内被析构（如程序退出、异常路径），lambda 中的 `&robot` 成为悬空引用。

同样的问题出现在 `Robot::initializeMotionExecutor()` 的 FK/IK lambda 中（`robot.cc:746-778`），那里捕获 `[this]`。虽然 `Robot` 持有 `motion_executor_`，析构时会先析构 executor（stop + joinWorker），但如果 executor 的 worker 线程正在执行 `update()` → `sample()` → `ik_callback_()` 的瞬间 Robot 开始析构，仍可能出现竞态。

**影响**：UAF（Use-After-Free），可能导致崩溃或数据损坏。

**修复建议**：使用 `std::weak_ptr` + `shared_from_this`，或在 `~Robot()` 中显式 stop + join worker 线程后再析构 executor。

#### C-3: MotionExecutor::stop() 不设置 stop_worker_

**文件**：`include/rocos_app/motion/motion_executor.h:204-227`

```cpp
MotionResult stop() {
    ...
    auto result = current_->stop();
    if (result.success) {
        task_status_ = MotionTaskStatus::Stopping;
    }
    return result;
}
```

`stop()` 调用 `current_->stop()`（触发 profile 减速），但**不设置 `stop_worker_ = true`**。workerLoop 只能依赖 command 自然进入 Stopped 状态后退出循环。如果 command 的 stop 实现有缺陷（如 profile 减速失败、IK 失败导致无法继续 sample），workerLoop 将永远运行。

**文件**：`motion_executor.h:252-257`

```cpp
void workerLoop() {
    for (;;) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!current_ || stop_worker_) { return; }  // stop_worker_ 只在 ~MotionExecutor 时设置
```

**影响**：如果 FiniteMotionCommand::stop() 成功但 profile 永远不完成（如内部 bug），workerLoop 不会退出，executor 无法接受新 command。

**修复建议**：`stop()` 在设置 `Stopping` 后同时设置 `stop_worker_ = true` 作为兜底；或增加超时机制。

#### C-4: initializeMotionExecutor() 可被重复调用，无互斥保护

**文件**：`src/robot.cc:716-786`

`initializeMotionExecutor()` 在 MoveJ/MoveL/MoveC 等多个入口被调用（robot.cc:1041, 1114, 1140, 1153, 1203, 1565, 1633），每次都重建整个 executor 栈（safety guard、FSM gateway、controller、context、model provider、executor）。如果两个 HTTP 线程同时调用 MoveL 和 MoveC，可能并发进入 `initializeMotionExecutor()`，导致 unique_ptr 被覆盖、旧对象被析构时仍有线程在使用。

```cpp
if (!motion_executor_) {
    initializeMotionExecutor();  // 无 mutex，多线程可并发进入
}
```

**影响**：数据竞争、double-free、UAF。

**修复建议**：使用 `std::call_once` + `std::once_flag`，或在 Robot 构造时一次性初始化。

### 2.2 High（影响正确性）

#### H-1: update() 第三参数 `bool required` 偏离设计的 `ReferenceSpace required`

**文件**：`motion_command.h:28-30`

设计文档意图是让 executor 把 active controller 的 `acceptedReferenceSpace()` 传给 command，command 据此决定产出 Joint 还是 Cartesian reference。实现用 `bool` 代替，丢失了"需要哪种 reference"的信息。

**影响**：当前所有 command 只产出 Joint reference、PositionController 只接受 Joint reference，所以暂时没有功能影响。但添加 CartesianImpedanceController 时会暴露此问题。

#### H-2: pause()/resume()/stop() 缺少 MotionContext& 参数

**文件**：`motion_command.h:32-42`

设计文档要求 `pause(ctx)`、`resume(ctx)`、`stop(ctx)`，使 command 可以在暂停/恢复/停止时读取机器人状态（如当前关节位置、使能状态）。实现中这些方法无参数。

**影响**：command 无法在 stop 时读取状态做安全决策（如判断是否需要紧急停止）。

#### H-3: MoveC velocity 标量广播到所有关节（物理不正确）

**文件**：`move_c_three_point_command.h:125-126`、`move_c_center_angle_command.h:131-132`

```cpp
ref.joint.velocity.assign(dof_, s_dot * path_length_);
ref.joint.acceleration.assign(dof_, s_ddot * path_length_);
```

所有关节的速度/加速度被设为同一个标量 `s_dot * path_length_`。不同关节在圆弧运动中实际速度差异很大（靠近旋转中心的关节速度慢，远离的快），标量广播会导致：
- PositionController 透传错误的速度前馈
- SafetyGuard 的命令速度检查基于错误的速度值

**对比 MoveLCommand**（虽然 MoveL 也有 C-1 的零速度问题，但至少没有广播错误标量）。

#### H-4: quaternionSlerp 结果未归一化

**文件**：`cartesian_geometry.h:47-81`

`detail::quaternionSlerp()` 计算插值后直接返回结果，**未做归一化**。浮点误差会导致四元数模长偏离 1.0，随着 s 从 0 到 1 逐步积累，中间姿态可能出现微小的旋转矩阵非正交。

**影响**：`KDL::Rotation::Quaternion()` 接受非单位四元数时会隐式归一化，但如果偏差较大可能产生精度问题。IK 求解器接收非正交旋转矩阵可能收敛变慢或失败。

#### H-5: workerLoop 使用 1ms sleep 而非实时同步

**文件**：`motion_executor.h:306`

```cpp
std::this_thread::sleep_for(std::chrono::milliseconds(1));
```

设计文档要求控制周期中调用 `ctx.waitControlCycle()`（封装 `hw_interface_->waitForSignal(0)`）同步硬件时钟。实现用固定 1ms sleep，造成：
- 实际周期 ≈ 1ms + update/dispatch 时间 ≈ 1.5-2ms（≈ 2 倍误差）
- 与硬件 EtherCAT 周期不同步
- 高优先级运动控制可能被 OS 调度延迟

**修复建议**：在 workerLoop 中增加 `context_->waitControlCycle()` 调用（需要先在 MotionContext 基类暴露该方法）。

#### H-6: circular interpolation 始终使用 start 朝向

**文件**：`cartesian_geometry.h:118`

```cpp
return KDL::Frame(start.M, center_frame * arc_pos);
```

`interpolateCircular()` 无论 `orientation_fixed` 参数如何，都使用 `start.M` 作为整个弧的朝向。虽然 command 层面在 `orientation_fixed=true` 时显式覆盖（`move_c_three_point_command.h:105-107`），但 `interpolateCircular` 本身的注释说"orientation 保持 start 的不变"，这意味着它**不支持弧线朝向插值**。

设计文档 `MoveLCommand::sample()` 描述 `pose_ref.M = slerp(R_start, R_goal, s)`，对 MoveC 应类似。当前实现无法在圆弧运动中同时改变位置和朝向。

### 2.3 Medium（影响健壮性）

#### M-1: producedReferenceSpace() 默认 None 绕过兼容性检查

**文件**：`motion_executor.h:350-353`

```cpp
const auto produced = command.producedReferenceSpace();
if (produced == ReferenceSpace::None) {
    return MotionResult::ok();  // 直接放行
}
```

设计文档建议默认 None 应阻止兼容性检查绕过，确保每个 command 都显式声明产出空间。当前实现中，如果新 command 忘记 override `producedReferenceSpace()`，它会默认 None 并绕过检查，直接进入执行——controller 可能收到无法处理的 reference 类型。

**修复建议**：`produced == None` 时应返回失败而非成功。

#### M-2: MoveL/MoveC Robot 入口无 speed/accel 范围验证

**文件**：`src/robot.cc:1213-1219`

```cpp
const auto submit_result = motion::submitMoveL(
    *this, *motion_executor_, pose, speed, acceleration, DELTA_T);
```

`Robot::MoveL` 直接将用户传入的 `speed` 和 `acceleration` 透传给 submission 函数，不做上限检查。submission 函数只检查 `> 0` 和 `isfinite`，不检查是否超出机器人物理极限。

**对比旧路径**：`MoveL_pos` 会调用 `CheckBeforeMove()` 做更全面的检查。

**影响**：过大的速度/加速度可能导致运动规划产生不切实际的轨迹。

#### M-3: submit() busy 检查存在 TOCTOU

**文件**：`motion_executor.h:79-88`

```cpp
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_) {
        auto result = MotionResult::fail(MotionResultCode::Busy, ...);
        return result;
    }
}
// 此处释放锁 → joinWorker() → 再次获取锁设置 current_
// 另一线程可在此窗口内 submit
```

busy 检查后释放锁、调用 `joinWorker()`，然后在 `requestStart()` 成功后才重新获取锁设置 `current_`。在这个窗口中，另一个线程的 submit 也可能通过 busy 检查。

**影响**：两个 HTTP 线程可能同时 submit 不同的 command，第二个会覆盖第一个。

#### M-4: MoveL/MoveC 同步等待无超时

**文件**：`src/robot.cc:1225-1233`

```cpp
if (!asynchronous) {
    while (motion_executor_->hasActiveCommand()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ...
}
```

同步调用忙等待，无超时机制。如果 executor 出错但 hasActiveCommand 一直返回 true（如 C-3 场景），HTTP 线程将永远阻塞。

#### M-5: MotionExecutor 缺少 task_id 生成

设计文档建议 `submit()` 返回 task_id 供 `/api/move/status?task_id=...` 查询。实现中 `submit()` 只返回 `MotionResult`，没有 task_id 字段。当前 MoveJ/MoveL/MoveC 的 task 查询仍依赖旧的 task map。

#### M-6: computeCircleCenter 叉积顺序 v2×v1

**文件**：`cartesian_geometry.h:132`

```cpp
KDL::Vector axis_z{v2 * v1};  // 注释标注"与 legacy circle_center 一致"
```

数学上三点圆的法向量应为 `(p2-p1) × (p3-p1)` 即 `v1 × v2`。当前使用 `v2 × v1`（反序），虽然与 legacy 保持一致，但会产生反向法向量，可能导致后续 `computeCircleArcParams` 中的 theta 方向与直觉相反。

### 2.4 Low（代码质量）

#### L-1: JointReference 使用 std::vector<double> 而非 KDL::JntArray

设计文档建议使用 `KDL::JntArray`，与运动学库类型一致。实现使用 `std::vector<double>` 简化了初始化和传递，但失去了 KDL 的类型安全和运算符重载。

#### L-2: MotionResultCode 未在设计文档中完全对应

设计文档定义了 `MotionResultCode` 枚举（Ok, Busy, InvalidCommand, InvalidState, Unsupported, PlanningFailed, ExecutionFailed, SafetyViolation, HardwareFault）。实现中额外增加了 `InvalidNumber`（用于 NaN/Inf 专用），这是合理的扩展但文档未同步。

#### L-3: 测试中 mock IK 不验证关节极限

`test/motion_core_test.cc` 中的 2-DOF planar IK mock 只检查数学可达性，不验证关节角度是否在物理限位内。这意味着测试无法发现 command 产出超限位关节角的问题。

#### L-4: cartesian_geometry.h 依赖 finite_motion_command.h

`cartesian_geometry.h` 只需要 `MotionProfileLimits` 结构体，但包含了整个 `finite_motion_command.h`（间接包含 `motion_command.h` 和 `UnitIntervalMotionProfile.h`）。应抽取 `MotionProfileLimits` 到独立头文件。

---

## 3. 各组件详细审查

### 3.1 MotionCommand

| 评估项 | 状态 | 说明 |
|-------|------|------|
| 接口抽象 | ✅ | prepare/start/update/pause/resume/stop 生命周期完整 |
| 不持有 Robot* | ✅ | command 通过 ctx/model 访问状态 |
| 能力声明 | ✅ | supportsPause/Resume/Stop 已实现 |
| 签名完整性 | ⚠ | update 第三参数 bool 简化；pause/resume/stop 缺 ctx |
| producedReferenceSpace | ⚠ | 默认 None 绕过了兼容性检查（M-1） |

### 3.2 FiniteMotionCommand

| 评估项 | 状态 | 说明 |
|-------|------|------|
| UnitIntervalMotionProfile 封装 | ✅ | 复用已有 S-curve profile |
| start/pause/resume/stop | ✅ | 统一实现，子类无需重复 |
| update 驱动 sample | ✅ | profile.update → sample(s, s_dot, s_ddot) |
| IK 失败处理 | ✅ | space=None → status_=Failed |
| stop 完成判断 | ✅ | `stopping_ && IsStopCompleted()` |
| pause 完成判断 | ✅ | `pause_requested_ && !IsActive() && IsStopped()` |
| profileLimits 签名 | ⚠ | 缺少 MotionContext& 参数（H-2 相关） |
| sample 签名 | ⚠ | 缺少 ctx/model/required 参数 |

### 3.3 MotionExecutor

| 评估项 | 状态 | 说明 |
|-------|------|------|
| submit 生命周期 | ✅ | null check → busy → FSM gate → prepare → compatibility → start → activate → worker |
| pause/resume/stop | ✅ | 检查 command 能力 → FSM gate → command 操作 |
| workerLoop dispatch | ✅ | Running → dispatchReference → Finished/Stopped/Failed/Paused |
| controller activate/deactivate | ✅ | submit 成功时 activate，finish/stop/fail 时 deactivate |
| FSM gateway 集成 | ✅ | 通过 gateway 抽象请求 FSM |
| 兼容性检查 | ✅ | producedReferenceSpace vs acceptedReferenceSpace |
| stop 兜底 | ❌ | 不设置 stop_worker_（C-3） |
| TOCTOU | ❌ | busy 检查后释放锁（M-3） |
| task_id | ❌ | 未生成（M-5） |
| workerLoop 同步 | ❌ | 用 1ms sleep 代替 waitControlCycle（H-5） |
| 多 controller 切换 | ❌ | 不支持运行时切换 controller |

### 3.4 ModelProvider

| 评估项 | 状态 | 说明 |
|-------|------|------|
| FK std::function | ✅ | 封装 `kinematics_.JntToCart()` |
| IK std::function | ✅ | 封装 `kinematics_.CartToJnt()` |
| getDof | ✅ | 返回 `jnt_num_` |
| Jacobian | ❌ | 未实现 |
| Dynamics | ❌ | 未实现 |
| 空 callback 验证 | ❌ | 构造时不检查 std::function 是否有效 |
| 类型安全 | ⚠ | 裸 struct 而非 class，无封装 |

### 3.5 MotionSafetyGuard

| 评估项 | 状态 | 说明 |
|-------|------|------|
| 使能检查 | ✅ | `!actual.enabled` → NotEnabled |
| NaN/Inf 检查 | ✅ | 逐一检查 target_position |
| 关节限位 | ✅ | min/max position limits |
| 命令速度 | ✅ | `(q_cmd - q_last_cmd) / dt` |
| 跟随误差 | ✅ | `q_cmd - q_actual` |
| 不悄悄 clamp | ✅ | 只检查，拒绝违规命令 |
| accept 更新 | ✅ | 只有 writeLowLevelCommand 成功后才 accept |
| 速度命令检查 | ❌ | 不检查 target_velocity |
| 力矩命令检查 | ❌ | 不检查 target_torque |
| 加速度检查 | ❌ | 不检查命令加速度 |
| 模式检查 | ❌ | 不检查 joint mode |
| 力矩变化率 | ❌ | 不检查 |

### 3.6 PositionController

| 评估项 | 状态 | 说明 |
|-------|------|------|
| Joint reference 消费 | ✅ | 提取 position/velocity |
| NaN/Inf 检查 | ✅ | allFinite 检查 |
| 维度检查 | ✅ | velocity/acceleration size 与 position 一致 |
| Reference space 检查 | ✅ | 拒绝非 Joint reference |
| activate/deactivate | ⚠ | 空实现（无参数设计签名简化后） |

### 3.7 MotionFsmGateway

| 评估项 | 状态 | 说明 |
|-------|------|------|
| 接口完整 | ✅ | requestStart/Pause/Resume/Stop + notify* |
| AcceptAllGateway | ✅ | 测试用默认实现 |
| BasicRobotFsmGateway | ✅ | 转发到 Robot FSM 方法 |

### 3.8 CartesianGeometry

| 评估项 | 状态 | 说明 |
|-------|------|------|
| computePathMetrics | ✅ | 平移+旋转，等效半径 |
| interpolateLinear | ⚠ | quaternionSlerp 结果未归一化（H-4） |
| interpolateCircular | ⚠ | 始终用 start.M 朝向（H-6） |
| computeCircleCenter | ⚠ | 叉积 v2×v1 反序（M-6） |
| computeCircleArcParams | ✅ | 中心+角度+局部坐标系 |
| computeNormalizedLimits | ✅ | path_length 归一化 |
| rotationAngle | ✅ | GetRotAngle 封装 |

### 3.9 MoveJCommand

| 评估项 | 状态 | 说明 |
|-------|------|------|
| 参数验证 | ✅ | NaN/Inf、维度、正数限制 |
| 归一化 limits | ✅ | delta 归一化，min 取最严 |
| sample 正确性 | ✅ | `s_dot * delta` 正确计算速度/加速度 |
| 零运动处理 | ✅ | `has_motion_` 标记，全零 delta 给默认 limits |

### 3.10 MoveLCommand

| 评估项 | 状态 | 说明 |
|-------|------|------|
| 参数验证 | ✅ | NaN/Inf、正数限制、q_current 非空 |
| path_metrics 计算 | ✅ | computePathMetrics |
| 归一化 limits | ✅ | computeNormalizedLimits |
| IK 失败 → space=None | ✅ | FiniteMotionCommand 基类处理 |
| q_seed 更新 | ✅ | IK 成功后 q_seed_ = q_out |
| 速度/加速度输出 | ❌ | 始终为零（C-1） |

### 3.11 MoveCThreePointCommand / MoveCCenterAngleCommand

| 评估项 | 状态 | 说明 |
|-------|------|------|
| 参数验证 | ✅ | NaN/Inf、正数、theta 非零、axis 范围 |
| 圆弧计算 | ✅ | computeCircleArcParams / buildCenterFrame |
| IK 集成 | ✅ | 与 MoveLCommand 相同模式 |
| orientation_fixed | ✅ | 覆盖 target_pose.M |
| 速度/加速度 | ⚠ | 标量广播（H-3），至少非零 |

### 3.12 Submission 适配函数

| 评估项 | 状态 | 说明 |
|-------|------|------|
| 参数验证 | ✅ | NaN/Inf、正数、joint_count > 0 |
| FK 起点计算 | ✅ | JntToCart → pose_start |
| IK lambda | ⚠ | 按引用捕获 robot（C-2） |
| 关节位置读取 | ✅ | getJointPosition 逐关节读取 |

### 3.13 Robot 集成

| 评估项 | 状态 | 说明 |
|-------|------|------|
| MoveJ executor 路径 | ✅ | time==0 && radius==0 |
| MoveL executor 路径 | ✅ | time==0 && radius==0 |
| MoveC executor 路径 | ✅ | 三点 + 圆心角度两个重载 |
| Drive state 检查 | ✅ | 每关节检查 OperationEnabled |
| 同步等待 | ⚠ | 忙等待无超时（M-4） |
| 初始化安全 | ❌ | 重复调用无互斥（C-4） |
| ModelProvider 注入 | ✅ | FK/IK/getDof lambda |
| 旧路径保留 | ✅ | time/radius != 0 走旧路径 |
| PauseMotion/ResumeMotion/StopMotion | ✅ | 转发到 executor |

---

## 4. 测试覆盖差距

### 4.1 Critical 缺失测试

| 缺失测试 | 影响 |
|---------|------|
| MoveL/MoveC 端到端 executor 集成测试 | 无法验证 MoveL/MoveC 在 workerLoop 中的完整执行链路 |
| Robot::MoveL/MoveC 仿真硬件集成测试 | `robot_motion_sim_test` 只覆盖 MoveJ，MoveL/MoveC 在真实硬件上的行为未验证 |
| IK lambda 生命周期测试 | 无法验证 Robot 析构后 IK callback 是否安全 |
| 并发 submit 测试 | 无法验证多线程 submit 的 TOCTOU 问题 |
| stop_worker_ 兜底测试 | 无法验证 command stop 后 workerLoop 是否退出 |

### 4.2 High 缺失测试

| 缺失测试 | 影响 |
|---------|------|
| MoveL sample() 速度/加速度非零验证 | C-1 零速度 bug 未被测试发现 |
| quaternionSlerp 归一化验证 | H-4 未归一化问题未被测试发现 |
| 大角度旋转插值测试 | 只测试了 90° 旋转，>180° 行为未验证 |
| MoveC 圆弧 > 180° 测试 | 只测试了小弧度，大半圆/整圆行为未验证 |
| orientation_fixed=false 的 MoveC 朝向插值 | H-6 导致朝向始终不变，测试未覆盖 |

### 4.3 Medium 缺失测试

| 缺失测试 | 影响 |
|---------|------|
| MoveL/MoveC 的 pause/resume/stop 在 executor 中的测试 | executor 级别的暂停/恢复/停止只被 MoveJ 验证过 |
| SafetyGuard 与 MoveL 零速度的交互 | C-1 零速度可能绕过速度检查 |
| 三点共线 / 三点重合 的 MoveC 测试 | computeCircleCenter 退化情况未验证 |
| Robot 非零 time/radius 参数走旧路径的测试 | 新旧路径分叉条件未验证 |

### 4.4 测试统计

| 测试文件 | 用例数 | 断言数 | 覆盖范围 |
|---------|-------|-------|---------|
| motion_core_test | 4 | 11788 | MoveJ + MoveL + MoveC sample + DianaApi 映射 |
| motion_executor_test | 11 | 73 | MoveJ executor 生命周期 |
| cartesian_geometry_test | 10 | ~25 | 几何函数单元 |
| motion_safety_guard_test | 6 | 22 | 位置模式安全检查 |
| motion_context_test | 4 | 17 | GuardedMotionContext |
| motion_controller_test | 4 | 16 | PositionController |
| robot_fsm_gateway_test | 3 | 14 | BasicRobotFsmGateway |
| robot_motion_context_test | 3 | 17 | RobotMotionContext 读写 |
| move_j_submission_test | 4 | 15 | submitMoveJ |
| robot_motion_sim_test | 5 | 37 | MoveJ 仿真硬件集成 |
| **总计** | **~54** | **~12024** | |

---

## 5. 设计场景验证

对照设计文档的 23 个场景逐一检查。

| # | 场景 | 状态 | 说明 |
|---|------|------|------|
| 1 | MoveJ 正常完成 | ✅ | robot_motion_sim_test 覆盖 |
| 2 | MoveL 正常完成 | ⚠ | motion_core_test 覆盖 sample，但无 executor 端到端 |
| 3 | MoveJ 暂停 | ✅ | robot_motion_sim_test + motion_executor_test |
| 4 | MoveJ 继续 | ✅ | 同上 |
| 5 | MoveL 暂停并继续 | ❌ | 无 MoveL 的 executor pause/resume 测试 |
| 6 | MoveJ 停止 | ✅ | robot_motion_sim_test |
| 7 | MoveJ 暂停后停止 | ✅ | motion_executor_test |
| 8 | 点动启动 | ❌ | JogCommand 未实现 |
| 9 | 点动暂停无效 | ❌ | JogCommand 未实现 |
| 10 | 点动继续无效 | ❌ | JogCommand 未实现 |
| 11 | 点动停止 | ❌ | JogCommand 未实现 |
| 12 | 相对距离点动 | ❌ | StepJogCommand 未实现 |
| 13 | 运行中再次提交 | ✅ | motion_executor_test busy check |
| 14 | 规划失败 | ✅ | motion_executor_test prepare fail + motion_core_test |
| 15 | 执行中 IK 失败 | ⚠ | motion_core_test 验证 space=None → Failed，但无 safe stop 路径 |
| 16 | 执行中安全检查失败 | ✅ | motion_executor_test context reject → Failed |
| 17 | 执行中驱动掉使能 | ❌ | SafetyGuard 检查 enabled，但 workerLoop 不主动检测 |
| 18 | 暂停期间驱动掉使能 | ❌ | 无暂停期监控机制 |
| 19 | 无任务时 Stop | ✅ | executor.stop() 返回 ok no-op |
| 20 | 无任务时 Pause/Resume | ✅ | executor.pause()/resume() 返回 InvalidState |
| 21 | ERROR_STATE 下提交运动 | ⚠ | 依赖 FSM gateway 拒绝，BasicRobotFsmGateway 已实现 |
| 22 | 初始化后未使能提交运动 | ⚠ | 同上 |
| 23 | 使能成功后提交运动 | ⚠ | 无集成测试 |

**场景覆盖**：10/23 已验证，6/23 部分验证或存疑，7/23 未实现（主要是 JogCommand 和硬件异常场景）。

---

## 6. 建议与优先级排序

### P0（上线前必修 — Critical）

| 编号 | 建议 | 预估工作量 |
|------|------|-----------|
| C-1 | MoveL sample() 输出正确的关节速度/加速度 | 1-2天 |
| C-2 | IK lambda 捕获安全（weak_ptr 或显式 stop+join 在析构前） | 0.5天 |
| C-3 | stop() 增加 stop_worker_ 兜底或超时 | 0.5天 |
| C-4 | initializeMotionExecutor() 使用 std::call_once | 0.5天 |

### P1（尽快修复 — High）

| 编号 | 建议 | 预估工作量 |
|------|------|-----------|
| H-3 | MoveC velocity 使用雅可比映射或数值差分计算各关节速度 | 1天 |
| H-4 | quaternionSlerp 结果归一化 | 0.5天 |
| H-5 | workerLoop 中调用 waitControlCycle() | 0.5天 |
| H-6 | interpolateCircular 支持朝向 SLERP | 1天 |
| H-1 | update() 第三参数改回 ReferenceSpace | 0.5天 |
| H-2 | pause/resume/stop 增加 MotionContext& 参数 | 0.5天 |

### P2（稳健性 — Medium）

| 编号 | 建议 | 预估工作量 |
|------|------|-----------|
| M-1 | producedReferenceSpace() 默认 None 应拒绝 | 0.5天 |
| M-2 | MoveL/MoveC Robot 入口增加速度/加速度上限检查 | 0.5天 |
| M-3 | submit() busy 检查与 current_ 设置使用同一把锁 | 1天 |
| M-4 | 同步等待增加超时 | 0.5天 |
| M-5 | submit() 生成 task_id，接入 /api/move/status | 1天 |

### P3（后续迭代）

- 实现 JogCommand / StepJogCommand（场景 8-12）
- 实现 CartesianReference + CartesianImpedanceController
- 实现 DynamicsAdapter
- 补全 MotionContext 方法
- HTTP 层全面迁移到 DianaApi 错误码
- 补充所有缺失测试（特别是 MoveL/MoveC 端到端和仿真硬件集成）
- 修复 unit_test 既有崩溃

---

## 7. 总结

当前实现完成了设计文档阶段 1-2 的核心目标：建立了 MotionCommand / FiniteMotionCommand / MotionExecutor / MotionSafetyGuard / MotionContext 的分层架构，成功将 MoveJ/MoveL/MoveC 三种运动类型接入 executor 路径。62 个测试覆盖了各个组件的单元行为和 MoveJ 的仿真硬件集成。

主要的架构偏差集中在：
1. **签名简化**——多个接口缺少 `MotionContext&` / `ModelProvider&` / `ReferenceSpace` 参数，这些参数在当前简单场景下不影响功能，但在引入复杂控制器（阻抗/导纳）和运行时状态读取时会成为阻碍。
2. **速度/加速度计算缺失**——MoveL 输出零速度（C-1），MoveC 输出不正确的标量速度（H-3），这两个问题直接影响硬件执行质量。
3. **实时性**——workerLoop 用 sleep 代替硬件同步（H-5），在非仿真硬件上会导致控制周期不准确。
4. **并发安全**——initializeMotionExecutor() 无互斥（C-4）、submit TOCTOU（M-3）、IK lambda 悬空引用（C-2）在真实多客户端环境下可能触发崩溃。

建议按 P0 → P1 → P2 的优先级逐步修复，P0 项目应在下一次集成测试前完成。
