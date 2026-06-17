# Motion Executor Code Review

本文档阅读并评价本次围绕 `docs/motion_fsm_executor_design.md` 落地的 motion core 代码修改。

## 修改范围

本次实现聚焦于 motion core 分层与 `Robot::MoveJ` 常规路径接入。`MoveL/MoveC/Dragging` 等旧执行路径尚未迁移。

新增文件：

- `include/rocos_app/motion/diana_error_codes.h`
- `include/rocos_app/motion/motion_types.h`
- `include/rocos_app/motion/motion_command.h`
- `include/rocos_app/motion/motion_fsm_gateway.h`
- `include/rocos_app/motion/finite_motion_command.h`
- `include/rocos_app/motion/move_j_command.h`
- `include/rocos_app/motion/motion_executor.h`
- `include/rocos_app/motion/motion_safety_guard.h`
- `include/rocos_app/motion/motion_context.h`
- `include/rocos_app/motion/motion_controller.h`
- `include/rocos_app/motion/position_controller.h`
- `include/rocos_app/motion/robot_fsm_gateway.h`
- `include/rocos_app/motion/robot_motion_context.h`
- `include/rocos_app/motion/move_j_submission.h`
- `test/motion_core_test.cc`
- `test/motion_executor_test.cc`
- `test/motion_safety_guard_test.cc`
- `test/motion_context_test.cc`
- `test/motion_controller_test.cc`
- `test/robot_fsm_gateway_test.cc`
- `test/robot_motion_context_test.cc`
- `test/move_j_submission_test.cc`
- `test/robot_motion_sim_test.cc`

修改文件：

- `CMakeLists.txt`
- `docs/motion_fsm_executor_design.md`
- `include/rocos_app/drive.h`
- `include/rocos_app/robot.h`
- `include/rocos_app/robot_http_server.h`
- `src/drive.cc`
- `src/drive_guard.cc`
- `src/robot.cc`
- `src/robot_http_server.cc`

## 设计符合度

已经符合设计文档的部分：

- 对外错误码开始使用 DianaApi 负数错误码，内部 `MotionResultCode` 通过 `toDianaErrorCode()` 映射到 API code。
- 新增 `MotionCommand` 抽象，command 不直接操作 FSM，也不持有 `Robot*`。
- 新增 `ReferenceSpace`，`MotionReference` 开始标记自身参考空间，`MotionCommand` 可通过 `producedReferenceSpace()` 声明产出的 reference 类型。
- 新增 `FiniteMotionCommand`，将 `UnitIntervalMotionProfile` 封装在有限路径命令基类内。
- 新增 `MoveJCommand`，按单位进度 `s/s_dot/s_ddot` 生成关节参考，并声明产出 `Joint` reference。
- 新增 `submitMoveJ()` 适配函数，可从 Robot-like runtime 读取当前关节位置和 jerk 限制，组装 `MoveJCommand` 并提交 executor。
- 新增 `MotionController` 抽象和 `PositionController`，开始落地 `MotionReference -> LowLevelCommand` 的控制器边界。
- `PositionController` 只消费 `JointReference`，输出位置模式 `LowLevelCommand.target_position`，并可选透传 `target_velocity`。
- 新增 `MotionExecutor`，拥有当前 command、任务状态、最后错误，并统一处理 submit/stop/pause/resume 的基本生命周期。
- `MotionExecutor` 已支持可选 active controller/context，控制周期中能执行 `command.update -> controller.update -> context.writeLowLevelCommand`。
- active controller 会在 submit 成功路径中 activate，并在任务 finish/stop/fail 时 deactivate。
- submit 阶段已检查 `command.producedReferenceSpace()` 和 active controller 的 `acceptedReferenceSpace()` 是否匹配，不匹配时以 DianaApi `-2325` 拒绝启动。
- controller 或 context 返回失败时，executor 会进入 `Failed`，保留原始 DianaApi 错误码，并通知 FSM error。
- executor 的 pause 语义已按设计修正为“command 真正返回 `Paused` 后，任务状态才变成 `Paused`”。
- 新增 `MotionFsmGateway` 抽象和默认 `AcceptAllMotionFsmGateway`，executor 已通过 gateway 请求 start/pause/resume/stop，并在 paused/stopped/error 时通知 FSM 边界。
- 新增 `BasicRobotFsmGateway`，可把 executor 的 start/pause/resume/stop/error 请求转发到 Robot 提供的受控 motion FSM 方法。
- `Robot` 新增 `requestMotionStart/requestMotionPause/requestMotionContinue/requestMotionStop/notifyMotionError`，旧 `enterRunning/enterStopped` 已复用这些边界。
- 新增 `LowLevelCommand`、`RobotStateSnapshot`、`MotionSafetyGuard`，开始落地最终下发前安全门禁。
- `MotionSafetyGuard` 遵循“不悄悄 clamp”的设计原则，只检查并拒绝违规命令，只有调用 `accept()` 后才更新上一条已接受命令。
- 新增 `MotionContext` 和 `GuardedMotionContext`，将“读取实际状态 -> safety guard 检查 -> 写底层命令 -> 成功后 accept”这条顺序固定在基类中。
- 新增 `RobotMotionContext`，可从 Robot-like runtime 读取关节位置、速度、力矩和使能状态，并将通过安全检查的 `LowLevelCommand` 写回底层关节目标。
- `Robot` 新增 `waitControlCycle()`，封装 `hw_interface_->waitForSignal(0)`，供 `RobotMotionContext` 在写入后同步控制周期。
- `Robot` 已持有 motion safety guard、FSM gateway、position controller、motion context 和 motion executor。
- `Robot::MoveJ` 在 `time == 0 && radius == 0` 的常规关节位置路径下，已改为调用 `submitMoveJ()` 提交 executor；同步调用会等待 executor 结束，异步调用提交后立即返回。
- `Robot` 已新增 `PauseMotion()`、`ResumeMotion()`、`StopMotion()`，用于对当前 executor 运动执行暂停、继续和停止。
- HTTP 层已新增 `POST /api/move/pause`、`POST /api/move/resume`，并将 `POST /api/move/stop` 接入 executor stop 路径；这些接口返回 `success/code/message`，失败时 `code` 透传 DianaApi 负数错误码。
- `Robot::MoveJ` 参数、模式、使能检查开始返回 DianaApi 负数错误码，而不是旧的 `-1`。
- `Drive` 不再拥有传入的 `HardwareInterface*`，避免多个关节 Drive 对同一硬件对象重复释放。
- `DriveGuard` 新增 Drive 注销和互斥保护，避免 Robot/Drive 析构后 guard 线程继续访问悬空 Drive 指针。

仍未完成的部分：

- `Robot::MoveJ` 的 `time/radius` 特殊语义尚未迁移到 executor 路径，目前返回 DianaApi `-3006`。
- `Robot::MoveL/MoveC/Dragging` 仍直接调用 `enterRunning()` 和 `enterStopped()`。
- `ModelProvider` 尚未落地。
- `MotionExecutor` 目前只支持手动注入 active controller/context，尚未支持 Robot 管理的 controller registry 和运行中控制器切换。
- `MoveLCommand`、`MoveCCommand`、`JogCommand` 尚未实现。
- HTTP 层只有新增的 pause/resume/stop executor 接口透传 DianaApi 错误码；其他旧 motion/task 接口仍使用旧 task map 和旧 4 位业务码，尚未整体迁移到 `MotionResult.api_error_code`。
- `MotionSafetyGuard` 当前只覆盖位置模式基础检查，尚未覆盖速度命令、力矩命令、加速度、力矩变化率和控制模式检查。

## 测试覆盖

新增测试：

- `motion_core_test`
  - DianaApi 错误码映射。
  - `MoveJCommand` 采样输出位置、速度、加速度。
  - `MoveJCommand` pause/resume/stop 由 `FiniteMotionCommand` 统一处理。
  - NaN 参数返回 `-2902`。

- `motion_executor_test`
  - 空 command 返回 `-2901`。
  - 忙碌提交返回 `-2215`。
  - prepare 失败保留 DianaApi 错误码并释放当前 command。
  - command 正常执行后进入 `Finished`。
  - finite command 在 executor 中 pause/resume 后仍保持 active command，并最终完成。
  - FSM 拒绝 start 时 executor 不会 prepare/start command。
  - executor 会向 FSM gateway 发送 pause/resume/stop 请求，并在 paused/stopped 后通知 gateway。
  - running reference 会经 `PositionController` 转成 `LowLevelCommand` 并写入 `MotionContext`。
  - active controller 正常 activate、周期 update，并在任务结束后 deactivate。
  - controller activate 失败时 submit 失败，保留 DianaApi 错误码且不写 context。
  - command/controller reference 空间不匹配时，submit 阶段拒绝启动并返回 DianaApi `-2325`。
  - context 拒绝 low-level command 时任务进入 `Failed`，保留 context 返回的 DianaApi 错误码。

- `motion_safety_guard_test`
  - 第一条合法位置命令可以通过。
  - 未使能机器人返回 `-2205`。
  - NaN/Inf 返回 `-2902`。
  - 位置越界返回 `-2308`。
  - 命令速度越界返回 `-2310`。
  - 跟随误差越界返回 `-2203`。
  - `reset()` 会清除上一条已接受命令历史。

- `motion_context_test`
  - safety check 通过后才调用底层写入。
  - safety check 失败时不写底层命令。
  - 底层写入失败时不会调用 `MotionSafetyGuard::accept()`。
  - safety failure 会转换为 `MotionResultCode::SafetyViolation` 并保留 DianaApi 错误码。

- `motion_controller_test`
  - `PositionController` 将 `JointReference` 转换成 `LowLevelCommand.target_position`。
  - 可选关节速度 reference 会透传到 `target_velocity`。
  - NaN/Inf reference 返回 DianaApi `-2902`。
  - reference/controller 空间不匹配返回 DianaApi `-2325`。

- `robot_fsm_gateway_test`
  - gateway 将 start/pause/resume/stop/error 请求转发到 Robot-like FSM 客户端。
  - FSM 拒绝请求时返回 `MotionResultCode::InvalidState` 和 DianaApi `-2205`。
  - `notifyStopped()` 会通过 `requestMotionStop()` 汇报运动停止。

- `robot_motion_context_test`
  - `RobotMotionContext` 可以读取 Robot-like runtime 的关节状态快照。
  - 通过 safety guard 后写入关节位置、速度，并等待一个控制周期。
  - safety guard 拒绝命令时不会写入底层关节目标。

- `move_j_submission_test`
  - `submitMoveJ()` 会创建 `MoveJCommand` 并提交 executor。
  - 目标维度不匹配返回 DianaApi `-2332`。
  - 速度、加速度、周期等标量非法时返回 DianaApi 错误码。
  - executor 的 DianaApi 失败结果会原样透传。

- `robot_motion_sim_test`
  - 参考 `src/rocosAppMain.cc` 使用 `HardwareSim(20)` 和 `Robot(config/robot.urdf, base_link, link_7)`。
  - 同步 `Robot::MoveJ` 会走 executor 路径，并在仿真硬件上到达目标关节位置。
  - 越界目标会返回 DianaApi `-2308`，机器人状态保持 `STOPPED`。
  - `Robot::MoveJ` 异步运动可通过 `PauseMotion()`、`ResumeMotion()`、`StopMotion()` 暂停、继续和停止。
  - HTTP API 可通过 `/api/move/pause`、`/api/move/resume`、`/api/move/stop` 控制异步 MoveJ。
  - 外部 `boost::shared_ptr<HardwareSim>` 管理硬件生命周期时，`Robot`/`Drive` 析构不会重复释放硬件，也不会让 `DriveGuard` 访问已析构的 Drive。

HTTP 暂停/继续/停止示例：

```bash
curl -X POST http://127.0.0.1:8080/api/move/pause
curl -X POST http://127.0.0.1:8080/api/move/resume
curl -X POST http://127.0.0.1:8080/api/move/stop
```

验证命令和结果：

```text
cmake --build build --target motion_core_test motion_executor_test motion_safety_guard_test motion_context_test motion_controller_test robot_fsm_gateway_test robot_motion_context_test move_j_submission_test robot_motion_sim_test robot -j 2
./build/bin/motion_core_test
./build/bin/motion_executor_test
./build/bin/motion_safety_guard_test
./build/bin/motion_context_test
./build/bin/motion_controller_test
./build/bin/robot_fsm_gateway_test
./build/bin/robot_motion_context_test
./build/bin/move_j_submission_test
./build/bin/robot_motion_sim_test
```

结果：

```text
motion_core_test: 4 test cases passed, 11788 assertions passed
motion_executor_test: 11 test cases passed, 73 assertions passed
motion_safety_guard_test: 6 test cases passed, 22 assertions passed
motion_context_test: 4 test cases passed, 17 assertions passed
motion_controller_test: 4 test cases passed, 16 assertions passed
robot_fsm_gateway_test: 3 test cases passed, 14 assertions passed
robot_motion_context_test: 3 test cases passed, 17 assertions passed
move_j_submission_test: 4 test cases passed, 15 assertions passed
robot_motion_sim_test: 5 test cases passed, 37 assertions passed
robot target: built successfully
```

全量验证：

```text
cmake --build build -j 2
ctest --test-dir build --output-on-failure
```

结果：

```text
build: passed
ctest: 61/62 passed
failed: unit_test
```

`unit_test` 在现有硬件/机器人综合测试中抛出 `Invalid argument` 并最终 `SIGSEGV`。失败发生在既有 `test/unit_test.cc` 的 Hardware、Drive、Async motion、Sync motion、Robot Motion Thread、HTTP communication、Kinematics 等用例中；新增的 motion core、executor、safety guard、context、HTTP pause/resume/stop 和仿真硬件集成测试均通过。

排除既有 `unit_test` 后：

```text
ctest --test-dir build --output-on-failure -E '^unit_test$'
```

结果：

```text
61/61 tests passed
```

## 代码评价

这次修改是合理的分层切入点。它先建立设计文档中的核心抽象：错误码、command、finite command、MoveJ 采样、executor 生命周期和最终安全门禁，然后把常规 `Robot::MoveJ` 路径接入 executor。这让后续迁移 `MoveL/MoveC/Dragging` 时有一个可测试的目标结构。

当前 `MotionExecutor` 仍是轻量版本，主要用于验证生命周期边界。它已经有可替换的 FSM gateway 抽象，并已能在启动阶段做 command/controller 能力匹配，在控制周期中串起 command、controller 和 context。`Robot` 已持有 `BasicRobotFsmGateway`、`RobotMotionContext`、`PositionController` 和 `MotionExecutor`，常规 `Robot::MoveJ` 已通过 `submitMoveJ()` 提交 executor。不过 `ModelProvider`、controller registry、MoveL/MoveC/Jog 迁移和 MoveJ 的 `time/radius` 特殊语义尚未完成，因此还不能视为完整运动执行器落地。

`MoveJCommand` 当前只处理关节空间 reference，并已声明 `ReferenceSpace::Joint`。它不做关节限位、模式、使能和实际状态读取，这些检查应该在后续 `prepare(ctx, model)` 和 `MotionSafetyGuard` 接入后补齐。

`PositionController` 已覆盖最直接的 `MoveJ + PositionController` 组合路径。它只做 reference 到 low-level command 的转换和基础输入合法性检查；真正的限位、速度、跟随误差等硬门禁仍由 `MotionSafetyGuard` 负责。

`MotionSafetyGuard` 当前实现聚焦位置控制路径。它已经具备 `q_last_cmd` 历史、速度检查、跟随误差检查和基础限位检查。`GuardedMotionContext` 已经固定了安全检查和 accept 的调用顺序，`RobotMotionContext` 已经在常规 `Robot::MoveJ` executor 路径中负责读 Robot 快照、执行安全检查，并写底层关节目标。

`Robot::MoveJ` 常规路径已经由 executor 接管，这是本轮最重要的集成变化。旧实现中 `MoveJ` 自己调用 `enterRunning()`、创建 `motion_thread_`、在 `RunMoveJ()` 末尾调用 `enterStopped()`；新路径改为由 `MotionExecutor` 通过 `BasicRobotFsmGateway<Robot>` 进入/退出 FSM，并由 `RobotMotionContext` 统一执行安全检查和底层写入。当前仍保留旧 `RunMoveJ()` 代码，供后续迁移和对照使用，但常规 `MoveJ` 不再直接调用它。

仿真硬件验证已经覆盖 `Robot::MoveJ -> MotionExecutor -> PositionController -> RobotMotionContext -> HardwareSim` 的完整链路，并覆盖了 Robot/Drive 正常析构路径。此前 `Drive` 会用多个 owning `boost::shared_ptr<HardwareInterface>` 接管同一个硬件指针，正常析构会重复 delete；本轮已改为非拥有裸指针，并让 `DriveGuard` 在 Drive 析构时注销指针。

## 下一阶段建议

1. 将 `MotionCommand::prepare/start/update` 改为接收 context/model，使 command 可以在 prepare 阶段读取真实状态、模型和限制。
2. 迁移 `Robot::MoveJ` 的 `time/radius` 特殊语义，或在 API 层明确声明暂不支持。
3. 继续迁移 `Robot::MoveL/MoveC/Dragging`，让所有运动占用都走 executor。
4. 增加 controller registry，由 `Robot` 管理 active controller，并限制只在 `IDLE`/`STOPPED` 等安全状态切换。
5. 将 HTTP 层其余旧 motion/task 接口的 `sendJson` 错误码从旧 4 位业务码迁移到 DianaApi 负数错误码。
6. 在 `unit_test` 的既有崩溃问题隔离后，再恢复完整 `ctest` 作为必须通过门禁。
