# Robot::motion 生命周期审查报告

> 审查日期: 2026-07-21  
> 审查范围: `std::unique_ptr<MotionInterface> motion` 的创建、使用、销毁全路径  
> 涉及文件: `src/robot.hpp`, `src/robot.cpp`, `src/executor.hpp`, `src/executor.cpp`

---

## 1. 背景

`Robot` 类中通过 `std::unique_ptr<MotionInterface> motion` 持有当前运动规划器对象:

```cpp
// src/robot.hpp:343
std::unique_ptr<MotionInterface> motion {nullptr}; // movej, movel, movec
```

`Executor` 类以裸指针引用同一个 motion 对象:

```cpp
// src/executor.hpp:98
MotionInterface *motion_{nullptr};  // 不管理生命周期，仅拼装调用
```

motion 对象的生命周期由 `Robot::motion` (unique_ptr) 管理, `Executor::motion_` 仅为使用方。

---

## 2. motion 创建与销毁路径全景

### 2.1 创建点 (6 处)

所有创建均使用 `std::make_unique`, 模式一致:

| 指令 | 创建类型 | 文件位置 |
|------|---------|---------|
| MoveJ | `MoveJoint` | `robot.cpp:901-910` |
| MoveL | `MoveLineOffline` | `robot.cpp:961-978` |
| MoveC (圆心+角度) | `MoveCircle` | `robot.cpp:1241-1249` |
| MoveC (三点圆弧) | `MoveCircle` | `robot.cpp:1272-1280` |
| MoveJogging | `MoveJog` | `robot.cpp:1392-1407` |
| MoveNullJogging | `MoveNullJog` | `robot.cpp:1439-1455` |
| MoveSvdJogging | `MoveSvdJog` | `robot.cpp:1488-1504` |

创建模式示例:

```cpp
auto new_motion = std::make_unique<MoveJoint>(...);
Result rc = new_motion->Reset();
if (rc != Result::NoError) return rc;       // 失败时 new_motion 自动析构, 正确
motion = std::move(new_motion);             // 旧 motion 在此被覆盖释放
executor->SwitchMotion(motion.get());       // executor 更新裸指针
```

### 2.2 销毁点 (仅 1 处)

```cpp
// src/robot.cpp:486-494  (on_fsm_reset 内)
if (motion) {
    const Result motion_reset = motion->Reset();
    if (motion_reset != Result::NoError && motion_reset != Result::PlanFinished) {
        log_ptr_->warn("ResetFault重置当前motion返回: {}", static_cast<int>(motion_reset));
    }
    motion.reset();                          // ← 唯一调用点
    if (executor) {
        executor->SwitchMotion(nullptr);     // 同步清空 executor 裸指针
    }
}
```

**结论: 整个代码库中仅 `on_fsm_reset()` (ResetFault 路径) 正确清理了 motion。**

---

## 3. 发现的问题

### 问题 1 [严重] — 运动正常结束后 motion 未销毁

**路径**: `RunCycle()` 检测到 `PlanFinished` → `EventSuccessed` → RUNNING → STOPPED

```cpp
// src/robot.cpp:861-871
if (r == Result::PlanFinished) {
    if (impl_->is(sml::state<class PAUSED>)) {
        // 暂停状态下到达目标, 仅更新执行器
        r = executor->Update();
        if (static_cast<int>(r) < 0) {
            impl_->process_event(EventErrorOccurred{});
        }
        return;
    }
    impl_->process_event(EventSuccessed{});  // 进入 STOPPED
    return;  // ← motion 对象未清理
}
```

**后果**:
- MoveJoint/MoveLine/MoveCircle 对象持续占用内存 (含 Ruckig 轨迹规划器内部缓冲区)
- 如果 STOPPED 状态下长时间无新运动指令, 等同于事实上的内存泄漏
- 旧 motion 对象仅在下一次运动指令执行 `motion = std::move(new_motion)` 时才被释放

---

### 问题 2 [严重] — 进入 ERROR 状态时 motion 未销毁

**路径**: `motion->Update()` 或 `executor->Update()` 返回错误 → `EventErrorOccurred` → ERROR_STATE

```cpp
// src/robot.cpp:873-884
if (static_cast<int>(r) < 0) {
    impl_->process_event(EventErrorOccurred{});   // 进入 ERROR_STATE
    return;                                        // ← motion 未清理
}
```

ERROR_STATE 的 `on_entry` 回调是空实现:

```cpp
// src/robot.cpp:236-237
const auto action_error = [](rocos::Robot &robot) {
}; //TODO: 进入错误状态时的必要处理   ← 开发者已知此处缺失
```

**后果**:
- motion 对象泄漏
- executor 内部 `motion_` 裸指针指向已处于错误状态的 motion
- 用户必须先调用 ResetFault (唯一清理路径) 才能恢复正常

---

### 问题 3 [严重] — 用户 Stop 操作不销毁 motion

**路径**: `StopMotion()` → `EventStopReq` → STOPPING → `on_fsm_stop()` → STOPPED

```cpp
// src/robot.cpp:430-439
void Robot::on_fsm_stop() {
    Result rc = executor->Stop();    // 仅调用 motion_->Stop(), 不销毁对象
    if (rc == Result::NoError) {
        log_ptr_->info("机器人停止成功，准备进入Stopped状态");
    } else {
        log_ptr_->error("机器人停止失败，退回ERROR状态");
        impl_->process_event(EventErrorOccurred{});
    }
    // ← motion 未 reset
}
```

**后果**: Stop 语义应为"终止并清理当前运动", 但实际仅调用了 `motion_->Stop()` 虚函数, 对象本身不被销毁。

---

### 问题 4 [中等] — motion 为 public 成员, 封装缺失

```cpp
// src/robot.hpp:340-345
public:
    std::unique_ptr<ModelInterface> model {nullptr};
    std::unique_ptr<HardwareInterface> hardware {nullptr};
    std::unique_ptr<MotionInterface> motion {nullptr};        // ← public
    std::unique_ptr<ControllerInterface> controller {nullptr};
    std::unique_ptr<Executor> executor {nullptr};
```

**风险**: 任何持有 `Robot&` 的代码都可以直接 `robot.motion.reset()`, 导致 `executor` 内部 `motion_` 裸指针悬空。虽然当前 `RobotHttpServer` (friend 类) 未直接访问 `motion`, 但架构上不设防。

---

### 问题 5 [轻微] — 多处 lambda 中存在不可达代码

以下位置在 `return Result::NoError;` 后有永远无法执行到的代码:

- `robot.cpp:981-982` (MoveL)
- `robot.cpp:1252-1253` (MoveC 圆心+角度)
- `robot.cpp:1283-1284` (MoveC 三点圆弧)
- `robot.cpp:1410-1411` (MoveJogging)
- `robot.cpp:1458-1459` (MoveNullJogging)
- `robot.cpp:1507-1508` (MoveSvdJogging)

示例:

```cpp
motion = std::move(new_motion);
executor->SwitchMotion(motion.get());
return Result::NoError;

log_ptr_->error("executor is nullptr");  // ← 永远不可达
return Result::Fatal;                      // ← 永远不可达
```

---

### 问题 6 [轻微] — on_fsm_stop() 未 join 控制线程

```cpp
void Robot::on_fsm_stop() {
    // if (control_thread_.joinable()) control_thread_.join();  ← 被注释掉
    Result rc = executor->Stop();
    ...
}
```

当前设计依赖控制线程 `while(IsControlActive())` 循环自然退出 (STOPPED 不在 `IsControlActive` 集合内), 线程在 `on_fsm_start()` 或析构函数中被 join。在此期间存在短暂的线程未回收窗口, 但不会导致数据竞争。

---

## 4. 安全确认

以下方面经审查确认为安全:

| 检查项 | 结论 | 说明 |
|--------|:----:|------|
| 析构顺序 | ✅ | executor 声明在 motion 之后 → 先析构 executor (`~Executor()` 调用 `motion_->Reset()` 时 motion 仍存活) → 后析构 motion |
| 空指针防御 | ✅ | `RunCycle()` 顶部 `if (!motion) return;`; Executor 所有方法均有 `if (motion_)` |
| 并发创建保护 | ✅ | 新运动指令在 lambda 中构造, 成功后通过 `motion = std::move(...)` 原子替换 |
| FSM 串行化 | ✅ | `Impl::process_event()` 使用 `std::recursive_mutex` 保护 |
| 析构时控制线程安全 | ✅ | `~Robot()` 先 join 控制线程, 再析构成员, 控制线程不会访问已销毁对象 |
| ResetFault 清理 | ✅ | 唯一同时调用 `motion.reset()` + `executor->SwitchMotion(nullptr)` 的路径, 实现正确 |

---

## 5. 修复方案

### 5.1 提取公共清理函数

在 `Robot` 类中新增私有辅助函数:

```cpp
// src/robot.hpp (private 区域)
void resetMotion();
```

```cpp
// src/robot.cpp
void Robot::resetMotion() {
    if (motion) {
        motion.reset();
    }
    if (executor) {
        executor->SwitchMotion(nullptr);
    }
}
```

### 5.2 在缺失点调用清理

**修改 1** — `on_fsm_stop()` 中增加清理:

```cpp
void Robot::on_fsm_stop() {
    Result rc = executor->Stop();
    if (rc == Result::NoError) {
        resetMotion();                              // ← 新增
        log_ptr_->info("机器人停止成功，准备进入Stopped状态");
    } else {
        log_ptr_->error("机器人停止失败，退回ERROR状态");
        impl_->process_event(EventErrorOccurred{});
    }
}
```

**修改 2** — `action_error` 回调中增加清理:

```cpp
const auto action_error = [](rocos::Robot &robot) {
    robot.resetMotion();                            // ← 新增
};
```

**修改 3** — `RunCycle()` 中 `PlanFinished` 分支增加清理:

```cpp
if (r == Result::PlanFinished) {
    if (impl_->is(sml::state<class PAUSED>)) {
        r = executor->Update();
        if (static_cast<int>(r) < 0) {
            impl_->process_event(EventErrorOccurred{});
        }
        return;
    }
    resetMotion();                                  // ← 新增
    impl_->process_event(EventSuccessed{});
    return;
}
```

### 5.3 改善封装

将 `motion` 移为 `private` 成员, 同时将 `RobotHttpServer` 已通过 friend 声明授权访问:

```cpp
// src/robot.hpp
private:
    std::unique_ptr<MotionInterface> motion {nullptr};
```

### 5.4 清理不可达代码

删除所有 `return Result::NoError;` 之后的不可达代码块 (6 处 lambda 中)。

---

## 6. 修改影响范围

| 修改点 | 影响 | 风险 |
|--------|------|------|
| 新增 `resetMotion()` | 无外部影响, 纯内部重构 | 低 |
| `on_fsm_stop()` 增加清理 | Stop 后 motion 被销毁, 后续任何依赖旧 motion 的操作需检查 | 低 (Stop 后本不应继续使用旧 motion) |
| `action_error` 增加清理 | 错误状态自动清理, 减少 ResetFault 前的手动清理需求 | 低 |
| `PlanFinished` 增加清理 | 正常结束后 motion 被销毁 | 需确认 `EventSuccessed` 后无其他代码再访问 motion |
| motion 改为 private | 编译期检查, 不影响 friend 类 RobotHttpServer | 低 (仅编译错误, 非运行时) |
