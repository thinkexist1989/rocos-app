# MoveServo 类实现方案

## Context

在 ROCOS 机器人控制器中新增基于 UDP 的伺服运动模式。核心需求：创建一个 `MoveServo` 类（继承 `MotionInterface`），通过 sockpp 库在 `localhost:8081` 上监听 UDP 二进制 `MotionGeneratorCommand` 指令，处理后在每个控制周期生成 `Reference`，并回复 `RobotState`。通信模式参考 libfranka 的 1kHz 实时 UDP 协议。

已有基础：`servo_type.hpp` 已定义 `MotionGeneratorCommand` 和 `RobotState`（`#pragma pack(1)`），FSM 已有 `SERVOING` 状态骨架，sockpp 库已存在于 `3rdparty/sockpp/` 但未集成到构建系统。

## 实现步骤

### 步骤 1: 添加错误码 (`src/result.hpp`)

在 `Result` 枚举中 `SocketOtherError = -1015` 之后添加：

```cpp
ServoBindFail = -1016,          // UDP 绑定失败
ServoThreadStartFailed = -1017, // UDP 接收线程创建失败
ServoNotInitialized = -1018,    // MoveServo 未正确初始化
```

在 `to_string()` 函数中添加对应的 `RESULT_CASE` 条目。

### 步骤 2: 集成 sockpp (`CMakeLists.txt`)

- 在第 77 行附近添加 `add_subdirectory(3rdparty/sockpp)`
- 在 `robot` 库的源文件列表中添加 `src/move_servo.cpp`（放在 `src/move_svd_jog.cpp` 之后）
- 在 `target_link_libraries(robot ...)` 末尾添加 `sockpp`

### 步骤 3: 创建 `src/move_servo.hpp`

```cpp
#pragma once

#include "motion_interface.hpp"
#include "model_interface.hpp"
#include "hardware_interface.hpp"
#include "result.hpp"
#include "servo_type.hpp"

#include <atomic>
#include <mutex>
#include <thread>
#include <array>

#include <sockpp/udp_socket.h>
#include <sockpp/inet_address.h>

namespace rocos {

class MoveServo : public MotionInterface {
 public:
  explicit MoveServo(HardwareInterface* hw,
                     ModelInterface* model = nullptr,
                     uint16_t listen_port = 8081);

  ~MoveServo() override;

  // ─── MotionInterface 接口 ───
  Result ValidateParameters() const override;
  Result Reset() override;
  Result Update() override;
  Result GenerateRef(Reference& ref_out) override;

  bool CanPause()  const override { return false; }
  bool CanResume() const override { return false; }
  bool CanStop()   const override { return true; }

  Result Pause() override;
  Result Resume() override;
  Result Stop() override;

  /// @brief 设置当前伺服模式（joint 或 cartesian）
  void SetMode(MotionMode mode) { current_mode_ = mode; }

 private:
  // ─── UDP 接收线程 ───
  void udpReceiveLoop();

  // ─── 矩阵转换辅助 ───
  static Frame       matrixToFrame(const std::array<double, 16>& mat);
  static void        frameToMatrix(const Frame& f, std::array<double, 16>& mat);

  // ─── 通信质量统计 ───
  void   recordSuccess(bool ok);
  double computeSuccessRate() const;

  // ─── 非拥有指针 ───
  HardwareInterface* hw_{nullptr};
  uint16_t            listen_port_{8081};

  // ─── UDP socket ───
  sockpp::udp_socket sock_;

  // ─── 共享数据（mtx_ 保护：UDP 线程写 / 控制线程读） ───
  mutable std::mutex     mtx_;
  MotionGeneratorCommand cmd_{};
  bool                   has_new_cmd_{false};
  sockpp::inet_address   client_addr_{};

  // ─── 伺服模式 ───
  MotionMode current_mode_{MotionMode::kNone};

  // ─── UDP 线程 ───
  std::thread thread_;

  // ─── 原子标志 ───
  std::atomic<bool> running_{false};
  std::atomic<bool> stopped_{false};

  // ─── 控制线程独享（不加锁） ───
  uint64_t message_id_counter_{0};
  int      joint_count_{0};

  // ─── 滑动窗口 ───
  static constexpr int         kSuccessWindowSize = 100;
  std::array<bool, kSuccessWindowSize> success_window_{};
  int success_index_{0};
  int success_count_{0};
};

}  // namespace rocos
```

### 步骤 4: 实现 `src/move_servo.cpp`

#### 4.1 构造函数

```cpp
MoveServo::MoveServo(HardwareInterface* hw, ModelInterface* model,
                     uint16_t listen_port)
    : MotionInterface(model), hw_(hw), listen_port_(listen_port)
{
    sockpp::initialize();                              // 幂等
    sockpp::inet_address addr("localhost", listen_port);
    sock_.bind(addr);                                 // 失败不抛异常，Reset() 检查

    if (hw_) {
        joint_count_ = std::max(0, hw_->GetDriveNum());
    }
    if (joint_count_ <= 0 && model_) {
        joint_count_ = model_->GetJointNum();
    }
    cmd_ = MotionGeneratorCommand{};
}
```

#### 4.2 析构函数

```cpp
MoveServo::~MoveServo() {
    running_.store(false);
    stopped_.store(true);
    if (thread_.joinable() && thread_.get_id() != std::this_thread::get_id()) {
        thread_.join();
    }
}
```

#### 4.3 UDP 接收线程（参照 libfranka 两阶段接收）

```cpp
void MoveServo::udpReceiveLoop() {
    auto recv_sock = sock_.clone();   // 独立句柄，避免与 send 端竞争
    recv_sock.set_non_blocking(true); // 全程非阻塞模式

    char                  buf[sizeof(MotionGeneratorCommand)];
    uint64_t              latest_msg_id = 0;
    sockpp::inet_address  addr;
    bool                  received = false;

    while (running_.load(std::memory_order_relaxed)) {
        // Phase 1: 非阻塞排空所有积压包，保留 message_id 最大的
        while (running_.load(std::memory_order_relaxed)) {
            ssize_t n = recv_sock.recv_from(buf, sizeof(buf), &addr);
            if (n <= 0) break;                              // 缓冲区已空
            if (n != sizeof(MotionGeneratorCommand)) continue;  // 畸形包

            auto* cmd = reinterpret_cast<MotionGeneratorCommand*>(buf);
            if (!received || cmd->message_id > latest_msg_id) {
                latest_msg_id = cmd->message_id;
                received = true;
            }
        }

        if (received) {
            std::lock_guard<std::mutex> lock(mtx_);
            cmd_ = *reinterpret_cast<MotionGeneratorCommand*>(buf);
            client_addr_ = addr;
            has_new_cmd_ = true;
            received = false;  // 重置，下轮等待新包
        }

        // Phase 2: 无包时短暂休眠避免忙等（~0.5ms）
        if (!received && running_.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }
}
```

#### 4.4 `Reset()` —— 初始化并启动线程

```cpp
Result MoveServo::Reset() {
    auto rc = ValidateParameters();
    if (rc != Result::NoError) return rc;

    if (!sock_.is_open()) return Result::ServoBindFailed;

    // 刷新 joint_count（硬件可能在构造后才就绪）
    if (hw_ && hw_->GetDriveNum() > 0) joint_count_ = hw_->GetDriveNum();
    if (joint_count_ <= 0 && model_) joint_count_ = model_->GetJointNum();
    if (joint_count_ <= 0) return Result::UnmatchedJointsNumber;

    {   // 重置共享缓冲区
        std::lock_guard<std::mutex> lock(mtx_);
        cmd_ = MotionGeneratorCommand{};
        has_new_cmd_ = false;
    }
    stopped_.store(false);
    running_.store(true);
    message_id_counter_ = 0;

    success_window_.fill(false);
    success_index_ = 0;
    success_count_ = 0;

    if (current_mode_ == MotionMode::kNone) {
        current_mode_ = MotionMode::kJointPosition;
    }

    // join 旧线程（复用场景）
    if (thread_.joinable()) {
        running_.store(false);
        thread_.join();
        running_.store(true);
    }

    try {
        thread_ = std::thread(&MoveServo::udpReceiveLoop, this);
    } catch (const std::system_error&) {
        return Result::ServoThreadStartFailed;
    }
    return Result::NoError;
}
```

#### 4.5 `Update()` —— 读取硬件状态、构建 RobotState、发送

```cpp
Result MoveServo::Update() {
    if (stopped_.load(std::memory_order_relaxed)) return Result::PlanFinished;
    if (!hw_) return Result::NoError;

    auto q    = hw_->GetPosition();
    auto dq   = hw_->GetVelocity();
    auto tau  = hw_->GetTorque();
    auto load = hw_->GetLoadTorque();

    RobotState state{};
    state.message_id = ++message_id_counter_;

    for (int i = 0; i < joint_count_ && i < static_cast<int>(MAX_DOF); ++i) {
        state.q[i]    = q(static_cast<unsigned int>(i));
        state.dq[i]   = dq(static_cast<unsigned int>(i));
        state.tau[i]  = tau(static_cast<unsigned int>(i));
        state.load[i] = load(static_cast<unsigned int>(i));
    }

    if (model_) {
        Frame flange;
        model_->ForwardKinematics(q, flange);
        frameToMatrix(flange, state.flange_to_base);
    }
    state.tcp_to_base = state.flange_to_base;  // TODO: 复合 tool_frame
    state.errors = {};
    state.mode   = current_mode_;
    state.control_command_success_rate = computeSuccessRate();

    {
        std::lock_guard<std::mutex> lock(mtx_);
        ssize_t sent = sock_.send_to(&state, sizeof(RobotState), client_addr_);
        recordSuccess(sent == sizeof(RobotState));
    }

    return Result::NoError;
}
```

#### 4.6 `GenerateRef()` —— 将指令转为 Reference

```cpp
Result MoveServo::GenerateRef(Reference& ref_out) {
    if (stopped_.load(std::memory_order_relaxed)) return Result::PlanFinished;

    MotionGeneratorCommand local_cmd;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!has_new_cmd_) return Result::NoError;
        local_cmd = cmd_;
    }

    switch (current_mode_) {
    case MotionMode::kJointPosition: {
        JntArray q_ref(static_cast<unsigned int>(joint_count_));
        for (int i = 0; i < joint_count_; ++i)
            q_ref(i) = local_cmd.q_c[static_cast<size_t>(i)];
        ref_out = std::move(q_ref);
        return Result::NoError;
    }
    case MotionMode::kCartesianPosition: {
        Frame f = matrixToFrame(local_cmd.tcp_c);
        ref_out = std::move(f);
        return Result::NoError;
    }
    default:
        return Result::NoError;
    }
}
```

#### 4.7 生命周期方法

- **`Pause()`** / **`Resume()`** → 返回 `Result::FunctionNotSupported`（伺服模式是连续流模式，不支持暂停）
- **`Stop()`** → 设置 `running_ = false`、`stopped_ = true`，`join` 线程
- **`ValidateParameters()`** → 检查 `hw_` 非空、`joint_count_ > 0`、`listen_port_ != 0`

#### 4.8 辅助方法

- **`matrixToFrame`**: 16 元素 row-major 4x4 → `KDL::Frame`（`KDL::Rotation` 以 column-major 传入）
- **`frameToMatrix`**: `KDL::Frame` → 16 元素 row-major 4x4
- **`recordSuccess`** / **`computeSuccessRate`**: 滑动窗口统计最近 100 次发送的成功率

### 步骤 5: Robot 集成 (`src/robot.hpp` + `src/robot.cpp`)

#### 5.1 声明 `MoveServoing()`

在 `robot.hpp` 中 `MoveSvdJogging` 声明之后添加：

```cpp
Result MoveServoing(uint16_t port = 8081);
```

#### 5.2 实现 `Robot::MoveServoing()`

```cpp
Result Robot::MoveServoing(uint16_t port) {
    data_ready_callback_ = [this, port]() -> Result {
        auto servo = std::make_unique<MoveServo>(
            hardware.get(), model.get(), port);
        Result rc = servo->Reset();
        if (rc != Result::NoError) return rc;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            motion = std::move(servo);
            executor->SwitchMotion(motion.get());
        }
        return Result::NoError;
    };

    if (!impl_->process_event(EventServoReq{})) {
        if (impl_->is(sml::state<class IDLE>)) return Result::NotEnabled;
        if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
        return Result::ConflictTaskRunning;
    }
    return Result::NoError;
}
```

#### 5.3 更新 `on_fsm_servoing()`

```cpp
void Robot::on_fsm_servoing() {
    log_ptr_->info("Robot is servoing...");
    if (data_ready_callback_) {
        Result rc = data_ready_callback_();
        if (rc != Result::NoError && rc != Result::PlanFinished) {
            log_ptr_->error("伺服模式启动失败: {}", static_cast<int>(rc));
            impl_->process_event(EventErrorOccurred{});
        }
    }
}
```

#### 5.4 `IsControlActive()` 添加 `SERVOING`

```cpp
bool Robot::IsControlActive() const {
    return /* ... existing states ... */
           || impl_->is(sml::state<class SERVOING>);
}
```

#### 5.5 添加 `#include "move_servo.hpp"` 到 `robot.cpp`

### 步骤 6: 构建验证

```bash
cmake -S . -B build && cmake --build build -j
```

验证编译通过，无警告。

## 数据流总结

```
UDP Thread (后台)            Control Thread (1kHz)
───────┬───────              ───────┬───────
  Phase 1: drain                      │
  Phase 2: 等待新包                   │
  ──► cmd_, client_addr_ (mtx_) ──►  Update():
                                       read hw → build RobotState
                                       send_to(client_addr_, state) → UDP
                                     GenerateRef():
                                       read cmd_ → JntArray | Frame
                                         → Controller → Hardware
```

## 涉及文件

| 文件 | 操作 |
|------|------|
| `src/result.hpp` | 添加 3 个错误码 + RESULT_CASE |
| `CMakeLists.txt` | 添加 sockpp subdir、move_servo.cpp 源文件、链接 sockpp |
| `src/move_servo.hpp` | **新建** |
| `src/move_servo.cpp` | **新建** |
| `src/robot.hpp` | 添加 `MoveServoing()` 声明 |
| `src/robot.cpp` | 添加 include、实现 MoveServoing、更新 on_fsm_servoing、IsControlActive |

## 验证

1. `cmake -S . -B build && cmake --build build -j` — 编译通过
2. `./build/bin/unit_test` — 现有测试全部通过（新类暂不引入单元测试，后续补充）
3. 运行 `./build/bin/rocosAppMain --sim=true --http_port=8080` 后，通过 UDP 客户端向 localhost:8081 发送 `MotionGeneratorCommand` 验证收发
