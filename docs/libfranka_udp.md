# libfranka UDP 通信协议设计

## 概述

libfranka 采用 **TCP + UDP 双通道** 架构与 Franka Robotics 机器人进行通信：

| 通道 | 方向 | 用途 | 端口 | 特点 |
|------|------|------|------|------|
| TCP | 双向 | 命令/响应 | 1337（固定） | 可靠、有序，请求-响应模式，超时 1s |
| UDP | 双向 | 实时数据流 | 客户端 OS 分配 | 低延迟、1kHz、允许丢包，超时 1s |

- **TCP 通道**负责：连接握手、版本协商、启停运动、设置碰撞阈值/阻抗/负载等参数、自动错误恢复
- **UDP 通道**负责：机器人状态推送（RobotState）和客户端控制指令下发（RobotCommand）

---

## 1. 连接建立流程

```
Client                                 Robot (Control)
  │                                          │
  │── TCP connect(1337) ────────────────────►│  建连
  │                                          │
  │── Connect(udp_port) ────────────────────►│  告知客户端 UDP 端口
  │                                          │
  │◄─ ConnectResponse(version, status) ──────│  版本协商
  │                                          │
  │◄─ UDP RobotState ────────────────────────│  开始 1kHz 持续推送
  │                                          │
  │── UDP RobotCommand ─────────────────────►│  收到状态后回复控制指令
```

关键代码（[src/network.cpp](src/network.cpp#L36-L37)）：

```cpp
udp_socket_.bind({"0.0.0.0", 0});   // OS 分配随机可用端口
udp_port_ = udp_socket_.address().port();
```

随后通过 TCP `Connect` 命令将此 UDP 端口告知 Robot（[src/robot_impl.cpp](src/robot_impl.cpp#L52-L53)）：

```cpp
connect<research_interface::robot::Connect, research_interface::robot::kVersion>(
    *network_, &ri_version_);
```

---

## 2. UDP 数据包结构

定义在 [common/include/research_interface/robot/rbk_types.h](common/include/research_interface/robot/rbk_types.h)。所有结构体使用 `#pragma pack(push, 1)` 确保紧凑二进制布局——**不做序列化，直接将 struct 字节通过 UDP socket 收发**。

### 2.1 RobotState（Robot → Client，约 840 bytes）

```cpp
#pragma pack(push, 1)

struct RobotState {
    // ── 同步字段 ──
    uint64_t message_id;                     // 单调递增，兼作时间戳（单位 ms）

    // ── 位姿矩阵（4x4 齐次变换，列主序，16 个 double） ──
    std::array<double, 16> O_T_EE;           // 末端在基坐标系中的实际位姿
    std::array<double, 16> O_T_EE_d;         // 末端在基坐标系中的期望位姿
    std::array<double, 16> F_T_EE;           // 末端在法兰坐标系中的位姿
    std::array<double, 16> EE_T_K;           // 刚度坐标系在末端坐标系中的位姿
    std::array<double, 16> F_T_NE;           // 名义末端在法兰坐标系中的位姿
    std::array<double, 16> NE_T_EE;          // 末端在名义末端坐标系中的位姿

    // ── 负载参数 ──
    double m_ee;                             // 末端执行器质量 [kg]
    std::array<double, 9> I_ee;              // 末端执行器惯性张量（3x3 列主序）
    std::array<double, 3> F_x_Cee;           // 末端执行器质心在法兰坐标系中的位置
    double m_load;                           // 外部负载质量 [kg]
    std::array<double, 9> I_load;            // 外部负载惯性张量
    std::array<double, 3> F_x_Cload;         // 外部负载质心位置

    // ── 肘部配置 ──
    std::array<double, 2> elbow;             // [关节3位置, 翻转方向]
    std::array<double, 2> elbow_d;           // 期望肘部
    std::array<double, 2> elbow_c;           // 指令肘部
    std::array<double, 2> delbow_c;          // 指令肘部速度
    std::array<double, 2> ddelbow_c;         // 指令肘部加速度

    // ── 关节扭矩 [Nm] ──
    std::array<double, 7> tau_J;             // 实测关节扭矩
    std::array<double, 7> tau_J_d;           // 期望关节扭矩（不含重力和摩擦）
    std::array<double, 7> dtau_J;            // 扭矩导数 [Nm/s]

    // ── 关节运动学 [rad] ──
    std::array<double, 7> q;                 // 实测关节位置
    std::array<double, 7> q_d;              // 期望关节位置
    std::array<double, 7> dq;               // 实测关节速度 [rad/s]
    std::array<double, 7> dq_d;             // 期望关节速度
    std::array<double, 7> ddq_d;            // 期望关节加速度 [rad/s²]

    // ── 碰撞/接触检测（0 为无接触，正值表示接触等级） ──
    std::array<double, 7> joint_contact;     // 关节接触（消失后归零）
    std::array<double, 6> cartesian_contact; // 笛卡尔接触 (x,y,z,R,P,Y)
    std::array<double, 7> joint_collision;   // 关节碰撞（保持直到 reset）
    std::array<double, 6> cartesian_collision;// 笛卡尔碰撞

    // ── 外力估计 ──
    std::array<double, 7> tau_ext_hat_filtered;  // 滤波后外部关节扭矩 [Nm]
    std::array<double, 6> O_F_ext_hat_K;         // 基坐标系下的外部 wrench
    std::array<double, 6> K_F_ext_hat_K;         // 刚度坐标系下的外部 wrench

    // ── 笛卡尔运动 ──
    std::array<double, 6> O_dP_EE_d;         // 期望末端 twist (vx,vy,vz,ωx,ωy,ωz)
    std::array<double, 3> O_ddP_O;           // 基座加速度（重力方向）[m/s²]
    std::array<double, 16> O_T_EE_c;         // 指令末端位姿
    std::array<double, 6> O_dP_EE_c;         // 指令末端 twist
    std::array<double, 6> O_ddP_EE_c;        // 指令末端加速度

    // ── 电机状态 ──
    std::array<double, 7> theta;             // 电机位置 [rad]
    std::array<double, 7> dtheta;            // 电机速度 [rad/s]

    // ── 模式与错误 ──
    MotionGeneratorMode motion_generator_mode;  // 运动生成器当前模式
    ControllerMode controller_mode;            // 控制器当前模式
    std::array<bool, 41> errors;               // 当前错误标志位集合
    std::array<bool, 41> reflex_reason;        // 反射停止原因（即 last_motion_errors）
    RobotMode robot_mode;                      // 机器人运行模式

    // ── 通信质量 ──
    double control_command_success_rate;       // 最近 100 个指令的接收成功率 [0, 1]
};
```

### 2.2 MotionGeneratorCommand（运动指令）

```cpp
struct MotionGeneratorCommand {
    std::array<double, 7> q_c;               // 关节位置指令 [rad]
    std::array<double, 7> dq_c;              // 关节速度指令 [rad/s]
    std::array<double, 16> O_T_EE_c;         // 笛卡尔位姿指令（4x4 列主序）
    std::array<double, 6> O_dP_EE_c;         // 笛卡尔速度指令 (vx,vy,vz,ωx,ωy,ωz)
    std::array<double, 2> elbow_c;           // 肘部指令
    bool valid_elbow;                         // 肘部指令是否有效
    bool motion_generation_finished;          // 运动完成标志（停止循环）
};
```

### 2.3 ControllerCommand（扭矩控制指令）

```cpp
struct ControllerCommand {
    std::array<double, 7> tau_J_d;           // 期望关节扭矩（不含重力/摩擦） [Nm]
    bool torque_command_finished;             // 控制器完成标志
};
```

### 2.4 RobotCommand（Client → Robot，组合指令包）

```cpp
struct RobotCommand {
    uint64_t message_id;                      // 与最后收到的 RobotState.message_id 对应
    MotionGeneratorCommand motion;
    ControllerCommand control;
};

#pragma pack(pop)
```

---

## 3. 控制循环协议

### 3.1 基本同步模式

UDP 通道形成了一个 **request-reply 同步模式**（虽然 UDP 是无连接协议，但 libfranka 实现了逻辑上的同步）：

```
时间 ─────────────────────────────────────────────────────►

Robot    ──[State #N]──────────[State #N+1]──────────[State #N+2]──►
              │                    │                    │
Client       ├─ 回调计算           ├─ 回调计算           ├─ 回调计算
              │                    │                    │
Robot    ◄──[Cmd #N]─────────◄──[Cmd #N+1]─────────◄──[Cmd #N+2]──
```

每次循环迭代：
1. Robot 通过 UDP 推送 `RobotState`（message_id = N）
2. 客户端用户回调函数根据状态计算控制输出
3. 客户端通过 UDP 回复 `RobotCommand`（message_id = N，携带运动/扭矩指令）
4. Robot 处理指令并推送下一个 `RobotState`（message_id = N+1）

### 3.2 ControlLoop 实现

控制循环的完整逻辑在 [src/control_loop.cpp](src/control_loop.cpp) 中：

```
ControlLoop::operator()()  [实时线程, SCHED_FIFO 最高优先级]
│
├─ robot_.update(nullptr, nullptr)           // 获取初始状态
│
└─ loop:
   ├─ motion = motion_callback(state, dt)    // 调用用户运动生成回调
   ├─ torque = control_callback(state, dt)   // 调用用户扭矩控制回调（可选）
   │
   ├─ 后处理（可选）:
   │   ├─ lowpassFilter()                    // 一阶低通滤波, 默认截止频率 100Hz
   │   └─ limitRate()                        // 速度/加速度/jerk 限幅
   │
   ├─ convertMotion() → MotionGeneratorCmd   // 类型转换
   ├─ createControllerCommand() → CtrlCmd    // 类型转换
   │
   ├─ robot_.update(&motion_cmd, &ctrl_cmd)  // UDP 收发:
   │   ├─ sendRobotCommand() ──UDP──► Robot
   │   └─ receiveRobotState() ◄──UDP── Robot
   │
   ├─ throwOnMotionError(state, motion_id)   // 错误检测
   │
   └─ if (motion_finished) → break           // 用户返回 MotionFinished() 则退出

│
└─ robot_.finishMotion(motion_id, ...)       // 发送 motion_generation_finished=true,
                                               等待 TCP Move 响应确认结束
```

---

## 4. receiveRobotState() 的接收策略

[src/robot_impl.cpp](src/robot_impl.cpp#L158-L180) 中的核心策略："总是取最新状态，丢弃积压旧包"：

```
receiveRobotState():
│
├─ 阶段 1：非阻塞排空
│   while (udpReceive(&state) == 有数据) {   // Poco::DatagramSocket.available()
│       if (state.message_id > latest.message_id)
│           latest = state                   // 保留最新的
│   }
│
├─ 阶段 2：阻塞兜底
│   while (latest.message_id == last_processed_id) {
│       latest = udpBlockingReceive()        // 阻塞等待新数据到达
│   }
│
└─ 返回 latest
```

**设计意图**：控制循环因用户回调计算耗时、系统调度延迟等原因，运行频率可能低于 1kHz。此时 UDP 缓冲区会积压多个 RobotState。采用此策略确保：
- 始终基于**最新**的机器人状态做控制决策
- 不因处理积压旧状态而引入额外延迟
- 如果完全没有新数据，阻塞等待（避免空转）

### 运动模式跟踪

每次收到 RobotState 后更新内部跟踪变量：

```cpp
void updateState(const RobotState& robot_state) {
    robot_mode_         = robot_state.robot_mode;           // 当前模式
    motion_generator_mode_ = robot_state.motion_generator_mode; // 运动生成器模式
    controller_mode_    = robot_state.controller_mode;      // 控制器模式
    message_id_         = robot_state.message_id;           // 最后处理的 ID
}
```

这些变量用于：
- 判断运动/控制器是否仍在运行（决定何时退出 `finishMotion` 循环）
- `receiveRobotState()` 中判断是否有新状态到达
- `sendRobotCommand()` 中校验是否可以发送指令

---

## 5. sendRobotCommand() 的发送逻辑

[src/robot_impl.cpp](src/robot_impl.cpp#L121-L156)：

```
sendRobotCommand(motion_cmd_ptr, control_cmd_ptr):
│
├─ 构建 RobotCommand{ message_id = message_id_ }
│
├─ 校验：
│   ├─ motion 指令非空但无运动生成器运行 → throw ControlException
│   ├─ control 指令非空但非 ExternalController 模式 → throw ControlException
│   └─ 运动+控制同时运行时只发了其中一半 → throw ControlException
│
├─ 如果 motion_cmd 和 control_cmd 都为空
│   └─ 不发送 UDP 包（仅读取状态，如 readOnce()）
│
└─ 否则: network_->udpSend<RobotCommand>(robot_command)
```

关键校验逻辑保证了**不能**在错误模式下发送指令，防止协议层面的误操作。

---

## 6. 运动生命周期管理（TCP + UDP 协作）

一段运动的完整协议交互：

```
Phase                     TCP                          UDP
──────────────────────────────────────────────────────────────
1. 启动运动
   Client ──Move(mode)──► Robot         (同步等待 Robot 确认)
                                                          state 持续推送
   Client ◄──MoveResponse── Robot       (TCP 响应)
                                         等待 mode 切换
                                                          state.motion_generator_mode == 目标模式
2. 控制循环
                                                        ◄── RobotState (1kHz)
                                                        ──► RobotCommand (1kHz)
   ... 循环直到 motion_finished == true ...
3. 结束运动
                                                        ──► RobotCommand{ motion_generation_finished=true }
                                                          等待 mode 切回 Idle
                                                        ◄── RobotState{ motion_generator_mode=kIdle }
   Client ◄──MoveResponse── Robot       (TCP 最终响应)

4. 异常取消
   Client ──StopMove()──► Robot         (同步 TCP)
                                                        ◄── RobotState 持续接收直到 mode=kIdle
```

### `startMotion()` 中的同步等待

[src/robot_impl.cpp](src/robot_impl.cpp#L205-L272) 中，发送 `Move` 命令后，通过 TCP 和 UDP 双重等待来确保运动已启动：

```cpp
// 1. 通过 TCP 发送 Move 命令
const uint32_t move_command_id = executeCommand<Move>(...);

// 2. 等待 Robot 确认启动（TCP 响应 或 UDP 状态变更）
while (motion_generator_mode_ != target_mode ||
       controller_mode_ != target_mode) {
    if (network_->tcpReceiveResponse<Move>(move_command_id, handler)) break;
    robot_state = update(nullptr, nullptr);  // 仅读取状态，不发送指令
}
```

### `finishMotion()` 中的结束等待

[src/robot_impl.cpp](src/robot_impl.cpp#L274-L311)：发送 `motion_generation_finished=true` 后，持续发送该指令直到 UDP 状态中 mode 切回 Idle，然后阻塞等待 TCP `Move` 最终响应以获取结束状态码。

---

## 7. 多模块 UDP 绑定

libfranka 支持同时控制 Robot、Gripper、VacuumGripper。每个模块实例化时创建独立的 `Network` 对象，因此拥有独立的 UDP 端口：

```
┌──────────┐  ┌──────────┐  ┌──────────────────┐
│  Robot   │  │ Gripper  │  │  VacuumGripper   │
│ TCP:1337 │  │TCP:1338  │  │  TCP:1339        │
│ UDP:???? │  │UDP:????  │  │  UDP:????        │
└────┬─────┘  └────┬─────┘  └───────┬──────────┘
     │              │                │
     └──────────────┼────────────────┘
                    │
               Robot Control
           (同一 IP, 不同端口)
```

- Robot 指令端口: `research_interface::robot::kCommandPort` (1337)
- Gripper 指令端口: `research_interface::gripper::kCommandPort` (1338)
- VacuumGripper 指令端口: `research_interface::vacuum_gripper::kCommandPort` (1339)

---

## 8. 协议设计要点总结

| 设计目标 | 实现手段 |
|---------|---------|
| **低延迟** | 原始 struct 二进制传输，零序列化/反序列化开销 |
| **实时性** | UDP 允许丢包；客户端始终消费最新状态，丢弃积压旧包 |
| **同步追踪** | `message_id` 贯穿 RobotState → RobotCommand → RobotState，形成闭环 |
| **可靠命令** | 启停运动、参数设置等关键操作走 TCP，带状态码响应和异常处理 |
| **紧凑数据** | `#pragma pack(push, 1)` 消除 padding，固定结构大小 |
| **线程安全** | `udp_mutex_` / `tcp_mutex_` 保护 socket 并发访问 |
| **连接监控** | 每次 `update()` 前通过 `tcpThrowIfConnectionClosed()` 检测 TCP 断开 |
| **可组合控制** | `RobotCommand` 同时携带运动指令和扭矩指令，支持外部控制器模式 |
| **模式安全** | `sendRobotCommand()` 在发送 UDP 前校验当前模式，防止无效指令 |
| **错误可追溯** | `ControlException` 携带 `vector<Record>`（状态+指令历史），含丢包统计 |
