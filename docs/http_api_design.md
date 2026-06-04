# ROCOS-App HTTP REST API 开发需求文档

## 1. 背景与目标

### 1.1 现状

ROCOS-App 当前使用 gRPC 作为唯一的外部通信接口（端口 30001），客户端通过 Protobuf 序列化与机器人控制器交互。存在以下痛点：

- gRPC 调试门槛高，需要专用工具（grpcurl、grpcui）
- 前端/Web 应用无法直接调用 gRPC
- 与第三方系统集成时 Protobuf 定义维护成本高
- 客户端必须依赖 gRPC 运行时库

### 1.2 目标

使用 HTTP/HTTPS RESTful API **完全替代** gRPC 通信，采用 JSON 格式，降低集成门槛，使任意语言/平台（浏览器、Python、curl、移动端）都能方便地控制机器人。

**核心设计原则：** 封装为一个独立、自包含的 C++ 类（`RobotHttpServer`），不依赖 gRPC/Protobuf，便于后期整体移植到其他项目。

---

## 2. 技术选型

使用 [cpp-httplib](https://github.com/yhirose/cpp-httplib) 作为 HTTP 服务器库：

- C++11 单头文件库（`httplib.h`），零外部依赖
- 无需 gRPC/Protobuf 依赖
- 支持 HTTP/HTTPS、WebSocket、SSE
- 阻塞式 I/O，适合本项目低频控制场景
- 嵌入简单，与现有 CMake 构建系统兼容

JSON 解析使用 [nlohmann/json](https://github.com/nlohmann/json)（同样是单头文件，零依赖）。

---

## 3. 类设计：RobotHttpServer

### 3.1 设计原则

- **独立单文件类**：`.h` + `.cc`，不依赖 gRPC/Protobuf 代码
- **仅依赖 Robot 核心类**：通过 `Robot*` 指针访问机器人功能
- **自包含路由注册**：所有路由在类内部注册，外部调用 `run()` 即启动
- **可整体移植**：拷贝 3 个文件（`httplib.h`、`robot_http_server.h`、`robot_http_server.cc`）即可在其他项目中使用

### 3.2 类接口

```cpp
namespace rocos {

class RobotHttpServer {
public:
    explicit RobotHttpServer(Robot* robot);
    ~RobotHttpServer();

    // 启动服务器（阻塞）
    void run(const std::string& host = "0.0.0.0", int port = 8080);

    // 非阻塞启动（独立线程）
    void runAsync(const std::string& host = "0.0.0.0", int port = 8080);

    // 停止服务器
    void stop();

private:
    void registerRoutes();

    // ---- 机器人状态 ----
    void handleGetRobotState(const httplib::Request& req, httplib::Response& res);
    void handleGetRobotInfo(const httplib::Request& req, httplib::Response& res);
    void handleGetRobotModel(const httplib::Request& req, httplib::Response& res);
    void handleGetLinkMesh(const httplib::Request& req, httplib::Response& res);

    // ---- 机器人控制 ----
    void handleEnable(const httplib::Request& req, httplib::Response& res);
    void handleDisable(const httplib::Request& req, httplib::Response& res);
    void handleIsEnabled(const httplib::Request& req, httplib::Response& res);
    void handleSetWorkMode(const httplib::Request& req, httplib::Response& res);

    // ---- 运动控制 ----
    void handleMoveJ(const httplib::Request& req, httplib::Response& res);
    void handleMoveJ_IK(const httplib::Request& req, httplib::Response& res);
    void handleMoveL(const httplib::Request& req, httplib::Response& res);
    void handleMoveL_FK(const httplib::Request& req, httplib::Response& res);
    void handleMoveC(const httplib::Request& req, httplib::Response& res);
    void handleMoveP(const httplib::Request& req, httplib::Response& res);
    void handleMovePath(const httplib::Request& req, httplib::Response& res);
    void handleStop(const httplib::Request& req, httplib::Response& res);

    // ---- 单轴/多轴控制 ----
    void handleSingleAxisEnable(const httplib::Request& req, httplib::Response& res);
    void handleSingleAxisDisable(const httplib::Request& req, httplib::Response& res);
    void handleSingleAxisMove(const httplib::Request& req, httplib::Response& res);
    void handleSingleAxisStop(const httplib::Request& req, httplib::Response& res);
    void handleMultiAxisEnable(const httplib::Request& req, httplib::Response& res);
    void handleMultiAxisDisable(const httplib::Request& req, httplib::Response& res);
    void handleMultiAxisMove(const httplib::Request& req, httplib::Response& res);
    void handleMultiAxisStop(const httplib::Request& req, httplib::Response& res);
    void handleMultiAxisSync(const httplib::Request& req, httplib::Response& res);

    // ---- 拖拽示教 ----
    void handleDragStart(const httplib::Request& req, httplib::Response& res);
    void handleDragStop(const httplib::Request& req, httplib::Response& res);

    // ---- 异步任务查询 ----
    void handleMoveStatus(const httplib::Request& req, httplib::Response& res);

    // ---- 标定 ----
    void handleSetPoseFrame(const httplib::Request& req, httplib::Response& res);
    void handleSetToolFrame(const httplib::Request& req, httplib::Response& res);
    void handleSetObjectFrame(const httplib::Request& req, httplib::Response& res);
    void handleCalibrationRun(const httplib::Request& req, httplib::Response& res);
    void handleCalibrationResult(const httplib::Request& req, httplib::Response& res);

    // ---- 工具函数 ----
    void sendJson(httplib::Response& res, int httpCode, bool success,
                  int businessCode, const std::string& message,
                  const nlohmann::json& data = nullptr);

    KDL::Frame jsonToFrame(const nlohmann::json& j);
    nlohmann::json frameToJson(const KDL::Frame& frame);

    // ---- 异步任务管理 ----
    std::string generateTaskId();
    void registerTask(const std::string& taskId, const std::string& type);
    void updateTaskStatus(const std::string& taskId, const std::string& status, int result = 0);
    nlohmann::json getTaskInfo(const std::string& taskId);

    Robot* robot_;
    std::unique_ptr<httplib::Server> server_;
    std::unique_ptr<std::thread> thread_;
    std::mutex taskMutex_;
    // taskId -> {type, status, result, message, create_time}
    std::map<std::string, nlohmann::json> taskMap_;
};

} // namespace rocos
```

### 3.3 在 main 中的使用

```cpp
#include <rocos_app/robot_http_server.h>

int main(int argc, char* argv[]) {
    // ... 初始化 Robot（与现有逻辑相同） ...
    Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);

    // 替换 gRPC：一行启动 HTTP 服务器
    rocos::RobotHttpServer httpServer(&robot);
    httpServer.run("0.0.0.0", 8080);  // 阻塞运行

    return 0;
}
```

---

## 4. API 总览

基础地址：`http://<host>:<port>`（默认端口 `8080`）

### 4.1 机器人状态

| 方法 | 路径 | 说明 |
|------|------|------|
| `GET` | `/api/robot/state` | 获取机器人完整状态（关节状态 + 笛卡尔位姿 + 硬件状态） |
| `GET` | `/api/robot/info` | 获取机器人信息（关节参数、单位等） |
| `GET` | `/api/robot/model` | 获取机器人 URDF 模型信息（连杆、关节类型、mesh） |
| `GET` | `/api/robot/model/mesh?path=<link_name>` | 获取连杆 mesh 文件（二进制） |

### 4.2 机器人控制

| 方法 | 路径 | 说明 |
|------|------|------|
| `POST` | `/api/robot/enable` | 使能机器人 |
| `POST` | `/api/robot/disable` | 禁用机器人 |
| `GET` | `/api/robot/enabled` | 查询使能状态 |
| `POST` | `/api/robot/workmode` | 设置工作模式 |

### 4.3 运动控制

| 方法 | 路径 | 说明 |
|------|------|------|
| `POST` | `/api/move/joint` | 关节空间运动 (MoveJ) |
| `POST` | `/api/move/joint_ik` | 笛卡尔位姿逆解运动 (MoveJ_IK) |
| `POST` | `/api/move/linear` | 笛卡尔直线运动 (MoveL) |
| `POST` | `/api/move/linear_fk` | 关节空间直线运动 (MoveL_FK) |
| `POST` | `/api/move/circle` | 圆弧运动 (MoveC) |
| `POST` | `/api/move/path` | 路径运动 (MoveP) |
| `POST` | `/api/move/pathway` | 多路点路径运动 (MovePath) |
| `POST` | `/api/move/stop` | 停止运动 |
| `GET` | `/api/move/status?task_id=<id>` | 查询异步运动任务状态 |

### 4.4 单轴/多轴控制

| 方法 | 路径 | 说明 |
|------|------|------|
| `POST` | `/api/axis/single/enable` | 单轴使能 |
| `POST` | `/api/axis/single/disable` | 单轴禁用 |
| `POST` | `/api/axis/single/move` | 单轴运动 |
| `POST` | `/api/axis/single/stop` | 单轴停止 |
| `POST` | `/api/axis/multi/enable` | 多轴使能 |
| `POST` | `/api/axis/multi/disable` | 多轴禁用 |
| `POST` | `/api/axis/multi/move` | 多轴运动 |
| `POST` | `/api/axis/multi/stop` | 多轴停止 |
| `POST` | `/api/axis/multi/sync` | 多轴同步 |

### 4.5 拖拽示教

| 方法 | 路径 | 说明 |
|------|------|------|
| `POST` | `/api/drag/start` | 启动拖拽示教 |
| `POST` | `/api/drag/stop` | 停止拖拽示教 |

### 4.6 标定

| 方法 | 路径 | 说明 |
|------|------|------|
| `POST` | `/api/calibration/pose` | 设置标定点 |
| `POST` | `/api/calibration/tool` | 设置工具坐标系 |
| `POST` | `/api/calibration/object` | 设置工件坐标系 |
| `POST` | `/api/calibration/run` | 执行标定 |
| `GET` | `/api/calibration/result` | 获取标定结果 |

---

## 5. 请求/响应格式

### 5.1 通用响应结构

所有 API 返回统一 JSON 格式：

```json
{
  "success": true,
  "code": 0,
  "message": "ok",
  "data": { ... }
}
```

失败时：

```json
{
  "success": false,
  "code": 2001,
  "message": "inverse kinematics: no solution found for target pose",
  "data": null
}
```

`code` 字段含义：`0` = 成功，`1xxx` = 参数错误，`2xxx` = 运动规划错误，`3xxx` = 机器人状态错误，`4xxx` = 标定错误，`5xxx` = 异步任务错误。详见第 7 节。

### 5.2 关键数据类型

**SE3Pose（位姿）：**

```json
{
  "position": { "x": 0.3, "y": 0.1, "z": 0.5 },
  "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
}
```

**JointState（关节状态）：**

```json
{
  "name": "joint_1",
  "position": 0.123,
  "velocity": 0.045,
  "acceleration": 0.01,
  "load": 2.3,
  "status": "ENABLED"
}
```

**WorkMode 枚举值：**

| 值 | 说明 |
|----|------|
| `Position` | 位置模式 |
| `EeAdmitTeach` | 末端导纳示教 |
| `JntAdmitTeach` | 关节导纳示教 |
| `JntImp` | 关节阻抗控制 |
| `CartImp` | 笛卡尔阻抗控制 |

---

## 6. 各 API 详细定义

### 6.1 `GET /api/robot/state`

**响应 `data`：**

```json
{
  "joint_states": [
    {
      "name": "joint_1",
      "position": 0.123,
      "velocity": 0.045,
      "acceleration": 0.01,
      "load": 2.3,
      "status": "ENABLED"
    }
  ],
  "flange": { "position": { "x":0, "y":0, "z":0 }, "orientation": { "x":0, "y":0, "z":0, "w":1 } },
  "tool": { "position": { "x":0, "y":0, "z":0 }, "orientation": { "x":0, "y":0, "z":0, "w":1 } },
  "object": { "position": { "x":0, "y":0, "z":0 }, "orientation": { "x":0, "y":0, "z":0, "w":1 } },
  "hw_state": {
    "hw_type": 0,
    "min_cycle_time": 0.0005,
    "max_cycle_time": 0.003,
    "current_cycle_time": 0.001,
    "slave_num": 8
  }
}
```

  > 注意：当前服务器实现（`src/robot_http_server.cc`）在 `GET /api/robot/state` 返回字段命名与设计文档略有差异：实现使用 `flange` (而非 `flange_pose`)、`tool`/`object` 而非 `tool_pose`/`object_pose`，以及 `hw_state` (而非 `hardware`)。已在规范中保留设计命名，但建议在发布版本中统一为一套命名 —— 当前变更流程为：以实现为准，文档标注 Not Implemented/字段差异，后续由产品/开发决定是否修改实现以匹配设计。

### 6.2 `POST /api/move/joint`

**请求体：**

```json
{
  "joints": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
  "speed": 1.05,
  "acceleration": 1.4,
  "time": 0.0,
  "radius": 0.0,
  "asynchronous": false
}
```

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `joints` | `double[]` | 是 | - | 目标关节角度（弧度） |
| `speed` | `double` | 否 | `1.05` | 速度限制 |
| `acceleration` | `double` | 否 | `1.4` | 加速度限制 |
| `time` | `double` | 否 | `0.0` | 最短运行时间（0 = 自动） |
| `radius` | `double` | 否 | `0.0` | 过渡半径 |
| `asynchronous` | `bool` | 否 | `false` | 是否异步执行 |

**响应 `data`（同步执行时）：**

```json
{
  "result": 0,
  "message": "ok"
}
```

**响应 `data`（异步执行时 `asynchronous: true`）：**

```json
{
  "task_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "status": "RUNNING",
  "message": "task created"
}
```

异步任务通过 `GET /api/move/status?task_id=<id>` 查询执行状态。

### 6.3 `POST /api/move/linear`

**请求体：**

```json
{
  "pose": {
    "position": { "x": 0.3, "y": 0.1, "z": 0.5 },
    "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
  },
  "speed": 1.05,
  "acceleration": 1.4,
  "time": 0.0,
  "radius": 0.0,
  "asynchronous": false
}
```

### 6.4 `GET /api/move/status`

查询异步运动任务的执行状态，供上层任务编排系统（如具身智能 LLM Agent、视觉策略规划器）进行状态追踪和错误恢复。

**查询参数：**

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `task_id` | `string` | 是 | 异步任务 ID（由运动接口异步返回） |

**响应 `data`：**

```json
{
  "task_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "type": "MoveJ",
  "status": "COMPLETED",
  "result": 0,
  "message": "ok",
  "create_time": "2025-06-03T14:30:00Z",
  "finish_time": "2025-06-03T14:30:02Z"
}
```

**`status` 取值：**

| 状态 | 说明 |
|------|------|
| `RUNNING` | 正在执行 |
| `COMPLETED` | 已完成（`result = 0`） |
| `FAILED` | 执行失败（`result != 0`，`message` 含具体错误码） |
| `STOPPED` | 被用户中断（`/api/move/stop`） |

### 6.5 `POST /api/move/circle`

**请求体：**

```json
{
  "pose_via": { "position": { "x":0, "y":0, "z":0 }, "orientation": { "x":0, "y":0, "z":0, "w":1 } },
  "pose_to": { "position": { "x":0, "y":0, "z":0 }, "orientation": { "x":0, "y":0, "z":0, "w":1 } },
  "speed": 0.25,
  "acceleration": 1.2,
  "time": 0.0,
  "radius": 0.0,
  "mode": "UNCONSTRAINED",
  "asynchronous": false
}
```

`mode` 取值：`UNCONSTRAINED`（姿态随动）或 `FIXED`（姿态固定）。

### 6.6 `POST /api/move/pathway`（多路点路径）

**请求体：**

```json
{
  "waypoints": [
    { "type": "joint", "joints": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6] },
    { "type": "linear", "pose": { "position": { "x":0, "y":0, "z":0 }, "orientation": { "x":0, "y":0, "z":0, "w":1 } } }
  ],
  "asynchronous": false
}
```

### 6.7 `POST /api/axis/single/move`

**请求体：**

```json
{
  "id": 0,
  "position": 1.57,
  "max_vel": 2.0,
  "max_acc": 5.0,
  "max_jerk": 10.0,
  "least_time": 0.5,
  "raw_data": false
}
```

### 6.8 `POST /api/drag/start`

**请求体：**

```json
{
  "flag": "J1",
  "direction": "POSITIVE",
  "max_speed": 1.0,
  "max_acceleration": 2.0
}
```

`flag` 取值：`J0`-`J6`、`TOOL_X`-`TOOL_YAW`、`FLANGE_X`-`FLANGE_YAW`、`OBJECT_X`-`OBJECT_YAW`、`BASE_X`-`BASE_YAW`、`NULLSPACE`

`direction` 取值：`POSITIVE`、`NEGATIVE`、`NONE`

### 6.9 `POST /api/calibration/run`

**请求体：**

```json
{
  "frame": "tool"
}
```

`frame` 取值：`tool` 或 `object`

---

## 7. 错误码

### 7.1 HTTP 状态码

| HTTP 状态码 | 说明 |
|------------|------|
| `200` | 请求成功 |
| `400` | 请求参数错误（JSON 解析失败、字段缺失等） |
| `404` | 接口不存在 |
| `500` | 服务器内部错误（未预期的异常） |

### 7.2 业务错误码

所有业务错误码通过 JSON 响应体的 `code` 字段返回（HTTP 状态码始终为 200，业务错误通过 `success: false` + `code` 区分）。格式为 4 位数字，按功能模块分段：

#### 请求参数类（1xxx）

| code | 说明 | LLM 可恢复建议 |
|------|------|----------------|
| `1001` | JSON 格式错误或缺少必填字段 | 检查请求体格式 |
| `1002` | 关节 ID 超出范围 | id 必须在 `[0, 关节数-1]` 内 |
| `1003` | 关节数据长度与机器人关节数不匹配 | joints 数组长度必须等于机器人关节数 |
| `1004` | 位姿数据格式错误 | 检查 position/orientation 字段 |
| `1005` | 速度/加速度参数超出安全范围 | 降低参数值 |
| `1006` | 不支持的工作模式 | 检查 WorkMode 枚举值 |

#### 运动规划类（2xxx）

| code | 说明 | LLM 可恢复建议 |
|------|------|----------------|
| `2001` | 运动学逆解（IK）无解 | 目标位姿不可达，尝试调整位姿或改用关节空间运动 |
| `2002` | 超出关节运动范围限位 | 目标关节角超出物理限位，缩小目标范围 |
| `2003` | 超出工作空间边界 | 目标点超出机器人可达空间 |
| `2004` | 运动轨迹规划失败 | 尝试降低速度/加速度，或增大 time 参数 |
| `2005` | 圆弧运动三点共线 | 调整中间点 pose_via 使三点不共线 |

#### 机器人状态类（3xxx）

| code | 说明 | LLM 可恢复建议 |
|------|------|----------------|
| `3001` | 机器人未使能，拒绝执行 | 先调用 `POST /api/robot/enable` |
| `3002` | 机器人处于故障状态 | 检查硬件状态，排除故障后复位 |
| `3003` | 正在执行其他运动指令 | 等待当前运动完成或调用 `/api/move/stop` 后重试 |
| `3004` | 运动线程启动失败 | 检查硬件连接或重启控制器 |
| `3005` | 工作模式不匹配 | 当前模式不支持该操作，先切换到正确的工作模式 |

#### 标定类（4xxx）

| code | 说明 | LLM 可恢复建议 |
|------|------|----------------|
| `4001` | 标定点数据不足 | 确保已设置所有必要的标定点 |
| `4002` | 标定计算失败（数值异常） | 标定点数据无效，重新采集 |
| `4003` | 标定误差过大 | 超过容差（0.1m），重新执行标定 |
| `4004` | 无效的坐标系类型 | frame 必须为 "tool" 或 "object" |

#### 异步任务类（5xxx）

| code | 说明 | LLM 可恢复建议 |
|------|------|----------------|
| `5001` | 任务 ID 不存在 | 检查 task_id 是否正确 |
| `5002` | 异步任务执行超时 | 任务可能卡死，调用 `/api/move/stop` 后重试 |

### 7.3 错误响应示例

**IK 无解：**

```json
{
  "success": false,
  "code": 2001,
  "message": "inverse kinematics: no solution found for target pose",
  "data": null
}
```

**未使能拒绝执行：**

```json
{
  "success": false,
  "code": 3001,
  "message": "robot is disabled, enable it first",
  "data": null
}
```

### 7.4 LLM/策略规划器错误恢复流程

```
运动指令返回 success=false
    │
    ├── code=2001 (IK无解)      → 降低目标精度 / 改用 MoveJ
    ├── code=2002 (超关节限位)   → 缩小目标范围
    ├── code=2003 (超工作空间)   → 调整路径规划
    ├── code=2004 (规划失败)     → 降低速度/加速度重试
    ├── code=3001 (未使能)       → 调用 enable → 重试
    ├── code=3003 (运动中)       → 轮询 /api/move/status 等待完成
    ├── code=3002 (故障)         → 报错给上层，需人工介入
    └── code=5001 (任务不存在)   → 检查 task_id
```

---

## 8. 项目结构变更

### 8.1 新增文件

```
rocos-app/
├── include/rocos_app/
│   └── robot_http_server.h          # HTTP 服务器类声明（自包含）
├── src/
│   ├── robot_http_server.cc         # HTTP 服务器实现（路由 + 处理函数）
│   └── rocosAppMain.cc              # 修改：用 RobotHttpServer 替代 gRPC
├── 3rdparty/
│   ├── httplib/
│   │   └── httplib.h                # cpp-httplib 头文件
│   └── json/
│       └── json.hpp                 # nlohmann/json 头文件
└── docs/
    └── http_api_design.md           # 本文档
```

### 8.2 可移除文件（不再需要）

- `protos/` 目录下所有 `.proto` 文件
- `cmake/common.cmake` 中 gRPC/Protobuf 相关配置
- `CMakeLists.txt` 中 gRPC 编译目标和 proto 代码生成规则
- `src/robot_service.cc`
- `include/rocos_app/robot_service.h`

### 8.3 CMakeLists.txt 变更

移除：
```cmake
# 以下全部移除
find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)
protobuf_generate_cpp(...)
add_library(rocos_protocol ...)
target_link_libraries(rocos_protocol gRPC::grpc++)
```

新增：
```cmake
# httplib 和 json 都是头文件，只需添加 include 路径
target_include_directories(rocos_app PRIVATE
    ${CMAKE_SOURCE_DIR}/3rdparty/httplib
    ${CMAKE_SOURCE_DIR}/3rdparty/json
)

# 替换链接库
target_link_libraries(rocos_app PRIVATE rocos::robot pthread)
```

---

## 9. 启动参数

```bash
rocos-app --urdf=robot.urdf --sim=true --http_port=8080 --http_host=0.0.0.0
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--http_port` | `8080` | HTTP 服务监听端口 |
| `--http_host` | `0.0.0.0` | HTTP 服务监听地址 |

---

## 10. 安全考虑

- **CORS**：支持跨域请求（前端开发阶段）
- **认证**：后续可扩展 JWT / API Key 认证中间件
- **输入校验**：所有 API 做参数范围检查（关节角范围、ID 范围等）
- **紧急停止**：`/api/move/stop` 接口 + SIGINT 信号处理

---

## 11. 开发阶段

| 阶段 | 内容 | 优先级 | 预估 |
|------|------|--------|------|
| **P0** | 基础框架（RobotHttpServer 类、路由注册、JSON 工具函数） | 高 | - |
| **P1** | 状态查询（`/api/robot/state`、`/api/robot/info`） | 高 | - |
| **P1** | 机器人控制（enable/disable/workmode） | 高 | - |
| **P1** | 运动控制（MoveJ、MoveL、MoveC、Stop） | 高 | - |
| **P2** | 单轴/多轴控制 | 中 | - |
| **P2** | 拖拽示教 | 中 | - |
| **P2** | 标定 | 中 | - |
| **P3** | URDF 模型 + Mesh 查询 | 低 | - |
| **P3** | WebSocket 实时状态推送 | 低 | - |
