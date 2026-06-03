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

    // ---- 标定 ----
    void handleSetPoseFrame(const httplib::Request& req, httplib::Response& res);
    void handleSetToolFrame(const httplib::Request& req, httplib::Response& res);
    void handleSetObjectFrame(const httplib::Request& req, httplib::Response& res);
    void handleCalibrationRun(const httplib::Request& req, httplib::Response& res);
    void handleCalibrationResult(const httplib::Request& req, httplib::Response& res);

    // ---- 工具函数 ----
    void sendJson(httplib::Response& res, int httpCode, bool success,
                  const std::string& message, const nlohmann::json& data = nullptr);

    KDL::Frame jsonToFrame(const nlohmann::json& j);
    nlohmann::json frameToJson(const KDL::Frame& frame);

    Robot* robot_;
    std::unique_ptr<httplib::Server> server_;
    std::unique_ptr<std::thread> thread_;
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
  "code": 200,
  "message": "ok",
  "data": { ... }
}
```

失败时：

```json
{
  "success": false,
  "code": 400,
  "message": "invalid joint id: 8 (max 6)",
  "data": null
}
```

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
  "flange_pose": { "position": { "x":0, "y":0, "z":0 }, "orientation": { "x":0, "y":0, "z":0, "w":1 } },
  "tool_pose": { "position": { "x":0, "y":0, "z":0 }, "orientation": { "x":0, "y":0, "z":0, "w":1 } },
  "object_pose": { "position": { "x":0, "y":0, "z":0 }, "orientation": { "x":0, "y":0, "z":0, "w":1 } },
  "hardware": {
    "type": "ETHERCAT",
    "cycle_time_avg": 0.001,
    "cycle_time_max": 0.003,
    "slave_count": 8
  }
}
```

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

**响应 `data`：**

```json
{
  "result": 0,
  "message": "ok"
}
```

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

### 6.4 `POST /api/move/circle`

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

### 6.5 `POST /api/move/pathway`（多路点路径）

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

### 6.6 `POST /api/axis/single/move`

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

### 6.7 `POST /api/drag/start`

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

### 6.8 `POST /api/calibration/run`

**请求体：**

```json
{
  "frame": "tool"
}
```

`frame` 取值：`tool` 或 `object`

---

## 7. 错误码

| HTTP 状态码 | 说明 |
|------------|------|
| `200` | 成功 |
| `400` | 请求参数错误 |
| `404` | 接口不存在 |
| `500` | 服务器内部错误（运动规划失败等） |

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
