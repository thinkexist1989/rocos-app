# ROCOS-App HTTP REST API 开发日志

## 2025-06-03: gRPC → HTTP REST API 迁移

### 背景

将 ROCOS-App 的外部通信接口从 gRPC（Protobuf 序列化）替换为 HTTP/JSON RESTful API，降低集成门槛，使前端/Python/curl 等任意客户端可直接调用。

### 变更概览

| 操作 | 文件 | 说明 |
|------|------|------|
| 新建 | `include/rocos_app/robot_http_server.h` | HTTP 服务器类声明 |
| 新建 | `src/robot_http_server.cc` | 全部 30+ 路由实现（1304 行） |
| 新建 | `3rdparty/httplib/httplib.h` | cpp-httplib v0.46.0 单头文件库 |
| 新建 | `3rdparty/json/json.hpp` | nlohmann/json 单头文件库 |
| 修改 | `CMakeLists.txt` | 移除 gRPC 编译目标，集成 httplib/json |
| 修改 | `src/rocosAppMain.cc` | 启动 HTTP 服务器替代 gRPC |
| 新建 | `docs/http_api_design.md` | API 设计文档 |
| 新建 | `docs/dev_log.md` | 本文件 |

### 设计决策

1. **RobotHttpServer 为独立类**：仅依赖 `Robot*` 指针，不依赖 gRPC/Protobuf，便于整体移植
2. **业务错误码分段**：`1xxx` 参数错误、`2xxx` 运动规划、`3xxx` 状态错误、`4xxx` 标定、`5xxx` 异步任务
3. **异步任务追踪**：异步运动返回 `task_id`，通过 `GET /api/move/status` 查询执行状态
4. **CORS 支持**：所有响应附带 `Access-Control-Allow-Origin: *`
5. **保留 gRPC 源文件**：`robot_service.cc`/`.h` 保留但不再编译

### 已实现接口（30+）

**机器人状态 (4)**：state, info, model, mesh
**机器人控制 (4)**：enable, disable, enabled, workmode
**运动控制 (9)**：joint, joint_ik, linear, linear_fk, circle, path, pathway, stop, status
**单轴控制 (4)**：enable, disable, move, stop
**多轴控制 (5)**：enable, disable, move, stop, sync
**拖拽示教 (2)**：start, stop
**标定 (5)**：pose, tool, object, run, result

### 已知限制

- `MovePath` 接口返回 501：`Robot::Path`/`PathEntry` 构造函数为 private，且 `MovePath()` 在 `robot.cc` 中为 stub
- `GetRobotModel` / `GetLinkMesh` 使用 `std::filesystem`（C++17），与其他部分的 C++11 标准不一致
- HTTP 服务器为阻塞式 I/O，高并发场景性能有限（本项目为低频控制场景，足够）
- 未实现 TLS/认证（后续可扩展）
