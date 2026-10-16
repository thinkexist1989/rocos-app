<!--
 Copyright (c) 2021 'Yang Luo, luoyang@sia.cn'

 This software is released under the MIT License.
 https://opensource.org/licenses/MIT
-->

<div align="center">
  <img src="./rocos-app.png" alt="" height="150">
  <h1>ROCOS-Application</h1>
  <blockquote>The Application of Robotics for ROCOS </blockquote>
</div>
:warning: Rocos-App is under development, not finished~ :thinking:

## Introduction

ROCOS-App is the application of robotics, which is designed as the robot controller for ROCOS. 



## Dependencies

- orocos-kdl
- ruckig
- nlopt
- trac_ik
- Eigen3
- protobuf
- grpc
- tinyFSM

EtherCAT shared-memory access uses the header-only
[SharedMemoryConfig](include/rocos_app/shared_memory_config.hpp); building and
using the installed CMake package do not require `rocos_ecm`. At runtime, an
EtherCAT master must create shared memory with the same layout before the
application connects.



## Usage

REST 接口支持情况、参数限制及 curl 示例见
[REST API 使用说明](REST_API_USAGE.md)。

从另一台电脑申请控制权、验证鉴权及控制仿真的 Python/curl 示例见
[跨电脑 HTTP 控制示例](examples/HTTP_CONTROL.md)，Python 示例无需安装第三方包。

Python SDK 只需复制 [rocos.py](examples/rocos.py)，即可使用
`with Robot(ip) as robot:`、`robot.movej(...)`、`robot.get_joints()`、`robot.get()`；
控制权和心跳由 SDK 自动处理。可运行 [robot_sdk_demo.py](examples/robot_sdk_demo.py)
体验仿真往返运动。

常用函数包括 `movej()`、`movel()`、`getflange()`、`gettool()`、`getjoints()`、
`is_moving()` 和 `stop()`。demo 提供 `--demo read/movej/movel/all`，可分别运行
查询、关节往返、法兰直线往返或全部流程。

完整的方法签名、参数、返回值和异常说明见
[Python SDK API 手册](examples/PYTHON_SDK_API.md)。

[直接类调用示例](examples/robot_class_demo.py) 展示初始化 `robot = Robot(ip)` 后，
调用 `robot.getjointposition()`、`robot.movej()`、`robot.movel()` 和
`robot.getflange()`；`getjointposition(0)` 可读取单个关节。

HTTP 控制权默认 300 秒超时，持有者每 5 秒调用
`POST /api/control/heartbeat`，同时携带 token 和 client_id；GET 查询不续期。
viz 与 Python 示例已支持独立心跳。可通过 `--http_control_ttl_seconds=30`
将失联超时调整为 30 秒，实际 TTL 和建议心跳间隔由控制权响应返回。

### HTTP 模块构建

```sh
cmake -S . -B build
cmake --build build -j2
ctest --test-dir build -R '^http_(server|control_heartbeat)_test$' --output-on-failure
```

HTTP 服务使用 C++17，构建目标为 `robot_http_server`（别名
`rocos::robot_http_server`），输出 `build/lib/librobot_http_server.so`。
`httplib.h` 和 `json.hpp` 已从本机参考项目迁入 `3rdparty/httplib`、
`3rdparty/json`，构建不依赖参考项目的绝对路径。

应用通过 `target_link_libraries(your_app PRIVATE rocos::robot_http_server)`
链接，并在已有 `Robot` 实例上启动：

```cpp
#include <rocos_app/robot_http_server.hpp>

// robot 必须比 http 活得更久。
rocos::RobotHttpServer http(&robot);
http.runAsync("127.0.0.1", 8080);
// 应用继续运行……退出前调用 http.stop()。
```

### 与 ROCOS-Viz 联调

`rocosAppMain` 默认同时启动 HTTP（`0.0.0.0:8080`）和 gRPC
（`0.0.0.0:30001`）。在本项目根目录启动 Talon 仿真：

```sh
./build/bin/rocosAppMain --sim=true --urdf=config/models/talon/robot.urdf
```

在另一个终端启动 viz：

```sh
cd /home/rocos/Videos/rocos-viz-2.0-sjc
ROBOT_HOST=127.0.0.1 ROBOT_PORT=8080 npm run dev -- --port 5173 --strictPort
```

浏览器访问 `http://localhost:5173`，连接地址填 `127.0.0.1`、端口填
`8080`。viz 的 `/api` 代理会转发到控制程序，并自动读取 URDF 和网格。
当前服务仅提供 HTTP，viz 的 WebSocket 尝试失败后会自动改用 HTTP 状态轮询。
从其他电脑访问时，浏览器和连接地址均使用运行控制程序的电脑 IP。

可通过 `--http_host`、`--http_port`、`--grpc_address` 修改监听地址，
`--grpc=false` 仅运行 HTTP，`--http=false` 仅运行 gRPC。
更换 HTTP 端口时须同时设置 viz 的 `ROBOT_PORT` 并重启 Vite。
HTTP 端口占用会明确报错并退出。

模型资源相对于 `--urdf` 所指文件的目录解析；推荐使用独立模型目录下的
URDF，以避免不同模型存在同名网格。HTTP 关节状态使用
`DISABLED`/`ENABLED`/`FAULT`/`UNKNOWN` 字符串，与 viz 兼容。

此 HTTP 模块适配当前目录的 `Robot`。以下写接口需要 HTTP 控制权：

| 接口 | 功能和请求示例 |
| --- | --- |
| `POST /api/robot/workmode` | `{"mode":"position"}`；也支持 `ee_admit_teach`、`jnt_admit_teach`、`jnt_imp`、`cart_imp`，或枚举值 0–4 |
| `POST /api/robot/single_axis/enable`、`disable` | `{"id":0}`；关节编号从 0 开始 |
| `POST /api/robot/multi_axis/enable`、`disable` | 整机使能/下使能 |
| `POST /api/robot/multi_axis/sync` | `{"sync":"none"}`、`time`、`phase`，或枚举值 0–2 |
| `POST /api/robot/jog/joint` | `{"joints":[1,0,0,0,0,0,0],"speed":0.2,"timeout":0.3}` |
| `POST /api/robot/jog/cartesian` | `{"twist":[0,0,1,0,0,0],"frame":"BASE","speed":0.02,"timeout":0.3}`；支持 BASE/FLANGE/TOOL/OBJECT |
| `POST /api/robot/jog/svd` | 七轴机械臂的单维零空间点动，如 `{"dim_speeds":[0.02],"timeout":0.3}` |
| `POST /api/robot/jog/nullspace` | 七轴关节意图向量 `joints` 投影到当前单维零空间，并转换为旧核心的臂角方向 |
| `POST /api/robot/dragging`（也可用 `/api/robot/jog`） | 旧格式，如 `{"flag":0,"dir":"POSITIVE","max_speed":0.2,"max_acceleration":1,"timeout":0.3}` |
| `POST /api/robot/jog/stop` | 停止当前点动，不会误停其他类型的运动 |
| `POST /api/robot/stop` | 根据当前核心运动类型停止离线轨迹、点动或示教；响应含 `stopped_motion_type` |
| `POST /api/robot/wait_move` | `{"timeout":10}`；等待运动结束，超时返回 5002 |
| `POST /api/calibration/pose` | `id`（1–9）和 `pose`；1–6 为工具标定点，7–9 为工件标定点 |
| `POST /api/calibration/run` | `{"frame":"tool"}` 或 `{"frame":"object"}`；检查标定点齐全及几何条件后调用原有标定方法 |
| `GET /api/calibration/result` | 返回最近计算的 `error_state` 和 `pose` |

在线轴运动接口 `moveSingleAxis()`、`moveMultiAxis()` 及其停止接口已移除。
HTTP 的 `single_axis/move`、`single_axis/stop`、`multi_axis/move`、`multi_axis/stop`
返回 3007（不支持）；对应 gRPC 请求返回 `UNIMPLEMENTED`。
`GET /api/robot/state?raw_data=true` 返回原始关节数据，同时返回硬件类型、周期、
从站数、法兰/工具/工件位姿和标定反馈；`torque` 字段用于力矩。

MoveJ、MoveJ_IK、MoveL、MoveL_FK 和 MoveC 沿用旧核心的 `speed`（或
`velocity`）、`acceleration`、`asynchronous` 参数。非零 `time` 返回
`PlanInput`，`radius` 尚未实现轨迹过渡。
MoveJ/MoveJ_IK 的 `jerk` 通过关节 jerk 限制生效。同步 HTTP 调用等待期间仍可
由另一个请求停止运动。停止成功表示停止请求已提交；通过 `wait_move` 或
`move_status` 确认减速结束后再发下一条动作。被停止的轨迹不会自动恢复。

关节和笛卡尔点动仅接受一个非零方向，其余方向组合明确返回未支持。
七轴单维零空间可以接收关节意图向量；奇异构型存在多个零空间方向时会拒绝，
不会任意选择另一方向。点动停止和 HTTP 超时监测均触发底层减速；原有核心的
心跳限制仍生效，因此实际超时可能比请求的 `timeout` 更短。

工作模式切换与驱动器 CSP/CSV/CST 切换是两套接口。
本次只支持工作模式切换，未增加 CSP/CSV/CST 切换。
`jnt_imp`、`cart_imp` 可以像 gRPC 一样选择枚举，但旧核心对应控制算法仍是
占位实现，响应 `controller_implemented:false`。关节示教优先读取
`/opt/rocos/yaml/joint_impedance_control.yaml`，不存在时在 URDF 目录及其上级目录
寻找同名配置。真实末端导纳示教需要六维力传感器；仿真使用零外力。

坐标系设置仍支持默认工具/工件坐标系。Lua、UDP 伺服、命名坐标系管理、
负载管理、阻抗参数查询、故障复位、暂停/恢复及笛卡尔轨迹的显式 `jerk`
仍返回 `success=false, code=3007`。viz 根据返回的 `api_profile:legacy_robot`
避免给笛卡尔运动自动附加新版默认 jerk；用户显式提供的 jerk 保留并由后端校验。

验证命令（只运行 HTTP 和 HardwareSim 测试，不连接真实机器人）：

```sh
ctest --test-dir build -R '^(http_(server_test|robot_integration_test)|motion_result_test)$' --output-on-failure
```

## Related Projects



## Contributor

:bust_in_silhouette: **Yang Luo (luoyang@sia.cn)**

:framed_picture: Icon Designed by :**Yuhan Ying (1027649507@qq.com)**

运动失败使用 `result.hpp` 中的具体 Result 码，HTTP 提供 `error_name`、
`data.error` 和 `motion_outcome`；同步及异步读取方式见
[REST API 使用说明第 16 节](REST_API_USAGE.md#16-运动错误码与异步结果)。
