# ROCOS 2.0.0 REST API 支持情况与 curl 使用说明

检查日期：2026-10-13。

本文以当前项目的 `src/robot_http_server.cpp`、`include/rocos_app/robot_http_server.hpp`、`src/robot.cc` 和 `include/rocos_app/robot.h` 为依据，对照根目录下的 `ROCOS  API.openapi.yaml` 整理。接口能力以当前源码为准，YAML 中的声明不代表功能已经实现。

本文的七轴运动数值用于 Talon 仿真示例。不同模型的关节数、关节限位和可达空间不同，不能直接套用七轴目标数组。各 curl 示例可以按需单独使用，不是必须全部连续执行的脚本。

目录：

- [支持情况总览](#1-支持情况总览)
- [启动服务与 curl 基本约定](#2-启动服务与-curl-基本约定)
- [控制权](#3-控制权写接口调用前的准备)
- [查询状态、参数和模型](#4-查询状态参数和模型)
- [使能和工作模式](#5-使能和工作模式)
- [离线轨迹运动](#6-离线轨迹运动)
- [停止、等待与结果查询](#7-停止等待与结果查询)
- [在线单轴、多轴接口](#8-yaml-未收录的在线单轴多轴接口)
- [点动接口](#9-点动接口)
- [默认工具系和工件系](#10-默认工具系和工件系)
- [标定](#11-标定)
- [未支持响应示例](#12-未支持接口的响应示例)
- [YAML 与实现的差异](#13-当前-yaml-与实现的差异清单)
- [常见错误与排查](#14-常见错误与排查)
- [项目验证与使用范围](#15-项目验证与使用范围)
- [运动错误码与异步结果](#16-运动错误码与异步结果)

## 1. 支持情况总览

YAML 中共有 **62 个接口操作**，按照“HTTP 方法 + 路径”分别计数。同一路径的 GET、POST、DELETE 是不同操作。

| 类别 | 数量 | 含义 |
| --- | ---: | --- |
| 功能可用 | 20 | 已接入当前 Robot；仍需满足使能、工作模式、运动限位等条件 |
| 部分支持 | 18 | 路由可调用，但参数、控制算法或行为有明确限制 |
| 未支持 | 24 | 路由已注册，功能处理返回未支持 |

上述“功能可用”不表示响应与 YAML 的字段类型完全一致。状态字段、控制权和参数默认值的差异见第 13 节。

当前 HTTP 实现另外提供了 YAML 未收录的 11 个可用操作：单轴 4 个、多轴 5 个、旧格式点动 2 个。还有 3 个 YAML 未收录的负载参数操作，当前均未支持。

### 1.1 功能可用的 20 个操作

| 方法 | 路径 | 功能及条件 |
| --- | --- | --- |
| GET | `/api/robot/state` | 机器人完整状态；支持 `raw_data=true` 查询参数 |
| GET | `/api/robot/info` | 关节编码器、传动比、零点偏移等参数 |
| GET | `/api/robot/urdf` | 当前 Robot 使用的 URDF XML |
| GET | `/api/robot/urdf/mesh` | 模型网格文件；必须提供 `path` |
| GET | `/api/robot/enabled` | 查询整机使能、下使能状态 |
| POST | `/api/robot/enable` | 整机使能；要求没有正在执行的运动 |
| POST | `/api/robot/disable` | 整机下使能；要求没有正在执行的运动 |
| POST | `/api/robot/movej` | 关节空间运动；支持关节 jerk 限制 |
| POST | `/api/robot/movej_ik` | 目标位姿逆解后执行 MoveJ；支持关节 jerk 限制 |
| POST | `/api/robot/stop` | 根据当前运动类型停止在线轴、离线轨迹、点动或示教 |
| POST | `/api/robot/wait_move` | 等待运动结束；可设置等待超时 |
| POST | `/api/robot/jog/stop` | 停止当前点动，不用于停止其他运动类型 |
| POST | `/api/calibration/pose` | 设置标定点；`id` 为 1–9 |
| POST | `/api/calibration/run` | 工具或工件标定；需提供足够且有效的标定点 |
| GET | `/api/calibration/result` | 获取最近的标定结果 |
| POST | `/api/control/acquire` | 申请控制权 |
| POST | `/api/control/release` | 释放控制权 |
| POST | `/api/control/takeover` | 抢占控制权，原 token 立即失效 |
| GET | `/api/control/status` | 查询当前控制权状态 |
| POST | `/api/control/heartbeat` | 校验 token 和 client_id 后续期，不改变运动状态 |

### 1.2 部分支持的 18 个操作

| 方法 | 路径 | 当前实际行为与限制 |
| --- | --- | --- |
| POST | `/api/robot/workmode` | 支持位置模式及两种导纳示教；关节/笛卡尔阻抗模式仅切换枚举，控制算法仍是占位 |
| POST | `/api/robot/movel` | 可执行直线运动；不能包含 `jerk` 或 `tool_name` |
| POST | `/api/robot/movel_fk` | 目标关节经正运动学转换后执行直线运动；不能包含 `jerk` 或 `tool_name` |
| POST | `/api/robot/movec` | 仅支持当前位姿作为起点，加经过点和终点；不能包含 `pose_start`、`center_frame` 或 `jerk` |
| POST | `/api/robot/jog/joint` | 向量只能有一个超过阈值的分量；分量正负决定方向；不读取 `direction` |
| POST | `/api/robot/jog/cartesian` | 6 维向量只能有一个超过阈值的分量；支持四种参考系；不读取 `direction` |
| POST | `/api/robot/jog/nullspace` | 仅支持七轴机械臂的单维零空间投影；不读取 `direction`；奇异构型可能拒绝 |
| POST | `/api/robot/jog/svd` | 仅支持七轴；`dim_speeds` 长度必须为 1 |
| GET | `/api/robot/move_status` | 可查询当前运动；未接通按任务 ID 追踪历史任务的流程 |
| POST | `/api/robot/servo/stop` | 实际转调统一停止，不能据此认为 UDP 伺服已经实现 |
| GET | `/api/robot/tool_frames` | 只返回 `default`，没有命名工具系列表 |
| GET | `/api/robot/object_frames` | 只返回 `default`，没有命名工件系列表 |
| GET | `/api/robot/tool_frame` | 只能查询默认工具系；名称省略或为 `default` |
| POST | `/api/robot/tool_frame` | 只能设置默认工具系，不能新增任意名称；要求运动空闲 |
| GET | `/api/robot/object_frame` | 只能查询默认工件系；名称省略或为 `default` |
| POST | `/api/robot/object_frame` | 只能设置默认工件系，不能新增任意名称；要求运动空闲 |
| POST | `/api/calibration/tool` | 与设置默认工具系使用同一处理函数 |
| POST | `/api/calibration/object` | 与设置默认工件系使用同一处理函数 |

### 1.3 未支持的 24 个操作

| 方法 | 路径 | 未支持的功能 |
| --- | --- | --- |
| GET | `/api/robot/impedance` | 阻抗参数查询 |
| POST | `/api/robot/reset` | 故障复位 |
| POST | `/api/robot/servo/start` | UDP 伺服启动 |
| POST | `/api/robot/servo/mode` | UDP 伺服的关节/笛卡尔空间选择 |
| POST | `/api/robot/pause` | 暂停运动 |
| POST | `/api/robot/resume` | 恢复运动 |
| DELETE | `/api/robot/tool_frame` | 删除命名工具系 |
| DELETE | `/api/robot/object_frame` | 删除命名工件系 |
| POST | `/api/robot/active_tool_frame` | 激活命名工具系 |
| POST | `/api/robot/active_object_frame` | 激活命名工件系 |
| POST | `/api/robot/frames/load` | 从指定文件加载坐标系集合 |
| POST | `/api/robot/frames/save` | 将坐标系集合保存到指定文件 |
| POST | `/api/script/upload` | Lua 上传、编译 |
| POST | `/api/script/run` | Lua 执行 |
| POST | `/api/script/pause` | Lua 暂停 |
| POST | `/api/script/resume` | Lua 继续 |
| POST | `/api/script/stop` | Lua 停止 |
| POST | `/api/script/step` | Lua 单步 |
| GET | `/api/script/status` | Lua 状态 |
| GET | `/api/script/list` | Lua 文件列表 |
| GET | `/api/script/download` | Lua 源码下载 |
| POST | `/api/script/breakpoint/add` | 添加 Lua 断点 |
| POST | `/api/script/breakpoint/remove` | 删除 Lua 断点 |
| POST | `/api/script/breakpoint/clear` | 清空 Lua 断点 |

这些写接口仍会先检查控制权。控制权有效时，未支持响应通常为 `success:false, code:3007`；没有控制权时可能先返回 3006。3007 也用于 token 无效或过期；现在通过 `error_name` 区分未支持（`FunctionNotSupported`）与控制权错误（`ControlTokenInvalidOrExpired`）。

## 2. 启动服务与 curl 基本约定

在项目根目录启动七轴 Talon 仿真，仅启用 HTTP：

```bash
./build/bin/rocosAppMain --sim=true --grpc=false \
  --urdf=config/models/talon/robot.urdf \
  --http_host=127.0.0.1 --http_port=8080
```

默认程序也可同时运行 HTTP 8080 与 gRPC 30001。HTTP 服务必须已经启动，以下 curl 请求才有响应。若控制程序已经在运行，使用其实际监听地址和端口即可。

在另一个终端设置地址：

```bash
BASE='http://127.0.0.1:8080'
```

最简单的状态查询：

```bash
curl -sS "$BASE/api/robot/state"
```

可选：使用 Python 3 格式化 JSON，便于查看。

```bash
curl -sS "$BASE/api/robot/state" | python3 -m json.tool
```

本文使用 Bash，`curl -d` 默认发 POST；写明 `-X POST` 是为了让请求方法更直观。带 JSON 请求体时应设置 `Content-Type: application/json`。

除模型 XML 和网格二进制之外，响应通常采用以下结构：

```json
{
  "success": true,
  "code": 0,
  "message": "ok",
  "data": {}
}
```

业务失败仍可能是 HTTP 200。因此，curl 执行成功、HTTP 200、`curl --fail` 未报错，都不能替代检查 JSON 的 `success`。

## 3. 控制权：写接口调用前的准备

### 3.1 申请控制权

状态查询不需要控制权。机器人控制、运动、点动、坐标系设置和标定等写接口需要控制权。

```bash
curl -sS -X POST "$BASE/api/control/acquire" \
  -H 'Content-Type: application/json' \
  -d '{"client_name":"curl-demo","client_id":"curl-demo-001"}'
```

成功响应的 `data` 中包含 `token` 和 `client_id`。在当前终端设置：

```bash
TOKEN='将返回的data.token粘贴到这里'
CLIENT_ID='curl-demo-001'
```

如果没有指定 `client_id`，服务端会生成一个，此时必须使用响应里的 `data.client_id`。

也可以用 Python 3 自动提取，替代上面的手动申请和粘贴步骤：

```bash
CONTROL_REPLY=$(curl -sS -X POST "$BASE/api/control/acquire" \
  -H 'Content-Type: application/json' \
  -d '{"client_name":"curl-demo","client_id":"curl-demo-001"}')
TOKEN=$(printf '%s' "$CONTROL_REPLY" | python3 -c \
  'import json,sys; r=json.load(sys.stdin); assert r["success"],r; print(r["data"]["token"])')
CLIENT_ID=$(printf '%s' "$CONTROL_REPLY" | python3 -c \
  'import json,sys; r=json.load(sys.stdin); assert r["success"],r; print(r["data"]["client_id"])')
```

每个终端的变量独立；在新的终端发送写请求时，需要设置相同的 `BASE`、`TOKEN` 和 `CLIENT_ID`。

### 3.2 有效期与查询

默认有效期为 **300 秒**，启动参数 `--http_control_ttl_seconds=30` 可调整为 30 秒。以响应中的 `ttl_seconds` 为准。持有者应按响应 `heartbeat_interval_seconds` 定时发送心跳（默认每 5 秒）：

```bash
curl -sS -X POST "$BASE/api/control/heartbeat" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{}'
```

心跳和通过控制权检查的写请求会续期；普通 GET 状态查询以及 `/api/control/status` 不会续期。无效或过期 token、错误 client_id 的心跳返回 3007，不会续期或恢复旧控制权。仅保持 TCP 连接不能保持控制权。

viz 和 Python 控制示例会独立发送心跳，因此不操作机器人也可持续持有。心跳中断达到超时后，其他客户端才能普通申请；主动 takeover 则立即替换持有者。控制权超时释放本身不会停止或改变机器人的运动。

```bash
curl -sS "$BASE/api/control/status"
```

已有持有者时，重复申请返回 3008，不会自动返回原 token。

### 3.3 释放与抢占

释放需要同时携带两个请求头：

```bash
curl -sS -X POST "$BASE/api/control/release" \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID"
```

需要主动替换当前持有者时才使用抢占：

```bash
curl -sS -X POST "$BASE/api/control/takeover" \
  -H 'Content-Type: application/json' \
  -d '{"client_name":"curl-demo","client_id":"curl-demo-001"}'
```

抢占成功后要更新 `TOKEN`；之前的 token 立即失效。本文使用正常控制权流程，不依赖调试旁路。

## 4. 查询状态、参数和模型

```bash
# 用户单位的关节状态，通常为 rad、rad/s
curl -sS "$BASE/api/robot/state"

# 原始编码器数据
curl -sS "$BASE/api/robot/state?raw_data=true"

# 传动比、编码器单位换算、零点偏移等
curl -sS "$BASE/api/robot/info"

# 是否整机使能
curl -sS "$BASE/api/robot/enabled"

# 当前运动状态；不要依赖 task_id
curl -sS "$BASE/api/robot/move_status"

# 保存 URDF XML，返回内容不是 StandardResponse JSON
curl -sS "$BASE/api/robot/urdf" -o /tmp/rocos-current-model.urdf
```

先查看 URDF 中实际的 mesh 路径，再下载网格：

```bash
MESH_PATH='将URDF中的mesh路径粘贴到这里'
curl -sS -G "$BASE/api/robot/urdf/mesh" \
  --data-urlencode "path=$MESH_PATH" -o /tmp/rocos-link.stl
```

网格下载成功时是二进制文件，失败时可能返回 JSON。因此下载后仍需确认文件内容，不能仅根据 `-o` 生成了文件判断成功。

状态中常用字段：

| 字段 | 当前含义 |
| --- | --- |
| `api_profile` | 当前为 `legacy_robot` |
| `joint_states[].id` | 从 0 开始的关节编号 |
| `joint_states[].name` | URDF 关节名，如 `joint_1`；viz 用它匹配三维模型 |
| `joint_states[].status` | `ENABLED`、`DISABLED`、`FAULT`、`UNKNOWN` 字符串 |
| `joint_states[].mode` | CSP/CSV/CST 状态信息；不表示 HTTP 支持切换 |
| `timestamp` | epoch 毫秒 |
| `work_mode` | 当前工作模式 |
| `motion_busy` | 当前是否有运动或示教控制运行 |
| `motion_type` | `none`、`axis`、`trajectory`、`dragging`、`teaching` |
| `stop_requested` | 是否已提交停止请求 |
| `result` | 最近一次 ROCOS 运动结果码；0 表示成功，具体负数含义见第 16 节 |
| `result_name` | 结果枚举名称，如 `NoError`、`IkCalcFail` |
| `motion_outcome` | `idle`、`running`、`completed`、`stopped`、`failed` |
| `error` | 运动错误详情；无错误时为 null |
| `control_active` | 当前实现与 `motion_busy` 相同，不表示客户端持有控制权 |
| `flange` / `tool` / `object` | 法兰、工具、工件位姿信息 |
| `hw_state` | 硬件类型、周期、从站数等信息 |

## 5. 使能和工作模式

### 5.1 整机使能、下使能

```bash
curl -sS -X POST "$BASE/api/robot/enable" \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID"
```

下使能要求当前没有运动。正在运动时，先按第 7 节停止并等待结束。

```bash
curl -sS -X POST "$BASE/api/robot/disable" \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID"
```

### 5.2 工作模式切换

```bash
curl -sS -X POST "$BASE/api/robot/workmode" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"mode":"position"}'
```

| 字符串 | 整数枚举 | 支持情况 |
| --- | ---: | --- |
| `position` | 0 | 支持位置运动 |
| `ee_admit_teach` | 1 | 支持末端导纳示教；真实硬件依赖六维力传感器，仿真使用零外力 |
| `jnt_admit_teach` | 2 | 支持关节导纳示教；依赖关节阻抗配置 |
| `jnt_imp` | 3 | 可选择枚举，控制算法仍是占位，`controller_implemented:false` |
| `cart_imp` | 4 | 可选择枚举，控制算法仍是占位，`controller_implemented:false` |

把请求体中的 `mode` 替换为上表值即可。普通运动过程中不允许切换模式；从示教模式切回 `position` 会先停止示教控制。

关节示教配置优先使用 `/opt/rocos/yaml/joint_impedance_control.yaml`，否则从 URDF 所在目录及其上级目录查找同名配置。切换失败时检查控制程序日志。

工作模式与驱动器 CSP/CSV/CST 是不同概念。当前没有增加 HTTP 驱动器模式切换接口。YAML 中 `/api/robot/servo/mode` 描述的是 UDP 伺服的 joint/cartesian 空间选择，该功能也未支持。

## 6. 离线轨迹运动

### 6.1 公共参数与执行方式

| 参数 | 当前说明 |
| --- | --- |
| `velocity` | 优先读取的速度参数；未提供时兼容 `speed`；默认 1.0 |
| `acceleration` | 加速度参数；默认 2.0 |
| `jerk` | 仅 MoveJ、MoveJ_IK 支持；必须大于 0 且不超过各关节物理限制 |
| `asynchronous` | 默认 `false`；为 `true` 时在启动运动后返回，仍需查询执行结果 |
| `time` | 当前统一拒绝非零值，返回 `PlanInput=-2316`；示例省略或设为 0 |
| `radius` | HTTP 接收并传给旧核心，但当前执行路径没有实现轨迹间圆滑过渡；省略或设为 0，不依赖它实现连续轨迹 |

位置使用米，关节角度使用弧度，时间使用秒。姿态使用四元数，推荐使用完整的嵌套格式并提供单位四元数：

```json
{
  "position": {"x": 0.1, "y": 0.2, "z": 0.3},
  "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
}
```

当前 MoveJ_IK/MoveL 直接使用旧核心运动学链末端位姿，示例从 `state.flange` 取目标；不要假设设置工具偏移后接口会自动把输入 TCP 目标换算成法兰目标。

以下示例均显式使用 `asynchronous:true`。运动响应成功表示启动被接受，不等于已经到达目标。通过 `wait_move` 或 `move_status` 确认结束，再发下一条动作。当前没有运动队列，也没有运动请求生成并返回 `task_id` 的流程。

### 6.2 MoveJ：七轴关节运动

```bash
curl -sS -X POST "$BASE/api/robot/movej" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"joints":[0.12,-0.15,0.08,-0.25,0.05,0.15,0],"velocity":0.2,"acceleration":0.6,"jerk":3,"asynchronous":true}'
```

`joints` 长度必须等于当前关节数。显式 `jerk` 会更新当前关节运动 jerk 限制，后续未显式传 jerk 的运动可能继续沿用该限制；它不是只在本次请求内生效的局部参数。

### 6.3 MoveJ_IK：位姿逆解后做关节运动

先生成当前法兰位姿的请求文件；这是一个用于验证格式和逆解接口的当前位姿目标，不要求产生明显移动：

```bash
curl -sS "$BASE/api/robot/state" | python3 -c \
  'import json,sys; r=json.load(sys.stdin); assert r["success"],r; print(json.dumps({"pose":r["data"]["flange"],"velocity":0.2,"acceleration":0.6,"jerk":3,"asynchronous":True}))' \
  > /tmp/rocos-movej-ik.json

curl -sS -X POST "$BASE/api/robot/movej_ik" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  --data-binary @/tmp/rocos-movej-ik.json
```

要实际改变位姿，应编辑文件中的 `pose` 为已确认可达的目标。逆运动学失败会返回业务失败。

### 6.4 MoveL：法兰直线运动

下面在当前法兰目标的 X 方向增加 2 mm。目标是否可达和轨迹是否可规划，仍取决于当前姿态及关节限制；优先在非奇异的 Talon 仿真构型下尝试。

```bash
curl -sS "$BASE/api/robot/state" | python3 -c \
  'import json,sys; r=json.load(sys.stdin); assert r["success"],r; p=r["data"]["flange"]; p["position"]["x"]+=0.002; print(json.dumps({"pose":p,"velocity":0.01,"acceleration":0.1,"asynchronous":True}))' \
  > /tmp/rocos-movel.json

curl -sS -X POST "$BASE/api/robot/movel" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  --data-binary @/tmp/rocos-movel.json
```

不能包含 `jerk`，即使值为 0 或 YAML 默认的 10 也会返回未支持。也不能包含 `tool_name`，即使名称为 `default`。

### 6.5 MoveL_FK：目标关节正解后走直线

```bash
curl -sS -X POST "$BASE/api/robot/movel_fk" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"joints":[0.12,-0.15,0.08,-0.25,0.05,0.15,0],"velocity":0.01,"acceleration":0.1,"asynchronous":true}'
```

这里的 `joints` 用于计算目标末端位姿，执行的是直线轨迹，不等同于 MoveJ。直线轨迹的冗余逆解也不保证最终关节角逐项等于输入数组。不能包含 `jerk` 或 `tool_name`。

### 6.6 MoveC：经过点和终点形式

接口只接受 `pose_via` 与 `pose_goal`，终点也可以写为 `pose_to`。起点固定为当前法兰位姿。

下面从当前法兰位姿生成一个小圆弧的经过点和终点，保持姿态不变。三点不共线，但仍需当前构型允许该轨迹：

```bash
curl -sS "$BASE/api/robot/state" | python3 -c \
  'import copy,json,sys; r=json.load(sys.stdin); assert r["success"],r; p=r["data"]["flange"]; v=copy.deepcopy(p); g=copy.deepcopy(p); v["position"]["x"]+=0.002; v["position"]["y"]+=0.002; g["position"]["x"]+=0.004; print(json.dumps({"pose_via":v,"pose_goal":g,"velocity":0.01,"acceleration":0.1,"asynchronous":True}))' \
  > /tmp/rocos-movec.json

curl -sS -X POST "$BASE/api/robot/movec" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  --data-binary @/tmp/rocos-movec.json
```

不支持显式 `pose_start`；不支持 `center_frame` + `theta` 形式；不支持显式 `jerk`。HTTP 当前固定传入旧核心的 `UNCONSTRAINED` 姿态模式，没有提供姿态模式选择参数。

## 7. 停止、等待与结果查询

统一停止：

```bash
curl -sS -X POST "$BASE/api/robot/stop" \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID"
```

响应 `data.stopped_motion_type` 表示提交停止前的运动类型。成功响应表示已提交停止请求，减速过程可能尚未结束。

等待最多 10 秒：

```bash
curl -sS --max-time 12 -X POST "$BASE/api/robot/wait_move" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"timeout":10}'
```

`timeout` 默认 30 秒，必须大于 0。客户端 `--max-time` 应大于服务端等待时间，否则 curl 可能先超时。成功后检查 `data.motion_busy:false` 和 `data.result`。

不阻塞地查询：

```bash
curl -sS "$BASE/api/robot/move_status"
```

`GET /api/robot/move_status?task_id=...` 虽然接收该查询参数，但当前运动没有登记任务，通常返回 `task:null`。不要将整个 JSON 的 `success:true` 理解为该任务存在。

停止后的轨迹不会自动恢复。`pause`、`resume` 未实现。`POST /api/robot/servo/stop` 是统一停止的别名，优先使用语义明确的 `/api/robot/stop`。

## 8. YAML 未收录的在线单轴、多轴接口

这组接口已接入当前 Ruckig 在线执行路径，可以使用。关节编号从 0 开始。

| 方法 | 路径 | 请求关键字段 |
| --- | --- | --- |
| POST | `/api/robot/single_axis/enable` | `id` |
| POST | `/api/robot/single_axis/disable` | `id` |
| POST | `/api/robot/single_axis/move` | `id`、`pos`；可选 `max_vel`、`max_acc`、`max_jerk`、`least_time`、`raw_data` |
| POST | `/api/robot/single_axis/stop` | `id` |
| POST | `/api/robot/multi_axis/enable` | 无必填请求体 |
| POST | `/api/robot/multi_axis/disable` | 无必填请求体 |
| POST | `/api/robot/multi_axis/move` | `target_pos` 数组；可选限制数组、`sync`、`least_time`、`raw_data` |
| POST | `/api/robot/multi_axis/stop` | 无必填请求体 |
| POST | `/api/robot/multi_axis/sync` | `sync`：`none`、`time`、`phase` 或 0、1、2 |

### 8.1 单轴使能与移动

```bash
curl -sS -X POST "$BASE/api/robot/single_axis/enable" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"id":0}'

curl -sS -X POST "$BASE/api/robot/single_axis/move" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"id":0,"pos":0.08,"max_vel":0.2,"max_acc":1,"max_jerk":5,"least_time":0}'
```

停止该在线轴：

```bash
curl -sS -X POST "$BASE/api/robot/single_axis/stop" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"id":0}'
```

把 `enable` 换成 `disable` 可下使能该轴，要求运动已结束。`single_axis/stop` 仅用于在线轴；离线协调轨迹、点动或示教应使用统一停止。

### 8.2 多轴同步与移动

```bash
curl -sS -X POST "$BASE/api/robot/multi_axis/sync" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"sync":"time"}'

curl -sS -X POST "$BASE/api/robot/multi_axis/move" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"target_pos":[0.12,-0.15,0.08,-0.25,0.05,0.15,0],"max_vel":[0.3,0.3,0.3,0.3,0.3,0.3,0.3],"max_acc":[1,1,1,1,1,1,1],"max_jerk":[5,5,5,5,5,5,5],"sync":"time","least_time":0}'
```

`target_pos` 也可写为 `joints`。所有显式数组长度必须等于当前关节数。`none` 为不同步，`time` 为时间同步，`phase` 为相位同步。

多轴停止、使能、下使能可按以下形式替换操作名称：

```bash
curl -sS -X POST "$BASE/api/robot/multi_axis/stop" \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID"
```

`multi_axis/enable`、`multi_axis/disable`、`multi_axis/sync`、`multi_axis/move` 要求没有正在执行的运动。单轴在线移动可以在已有在线轴运动期间更新目标，但不能与离线轨迹或点动混用。

### 8.3 原始编码器单位

单轴、多轴移动可设置 `raw_data:true`。位置和显式传入的速度、加速度、jerk 按该轴 `cnt_per_unit` 换算；未显式传入的限制仍使用控制器当前用户单位限制。

先通过 `/api/robot/info` 获取 `cnt_per_unit`，不能把 rad 数值直接当编码器数值发送。`least_time` 单位仍为秒，不是编码器单位。默认使用 `raw_data:false` 更便于调试。

## 9. 点动接口

### 9.1 当前共同限制

- 必须处于已使能的 `position` 工作模式。
- 关节和笛卡尔点动只能选择一个方向，不支持多方向合成运动。
- `timeout` 默认 0.3 秒，要求 `0 < timeout <= 5`。
- `dir_threshold` 默认 `1e-6`，必须非负；不要自动发送 YAML 中的 `0.99` 默认值。
- `speed` 默认 0.25，`acceleration` 默认 1.0；可显式设小值。
- 旧核心会按点动类型对速度做缩放，尤其笛卡尔平移和旋转使用不同系数；`speed` 不是严格的最终末端速度承诺。
- 原有底层心跳仍生效，所以实际停止时间可能短于请求的 `timeout`。
- 连续点动要定期重复发送同一个方向，例如每 50 ms；停止后或切换轴/参考系前先等待当前点动结束。
- 向量接口不读取 `direction`；正负方向写在向量分量中。

### 9.2 关节正向、反向与停止

关节 0 正向点动：

```bash
curl -sS -X POST "$BASE/api/robot/jog/joint" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"joints":[1,0,0,0,0,0,0],"speed":0.1,"acceleration":0.5,"timeout":0.3}'
```

反向把第一项改为 `-1`，不使用 `direction:"NEGATIVE"`。向量幅度主要用于方向判断，实际速度由 `speed` 指定。

停止：

```bash
curl -sS -X POST "$BASE/api/robot/jog/stop" \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID"
```

一次重复发送的仿真示例：

```bash
for i in {1..6}; do
  curl -sS -X POST "$BASE/api/robot/jog/joint" \
    -H 'Content-Type: application/json' \
    -H "X-Rocos-Control-Token: $TOKEN" \
    -H "X-Rocos-Client-Id: $CLIENT_ID" \
    -d '{"joints":[1,0,0,0,0,0,0],"speed":0.1,"timeout":0.3}'
  sleep 0.05
done
curl -sS -X POST "$BASE/api/robot/jog/stop" \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID"
```

### 9.3 笛卡尔点动

`twist` 顺序为 `[vx, vy, vz, wx, wy, wz]`。下例在 BASE 参考系选择 Z 正向：

```bash
curl -sS -X POST "$BASE/api/robot/jog/cartesian" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"twist":[0,0,1,0,0,0],"frame":"BASE","speed":0.02,"timeout":0.3}'
```

`frame` 可替换为 `FLANGE`、`TOOL` 或 `OBJECT`，默认为 BASE。同时给 X、Z 非零值会返回未支持；全零向量会请求停止当前点动。

### 9.4 七轴零空间点动

关节意图投影形式：

```bash
curl -sS -X POST "$BASE/api/robot/jog/nullspace" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"joints":[1,-1,0,0,0,0,0],"speed":0.03,"timeout":0.3}'
```

这里允许多个意图分量，然后投影到当前单维零空间。机械臂必须有 7 个关节；Jacobian 不满足单维零空间映射条件时返回 2001。

单维速度形式：

```bash
curl -sS -X POST "$BASE/api/robot/jog/svd" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"dim_speeds":[0.02],"timeout":0.3}'
```

`dim_speeds` 只能包含一个元素。其绝对值作为旧核心臂角速度输入，正负决定方向；不代表任意维数的通用 SVD 伺服。

### 9.5 YAML 未收录的旧格式点动

`/api/robot/dragging` 和 `/api/robot/jog` 使用相同处理函数。这组接口读取 `dir` 或 `direction`，与上面的向量接口不同。

```bash
curl -sS -X POST "$BASE/api/robot/dragging" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"flag":0,"dir":"POSITIVE","max_speed":0.1,"max_acceleration":0.5,"timeout":0.3}'
```

| flag | 含义 |
| --- | --- |
| 0–6 | 七轴的关节 0–6 |
| 100–105 | TOOL 的 X/Y/Z/ROLL/PITCH/YAW |
| 200–205 | FLANGE 的 X/Y/Z/ROLL/PITCH/YAW |
| 300–305 | OBJECT 的 X/Y/Z/ROLL/PITCH/YAW |
| 400–405 | BASE 的 X/Y/Z/ROLL/PITCH/YAW |
| 500 | 七轴单维零空间 |

`dir` 支持 `POSITIVE`、`NEGATIVE`、`NONE`，也可使用整数 1、-1、0。`NONE` 表示停止当前点动。

## 10. 默认工具系和工件系

当前只有默认坐标系，以下查询可直接使用：

```bash
curl -sS "$BASE/api/robot/tool_frames"
curl -sS "$BASE/api/robot/object_frames"
curl -sS "$BASE/api/robot/tool_frame?name=default"
curl -sS "$BASE/api/robot/object_frame?name=default"
```

工具系示例为法兰 Z 方向偏移 0.1 m，姿态不变：

```bash
curl -sS -X POST "$BASE/api/robot/tool_frame" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"name":"default","frame":{"position":{"x":0,"y":0,"z":0.1},"orientation":{"x":0,"y":0,"z":0,"w":1}}}'
```

工件系示例：

```bash
curl -sS -X POST "$BASE/api/robot/object_frame" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"name":"default","frame":{"position":{"x":0.1,"y":0.2,"z":0},"orientation":{"x":0,"y":0,"z":0,"w":1}}}'
```

`name` 可以省略，但若提供必须是 `default`。`frame` 可用 `pose` 作为别名。把路径替换为 `/api/calibration/tool`、`/api/calibration/object`，请求体保持相同，即可使用对应别名接口。

设置接口调用旧核心更新当前坐标变换，并尝试写入旧核心的配置文件；HTTP 返回成功不能保证磁盘写入成功，旧核心未检查文件流错误。它不等于支持 `/api/robot/frames/save` 的命名坐标系集合持久化。

## 11. 标定

### 11.1 点位编号

| id | 用途 |
| --- | --- |
| 1–6 | 工具标定点 |
| 7–9 | 工件标定点 |

设置点位本身不驱动机器人移动，只记录提交的位姿。真实标定应提交实际测量点；下例为仿真中的数据格式示例。

### 11.2 工件标定完整 curl 示例

```bash
curl -sS -X POST "$BASE/api/calibration/pose" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"id":7,"pose":{"position":{"x":0.1,"y":0.2,"z":0},"orientation":{"x":0,"y":0,"z":0,"w":1}}}'

curl -sS -X POST "$BASE/api/calibration/pose" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"id":8,"pose":{"position":{"x":0.2,"y":0.2,"z":0},"orientation":{"x":0,"y":0,"z":0,"w":1}}}'

curl -sS -X POST "$BASE/api/calibration/pose" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"id":9,"pose":{"position":{"x":0.1,"y":0.3,"z":0},"orientation":{"x":0,"y":0,"z":0,"w":1}}}'

curl -sS -X POST "$BASE/api/calibration/run" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"frame":"object"}'

curl -sS "$BASE/api/calibration/result"
```

以上三个位置点不重合、不共线。检查计算响应中的 `success`、`data.error_state` 和 `data.pose`。

### 11.3 工具标定

同样通过 `/api/calibration/pose` 提交 1–6 号点，再执行：

```bash
curl -sS -X POST "$BASE/api/calibration/run" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"frame":"tool"}'
```

工具标定需要 1–4 号点提供足够的姿态变化来约束工具偏移；4–6 号点还用于确定方向。全部使用相同四元数，即使位置不同，也可能因旋转约束不满秩而失败。点位不齐返回 4001，几何条件不满足或计算失败返回 4002。

标定计算与设置当前工具/工件系是不同接口。不要把 `run` 返回的计算成功视为已经成功保存所有配置；需要应用结果时，通过默认坐标系设置接口明确提交 `data.pose`。`result` 返回最近一次结果，不区分多个历史标定任务。

## 12. 未支持接口的响应示例

只读阻抗查询不需要控制权，可用于查看未支持响应格式：

```bash
curl -sS "$BASE/api/robot/impedance"
```

典型响应：

```json
{
  "success": false,
  "code": 3007,
  "message": "Impedance parameter query is not supported by this Robot version",
  "error_name": "FunctionNotSupported",
  "data": null
}
```

暂停接口即使持有有效控制权，也不能暂停轨迹：

```bash
curl -sS -X POST "$BASE/api/robot/pause" \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID"
```

Lua 查询也未支持：

```bash
curl -sS "$BASE/api/script/status"
```

YAML 未收录的负载接口同样未支持：

| 方法 | 路径 |
| --- | --- |
| GET | `/api/robot/load_params` |
| POST | `/api/robot/load_params` |
| POST | `/api/robot/load_params/save` |

## 13. 当前 YAML 与实现的差异清单

| 项目 | YAML 声明 | 当前实现 / 客户端处理方式 |
| --- | --- | --- |
| 功能范围 | 列出全部 62 个操作 | 24 个未支持，18 个部分支持；按第 1 节选择接口 |
| 控制权有效期 | 默认 300 秒，可配置 | 参考响应中的 `ttl_seconds` |
| 控制权绑定 | token 和 client_id | TCP 源端口仅供审计，换连接不影响有效凭据 |
| 写接口请求头 | ControlToken 与 ClientId 同时提供 | 对应两个 `X-Rocos-*` 请求头 |
| 续期 | 心跳和通过鉴权的写请求续期 | 普通 GET 不续期；心跳不改变运动状态 |
| 关节 status | integer | 实际为字符串 ENABLED/DISABLED/FAULT/UNKNOWN |
| robot_state | 声明完整新版 FSM 状态集合 | 当前主要返回 DISABLED/STOPPED/RUNNING/FAULT/UNKNOWN，不提供完整新版状态机 |
| timestamp | epoch seconds | 实际 epoch 毫秒；需要秒时除以 1000 |
| MoveL/MoveL_FK/MoveC jerk | 默认 10 | 不能发送该字段；显式存在就返回未支持 |
| MoveL/MoveL_FK tool_name | 可指定工具名称 | 不能发送该字段，包括 default |
| MoveC 输入形式 | 起点、圆心形式、经过点形式 | 仅当前起点 + 经过点 + 终点；无显式起点和圆心形式 |
| 向量点动 direction | 文档描述整体乘正负号 | 当前未读取；直接改变向量符号 |
| 点动方向合成 | 向量描述未体现旧核心限制 | 关节/笛卡尔只能一个方向；零空间使用专门投影 |
| 点动默认 speed | 关节/笛卡尔/零空间默认 1 | 当前默认 0.25；SVD 非零速度取 dim_speeds 的绝对值 |
| 点动默认 timeout | 0.1 秒 | 当前默认 0.3 秒；范围为 (0, 5]，底层心跳可能更早停止 |
| 点动默认 dir_threshold | 0.99 | 当前 1e-6；默认值不要由客户端自动填充为 0.99 |
| task_id | move_status 可以带 task_id | 运动接口未登记并返回任务 ID；使用当前运动状态查询 |
| 标定 pose 请求体 | 未定义 | 实际必须传整数 id 和 pose |
| 标定 run 请求体 | 未定义 | 实际必须传 frame: tool/object |
| 命名坐标系 | 支持名称列表、增删、激活和持久化 | 当前只有 default 的查询和设置 |
| time/radius | YAML 未完整描述旧版扩展参数 | time 非零返回 PlanInput；radius 未实现轨迹过渡，不依赖非零值 |
| 单轴/多轴/旧格式点动 | 未收录 | 当前可用，见第 8、9.5 节 |
| 负载管理 | 未收录 | 当前 3 个操作均未支持 |

开发基于 YAML 的自动生成客户端时，需要按上表处理差异；仅导入原始 YAML 无法得到完全符合当前实现的客户端。

## 14. 常见错误与排查

| 现象或 code | 常见原因 | 处理方式 |
| --- | --- | --- |
| curl 连接失败 | HTTP 未启动、端口不一致或地址错误 | 检查控制程序启动日志、`--http_port` 和 BASE |
| 3006 | 没有控制权 token | 先 acquire，再携带两个请求头 |
| 3007，error_name 为 ControlTokenInvalidOrExpired | token 无效、过期或 client_id 不匹配 | 查询控制权，重新申请或确认两个请求头 |
| 3007，error_name 为 FunctionNotSupported | 未支持的接口或参数 | 对照第 1、13 节，不重复重试相同参数 |
| 3008 | 控制权已被其他客户端持有 | 原持有者 release，或按需要显式 takeover |
| 3003 | 当前有另一种运动或正在停止 | 统一 stop，wait_move 完成后再发新运动 |
| 3005 | 工作模式或驱动器状态不符合要求 | 切回 position，核对关节使能和 mode |
| 3001 | 在线轴或点动所需关节未使能 | 检查 enabled 和 joint_states 状态 |
| 1001 / 1002 / 1003 | JSON、数值、编号或数组长度错误 | 核对 JSON 类型、限位、关节数和 id |
| 2001 | 零空间映射失败 | 当前构型不满足单维映射条件 |
| 4001 / 4002 | 标定点不齐、共线、姿态约束不足或计算失败 | 核对点位编号、位置与姿态变化 |
| 5002 | wait_move 等待超时 | 查询 motion_busy；等待超时本身不会停止运动 |
| 负数 Result 码 | 逆解、限位、规划或执行失败 | 通过 error_name 和 data.error 读取具体原因，见第 16 节 |
| 点动请求成功但没有明显移动 | 单次请求很快触发心跳停止、方向被阈值过滤或速度较低 | 检查向量、dir_threshold，并按固定周期续发 |
| 角度变了但 viz 模型不动 | 老进程仍返回硬件名称或模型与关节名不匹配 | 更新并重启控制程序，确认 name 对应 URDF 关节名，刷新 viz |

`wait_move` 成功只表示当前运动已结束，不保证每个关节已经到达最初目标：运动可能被 stop 提前结束。判断到位还要比较 `joint_states[].position` 与目标。

## 15. 项目验证与使用范围

当前项目已有 HTTP 冒烟测试和隔离 HardwareSim 集成测试，覆盖控制权、未支持响应、在线轴实际运动、停止、点动心跳、工作模式、标定以及状态/参数中的 URDF 关节名。

```bash
cmake -S . -B build
cmake --build build -j2
ctest --test-dir build -R '^(http_(server_test|robot_integration_test)|motion_result_test)$' --output-on-failure
```

集成测试自行启动临时端口的 HardwareSim，不连接真实硬件。本文完整接口清单来自源码对照；并非每种模型、每种姿态下的所有运动示例都经过执行验证。

整理本文时另用临时端口的 HardwareSim 实际验证了文档中的控制权申请/释放、整机使能、位置模式切换、MoveJ、单轴移动、等待、连续关节点动与停止、工件标定和未支持响应示例。其余 Shell 代码块做了语法检查，字面 JSON 请求体做了解析检查。

当前服务提供 HTTP REST，未实现 WebSocket 状态推送；viz 使用 HTTP 轮询获取状态。对接客户端应以 `api_profile:legacy_robot` 和本文描述的实际字段为准。


## 16. 运动错误码与异步结果

运动检查、MoveJ/MoveJ_IK、MoveL/MoveL_FK/MoveC、Ruckig 在线轴及轨迹执行现在接入 `include/rocos_app/result.hpp`。错误在发生的分支记录，不依赖 HTTP 从一个 `-1` 猜测原因。现有 HTTP 参数、控制权、标定和等待超时的正数码保留。

### 16.1 已接通的核心错误码

| Result 枚举 | code | 触发原因 |
| --- | ---: | --- |
| `NoError` / `PlanFinished` | 0 | 成功；PlanFinished 的内部值 100 在 REST 中归一化为 0 |
| `JointStateError` | -2017 | 当前关节模式不支持该运动，或执行中模式改变 |
| `NotEnabled` | -2212 | 核心检查发现未使能，或执行过程中丢失使能 |
| `ConflictTaskRunning` | -2215 | 原生运动调用发现另一个任务正在运行 |
| `Fatal` | -2307 | 异步运动线程或在线规划发生未分类异常 |
| `PosLimit` | -2308 | 目标或轨迹关节位置超过限制 |
| `SpeedLimit` | -2310 | 速度限制或轨迹采样速度超限 |
| `AccLimit` | -2311 | 加速度超过关节限制 |
| `JerkLimit` | -2312 | 当前关节 jerk 限制非法或超过物理限制 |
| `IkTrack` | -2314 | 轨迹中间点的位置逆运动学失败；保留最后一次重试原因 |
| `PlanInput` | -2316 | 不支持的非零 time、无效重试次数，或 Ruckig 输入无效 |
| `PlanMoveJ` | -2317 | MoveJ 的 DoubleS 轨迹不可行或时长非法 |
| `PlanMoveL` | -2318 | 直线轨迹生成失败 |
| `PlanMoveC` | -2319 | 圆弧轨迹生成失败，例如点位重合/共线 |
| `MoveUnknown` | -2325 | 本次范围之外的旧接口仍返回未分类码时，HTTP 的明确兜底；原值放 native_code |
| `UnmatchedJointsNumber` | -2332 | 原生调用的关节数组长度错误 |
| `IkCalcFail` | -2349 | 目标位姿逆解失败 |
| `FkCalcFail` | -2350 | 目标或当前位姿正运动学失败 |
| `JacobianCalcFail` | -2353 | 速度模式的 Jacobian 逆解失败 |
| `ParameterNanOrInf` | -2902 | 原生运动输入含 NaN/Inf |
| `IllegalParameter` | -3006 | 原生运动输入不合法，例如速度非正 |
| `ResourceUnavailable` | -3001 | 运动线程启动失败 |
| `PlannerError` | -2305 | Ruckig 其他规划/时长/同步计算失败 |

HTTP 入口的参数校验仍可能先返回 1001/1002/1003；HTTP 未使能和忙状态检查也保留 3001/3003。上表是到达核心后的具体返回码，不应认为所有校验都已经改成负数码。

Ruckig 原始值不再直接充当业务码：`ErrorInvalidInput=-100` 映射为 `PlanInput=-2316`，位置限制错误映射为 `PosLimit=-2308`，其他规划错误映射为 `PlannerError=-2305`。原始值保存在 `native_code`，便于算法调试。

### 16.2 同步失败的格式与 curl 示例

不可达的目标位姿可以在仿真中触发逆解失败：

```bash
curl -sS -X POST "$BASE/api/robot/movej_ik" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"pose":{"position":{"x":100,"y":100,"z":100},"orientation":{"x":0,"y":0,"z":0,"w":1}},"velocity":0.2,"acceleration":0.6,"asynchronous":true}'
```

目标逆解发生在运动启动前，所以即使设置 asynchronous:true，该请求也会直接返回失败。示意响应（原始求解器码以实际返回为准）：

```json
{
  "success": false,
  "code": -2349,
  "error_name": "IkCalcFail",
  "message": "Target inverse kinematics calculation failed",
  "data": {
    "motion_busy": false,
    "result": -2349,
    "result_name": "IkCalcFail",
    "motion_outcome": "failed",
    "error": {
      "code": -2349,
      "error_name": "IkCalcFail",
      "message": "Target inverse kinematics calculation failed",
      "command": "movej_ik",
      "stage": "ik"
    }
  }
}
```

上例省略部分状态字段；实际求解器返回失败值时还包含 `data.error.native_code`。

### 16.3 异步失败如何获取

asynchronous:true 的启动成功只表示请求已经进入运动流程。运行中的规划、插补或失去使能导致的失败，记录在最近一次运动报告中。

```bash
curl -sS "$BASE/api/robot/move_status" | python3 -m json.tool
```

查询接口自身成功时，顶层 success:true/code:0；若运动失败，`data.result` 非零，`data.motion_outcome` 为 failed，`data.error` 包含原因。不能只检查该查询响应的顶层 success。

```bash
curl -sS -X POST "$BASE/api/robot/wait_move" \
  -H 'Content-Type: application/json' \
  -H "X-Rocos-Control-Token: $TOKEN" \
  -H "X-Rocos-Client-Id: $CLIENT_ID" \
  -d '{"timeout":10}'
```

wait_move 等待运动结束后，会把具体运动失败码放到顶层 code，并返回 success:false。等待自身超时则保持 5002，不覆盖实际运动报告。

| 详情字段 | 含义 |
| --- | --- |
| `command` | movej、movej_ik、movel、movel_fk、movec、single_axis、multi_axis 等 |
| `stage` | validation、fk、ik、planning、startup、execution、stopping 等失败阶段 |
| `joint` | 关联关节编号，从 0 开始；能确定时才返回 |
| `waypoint` | 关联轨迹点编号，从 0 开始；能确定时才返回 |
| `native_code` | 第三方原始返回码；有真实原始值时才返回 |

运动报告在 motion_busy 变为 false 之前保存完成，避免读到结束状态却丢掉失败原因。一次新运动启动时清除上一次错误；启动前的参数拒绝不会替代正在执行运动的结果。

### 16.4 正常完成、主动停止与失败

| motion_outcome | result | 含义 |
| --- | --- | --- |
| idle | 0 | 尚无运动结果 |
| running | 0 | 正在规划或执行 |
| completed | 0 | 正常完成 |
| stopped | 0 | 接收停止请求后结束，不表示到达原始目标 |
| failed | 非零 | 具体失败；查看 error |

`motion_busy` 表示是否仍有控制流程运行。规划重试期间可能已记录最近一次失败原因而仍处于 busy，应等待终态再判断最终执行结果。成功启动、成功停止和成功到位是不同含义。

成功不能用 code<1000 或 code>=0 判断，因为 Result 既有负数错误，也有正数 FunctionNotSupported=3007。当前显式识别 NoError、PlanFinished，REST 成功统一 code:0。
