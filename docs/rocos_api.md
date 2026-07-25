# ROCOS API 接口定义说明文档

> 本文档依据 [ROCOS API.openapi.yaml](ROCOS%20%20API.openapi.yaml)（OpenAPI 3.0.1，版本 2.0.0）生成，
> 描述机器人控制器的 HTTP/JSON 接口。该接口替代了原 gRPC 方案，支持多语言、跨平台集成。

## 目录

- [1. 通用约定](#1-通用约定)
- [2. 数据模型](#2-数据模型)
- [3. 机器人状态与信息](#3-机器人状态与信息)
- [4. 基础控制](#4-基础控制)
- [5. 空间运动控制 (Motion)](#5-空间运动控制-motion)
- [6. 点动控制 (Jogging)](#6-点动控制-jogging)
- [7. 坐标系管理 (Frame)](#7-坐标系管理-frame)
- [8. 标定](#8-标定)
- [9. Lua 脚本](#9-lua-脚本)

---

## 1. 通用约定

### 1.1 标准响应结构 (StandardResponse)

除二进制/纯文本接口外，所有接口均返回统一 JSON 结构：

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | boolean | 请求是否成功 |
| `code` | integer | 业务状态码，见下表 |
| `message` | string | 结果描述信息 |
| `data` | object \| null | 业务数据，具体结构因接口而异 |

### 1.2 业务状态码分段

| 码段 | 含义 |
|------|------|
| `0` | 成功 |
| `1xxx` | 参数错误 |
| `2xxx` | 运动 / 规划错误 |
| `3xxx` | 机器人状态错误 |
| `4xxx` | 标定错误 |
| `5xxx` | 异步任务错误 |
| `-6xxx` | Lua 脚本错误 |

### 1.3 单位约定

- 关节角度：**rad**（弧度）
- 位姿位置：**m**（米）
- 位姿姿态：**四元数** `(x, y, z, w)`
- 角度量（如 MoveC 的 `theta`）：**rad**

---

## 2. 数据模型

### 2.1 Pose（位姿）

```json
{
  "position":    { "x": 0.0, "y": 0.0, "z": 0.0 },
  "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
}
```

- `position`：位置坐标（米），各分量默认 `0`
- `orientation`：四元数姿态，`w` 默认 `1`，其余默认 `0`

### 2.2 RobotStateData（机器人状态快照）

| 字段 | 类型 | 说明 |
|------|------|------|
| `robot_state` | string | FSM 状态，见下表 |
| `is_enabled` | boolean | 是否已使能 |
| `is_running` | boolean | 是否正在运行 |
| `control_active` | boolean | 控制是否激活 |
| `motion_busy` | boolean | 是否有运动在执行 |
| `work_mode` | string | 当前工作模式，取值：`position`、`jnt_imp`、`jnt_admit_teach`、`cart_imp` |
| `timestamp` | number | 快照时刻（epoch 秒） |
| `joint_states` | JointState[] | 各关节状态数组 |
| `flange` | Pose | 法兰位姿 |
| `active_tool_frame_name` | string | 激活工具坐标系名称 |
| `active_tool_frame` | Pose | 激活工具坐标系位姿 |
| `active_object_frame_name` | string | 激活工件坐标系名称 |
| `active_object_frame` | Pose | 激活工件坐标系位姿 |
| `hw_state` | HardwareState | 硬件总线摘要 |

**robot_state 枚举**：
`IDLE`、`ENABLING`、`DISABLING`、`STARTING`、`STOPPING`、`PAUSING`、`RESUMING`、`RESETTING`、`RUNNING`、`PAUSED`、`STOPPED`、`SERVOING`、`ERROR_STATE`

### 2.3 JointState（关节状态）

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | integer | 关节 ID |
| `name` | string | 关节名称 |
| `position` | number | 当前位置 |
| `velocity` | number | 当前速度 |
| `torque` | number | 当前力矩 |
| `load_torque` | number | 负载力矩 |
| `status` | integer | `0`: disabled，`1`: fault，`2`: enabled |

### 2.4 HardwareState（硬件状态）

| 字段 | 类型 | 说明 |
|------|------|------|
| `joint_num` | integer | 关节数量 |
| `state` | integer | `0`: DISABLED，`1`: ENABLED，`2`: ERROR |

### 2.5 RobotInfoData（机器人参数信息）

`joint_infos` 数组，每项包含：

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | integer | 关节 ID |
| `name` | string | 关节名称 |
| `cnt_per_unit` | number | 每单位编码器脉冲数 |
| `torque_per_unit` | number | 每单位力矩系数 |
| `ratio` | number | 传动比 |
| `unit_name` | string | 单位名称 |
| `zero_offset` | integer | 零点偏移 |

### 2.6 MotionResponse（运动类接口响应）

在 StandardResponse 基础上，`data` 包含：

| 字段 | 类型 | 说明 |
|------|------|------|
| `robot_state` | string | 当前 FSM 状态 |
| `control_active` | boolean | 控制是否激活 |
| `motion_busy` | boolean | 是否有运动在执行（WaitMove 专用） |

### 2.7 坐标系相关模型

**FrameNameList**（坐标系名称列表）：

| 字段 | 类型 | 说明 |
|------|------|------|
| `names` | string[] | 坐标系名称列表 |
| `active` | string | 当前激活的坐标系名称 |

**NamedFrame**（命名坐标系）：

| 字段 | 类型 | 说明 |
|------|------|------|
| `name` | string | 坐标系名称 |
| `frame` | Pose | 坐标系位姿 |

**FramesSnapshot**（坐标系快照，用于加载/保存）：

| 字段 | 类型 | 说明 |
|------|------|------|
| `path` | string | YAML 文件路径 |
| `tool_frames` | string[] | 工具坐标系名称列表 |
| `object_frames` | string[] | 工件坐标系名称列表 |
| `active_tool_frame` | string | 激活的工具坐标系 |
| `active_object_frame` | string | 激活的工件坐标系 |

### 2.8 Lua 脚本相关模型

**ScriptStatusData**（脚本状态）：

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | string | 脚本状态机状态，枚举：`EMPTY`、`LOADED`、`RUNNING`、`PAUSING`、`PAUSED`、`STOPPING`、`COMPLETED`、`FAILED`、`STOPPED` |
| `script_id` | string | 脚本 ID |
| `filename` | string | 脚本文件名 |
| `line` | integer | 当前执行行号（≥0） |
| `error` | string | 错误信息 |
| `motion_active` | boolean | 脚本触发的运动是否仍在执行 |
| `breakpoints` | ScriptBreakpoint[] | 断点列表，每项为 `{filename, line}`（line ≥ 1） |

**ScriptResponse**：StandardResponse + `data` 为 ScriptStatusData。

---

## 3. 机器人状态与信息

### 3.1 获取机器人完整状态快照

```
GET /api/robot/state
```

返回 FSM 状态、关节数据、笛卡尔位姿（法兰、激活工具系、激活工件系）及硬件总线摘要。

- **响应**：`200` — StandardResponse，`data` 为 [RobotStateData](#22-robotstatedata机器人状态快照)

### 3.2 获取机器人参数信息

```
GET /api/robot/info
```

返回各关节的传动比、编码器脉冲数、零点偏移等配置参数。

- **响应**：`200` — StandardResponse，`data` 为 [RobotInfoData](#25-robotinfodata机器人参数信息)

### 3.3 获取机器人 URDF 模型 XML

```
GET /api/robot/urdf
```

- **响应**：`200` — `text/xml`，URDF XML 文本

### 3.4 获取连杆 3D Mesh 文件 (STL)

```
GET /api/robot/urdf/mesh?path=<path>
```

| 参数 | 位置 | 必填 | 说明 |
|------|------|------|------|
| `path` | query | 是 | STL 文件相对路径，支持 `package://` 格式 |

- **响应**：`200` — `application/octet-stream`，二进制文件流

### 3.5 查询使能状态

```
GET /api/robot/enabled
```

- **响应**：`200` — StandardResponse，`data` 结构：

| 字段 | 类型 | 说明 |
|------|------|------|
| `enabled` | boolean | 是否已使能 |
| `disabled` | boolean | 是否已下电 |
| `robot_state` | string | 当前 FSM 状态 |

---

## 4. 基础控制

### 4.1 全局使能机器人

```
POST /api/robot/enable
```

- **响应**：`200` — StandardResponse（使能成功）

### 4.2 全局下电 / 关闭使能

```
POST /api/robot/disable
```

- **响应**：`200` — StandardResponse（下电成功）

### 4.3 清除报警并复位

```
POST /api/robot/reset
```

仅用于 `ERROR_STATE`。触发状态机 `ERROR_STATE → RESETTING`，清除硬件 Fault/Alarm 后重新使能；成功进入 `STOPPED`，失败保持 `ERROR_STATE`。

> **注意**：`Stop` 只负责停止运动，不负责清除报警；清除报警必须调用本接口。

- **响应**：`200` — StandardResponse，`data` 结构：

| 字段 | 类型 | 说明 |
|------|------|------|
| `robot_state` | string | 复位后状态，枚举：`STOPPED`、`ERROR_STATE` |
| `enabled` | boolean | 是否已使能 |

### 4.4 设置工作模式

```
POST /api/robot/workmode
```

**请求体**：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `mode` | string | 是 | 工作模式，见下表 |

**mode 枚举**：

| 值 | 含义 |
|----|------|
| `position` | 位置模式 |
| `ee_admit_teach` | 末端导纳拖拽示教 |
| `jnt_admit_teach` | 关节导纳拖拽示教 |
| `jnt_imp` | 关节阻抗控制 |
| `cart_imp` | 笛卡尔阻抗控制 |

- **响应**：`200` — 模式切换成功

---

## 5. 空间运动控制 (Motion)

所有运动指令均为**异步受理**：接口返回表示请求已受理，实际运动进度通过 [5.10 查询运动状态](#510-查询运动任务状态) 或 [5.9 WaitMove](#59-等待当前运动结束阻塞) 跟踪。

运动类接口的公共速度参数（除特别说明外）：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `velocity` | number | 1.0 | 速度限制 |
| `acceleration` | number | 2.0 | 加速度限制 |
| `jerk` | number | 10.0 | 加加速度限制 |

### 5.1 关节空间运动 (MoveJ)

```
POST /api/robot/movej
```

**请求体**：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `joints` | number[] | 是 | 目标关节角度 [rad]，长度必须等于机器人关节数 |
| `velocity` / `acceleration` / `jerk` | number | 否 | 公共速度参数 |

- **响应**：`200` — [MotionResponse](#26-motionresponse运动类接口响应)

### 5.2 笛卡尔位姿逆解运动 (MoveJ_IK)

```
POST /api/robot/movej_ik
```

对目标笛卡尔位姿做逆运动学求解后，以关节空间方式运动。

**请求体**：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `pose` | Pose | 是 | 目标笛卡尔位姿 |
| `velocity` / `acceleration` / `jerk` | number | 否 | 公共速度参数 |

- **响应**：`200` — MotionResponse

### 5.3 笛卡尔直线运动 (MoveL)

```
POST /api/robot/movel
```

**请求体**：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `pose` | Pose | 是 | 目标笛卡尔位姿 |
| `tool_name` | string | 否 | 指定使用的工具坐标系名称 |
| `velocity` / `acceleration` / `jerk` | number | 否 | 公共速度参数 |

- **响应**：`200` — MotionResponse

### 5.4 关节空间直线运动 (MoveL_FK)

```
POST /api/robot/movel_fk
```

给定目标关节角，末端沿笛卡尔直线运动到对应位姿。

**请求体**：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `joints` | number[] | 是 | 目标关节角度 [rad] |
| `tool_name` | string | 否 | 指定使用的工具坐标系名称 |
| `velocity` / `acceleration` / `jerk` | number | 否 | 公共速度参数 |

- **响应**：`200` — MotionResponse

### 5.5 笛卡尔圆弧运动 (MoveC)

```
POST /api/robot/movec
```

支持两种圆弧定义模式：

1. **圆心 + 角度**：`center_frame` + `theta`
2. **三点圆弧**：`pose_via` + `pose_goal`

`pose_start` 可选，默认为当前法兰位姿。

**请求体**：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `pose_start` | Pose | 否 | 圆弧起点，默认当前法兰位姿 |
| `center_frame` | Pose | 条件 | 圆心位姿（与 `theta` 配合） |
| `theta` | number | 条件 | 圆弧角度 [rad]（与 `center_frame` 配合） |
| `pose_via` | Pose | 条件 | 圆弧经过点（与 `pose_goal` 配合） |
| `pose_goal` | Pose | 条件 | 圆弧终点（与 `pose_via` 配合） |
| `pose_to` | Pose | 否 | `pose_goal` 的别名 |
| `velocity` / `acceleration` / `jerk` | number | 否 | 公共速度参数 |

- **响应**：`200` — MotionResponse

### 5.6 暂停运动

```
POST /api/robot/pause
```

- **响应**：`200` — MotionResponse

### 5.7 继续运动

```
POST /api/robot/resume
```

- **响应**：`200` — MotionResponse

### 5.8 停止运动

```
POST /api/robot/stop
```

- **响应**：`200` — MotionResponse

### 5.9 等待当前运动结束（阻塞）

```
POST /api/robot/wait_move
```

先延时 20ms，再轮询 FSM 直到运动完成或进入 `ERROR_STATE`。为阻塞式接口。

- **响应**：`200` — MotionResponse（运动结束或出错）

### 5.10 查询运动/任务状态

```
GET /api/robot/move_status?task_id=<task_id>
```

| 参数 | 位置 | 必填 | 说明 |
|------|------|------|------|
| `task_id` | query | 否 | 指定查询的任务 ID，缺省查询整体状态 |

- **响应**：`200` — StandardResponse，`data` 结构：

| 字段 | 类型 | 说明 |
|------|------|------|
| `robot_state` | string | 当前 FSM 状态 |
| `is_running` | boolean | 是否正在运行 |
| `control_active` | boolean | 控制是否激活 |
| `task_id` | string | 任务 ID |
| `task` | object \| null | 任务详细信息 |

---

## 6. 点动控制 (Jogging)

> 点动接口全部使用**向量**方式传递运动方向。

点动公共参数：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `direction` | string | `POSITIVE` | 向量整体缩放符号（乘到每个元素上），枚举：`POSITIVE`、`NEGATIVE` |
| `speed` | number | 1.0 | 点动速度 |
| `timeout` | number | 0.1 | 超时时间 [s]，超时未收到新指令自动停止 |
| `dir_threshold` | number | 0.99 | 方向判定阈值 |

### 6.1 关节点动

```
POST /api/robot/jog/joint
```

传入与关节数等长的方向向量，每个元素为该关节的速度方向。

**请求体**：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `joints` | number[] | 是 | 关节方向向量，长度等于机器人关节数 |
| `direction` / `speed` / `timeout` / `dir_threshold` | — | 否 | 点动公共参数 |

- **响应**：`200` — MotionResponse

### 6.2 笛卡尔点动

```
POST /api/robot/jog/cartesian
```

传入 6 维 twist 向量 `[vx, vy, vz, wx, wy, wz]`，前 3 个为线速度，后 3 个为角速度。Robot 层会将 FLANGE/TOOL/OBJECT 下的 twist 旋转到 BASE 后执行。

**请求体**：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `twist` | number[6] | 是 | 6 维 twist `[vx, vy, vz, wx, wy, wz]` |
| `frame` | string | 否 | 点动参考坐标系，默认 `BASE`，见下表 |
| `direction` / `speed` / `timeout` / `dir_threshold` | — | 否 | 点动公共参数 |

**frame 枚举**：

| 值 | 含义 |
|----|------|
| `BASE` | twist 已在基坐标系表达 |
| `FLANGE` | twist 在当前法兰坐标系表达 |
| `TOOL` | twist 在当前激活工具坐标系表达 |
| `OBJECT` | twist 在当前激活工件坐标系表达 |

- **响应**：`200` — MotionResponse

### 6.3 零空间点动

```
POST /api/robot/jog/nullspace
```

传入与关节数等长的方向向量，在不影响末端位姿的零空间内运动（适用于冗余自由度机器人调整构型）。

**请求体**：同 [6.1 关节点动](#61-关节点动)。

- **响应**：`200` — MotionResponse

### 6.4 SVD 维度速度点动

```
POST /api/robot/jog/svd
```

沿雅可比矩阵 SVD 分解的各维度方向点动，传入各维度的速度标量数组。

**请求体**：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `dim_speeds` | number[] | 是 | 各维度速度标量 |
| `timeout` | number | 否 | 默认 0.1 |
| `dir_threshold` | number | 否 | 默认 0.99 |

- **响应**：`200` — MotionResponse

### 6.5 停止点动

```
POST /api/robot/jog/stop
```

- **响应**：`200` — MotionResponse

---

## 7. 坐标系管理 (Frame)

### 7.1 获取所有工具坐标系名称列表

```
GET /api/robot/tool_frames
```

- **响应**：`200` — StandardResponse，`data` 为 [FrameNameList](#27-坐标系相关模型)

### 7.2 获取所有工件坐标系名称列表

```
GET /api/robot/object_frames
```

- **响应**：`200` — StandardResponse，`data` 为 FrameNameList

### 7.3 工具坐标系 查询 / 设置 / 删除

**查询指定工具坐标系**：

```
GET /api/robot/tool_frame?name=<name>
```

| 参数 | 位置 | 必填 | 说明 |
|------|------|------|------|
| `name` | query | 是 | 坐标系名称 |

- **响应**：`200` — StandardResponse（成功或未找到），`data` 为 NamedFrame

**设置 / 新增工具坐标系**：

```
POST /api/robot/tool_frame
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `name` | string | 是 | 坐标系名称（字母/数字/下划线/短横/点号，≤64 字符） |
| `frame` | Pose | 是 | 坐标系位姿 |
| `pose` | Pose | 否 | `frame` 的别名 |

- **响应**：`200` — StandardResponse，`data` 为 NamedFrame

**删除指定工具坐标系**：

```
DELETE /api/robot/tool_frame?name=<name>
```

- **响应**：`200` — StandardResponse

### 7.4 工件坐标系 查询 / 设置 / 删除

接口形式与工具坐标系完全对称：

```
GET    /api/robot/object_frame?name=<name>   # 查询
POST   /api/robot/object_frame               # 设置/新增（body: {name, frame | pose}）
DELETE /api/robot/object_frame?name=<name>   # 删除
```

响应结构同 7.3。

### 7.5 激活指定工具坐标系

```
POST /api/robot/active_tool_frame
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `name` | string | 是 | 要激活的工具坐标系名称 |

- **响应**：`200` — StandardResponse

### 7.6 激活指定工件坐标系

```
POST /api/robot/active_object_frame
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `name` | string | 是 | 要激活的工件坐标系名称 |

- **响应**：`200` — StandardResponse

### 7.7 从 YAML 文件加载坐标系

```
POST /api/robot/frames/load
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `path` | string | 是 | YAML 文件路径 |

- **响应**：`200` — StandardResponse，`data` 为 [FramesSnapshot](#27-坐标系相关模型)

### 7.8 保存坐标系到 YAML 文件

```
POST /api/robot/frames/save
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `path` | string | 是 | YAML 文件路径 |

- **响应**：`200` — StandardResponse，`data` 为 FramesSnapshot

---

## 8. 标定

### 8.1 设置标定点位 ⚠️ 已废弃

```
POST /api/calibration/pose
```

**deprecated**：当前未实现，返回 `code=1004`。

### 8.2 设置工具坐标系

```
POST /api/calibration/tool
```

功能等同 `POST /api/robot/tool_frame`。

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `name` | string | 是 | 坐标系名称 |
| `frame` | Pose | 是 | 坐标系位姿 |
| `pose` | Pose | 否 | `frame` 的别名 |

- **响应**：`200` — StandardResponse

### 8.3 设置工件坐标系

```
POST /api/calibration/object
```

功能等同 `POST /api/robot/object_frame`，请求体同 8.2。

- **响应**：`200` — StandardResponse

### 8.4 执行标定计算 ⚠️ 已废弃

```
POST /api/calibration/run
```

**deprecated**：当前未实现，返回 `code=1004`。

### 8.5 获取标定结果 ⚠️ 已废弃

```
GET /api/calibration/result
```

**deprecated**：当前未实现，返回 `code=1004`。

---

## 9. Lua 脚本

脚本生命周期：上传编译（`LOADED`）→ 异步执行（`RUNNING`）→ 完成/失败/停止。执行、暂停、单步等操作均通过 `/api/script/status` 查询状态。

### 9.1 上传并编译 Lua 脚本

```
POST /api/script/upload
```

脚本仅加载到内存，**不写入文件系统**；成功后进入 `LOADED` 状态。

**请求体**：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `filename` | string | 是 | 脚本文件名 |
| `source` | string | 是 | UTF-8 Lua 源码 |

- **响应**：`200` — ScriptResponse

### 9.2 异步执行已加载的脚本

```
POST /api/script/run
```

立即返回；通过 `status` 接口查询进度。

- **响应**：`200` — ScriptResponse

### 9.3 暂停脚本

```
POST /api/script/pause
```

- **响应**：`200` — ScriptResponse

### 9.4 继续脚本

```
POST /api/script/resume
```

- **响应**：`200` — ScriptResponse

### 9.5 停止脚本

```
POST /api/script/stop
```

- **响应**：`200` — ScriptResponse

### 9.6 单步执行脚本

```
POST /api/script/step
```

执行至下一个有效行后暂停（需脚本处于暂停状态）。

- **响应**：`200` — ScriptResponse

### 9.7 获取脚本状态

```
GET /api/script/status
```

- **响应**：`200` — ScriptResponse，`data` 为 [ScriptStatusData](#28-lua-脚本相关模型)

### 9.8 添加断点

```
POST /api/script/breakpoint/add
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `filename` | string | 是 | 脚本文件名 |
| `line` | integer | 是 | 行号（≥1） |

- **响应**：`200` — ScriptResponse

### 9.9 删除断点

```
POST /api/script/breakpoint/remove
```

请求体同 9.8。

- **响应**：`200` — ScriptResponse

### 9.10 清空所有断点

```
POST /api/script/breakpoint/clear
```

- **响应**：`200` — ScriptResponse

---

## 附录：接口一览表

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/api/robot/state` | 获取机器人完整状态快照 |
| GET | `/api/robot/info` | 获取机器人参数信息 |
| GET | `/api/robot/urdf` | 获取 URDF 模型 XML |
| GET | `/api/robot/urdf/mesh` | 获取连杆 STL 文件 |
| GET | `/api/robot/enabled` | 查询使能状态 |
| POST | `/api/robot/enable` | 全局使能 |
| POST | `/api/robot/disable` | 全局下电 |
| POST | `/api/robot/reset` | 清除报警并复位 |
| POST | `/api/robot/workmode` | 设置工作模式 |
| POST | `/api/robot/movej` | 关节空间运动 |
| POST | `/api/robot/movej_ik` | 笛卡尔位姿逆解运动 |
| POST | `/api/robot/movel` | 笛卡尔直线运动 |
| POST | `/api/robot/movel_fk` | 关节空间直线运动 |
| POST | `/api/robot/movec` | 笛卡尔圆弧运动 |
| POST | `/api/robot/pause` | 暂停运动 |
| POST | `/api/robot/resume` | 继续运动 |
| POST | `/api/robot/stop` | 停止运动 |
| POST | `/api/robot/wait_move` | 等待运动结束（阻塞） |
| GET | `/api/robot/move_status` | 查询运动/任务状态 |
| POST | `/api/robot/jog/joint` | 关节点动 |
| POST | `/api/robot/jog/cartesian` | 笛卡尔点动 |
| POST | `/api/robot/jog/nullspace` | 零空间点动 |
| POST | `/api/robot/jog/svd` | SVD 维度速度点动 |
| POST | `/api/robot/jog/stop` | 停止点动 |
| GET | `/api/robot/tool_frames` | 工具坐标系名称列表 |
| GET | `/api/robot/object_frames` | 工件坐标系名称列表 |
| GET/POST/DELETE | `/api/robot/tool_frame` | 工具坐标系 查询/设置/删除 |
| GET/POST/DELETE | `/api/robot/object_frame` | 工件坐标系 查询/设置/删除 |
| POST | `/api/robot/active_tool_frame` | 激活工具坐标系 |
| POST | `/api/robot/active_object_frame` | 激活工件坐标系 |
| POST | `/api/robot/frames/load` | 从 YAML 加载坐标系 |
| POST | `/api/robot/frames/save` | 保存坐标系到 YAML |
| POST | `/api/calibration/pose` | ⚠️ 已废弃（未实现） |
| POST | `/api/calibration/tool` | 设置工具坐标系 |
| POST | `/api/calibration/object` | 设置工件坐标系 |
| POST | `/api/calibration/run` | ⚠️ 已废弃（未实现） |
| GET | `/api/calibration/result` | ⚠️ 已废弃（未实现） |
| POST | `/api/script/upload` | 上传并编译 Lua 脚本 |
| POST | `/api/script/run` | 异步执行脚本 |
| POST | `/api/script/pause` | 暂停脚本 |
| POST | `/api/script/resume` | 继续脚本 |
| POST | `/api/script/stop` | 停止脚本 |
| POST | `/api/script/step` | 单步执行 |
| GET | `/api/script/status` | 获取脚本状态 |
| POST | `/api/script/breakpoint/add` | 添加断点 |
| POST | `/api/script/breakpoint/remove` | 删除断点 |
| POST | `/api/script/breakpoint/clear` | 清空断点 |
