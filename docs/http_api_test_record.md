# ROCOS HTTP API 测试记录

> 测试日期: 2026-07-17
> 服务器: `rocosAppMain --sim=true --http_port=8080`
> 测试工具: curl
> 基准文档: [ROCOS API.openapi.yaml](ROCOS%20%20API.openapi.yaml), [src/robot_http_server.hpp](../src/robot_http_server.hpp), [src/robot.hpp](../src/robot.hpp)

---

## 1. 机器人状态与信息

### 1.1 GET /api/robot/state — 获取机器人完整状态

```bash
curl -s http://localhost:8080/api/robot/state | python3 -m json.tool
```

<details><summary>输出</summary>

```json
{
    "code": 0,
    "message": "ok",
    "success": true,
    "data": {
        "robot_state": "STOPPED",
        "is_enabled": true,
        "is_running": false,
        "control_active": false,
        "motion_busy": false,
        "timestamp": 1784292216.6782908,
        "joint_states": [
            {"id": 0, "name": "joint_1", "position": 0.0, "velocity": 0.0, "torque": 0.0, "load_torque": 0.0, "status": 0},
            {"id": 1, "name": "joint_2", "position": 0.0, "velocity": 0.0, "torque": 0.0, "load_torque": 0.0, "status": 0},
            {"id": 2, "name": "joint_3", "position": 0.0, "velocity": 0.0, "torque": 0.0, "load_torque": 0.0, "status": 0},
            {"id": 3, "name": "joint_4", "position": 0.0, "velocity": 0.0, "torque": 0.0, "load_torque": 0.0, "status": 0},
            {"id": 4, "name": "joint_5", "position": 0.0, "velocity": 0.0, "torque": 0.0, "load_torque": 0.0, "status": 0},
            {"id": 5, "name": "joint_6", "position": 0.0, "velocity": 0.0, "torque": 0.0, "load_torque": 0.0, "status": 0},
            {"id": 6, "name": "joint_7", "position": 0.0, "velocity": 1026.44, "torque": 0.0, "load_torque": 0.0, "status": 0}
        ],
        "flange": {
            "position": {"x": 0.0, "y": 2.31e-16, "z": 1.389},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        },
        "active_tool_frame_name": "",
        "active_tool_frame": {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        },
        "active_object_frame_name": "",
        "active_object_frame": {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        },
        "hw_state": {
            "joint_num": 7,
            "state": 1
        }
    }
}
```

</details>

> **注意**: 此端点实际返回字段已超出 OpenAPI `RobotStateData` schema 定义，新增了 `robot_state`, `is_enabled`, `is_running`, `control_active`, `motion_busy`, `timestamp`, `active_tool_frame(_name)`, `active_object_frame(_name)`。OpenAPI 中的 `tool`/`object` 字段已不再返回。

---

### 1.2 GET /api/robot/info — 获取机器人参数信息

```bash
curl -s http://localhost:8080/api/robot/info | python3 -m json.tool
```

<details><summary>输出（7轴）</summary>

```json
{
    "code": 0,
    "message": "ok",
    "success": true,
    "data": {
        "joint_infos": [
            {"id": 0, "name": "joint_1", "cnt_per_unit": 156455.678, "torque_per_unit": 1.0, "ratio": 1.0, "unit_name": "rad", "zero_offset": 0},
            {"id": 1, "name": "joint_2", "cnt_per_unit": 156455.678, "torque_per_unit": 1.0, "ratio": 1.0, "unit_name": "rad", "zero_offset": 0},
            {"id": 2, "name": "joint_3", "cnt_per_unit": 156455.678, "torque_per_unit": 1.0, "ratio": 1.0, "unit_name": "rad", "zero_offset": 0},
            {"id": 3, "name": "joint_4", "cnt_per_unit": 156455.678, "torque_per_unit": 1.0, "ratio": 1.0, "unit_name": "rad", "zero_offset": 0},
            {"id": 4, "name": "joint_5", "cnt_per_unit": 156455.678, "torque_per_unit": 1.0, "ratio": 1.0, "unit_name": "rad", "zero_offset": 0},
            {"id": 5, "name": "joint_6", "cnt_per_unit": 130379.73,  "torque_per_unit": 1.0, "ratio": 1.0, "unit_name": "rad", "zero_offset": 0},
            {"id": 6, "name": "joint_7", "cnt_per_unit": 130379.73,  "torque_per_unit": 1.0, "ratio": 1.0, "unit_name": "rad", "zero_offset": -133826946}
        ]
    }
}
```

</details>

---

### 1.3 GET /api/robot/urdf — 获取 URDF 模型

```bash
curl -s -o /dev/null -w "HTTP %{http_code}, size: %{size_download} bytes" http://localhost:8080/api/robot/urdf
```

输出: `HTTP 200, size: 17877 bytes` （返回 XML 文本）

---

### 1.4 GET /api/robot/urdf/mesh — 获取连杆 3D Mesh

```bash
# 正确调用（带 path 参数）
curl -s -o /dev/null -w "HTTP %{http_code}, size: %{size_download} bytes" \
  'http://localhost:8080/api/robot/urdf/mesh?path=link1.stl'
```

输出: `HTTP 200, size: 84 bytes`

```bash
# 缺少 path 参数
curl -s http://localhost:8080/api/robot/urdf/mesh
```

```json
{"code": 1001, "message": "Missing 'path' query parameter", "success": false, "data": null}
```

---

### 1.5 GET /api/robot/enabled — 查询使能状态

```bash
curl -s http://localhost:8080/api/robot/enabled | python3 -m json.tool
```

```json
{
    "code": 0,
    "message": "ok",
    "success": true,
    "data": {
        "enabled": true,
        "disabled": false,
        "robot_state": "STOPPED"
    }
}
```

> **注意**: 实际返回比 OpenAPI `RobotEnabledData` 多出 `disabled` 和 `robot_state` 字段。

---

## 2. 基础控制

### 2.1 POST /api/robot/enable — 全局使能

```bash
curl -s -X POST http://localhost:8080/api/robot/enable | python3 -m json.tool
```

<details><summary>使能成功（STOPPED 状态）</summary>

```json
{"code": 0, "message": "Robot enabled", "success": true, "data": {"enabled": true, "robot_state": "STOPPED"}}
```

</details>

<details><summary>重复使能（已使能时）</summary>

```json
{"code": -2017, "message": "Robot enable failed", "success": false, "data": {"enabled": true, "robot_state": "STOPPED"}}
```

错误码 `-2017` = `JointStateError`

</details>

---

### 2.2 POST /api/robot/disable — 全局下电

```bash
curl -s -X POST http://localhost:8080/api/robot/disable | python3 -m json.tool
```

```json
{"code": 0, "message": "Robot disabled", "success": true, "data": {"enabled": false, "robot_state": "IDLE"}}
```

---

### 2.3 POST /api/robot/workmode — 设置工作模式

```bash
curl -s -w '\nHTTP_CODE: %{http_code}' -X POST http://localhost:8080/api/robot/workmode \
  -H 'Content-Type: application/json' \
  -d '{"mode": "position"}'
```

输出: `HTTP_CODE: 200`（空 body）

支持的模式: `position`, `ee_admit_teach`, `jnt_admit_teach`, `jnt_imp`, `cart_imp`

---

## 3. 空间运动控制 (Motion)

### 3.1 POST /api/move/joint — 关节空间运动 (MoveJ)

```bash
curl -s -X POST http://localhost:8080/api/move/joint \
  -H 'Content-Type: application/json' \
  -d '{"joints": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], "speed": 1.0, "acceleration": 2.0}' \
  | python3 -m json.tool
```

```json
{"code": 0, "message": "MoveJ accepted", "success": true, "data": {"robot_state": "RUNNING", "control_active": true}}
```

---

### 3.2 POST /api/move/joint_ik — 笛卡尔逆解运动 (MoveJ_IK)

```bash
curl -s -X POST http://localhost:8080/api/move/joint_ik \
  -H 'Content-Type: application/json' \
  -d '{"pose": {"position": {"x": 0.5, "y": 0.0, "z": 0.8}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}, "speed": 1.0}' \
  | python3 -m json.tool
```

运动冲突时:
```json
{"code": -2215, "message": "MoveJ_IK failed", "success": false, "data": {"robot_state": "RUNNING", "control_active": true}}
```

错误码 `-2215` = `ConflictTaskRunning`

---

### 3.3 POST /api/move/linear — 笛卡尔直线运动 (MoveL)

```bash
curl -s -X POST http://localhost:8080/api/move/linear \
  -H 'Content-Type: application/json' \
  -d '{"pose": {"position": {"x": 0.5, "y": 0.1, "z": 0.9}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}, "speed": 1.0}' \
  | python3 -m json.tool
```

---

### 3.4 POST /api/move/linear_fk — 关节空间直线运动 (MoveL_FK)

```bash
curl -s -X POST http://localhost:8080/api/move/linear_fk \
  -H 'Content-Type: application/json' \
  -d '{"joints": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], "speed": 1.0}' \
  | python3 -m json.tool
```

---

### 3.5 POST /api/move/stop — 停止运动

```bash
curl -s -X POST http://localhost:8080/api/move/stop | python3 -m json.tool
```

STOPPED 状态下调用:
```json
{"code": -2307, "message": "Motion stop failed", "success": false, "data": {"robot_state": "STOPPED", "control_active": false}}
```

> **注意**: `Fatal`(-2307) 是因为已在 STOPPED 状态。运动中调用则正常停止。

---

### 3.6 POST /api/move/pause — 暂停运动

```bash
curl -s -X POST http://localhost:8080/api/move/pause | python3 -m json.tool
```

_注: OpenAPI 中缺少此端点，需补充。_

---

### 3.7 POST /api/move/resume — 继续运动

```bash
curl -s -X POST http://localhost:8080/api/move/resume | python3 -m json.tool
```

_注: OpenAPI 中缺少此端点，需补充。_

---

### 3.8 GET /api/move/status — 查询运动任务状态

```bash
curl -s 'http://localhost:8080/api/move/status?task_id=test' | python3 -m json.tool
```

```json
{
    "code": 0,
    "message": "ok",
    "success": true,
    "data": {
        "task_id": "test",
        "task": null,
        "robot_state": "STOPPED",
        "is_running": false,
        "control_active": false
    }
}
```

---

## 4. 拖拽示教 (Drag / Jogging)

### 4.1 POST /api/drag/start — 启动点动

```bash
# 关节 J0 正方向点动
curl -s -X POST http://localhost:8080/api/drag/start \
  -H 'Content-Type: application/json' \
  -d '{"flag": "J0", "direction": "POSITIVE", "max_speed": 1.0}' \
  | python3 -m json.tool
```

```json
{"code": 0, "message": "Dragging started", "success": true, "data": {"robot_state": "RUNNING", "control_active": true}}
```

```bash
# 笛卡尔 TOOL_X 负方向点动
curl -s -X POST http://localhost:8080/api/drag/start \
  -H 'Content-Type: application/json' \
  -d '{"flag": "TOOL_X", "direction": "NEGATIVE", "max_speed": 0.5}' \
  | python3 -m json.tool
```

**支持的 flag 枚举**:
| 类别 | 值 |
|------|----|
| 关节 | `J0` `J1` `J2` `J3` `J4` `J5` `J6` |
| 笛卡尔-工具 | `TOOL_X` `TOOL_Y` `TOOL_Z` `TOOL_ROLL` `TOOL_PITCH` `TOOL_YAW` |
| 笛卡尔-法兰 | `FLANGE_X` `FLANGE_Y` `FLANGE_Z` `FLANGE_ROLL` `FLANGE_PITCH` `FLANGE_YAW` |
| 笛卡尔-工件 | `OBJECT_X` `OBJECT_Y` `OBJECT_Z` `OBJECT_ROLL` `OBJECT_PITCH` `OBJECT_YAW` |
| 笛卡尔-基座 | `BASE_X` `BASE_Y` `BASE_Z` `BASE_ROLL` `BASE_PITCH` `BASE_YAW` |
| 零空间 | `NULLSPACE` |

**direction 枚举**: `POSITIVE` / `NEGATIVE` / `NONE`

---

### 4.2 POST /api/drag/stop — 停止点动

```bash
curl -s -X POST http://localhost:8080/api/drag/stop | python3 -m json.tool
```

```json
{"code": 0, "message": "Dragging stopped", "success": true, "data": {"robot_state": "STOPPING", "control_active": true}}
```

---

## 5. 标定与坐标系

### 5.1 POST /api/calibration/pose — 设置标定点位

```bash
curl -s -X POST http://localhost:8080/api/calibration/pose \
  -H 'Content-Type: application/json' \
  -d '{"id": 0, "pose": {"position": {"x": 0.5, "y": 0, "z": 0.8}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}}' \
  | python3 -m json.tool
```

```json
{"code": 1004, "message": "Pose frame calibration endpoint is not implemented", "success": false, "data": null}
```

---

### 5.2 POST /api/calibration/tool — 设置工具坐标系

```bash
# 注意: 需同时传 name 和 pose/frame 字段
curl -s -X POST http://localhost:8080/api/calibration/tool \
  -H 'Content-Type: application/json' \
  -d '{"name": "tcp_default", "pose": {"position": {"x": 0, "y": 0, "z": 0.2}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}}' \
  | python3 -m json.tool
```

```json
{
    "code": 0,
    "message": "Tool frame set",
    "success": true,
    "data": {
        "name": "tcp_default",
        "frame": {"position": {"x": 0.0, "y": 0.0, "z": 0.2}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
    }
}
```

---

### 5.3 POST /api/calibration/object — 设置工件坐标系

```bash
curl -s -X POST http://localhost:8080/api/calibration/object \
  -H 'Content-Type: application/json' \
  -d '{"name": "table_1", "pose": {"position": {"x": 0.5, "y": 0.3, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}}' \
  | python3 -m json.tool
```

```json
{
    "code": 0,
    "message": "Object frame set",
    "success": true,
    "data": {
        "name": "table_1",
        "frame": {"position": {"x": 0.5, "y": 0.3, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
    }
}
```

---

### 5.4 POST /api/calibration/run — 执行标定计算

```bash
curl -s -X POST http://localhost:8080/api/calibration/run \
  -H 'Content-Type: application/json' \
  -d '{"frame": "tool"}' | python3 -m json.tool
```

```json
{"code": 1004, "message": "Calibration run endpoint is not implemented", "success": false, "data": null}
```

---

### 5.5 GET /api/calibration/result — 获取标定结果

```bash
curl -s http://localhost:8080/api/calibration/result | python3 -m json.tool
```

```json
{"code": 1004, "message": "Calibration result endpoint is not implemented", "success": false, "data": null}
```

---

## 6. Lua 脚本

### 6.1 POST /api/script/upload — 上传脚本

```bash
curl -s -X POST http://localhost:8080/api/script/upload \
  -H 'Content-Type: application/json' \
  -d '{"filename": "test.lua", "source": "robot.Sleep(100)\n"}' \
  | python3 -m json.tool
```

```json
{
    "code": 0,
    "message": "script uploaded",
    "success": true,
    "data": {
        "script_id": "script_1",
        "state": "LOADED",
        "filename": "test.lua",
        "line": 0,
        "error": "",
        "motion_active": false,
        "breakpoints": []
    }
}
```

---

### 6.2 GET /api/script/status — 获取脚本状态

```bash
curl -s http://localhost:8080/api/script/status | python3 -m json.tool
```

```json
{
    "code": 0,
    "message": "ok",
    "success": true,
    "data": {
        "script_id": "script_1",
        "state": "LOADED",
        "filename": "test.lua",
        "line": 0,
        "error": "",
        "motion_active": false,
        "breakpoints": []
    }
}
```

---

### 6.3 POST /api/script/run — 异步执行脚本

```bash
curl -s -X POST http://localhost:8080/api/script/run | python3 -m json.tool
```

```json
{"code": 0, "message": "script started", "success": true, "data": {..., "state": "RUNNING", ...}}
```

---

### 6.4 POST /api/script/pause — 暂停脚本

```bash
curl -s -X POST http://localhost:8080/api/script/pause | python3 -m json.tool
```

```json
{"code": 0, "message": "script pause requested", "success": true, "data": {..., "state": "PAUSING", ...}}
```

---

### 6.5 POST /api/script/resume — 继续脚本

```bash
curl -s -X POST http://localhost:8080/api/script/resume | python3 -m json.tool
```

已完成状态下:
```json
{"code": -6001, "message": "script state conflict", "success": false, "data": {..., "state": "COMPLETED", ...}}
```

错误码 `-6001` = `LuaStateConflict`

---

### 6.6 POST /api/script/stop — 停止脚本

```bash
curl -s -X POST http://localhost:8080/api/script/stop | python3 -m json.tool
```

---

### 6.7 POST /api/script/step — 单步执行

```bash
curl -s -X POST http://localhost:8080/api/script/step | python3 -m json.tool
```

---

### 6.8 POST /api/script/breakpoint/add — 添加断点

```bash
curl -s -X POST http://localhost:8080/api/script/breakpoint/add \
  -H 'Content-Type: application/json' \
  -d '{"filename": "debug_test.lua", "line": 2}' \
  | python3 -m json.tool
```

```json
{"code": 0, "message": "breakpoint added", "success": true, "data": {"breakpoints": [{"filename": "debug_test.lua", "line": 2}], ...}}
```

---

### 6.9 POST /api/script/breakpoint/remove — 删除断点

```bash
curl -s -X POST http://localhost:8080/api/script/breakpoint/remove \
  -H 'Content-Type: application/json' \
  -d '{"filename": "debug_test.lua", "line": 2}' \
  | python3 -m json.tool
```

---

### 6.10 POST /api/script/breakpoint/clear — 清空断点

```bash
curl -s -X POST http://localhost:8080/api/script/breakpoint/clear | python3 -m json.tool
```

---

## 7. 对照结论

### 7.1 OpenAPI 与实际返回不符

| OpenAPI 字段 | 实际返回 | 状态 |
|-------------|---------|------|
| `RobotStateData.tool` | 不存在 | ❌ 已删除，改用 `active_tool_frame` |
| `RobotStateData.object` | 不存在 | ❌ 已删除，改用 `active_object_frame` |
| `RobotStateData.hw_state.hw_type` | 不存在 | ❌ 未实现 |
| `RobotStateData.hw_state.*_cycle_time` | 不存在 | ❌ 未实现 |
| `robot_state` | 存在 | ⚠️ OpenAPI 缺失 |
| `is_enabled/is_running/motion_busy/...` | 存在 | ⚠️ OpenAPI 缺失 |
| `timestamp` | 存在 | ⚠️ OpenAPI 缺失 |

### 7.2 OpenAPI 有但未实现的端点

| 端点 | 说明 |
|------|------|
| `/api/move/circle` | MoveC 未暴露 HTTP |
| `/api/move/path` | MoveP 不存在 |
| `/api/axis/single/*` (4个) | 单轴控制整组缺失 |
| `/api/axis/multi/*` (5个) | 多轴控制整组缺失 |

### 7.3 HTTP Server 有但 OpenAPI 缺失

| 端点 | 说明 |
|------|------|
| `POST /api/move/pause` | 需补文档 |
| `POST /api/move/resume` | 需补文档 |

### 7.4 路由风格待统一

当前分散在 `/api/move/*`, `/api/drag/*`, `/api/calibration/*`, `/api/script/*`，需统一到 `/api/robot/*` 前缀下。
