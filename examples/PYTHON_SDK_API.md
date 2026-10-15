# ROCOS Python SDK API 手册

适用实现：[rocos.py](rocos.py)。更新日期：2026-10-15。

本手册面向使用 Python 控制机器人的应用开发者。普通调用使用 `Robot`、`Pose` 和运动方法即可；连接管理、控制权申请和心跳由 SDK 处理。需要对接其他接口时，可使用通用 JSON 方法。

`Robot` 是 [rocos.py](rocos.py) 中的实际类，查询和运动函数都是它的实例方法。直接创建对象后调用即可：

```python
from rocos import Robot

robot = Robot("192.168.1.117")
try:
    robot.enable()
    joints = robot.getjointposition()     # 全部关节角度
    first_joint = robot.getjointposition(0)
    flange = robot.getflange()
    robot.movej(joints)                   # 绝对关节目标，默认等到完成
finally:
    robot.close()
```

完整的“初始化类 → 调用类方法 → MoveJ / MoveL → 退出”程序见 [robot_class_demo.py](robot_class_demo.py)。它直接调用 `robot` 对象，不经过 `demo_*` 辅助函数。默认目标用于空闲七轴 Talon 仿真：

```sh
python3 robot_class_demo.py 192.168.1.117:8080
```

相关资料：[可运行示例](robot_sdk_demo.py)、[跨电脑连接与 curl](HTTP_CONTROL.md)、[REST 支持范围](../REST_API_USAGE.md)、[OpenAPI JSON](../ROCOS%20API.openapi.json)。

## 目录

- [1. 安装与快速开始](#1-安装与快速开始)
- [2. Robot 构造与使用约定](#2-robot-构造与使用约定)
- [3. 方法速查](#3-方法速查)
- [4. 状态查询](#4-状态查询)
- [5. 使能与工作模式](#5-使能与工作模式)
- [6. MoveJ 关节运动](#6-movej-关节运动)
- [7. Pose 与笛卡尔运动](#7-pose-与笛卡尔运动)
- [8. 等待与停止](#8-等待与停止)
- [9. 通用 JSON 调用](#9-通用-json-调用)
- [10. 返回值与异常](#10-返回值与异常)
- [11. 连接与控制权管理](#11-连接与控制权管理)
- [12. 心跳与诊断属性](#12-心跳与诊断属性)
- [13. 完整流程示例](#13-完整流程示例)
- [14. 兼容性、限制与验证](#14-兼容性限制与验证)

## 1. 安装与快速开始

### 1.1 环境

- Python 3.7 或更高版本，仅使用标准库。
- 将 `rocos.py` 放在应用脚本同一目录，或放入 Python 的模块搜索路径。
- 控制电脑运行支持 HTTP API 的 ROCOS app，另一台电脑能访问其 HTTP 端口，默认 8080。
- Windows 可把命令中的 `python3` 换成 `py -3`。

目录示例：

```text
my_robot_app/
├── rocos.py
└── main.py
```

### 1.2 最简运动程序

```python
from rocos import Robot

with Robot("192.168.1.117", port=8080) as robot:
    robot.enable()
    original = robot.get_joints()
    target = original.copy()
    target[0] += 0.03

    robot.movej(target, velocity=0.1, acceleration=0.3)
    print("到达目标：", robot.get_joints())
    robot.movej(original, velocity=0.1, acceleration=0.3)
```

这里的目标是相对于读取位置增加 0.03 rad；`movej()` 的参数本身始终是**绝对关节目标**。默认调用会等待运动完成。目标必须位于实际机器人的关节范围内。

可直接运行的往返程序仅允许在空闲 HardwareSim 上执行，并包含停止和状态恢复逻辑：

```sh
python3 robot_sdk_demo.py --host 192.168.1.117 --port 8080 --read-only
python3 robot_sdk_demo.py --host 192.168.1.117 --port 8080
python3 robot_sdk_demo.py --host 192.168.1.117 --demo movel
python3 robot_sdk_demo.py --host 192.168.1.117 --demo all
```

`--demo read/movej/movel/all` 分别执行只读查询、关节往返、直线往返和全部示例。默认是 `movej`，原 `--read-only` 参数仍可使用。MoveL 示例先 MoveJ 到七轴 Talon 的预备关节姿态 `[0.2, -0.4, 0.3, -0.55, 0.2, 0.4, 0.1]`，再沿基坐标 Z 方向执行默认 `-0.005 m` 的法兰直线往返，最后恢复初始关节位置。其他模型通过 `--prepare-joints` 指定适合自己的预备姿态。

`--joint`、`--joint-delta` 可调整 MoveJ 关节及角度；`--dz` 调整 MoveL 位移。示例没有通过降级或放宽限位绕过奇异姿态的规划错误。

### 1.3 只读程序

```python
from rocos import Robot

robot = Robot("192.168.1.117")
print(robot.get_joints())
print(robot.get_pose())
print(robot.is_enabled())
```

仅构造对象和查询状态不申请控制权。进入 `with` 会申请控制权，即使块内只有读取操作。

## 2. Robot 构造与使用约定

### 2.1 构造函数

```text
Robot(host="127.0.0.1", port=8080, *, timeout=5,
      client_name="python-robot", client_id=None, auto_acquire=True)
```

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `host` | `str` | `"127.0.0.1"` | 控制器主机、主机加端口，或完整 HTTP/HTTPS 地址 |
| `port` | `int` | `8080` | 主机参数未指定端口且未包含协议时使用 |
| `timeout` | 正有限数 | `5` | 单次 HTTP 请求的网络超时，秒；不是完整运动的等待时间 |
| `client_name` | `str` | `"python-robot"` | 出现在控制器控制权状态和日志中的客户端名称 |
| `client_id` | `str` 或 `None` | `None` | 高级会话标识；通常保持默认，SDK 自动生成 |
| `auto_acquire` | `bool` | `True` | 首次通过 SDK 发起普通控制操作时自动申请控制权 |

`timeout`、`client_name`、`client_id`、`auto_acquire` 是仅限关键字参数。

```python
from rocos import Robot

robot = Robot("192.168.1.117", port=8080, timeout=5)
other = Robot("http://192.168.1.117:8080", client_name="inspection-app")
```

传完整 URL 时使用 URL 自身的端口；例如 `http://host` 使用 HTTP 默认端口 80，而不是补上 8080。URL 不包含 `/api`、其他路径、用户名、密码或查询参数。构造函数不发网络请求，地址或参数错误可能抛出 `ValueError`。

当前示例服务提供 HTTP。SDK 接受 HTTPS 地址的前提是目标服务或反向代理已经配置 HTTPS。

### 2.2 推荐生命周期

```python
from rocos import Robot

with Robot("192.168.1.117") as robot:
    robot.enable()
    print(robot.get_joints())
```

进入块时调用 `connect()`，退出时调用 `close()`。异常退出也会尝试释放控制权，并保留应用的原异常。

不用 `with` 时：

```python
from rocos import Robot

robot = Robot("192.168.1.117")
try:
    robot.enable()  # 首次控制操作自动建立控制会话
    print(robot.get())
finally:
    robot.close()
```

`close()` 负责释放控制会话。异步运动应在退出前通过 `wait_move()` 等待完成，或通过 `stop()` 停止。关闭会话不会自动下使能，也不会恢复之前的工作模式。

### 2.3 单位和执行约定

| 数据 | 单位 / 约定 |
| --- | --- |
| 关节位置 | rad，按控制器关节顺序排列；关节数由实际机器人决定 |
| MoveJ、MoveJ_IK 速度 | rad/s |
| MoveJ、MoveJ_IK 加速度 | rad/s² |
| 法兰 / TCP 位置 | m |
| MoveL 速度 | m/s |
| MoveL 加速度 | m/s² |
| 姿态 | 四元数 `qx, qy, qz, qw`；不是欧拉角或旋转向量 |
| SDK 超时 | 秒 |
| `robot.get()` 的 `timestamp` | 当前服务返回 Unix epoch 毫秒 |

业务控制建议在同一线程中按顺序调用。内部心跳独立运行；当前 SDK 没有提供让多个业务线程同时改变连接状态或提交运动的同步机制。

## 3. 方法速查

| 类别 | 方法 | 默认返回 |
| --- | --- | --- |
| 状态 | `get()` / `get_state()` / `state()` | 状态 `dict` |
| 状态 | `get_joints()` / `getjoints()` / `get_joint_positions()` | 关节位置 `list` |
| 状态 | `getjointposition(joint=None)` / `get_joint_position(...)` | 全部位置列表，或单个关节位置 |
| 状态 | `get_joint_velocities()` / `get_joint_torques()` | 关节速度 / 力矩数据 `list` |
| 状态 | `getjointvelocity(joint=None)` / `get_joint_velocity(...)` | 全部速度列表，或单个关节速度 |
| 状态 | `getjointtorque(joint=None)` / `get_joint_torque(...)` | 全部力矩字段列表，或单个关节值 |
| 状态 | `get_flange()` / `getflange()` | 法兰 `Pose`，不含工具偏置 |
| 状态 | `get_pose()` / `getpose()` / `get_tool()` / `gettool()` | 当前 TCP `Pose` |
| 状态 | `get_info()` | 机器人基本信息 `dict` |
| 状态 | `is_moving()` | 是否正在运动，`bool` |
| 状态 | `is_enabled()` | `bool` |
| 基础控制 | `enable()`、`disable()` | 操作结果 `dict` |
| 工作模式 | `set_work_mode(mode)` | 模式结果 `dict` |
| 关节运动 | `movej(joints, ...)`、别名 `movrj(...)` | 运动结果 `dict` |
| 直线运动 | `movel(pose, ...)` | 运动结果 `dict` |
| 逆解关节运动 | `movej_ik(pose, ...)` | 运动结果 `dict` |
| 等待 / 停止 | `wait_move(...)`、`stop(...)` | 运动状态 `dict` |
| JSON 扩展 | `get(path, ...)`、`post(path, ...)` | 响应中的 `data` |
| 低层请求 | `request(path, ...)` | 完整标准响应 `dict` |
| 会话 | `connect(...)`、`acquire(...)`、`takeover(...)` | 控制权信息 `dict` |
| 会话 | `release()` | 控制权结果 `dict` 或 `None` |
| 会话 | `close()` | `None` |
| 诊断 | `start_heartbeat(owner)`、`stop_heartbeat()` | `None` |

普通应用主要使用状态查询和运动方法；会话和心跳方法通常由 SDK 自动调用。

## 4. 状态查询

### 4.1 `get()`、`get_state()`、`state()`

```text
get(path="state", params=None, *, raw=False, checked=True, timeout=None)
get_state()
state()
```

`robot.get()` 获取完整状态。`get_state()` 和 `state()` 是无参数快捷方法，与默认 `get()` 等价。

```python
state = robot.get()
print(state["is_enabled"])
print(state["motion_busy"])
print(state["work_mode"])
print(state["joint_states"][0]["position"])
```

常用返回字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `is_enabled` | `bool` | 实际关节使能汇总 |
| `robot_state` | `str` | 当前主要为 `DISABLED/STOPPED/RUNNING/FAULT/UNKNOWN` |
| `work_mode` | `str` | 当前工作模式，例如 `position` |
| `motion_busy` | `bool` | 控制器是否正在执行运动 |
| `motion_type` | `str` | `none/axis/trajectory/dragging/teaching` |
| `motion_outcome` | `str` | `idle/running/completed/stopped/failed` |
| `result`、`result_name` | `int`、`str` | 当前 / 上次运动的结果 |
| `error` | `dict` 或 `None` | 错误详情，可能包含阶段、关节、规划点和底层错误码 |
| `joint_states` | `list[dict]` | 每个关节的位置、速度、状态、模式等 |
| `flange`、`tool` | `dict` | 法兰和当前 TCP 位姿 |
| `hw_state` | `dict` | 硬件状态；当前 `hw_type=1` 表示 HardwareSim |
| `timestamp` | `int` | 状态时间戳，毫秒 |

GET 成功说明查询成功。即使 `error` 中保留上次运动失败，查询本身仍可成功；应根据当前业务同时检查运动结果字段。

完整字段及服务版本差异参见 [REST 手册](../REST_API_USAGE.md)。

### 4.2 `get_joints()` / `getjoints()` / `get_joint_positions()`

```text
get_joints() -> list
getjoints() -> list
get_joint_positions() -> list
```

返回按关节顺序排列的当前位置，单位 rad。每次调用都会重新查询状态。

```python
joints = robot.get_joints()
print("关节数：", len(joints))
print("第一个关节：", joints[0])
```

也可以直接使用下面的单关节 / 全部关节方法：

```text
get_joint_position(joint=None) / getjointposition(joint=None)
get_joint_velocity(joint=None) / getjointvelocity(joint=None)
get_joint_torque(joint=None) / getjointtorque(joint=None)
```

省略 `joint` 或传入 `None` 返回全部关节的列表；传入从 0 开始的整数索引返回单个数值。越界（包括负索引）抛出 `IndexError`，非整数或布尔索引抛出 `TypeError`。位置单位 rad，速度 rad/s；力矩仍使用当前旧版控制器返回的原始计数。

```python
positions = robot.getjointposition()
position_0 = robot.getjointposition(0)
velocities = robot.getjointvelocity()
torque_0 = robot.getjointtorque(0)
```

### 4.3 `get_pose()` / `getpose()` / `get_tool()` / `gettool()`

```text
get_pose() -> Pose
getpose() -> Pose
get_tool() -> Pose
gettool() -> Pose
```

从当前状态中的 `tool` 字段构造 TCP 位姿对象。

```python
pose = robot.get_pose()
print(pose.x, pose.y, pose.z)
print(pose.qx, pose.qy, pose.qz, pose.qw)
```

这是控制器当前 TCP 的位姿。SDK 不额外进行坐标系变换；工具定义和坐标系语义以当前控制器配置为准。

### 4.4 `is_enabled()`

```text
is_enabled() -> bool
```

返回状态中的 `is_enabled`。每次调用都会查询控制器。

### 4.5 `get_flange()` / `getflange()`

```text
get_flange() -> Pose
getflange() -> Pose
```

返回运动学基坐标系下的法兰位姿，直接来自控制器 `flange` 字段，不含工具偏置。与 TCP 的关系为 `T_base_tcp = T_base_flange × T_flange_tool`。

```python
flange = robot.getflange()
tcp = robot.gettool()
print("法兰：", flange)
print("TCP：", tcp)
```

当前旧版控制器的 MoveL / MoveJ_IK 按法兰目标进行规划。生成这些运动目标应使用 `get_flange()`，尤其是在配置了非零工具偏置时；SDK 不隐式把 TCP 目标转换为法兰目标。

### 4.6 `get_joint_velocities()` / `get_joint_torques()`

```text
get_joint_velocities() -> list
get_joint_torques() -> list
```

按相同关节顺序返回当前 `velocity` / `torque` 字段。速度单位 rad/s。当前旧版 `Drive::getTorque()` 仍直接返回原始力矩计数，SDK 原样返回，不能将该列表直接当成 N·m。读这些数据不会申请控制权。

### 4.7 `get_info()` / `is_moving()`

```text
get_info() -> dict
is_moving() -> bool
```

`get_info()` 返回包含 `joint_infos`、`api_profile`、`capabilities` 等字段的基本信息。`is_moving()` 返回最新状态的 `motion_busy`。两者每次调用都查询控制器。

## 5. 使能与工作模式

### 5.1 `enable()` / `disable()`

```text
enable() -> dict
disable() -> dict
```

自动建立普通控制会话后，发送使能 / 下使能命令。当前服务要求修改使能时运动空闲，否则抛出 `ApiError`。

典型返回：

```python
{"robot_state": "STOPPED", "enabled": True}
```

```python
robot.enable()
print(robot.is_enabled())
robot.stop()
robot.disable()
```

运动方法本身不会自动使能。未使能时调用 SDK 运动方法，会抛出提示先调用 `enable()` 的 `RuntimeError`。

### 5.2 `set_work_mode(mode)`

```text
set_work_mode(mode) -> dict
```

| `mode` | 当前含义 |
| --- | --- |
| `"position"` / `0` | 位置工作模式 |
| `"ee_admit_teach"` / `1` | 末端导纳示教，激活依赖控制器配置 |
| `"jnt_admit_teach"` / `2` | 关节导纳示教，激活依赖控制器配置 |
| `"jnt_imp"` / `3` | 当前仅切换枚举，关节阻抗控制算法未完整实现 |
| `"cart_imp"` / `4` | 当前仅切换枚举，笛卡尔阻抗控制算法未完整实现 |

```python
result = robot.set_work_mode("position")
print(result["mode"])
print(result["controller_implemented"])
```

当前 SDK 的 `movej/movel/movej_ik` 会在运动前自动准备 `position` 模式，普通点到点程序不需要单独调用本方法。模式切换失败会抛出 `ApiError`。这是机器人工作模式，不提供 CSP/CSV/CST 驱动模式切换。

## 6. MoveJ 关节运动

### 6.1 签名与参数

```text
movej(joints, velocity=0.1, acceleration=0.3, timeout=20, *, wait=True, **parameters)
movrj(...)   # movej 的兼容别名
```

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `joints` | 数值序列或 `dict` | 必填 | 全部关节的绝对目标，rad；长度必须与机器人关节数一致 |
| `velocity` | 正有限数 | `0.1` | 关节运动速度，rad/s |
| `acceleration` | 正有限数 | `0.3` | 关节运动加速度，rad/s² |
| `timeout` | 正有限数 | `20` | SDK 等待完成的时间预算，秒 |
| `wait` | `bool` | `True` | `True` 等待完成，`False` 受理后返回 |
| `parameters` | 关键字参数 | 无 | 其他服务端运动参数，见下文 |

执行顺序：准备控制会话、查询使能、按需准备 position 模式、提交运动、按 `wait` 等待。SDK 内部始终异步提交，Python 层负责等待和心跳。

```python
target = robot.get_joints()
target[0] += 0.03
result = robot.movej(target, velocity=0.1, acceleration=0.3, timeout=30)
```

`wait=True` 时返回最终运动状态，并额外查询实际关节位置，与目标逐关节比较，容差为 `1e-4 rad`。最终状态是 `stopped` 或位置不符时抛出 `RuntimeError`；控制器报告失败时抛出 `ApiError`。

`wait=False` 只返回受理时的状态。调用 `wait_move()` 可检查后续结果，但不会额外执行同步 `movej()` 中的目标位置比较。

### 6.2 字典参数

```python
command = {
    "joints": robot.get_joints(),
    "velocity": 0.1,
    "acceleration": 0.3,
}
result = robot.movej(command)
```

SDK 不修改传入字典。字典已包含 `velocity/acceleration` 时，以字典中的值为准；函数参数只补充缺失值。其余 `**parameters` 覆盖同名字典字段。不要传 `asynchronous` 来选择 SDK 阻塞行为，使用 `wait` 即可。

### 6.3 扩展参数与当前限制

| 参数 | 当前行为 |
| --- | --- |
| `time` | 仅 `0` 支持；非零在当前规划器中返回 `PlanInput` |
| `radius` | 接口接受非负值，但当前实现不提供有效的路径混合；保持默认 0 |
| `jerk` | MoveJ 可显式传入正值；默认不发送、不覆盖控制器配置 |

当前 HTTP 服务会把显式 MoveJ `jerk` 写入共享的关节 jerk 约束并保留，不能把它视为只影响单次命令的临时参数。普通调用建议沿用控制器配置。

SDK 不根据 `speed` 自动换算速度比例。使用明确的 `velocity`；不要同时传入两种字段。

### 6.4 同步、异步和超时

```python
robot.movej(target, wait=False)
print(robot.get()["motion_busy"])
robot.wait_move(timeout=30)
```

`timeout` 不改变轨迹速度或执行时间，也不会在超时时自动停止机器人。若应用决定取消，可显式调用 `stop()`；此时仍需具备有效控制权。

等待预算从提交受理后的等待阶段计算，最后一次网络请求可能使实际返回时间略长于该预算，因此它不是硬实时截止时间。`wait=False` 不执行等待，但 `timeout` 参数仍须合法。

## 7. Pose 与笛卡尔运动

### 7.1 `Pose`

```text
Pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
```

这是可修改的 dataclass，字段为：

| 字段 | 含义 | 单位 |
| --- | --- | --- |
| `x/y/z` | 该位姿所描述参考点的位置；法兰或 TCP 由调用接口决定 | m |
| `qx/qy/qz/qw` | 姿态四元数分量 | 无量纲 |

```python
from rocos import Pose

target = Pose(0.3, 0.0, 0.5)
pose = robot.get_pose()
pose.z -= 0.01
```

建议使用单位四元数。`Pose.to_dict()` 检查全部分量有限且四元数不全为零，但不执行四元数归一化。

#### `Pose.copy()` / `Pose.translated()`

```text
copy() -> Pose
translated(dx=0.0, dy=0.0, dz=0.0) -> Pose
```

`copy()` 返回独立副本。`translated()` 返回沿当前参考坐标轴平移后的新位姿，单位 m，姿态保持不变，原对象不改变。它不沿工具自身坐标轴平移。

```python
start = robot.getflange()
target = start.translated(dz=-0.005)
another = start.copy()
another.x += 0.01
```

#### `Pose.to_dict()`

返回新的嵌套 Python 字典：

```python
target = Pose(0.3, 0.0, 0.5)
value = target.to_dict()
# {
#   "position": {"x": 0.3, "y": 0.0, "z": 0.5},
#   "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
# }
```

非法数值会在这里或运动序列化阶段抛出 `ValueError` / `TypeError`。

#### `Pose.from_dict(value)`

类方法，从包含 `position.x/y/z` 和 `orientation.x/y/z/w` 的完整字典构造对象。缺少必需字段时可抛出 `KeyError`；构造时不执行 `to_dict()` 的数值验证。

```python
copy_pose = Pose.from_dict(target.to_dict())
```

### 7.2 `movel()`

```text
movel(pose, velocity=0.05, acceleration=0.1, timeout=20, *, wait=True, **parameters)
```

| 参数 | 说明 |
| --- | --- |
| `pose` | `Pose` 对象或控制器支持的位姿字典；当前旧版控制器使用法兰目标 |
| `velocity` | 默认 `0.05 m/s` |
| `acceleration` | 默认 `0.1 m/s²` |
| `timeout`、`wait` | 与 MoveJ 的等待语义相同 |
| `parameters` | 当前可沿用 `time=0`、`radius=0`；不支持逐命令 `jerk` 或 `tool_name` |

```python
target = robot.get_flange()
target.z -= 0.01
result = robot.movel(target, velocity=0.03, acceleration=0.1, timeout=30)
```

机器人按控制器的直线规划执行。可达性、奇异性和途中逆解由控制器判定；读取当前位姿后增加任意偏移，并不保证新路径可规划。

返回最终或受理时的运动状态 `dict`。同步 MoveL 检查终态，但 SDK 不另外实现笛卡尔位置和姿态误差验证。运行可达性更明确的演示可使用 `--demo movel`，它会先进入指定的非奇异预备姿态。

### 7.3 `movej_ik()`

```text
movej_ik(pose, velocity=0.1, acceleration=0.3, timeout=20, *, wait=True, **parameters)
```

先由控制器计算目标法兰位姿的逆解，再执行关节空间运动。`pose` 可为 `Pose` 或位姿字典；速度和加速度单位分别为 rad/s、rad/s²。TCP 路径不保证是直线。

```python
target = robot.get_flange()
target.z -= 0.01
result = robot.movej_ik(target, velocity=0.1, acceleration=0.3)
```

`time/radius/jerk` 的当前限制参照 MoveJ。逆解失败等结果通过 `ApiError` 返回。同步调用检查终态，SDK 不额外校验 TCP 误差或逆解关节目标。

## 8. 等待与停止

### 8.1 `wait_move(timeout=20)`

```text
wait_move(timeout=20) -> dict
```

等待控制器当前运动结束。单次服务端等待最多约 2 秒；返回业务码 5002 且仍未结束时，SDK 在总等待预算内继续查询。最终运动失败时抛出 `ApiError`，总预算耗尽时抛出 `TransportError`。

```python
robot.movej(target, wait=False)
result = robot.wait_move(timeout=30)
print(result["motion_outcome"])
```

这是等待**当前运动**，不是按独立任务 ID 等待某个历史命令。应用应串行提交运动。被停止的运动可返回 `motion_outcome="stopped"`；`wait_move()` 本身不会仅因这个字段抛异常，需要调用方区分“完成”和“停止”。

### 8.2 `stop()`

```text
stop(*, wait=True, timeout=20) -> dict
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `wait` | `True` | 发出停止后，轮询状态直至 `motion_busy=False` |
| `timeout` | `20` | 轮询停止结果的时间预算，秒 |

停止方式由当前控制器根据运动类型处理。

```python
robot.stop()                 # 确认控制器退出忙状态后返回
robot.stop(wait=False)       # 发出请求后返回
```

返回值是**最初停止请求的响应快照**，因此其中的 `motion_busy` 仍可能为 `True`，即使 `wait=True` 已成功等到停止。需要最新状态时调用 `robot.get()`。本方法确认退出忙状态，不替代检查最终 `error/result` 字段。

## 9. 通用 JSON 调用

普通使用优先调用机器人方法。本节用于访问尚未封装的方法、获取完整响应或排查接口问题。

### 9.1 路径规则

下面四种路径等价：

```python
robot.get("state")
robot.get("robot/state")
robot.get("/robot/state")
robot.get("/api/robot/state")
```

规则如下：

- `api/...` 保持为 `/api/...`。
- `robot/...`、`control/...`、`calibration/...`、`script/...` 自动补 `/api/`。
- 其他短路径自动补 `/api/robot/`。
- 路径可以带查询字符串，但不接受完整 URL、fragment 或 `.` / `..` 路径段。

```python
info = robot.get("info")
owner = robot.get("control/status")
```

### 9.2 `get()`

```text
get(path="state", params=None, *, raw=False, checked=True, timeout=None)
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `path` | `"state"` | 当前控制器的接口路径 |
| `params` | `None` | 查询参数字典，布尔值编码为小写 `true/false`；忽略 `None` 值；列表展开为同名多值参数 |
| `raw` | `False` | 返回完整标准响应，而不是只返回 `data` |
| `checked` | `True` | 对业务失败抛出 `ApiError`；`False` 时保留完整响应供调用方判断 |
| `timeout` | `None` | 覆盖本次请求的网络超时；`None` 沿用构造时的值 |

```python
state = robot.get("state", params={"raw_data": False})
reply = robot.get("state", raw=True)
print(reply["success"], reply["code"], reply["data"])
```

`raw_data=True` 返回控制器原始用户单位转换前的数据，不再是 `get_joints()` 约定的 rad；只有需要原始计数时才使用。

### 9.3 `post()`

```text
post(path, json=None, *, raw=False, checked=True, timeout=None)
```

`json` 为 Python 字典或其他 JSON 可序列化值。省略或传入 `None` 时发送空对象 `{}`，不是 JSON `null`。其他参数含义同 `get()`。

普通控制路径按需自动申请控制权，写请求自动附加会话信息。`control/...` 这一组底层管理接口不会触发自动申请；管理会话应优先使用 SDK 的 `connect/close/takeover`。

```python
target = robot.get_joints()
target[0] += 0.03
result = robot.post("movej", {
    "joints": target,
    "velocity": 0.1,
    "acceleration": 0.3,
    "asynchronous": True,
})
robot.wait_move(timeout=30)
```

通用 `post()` 原样遵循服务端参数语义，不执行运动快捷方法的自动 position 模式准备、终态检查或目标位置比较。原始 MoveJ 未指定 `asynchronous` 时，当前服务默认阻塞返回；长运动可能超过单次网络超时。上例显式异步，再由 SDK 等待。

返回规则：

| 调用选项 | 业务成功 | 业务失败 |
| --- | --- | --- |
| 默认 | 返回 `data` | 抛出 `ApiError` |
| `raw=True` | 返回完整响应 | 仍抛出 `ApiError` |
| `checked=False` | 返回完整响应 | 返回完整响应，不为该业务响应抛异常 |
| `raw=True, checked=False` | 返回完整响应 | 返回完整响应 |

`checked=False` 不会屏蔽连接失败、非 JSON 响应，也不会屏蔽请求前自动申请控制权失败或已经记录的会话失效异常。它只改变本次请求收到业务响应后的处理方式。

### 9.4 `request()`：低层兼容入口

```text
request(path, body=None, headers=None, timeout=None, checked=True,
        *, method=None, params=None) -> dict
```

| 参数 | 说明 |
| --- | --- |
| `path` | 使用相同的路径展开规则 |
| `body` | JSON 请求体；`None` 表示不发送请求体 |
| `headers` | `None` 时使用 SDK 会话头；显式字典会替换自动会话头，适合鉴权诊断 |
| `method` | 指定 HTTP 方法；未指定时，`body is None` 选 GET，否则选 POST |
| `params` | 查询参数，编码规则同 `get()` |
| `checked` | 是否对业务失败抛出异常 |
| `timeout` | 本次网络超时 |

本方法总是返回完整标准响应，不会自动申请控制权。`headers={}` 表示不附加当前凭据。

```python
reply = robot.request("state")
print(reply["data"]["is_enabled"])

# 诊断缺失鉴权；空 MoveJ 不会执行运动。
reply = robot.request("movej", {}, headers={}, checked=False)
print(reply["code"])
```

当前通用客户端只解析 ROCOS 标准 JSON。URDF XML、网格文件等二进制 / 文本下载需使用独立的 HTTP 下载方式。SDK 不自动跟随重定向，以避免转发控制凭据或重放请求。

## 10. 返回值与异常

### 10.1 运动结果

`movej/movel/movej_ik/wait_move/stop` 返回运动状态字典，常用字段如下：

```python
{
    "robot_state": "STOPPED",
    "motion_busy": False,
    "motion_type": "none",
    "motion_outcome": "completed",
    "result": 0,
    "result_name": "NoError",
    "error": None,
}
```

这是典型字段示意，实际响应还包含其他字段；`result_name`、运动类型和最终时刻取决于控制器状态，不要求严格等于此示例。

完整 HTTP 标准响应则包含 `success/code/message/data`，失败时还可能包含 `error_name`。HTTP 200 不能替代业务成功判断；默认 SDK 方法已经执行该判断。

### 10.2 `ApiError`

控制器返回业务失败时抛出，继承 `RuntimeError`。

| 属性 | 含义 |
| --- | --- |
| `code` | 控制器业务码 / 运动结果码 |
| `error_name` | 错误名称；未返回时为空字符串 |
| `message` | 控制器错误说明 |
| `data` | 控制器错误详情，可能为字典、列表或 `None` |
| `response` | 完整标准响应 |
| `path` | 对应接口路径 |
| `http_status` | 对应 HTTP 状态码；通常为 200 |

```python
from rocos import ApiError

try:
    robot.movej(target)
except ApiError as error:
    print("错误码：", error.code)
    print("错误名：", error.error_name)
    print("说明：", error.message)
    print("详情：", error.data)
```

HTTP 4xx/5xx 中返回的标准 JSON 业务错误也会尽量保留为 `ApiError`。当前 `wait_move()` 对最终业务响应自行构造异常，该路径的 `http_status` 使用默认值 200。

常见错误：

| 码 / 名称 | 含义 |
| --- | --- |
| `1001` | HTTP 参数缺失、类型错误或 JSON 不合法等 |
| `3003` | 当前运动与新操作冲突 |
| `3005` | 当前工作模式或状态不满足要求 |
| `3006` | 缺少控制权凭据 |
| `3007` + `ControlTokenInvalidOrExpired` | 会话过期、被接管或客户端身份不匹配 |
| `3007` + `FunctionNotSupported` | 当前服务未实现功能；不能当成会话失效 |
| `3008` | 其他客户端持有控制权 |
| `-2212` / `NotEnabled` | 执行过程中未使能或丢失使能 |
| `-2215` / `ConflictTaskRunning` | 原生运动任务冲突 |
| `-2308` / `PosLimit` | 关节位置越限 |
| `-2310` / `SpeedLimit` | 速度超限 |
| `-2316` / `PlanInput` | 规划输入无效或当前规划器不支持 |
| `-2317` / `PlanMoveJ` | 关节轨迹规划失败 |
| `-2318` / `PlanMoveL` | 直线轨迹规划失败 |
| `-2349` / `IkCalcFail` | 目标逆解失败 |
| `-2314` / `IkTrack` | 路径跟踪中的逆解失败 |

错误码存在正负值，3007 还存在不同含义。按 `code` 和 `error_name` 判断，不要只按正负号判断成功。更多映射参见 [REST 运动错误说明](../REST_API_USAGE.md#16-运动错误码与异步结果)。

### 10.3 `TransportError`

网络连接失败、读取超时、响应不是标准 JSON、等待超时等情况抛出，继承 `RuntimeError`。

```python
from rocos import TransportError

try:
    robot.movej(target, timeout=30)
except TransportError as error:
    print("请求结果需要核对：", error)
```

发送控制请求后超时，不能据此推断命令未执行。SDK 不自动重发运动，也不在超时时自动停止。应用应重新查询状态后决定等待、停止或重新提交。

### 10.4 本地校验和终态错误

以下情况可能是普通 Python 异常，而不是 `ApiError`：

- `ValueError`：非法地址、非正超时、非有限 JSON 数值、无效 Pose 等。
- `TypeError`：参数不能转换成 JSON、Pose 分量类型不正确等。
- `RuntimeError`：机器人未使能、同步运动被停止、同步 MoveJ 的最终位置不符等。
- `KeyError`：`Pose.from_dict()` 缺字段，或服务返回了不符合预期的必要字段。

不要假设所有失败都具有 `error.code`。只有 `ApiError` 保证包含上述业务异常属性。

## 11. 连接与控制权管理

这部分用于显式重连、操作员主动接管或诊断。普通程序用 `with Robot(...)` 即可。

### 11.1 `connect()`

```text
connect(*, takeover=False, keep_alive=True) -> dict
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `takeover` | `False` | `True` 表示显式接管；默认普通申请，不替换其他持有者 |
| `keep_alive` | `True` | 建立会话后启动独立心跳 |

已有本客户端有效凭据时，验证并复用当前会话；凭据失效时，显式 `connect()` 会清理旧会话并尝试普通申请。其他客户端仍持有时返回 `ApiError(3008)`，不会自动升级为接管。

```python
robot.connect()
print(robot.get_joints())
robot.close()
```

返回控制权信息字典，包含 `has_owner`、`owner_name`、`owner_client_id`、`ttl_seconds`、`heartbeat_interval_seconds` 等。新申请 / 接管时还包含 `token` 和 `client_id`；复用会话的心跳响应不包含新 token。

### 11.2 `acquire(keep_alive=True)`

直接普通申请控制权。其他端或同一端已持有时，服务端可返回 3008。需要复用本对象已有会话时调用 `connect()`。

```python
owner = robot.acquire()
```

通常不需要自己调用，首次控制和进入 `with` 都会自动处理。

### 11.3 `takeover(keep_alive=True)`

显式替换当前持有者。旧凭据立即失效，SDK 保存新会话并按需启动心跳。

```python
robot.takeover()
```

也可调用 `robot.connect(takeover=True)`。自动连接没有自动接管行为。

### 11.4 `release()`

停止并等待心跳线程退出，然后释放当前控制权。成功返回服务端控制权状态字典；本对象没有凭据时返回 `None`。

会话已经过期或被接管时，`release()` 会清理本对象旧凭据并向调用方抛出相应 `ApiError`。网络失败也会向上传递。

### 11.5 `close()`

停止心跳并尝试释放。与 `release()` 相比，会忽略“本对象会话已经失效”的鉴权错误，便于退出清理；其他错误仍会抛出。返回 `None`。

`with` 正常退出时清理失败会向上传递；若块内已有异常，则尝试清理、记录清理错误，并保留原异常。

关闭后对象没有被永久禁用。再次显式 `connect()` 可建立会话；默认 `auto_acquire=True` 且没有残留会话失效异常时，再次控制调用也可能建立新会话。建议按一个业务会话使用一个 `with` 块。

## 12. 心跳与诊断属性

### 12.1 自动行为

- 默认控制权有效期为 300 秒，可由服务端配置。
- SDK 按服务端返回的 `heartbeat_interval_seconds` 发送心跳，默认每 5 秒。
- GET 状态查询不续期；保持网络连接也不替代心跳。
- 心跳网络失败会记录日志并重试心跳；业务控制请求本身不会自动重发。
- 心跳返回会话失效码 3006/3007 时停止心跳、保存错误；后续普通 SDK 控制调用会抛出该错误。
- 明确恢复时由应用调用 `connect()`；不会在后台自动夺回控制权或重放运动。

### 12.2 `start_heartbeat(owner)` / `stop_heartbeat()`

```text
start_heartbeat(owner) -> None
stop_heartbeat() -> None
```

`owner` 是服务端控制权信息；其中 `heartbeat_interval_seconds` 如提供，必须是正有限数，未提供时使用 5 秒。调用前应已建立有效控制会话。

`start_heartbeat()` 先停止旧线程，再启动新线程。`stop_heartbeat()` 设置停止事件，并等待正在进行的请求结束；等待预算约为构造时 `timeout + 1` 秒，未能退出时抛出 `TransportError`。

停止心跳不会立即释放控制权。测试静默超时可用 `connect(keep_alive=False)`；普通应用保持自动管理即可。

### 12.3 可读诊断属性

| 属性 | 说明 |
| --- | --- |
| `base_url` | SDK 实际使用的控制器地址 |
| `timeout` | 默认网络超时 |
| `client_name` | 客户端名称 |
| `client_id` | 本会话客户端身份 |
| `token` | 当前凭据，未持有时为 `None`；不应打印或分发 |
| `heartbeat_error` | 心跳确认失权后的异常；无记录时为 `None` |

这些属性用于诊断，不建议应用手动改写凭据或底层 `http` 对象。

```python
import logging

logging.basicConfig(level=logging.WARNING)
```

SDK 的心跳异常通过 Python logging 输出，正常申请和释放不会打印凭据。

## 13. 完整流程示例

### 13.1 读取、运动、等待、退出

```python
from rocos import Robot

with Robot("192.168.1.117") as robot:
    robot.enable()
    start = robot.get_joints()
    target = start.copy()
    target[0] += 0.03
    try:
        robot.movej(target, wait=False)
        print("运动状态：", robot.get()["motion_busy"])
        result = robot.wait_move(timeout=30)
        if result["motion_outcome"] != "completed":
            raise RuntimeError("运动未正常完成：" + result["motion_outcome"])
        robot.movej(start)
    finally:
        robot.stop()
```

这里只用高级 SDK 方法。需要恢复原来的使能和工作模式时，参照 [robot_sdk_demo.py](robot_sdk_demo.py) 中的完整仿真示例。

### 13.2 业务错误处理

```python
from rocos import ApiError, Robot, TransportError

try:
    with Robot("192.168.1.117") as robot:
        robot.enable()
        target = robot.get_joints()
        target[0] += 0.03
        robot.movej(target)
except ApiError as error:
    print("控制器拒绝或执行失败：", error.code, error.error_name, error.message)
except TransportError as error:
    print("通信或等待失败，需要核对实际状态：", error)
except (ValueError, TypeError, RuntimeError) as error:
    print("参数、状态或执行结果错误：", error)
```

示例没有在异常后自动重试运动。是否停止或重连，应由业务根据当前状态决定。

## 14. 兼容性、限制与验证

### 14.1 已封装与通用扩展

专用运动方法为 `movej/movrj/movel/movej_ik`。圆弧、点动、坐标系、标定等尚未提供专用 Python 方法，可在服务端支持时通过 `get/post` 调用。

通用 JSON 入口不会让服务端未实现的功能变为可用。当前主要限制包括命名工具 / 工件系、新版阻抗算法、Lua 脚本、部分伺服接口和暂停恢复等，详见 [REST 支持范围](../REST_API_USAGE.md)。

本 SDK 不提供 CSP/CSV/CST 驱动模式切换。HTTP 控制权当前不约束其他客户端的 gRPC 控制调用。

### 14.2 旧示例兼容

```text
RocosClient(base_url, timeout=5)
```

`RocosClient` 是 `Robot` 的兼容子类，供旧 `http_control.py` 使用，默认 `auto_acquire=False`。新应用使用 `from rocos import Robot`，无需使用该类。

运行原命令行示例时，`http_control.py` 和 `rocos.py` 必须位于同一目录或可被 Python 正常导入。

### 14.3 验证

SDK 有 16 项测试，覆盖真实 HardwareSim 运动、心跳、会话占用、接管后的错误处理、同步 / 异步、停止、参数和传输失败。也覆盖单关节 / 全部关节查询、非零工具偏置下区分法兰与 TCP、真实 MoveL 往返、模块化 demo 和直接类调用示例。完整方法实现以 [rocos.py](rocos.py) 为准。

项目根目录执行：

```sh
cmake -S . -B build
ctest --test-dir build -R '^python_robot_client_test$' --output-on-failure
```

测试创建独立仿真，不需要占用用户当前运行的控制器。手册中的示例目标仍需结合实际机器人和坐标配置使用。
