# 七轴关节控制 JSON 协议设计

本文整理两类消息：控制端发送给上层的**状态反馈**，以及上层发送给控制端的**控制指令**。

**实现入口：[src/jointTcpClientMain.cc](../src/jointTcpClientMain.cc)。** 程序作为 TCP 客户端主动连接上位机服务端，接收 10 Hz 指令，用 Ruckig 以 1 ms 步长生成连续参考位置，并通过同一连接以约 100 Hz 回传反馈。原 `jointImpedanceMain.cc` 保留独立位置保持功能。

**一问一答版本：[src/jointTcpRequestReplyClientMain.cc](../src/jointTcpRequestReplyClientMain.cc)。** 使用相同的连接参数和 JSON 格式，不主动发送状态；每收到一条完整且校验通过的 command，回复一行最新采样状态 JSON，不等待目标运动完成。非法消息不回复，也不刷新指令超时。TCP 粘包中的每条有效指令各回复一帧，控制目标仍只保留最新一条。首帧采样前的请求会等待真实状态；待回复请求上限为 1024，积压达到上限后暂停读取新指令，待回复发出后继续读取。

一问一答版本中，`--command_timeout`（默认 1 秒）仅用于告警：超时后保持程序和 TCP 连接运行，继续执行并保持最后一条有效指令的目标，新指令到来时正常更新。发送一帧状态耗时超过 1 秒也仅告警，保留原帧及已发送偏移并继续重试，避免丢帧或重复发送帧前缀。每次超时只告警一次，恢复时打印恢复日志。实际断线、配置错误及机器人保护故障仍会故障退出；原 `jointTcpClientMain.cc` 的超时退出规则保持原样。

一问一答版本另支持 `mode="free"` 零力模式：固定 `kp=[0,0,0,0,0,0,0]`、`kd=[4,4,4,3,2,1.5,1]`、摩擦补偿比例 `0.6`。参数独立于 YAML 中普通阻抗模式的增益与摩擦比例，切换规则同 `impedance`，详见 3.5 节。

可用 HardwareSim 回归验证五种模式在指令超时后继续运行，并在新请求到来时回复：

```bash
python3 test/joint_tcp_request_reply_client_test.py \
  --binary=build/bin/jointTcpRequestReplyClientMain --config=config/identified_impedance.yaml
```

```bash
cmake --build build --target jointTcpRequestReplyClientMain -j2
./build/bin/jointTcpRequestReplyClientMain --config=config/identified_impedance.yaml \
  --host=127.0.0.1 --port=9000 --sim
```

一问一答版本默认打印连接信息及每帧收发 JSON：日志带本地时间（毫秒和时区）、服务端配置、实际远端/本地 IP 与端口、帧编号、字节数和缩进 JSON。TX 日志在整帧写入 socket 后输出；拒绝指令会显示原因，网络线程退出时打印收发统计。使用 `--notcp_log_data` 可关闭逐帧数据日志，保留连接、错误和统计信息。网络传输仍是一行一个 JSON，日志中的缩进仅用于终端阅读。

本文的 `stop` 已按实际需求更新为 **CSP 下执行 MoveJ 到指定关节角并保持**，不再表示下使能；原四种指令都必须携带目标 `q`，新增 `free` 可省略 `q`。位置和 stop 模式的两个计算力矩字段在一问一答版本中为 `[0,0,0,0,0,0,0]`，原 100 Hz 版本中为 `null`。

## 1. 通用约定

### 1.1 编码与数据类型

- 消息使用 UTF-8 编码，每条消息的顶层为一个 JSON 对象。
- 字段名称和模式名称区分大小写。
- 角度、速度和力矩使用 JSON 数字，不能写成带单位的字符串。
- 数字必须有限，不允许 `NaN`、`Infinity` 或 `-Infinity`。
- JSON 不支持十六进制数字字面量，因此帧头使用字符串，例如 `"0xAA55"`。
- 本文示例均为格式示意，不对应当前真机采样。

### 1.2 关节顺序与单位

控制使用的 `q` 和状态反馈中的关节数组固定为 **7 个元素**，顺序统一如下，不按硬件 ID 或接收顺序重新排列：

| 数组下标 | 关节名称 |
| --- | --- |
| 0 | `joint_1` |
| 1 | `joint_2` |
| 2 | `joint_3` |
| 3 | `joint_4` |
| 4 | `joint_5` |
| 5 | `joint_6` |
| 6 | `joint_7` |

| 数据 | 协议单位 | 说明 |
| --- | --- | --- |
| 关节角 | rad | 弧度，不能直接传入角度制数据 |
| 关节速度 | rad/s | 关节输出端角速度 |
| 关节力矩 | Nm | 关节输出端力矩，不是驱动器千分比计数 |
| 时间戳 | μs | Unix 时间戳，微秒 |

关节零位和正方向必须与控制端实际加载的 URDF 及 Robot 映射一致。上层若使用不同顺序或方向，应在生成消息前完成转换。

### 1.3 帧头

| 消息类型 | `frame_header` 固定值 | 方向 |
| --- | --- | --- |
| 状态反馈 | `"0xAA55"` | 控制端 → 上层 |
| 控制指令 | `"0xAA56"` | 上层 → 控制端 |

帧头是 JSON 内的消息标识，不代表在 JSON 前额外发送两个二进制字节，也不是校验和。

### 1.4 时间戳

`timestamp_us` 使用自 `1970-01-01T00:00:00Z` 起经过的整数微秒数：

```text
timestamp_us = Unix 时间戳（秒）× 1,000,000
```

- 状态反馈填写该帧数据的采样时间。
- 控制指令填写上层生成该指令的时间。
- 时间戳与时区无关，不要叠加北京时间的 8 小时偏移。
- 如果需要比较两台计算机之间的指令延迟，应先同步两台计算机的时钟。
- 时间戳用于记录和对齐，不是期望执行时刻；本文不定义轨迹排队或定时执行功能。
- 控制周期和位置差分速度仍应使用控制端的单调时钟间隔，不能使用可能受系统校时影响的 Unix 时间戳，也不能使用指令发送间隔代替实际采样间隔。

## 2. 状态反馈 JSON

### 2.1 完整示例

```json
{
  "frame_header": "0xAA55",
  "timestamp_us": 1792216800000000,
  "q": [0.1, -0.2, 0.3, -0.4, 0.2, -0.1, 0.0],
  "dq": [0.01, -0.02, 0.0, 0.03, -0.01, 0.0, 0.02],
  "tau_actual": [1.2, -4.5, 0.8, 3.1, 0.2, -0.3, 0.1],
  "tau_calculated": [1.3, -4.4, 0.7, 3.0, 0.25, -0.28, 0.12],
  "tau_friction": [0.2, -0.3, 0.0, 0.25, -0.05, 0.0, 0.04]
}
```

### 2.2 字段说明

反馈消息的以下字段全部必填。

| 字段 | JSON 类型 | 单位 | 含义 |
| --- | --- | --- | --- |
| `frame_header` | string | — | 固定为 `"0xAA55"` |
| `timestamp_us` | integer | μs | 当前反馈帧的采样时间 |
| `q` | number[7] | rad | 七轴实际位置，来自 `robot.getJointPosition()` |
| `dq` | number[7] | rad/s | 一问一答版本直接读取 `robot.getJointVelocity()`；原 100 Hz 版本使用实际位置差分 |
| `tau_actual` | number[7] | Nm | 七轴实际力矩反馈，经额定力矩换算 |
| `tau_calculated` | number[7] 或 null | Nm | 阻抗模式为限幅前总请求力矩；位置和 stop 模式在一问一答版本中为七轴零向量，原 100 Hz 版本中为 `null` |
| `tau_friction` | number[7] 或 null | Nm | 阻抗模式为实际采用的摩擦补偿，已计入模式和系数；位置和 stop 模式在一问一答版本中为七轴零向量，原 100 Hz 版本中为 `null` |

### 2.3 实际速度 `dq`

一问一答版本直接读取驱动器实际速度，按硬件 ID 映射到协议关节顺序：

```text
dq[j] = robot.getJointVelocity(indices[j])
```

`getJointVelocity()` 已在 Drive 层按 URDF 中的 `vel_per_unit` 将驱动器原始速度反馈换算为 rad/s。状态 JSON、阻抗阻尼、刚体/摩擦补偿、静止判断及速度保护统一使用该速度；入口不再对位置求差分，也不额外滤波。模式切换预置补偿同样使用驱动器实际速度。

原 100 Hz 版本沿用位置差分方法：

```text
dq[j] = (q_current[j] - q_previous[j]) / measured_dt
```

其中 `measured_dt` 是相邻两次位置采样的实际间隔，单位为秒。原 100 Hz 版本没有额外在线低通滤波，也不读取驱动器的 `getJointVelocity()` 作为控制速度。

位置差分仍会放大量化噪声，因此 `dq` 的抖动不一定表示关节发生了同等幅度的真实速度变化。

### 2.4 实际力矩 `tau_actual`

当前 `getJointTorque()` 返回额定力矩千分比计数，反馈到 JSON 前需要换算一次：

```text
tau_actual[j] = robot.getJointTorque(robot_index[j]) * rated_torque[j] / 1000
```

当前七轴额定力矩按关节顺序为：

```text
rated_torque = [48, 48, 40, 40, 7, 7, 7] Nm
```

例如，joint_3 的额定力矩为 40 Nm，反馈计数为 150，则 `tau_actual[2] = 6.0` Nm。

这里使用 `getJointTorque()`，不使用调试输出中的 `getJointLoadTorque()`。接收端拿到的数据已经是 Nm，不要再次乘额定力矩或除以 1000。

### 2.5 计算力矩与摩擦力矩的关系

阻抗模式下，协议中的总请求力矩定义为：

```text
tau_calculated = tau_rigid
               + tau_friction
               + Kp * (q_desired - q_actual)
               - Kd * dq_actual
```

上述乘法按关节逐元素计算。其中：

- `tau_rigid = Y(q, dq, 0) * P * beta`，为辨识刚体模型补偿。
- `tau_friction` 是本周期采用的摩擦补偿，已乘 YAML 系数。
- `tau_calculated` **已经包含** `tau_friction`，上层不能将二者再次相加。
- `tau_calculated` 不是限幅后的最终下发力矩，也不是实际力矩测量值。

TCP 客户端直接对 `computeTorque()` 输出执行力矩限幅及变化率限制。上位机指令的 `tau` 在所有模式下均被忽略，不读取、不存储、不叠加。

反馈与当前实际执行的模式对应，而不是尚在等待切换的指令模式：

| 实际执行模式 | `tau_calculated` | `tau_friction` |
| --- | --- | --- |
| `position` | 七轴零向量（一问一答）/ `null`（原 100 Hz） | 七轴零向量（一问一答）/ `null`（原 100 Hz） |
| `stop`（MoveJ 和到位保持） | 七轴零向量（一问一答）/ `null`（原 100 Hz） | 七轴零向量（一问一答）/ `null`（原 100 Hz） |
| `impedance` | 七轴总请求力矩 | 七轴零向量 |
| `impedance_friction` | 七轴总请求力矩 | 七轴缩放后的摩擦补偿 |
| `free`（仅一问一答） | 刚体补偿 + 0.6 倍摩擦补偿 − 固定阻尼 | 七轴全量摩擦需求的 0.6 倍 |

位置和 stop 模式下的七轴零向量（一问一答版本）或 `null`（原 100 Hz 版本）表示未执行阻抗力矩计算，不表示实际力矩为零；`q`、`dq` 和 `tau_actual` 在所有模式下都正常反馈。

## 3. 控制指令 JSON

### 3.1 完整示例

```json
{
  "frame_header": "0xAA56",
  "timestamp_us": 1792216800000000,
  "mode": "impedance_friction",
  "q": [0.1, -0.2, 0.3, -0.4, 0.2, -0.1, 0.0],
  "tau": [0, 0, 0, 0, 0, 0, 0]
}
```

### 3.2 字段说明

| 字段 | JSON 类型 | 单位 | 必填条件 | 含义 |
| --- | --- | --- | --- | --- |
| `frame_header` | string | — | 始终必填 | 固定为 `"0xAA56"` |
| `timestamp_us` | integer | μs | 始终必填 | 指令生成时间 |
| `mode` | string | — | 始终必填 | 工作模式，必须为下表取值之一；`free` 仅一问一答版本支持 |
| `q` | number[7] | rad | `free` 可省略，其余必填 | 七轴目标位置；阻抗模式下为弹簧参考位置；stop 下为 MoveJ 目标；free 下不用于位置控制 |
| `tau` | 任意合法 JSON 值 | — | 可省略 | 旧消息兼容字段，所有模式均忽略，建议不发送 |

`tau` 不作为控制输入，不检查其关节数组格式。是否携带及其数值均不影响预置、模型补偿或阻抗力矩；消息整体仍须满足合法 JSON、无重复字段及长度/嵌套限制。当前协议没有独立的“直接力矩控制”模式。

除 `free` 外，指令缺少 `q` 时都会被拒绝，不能默认补成七轴零位置，也不能将其当作增量位移。`free` 若携带 `q`，仍校验七轴有限数值及位置限位，但不跟踪该目标。

### 3.3 工作模式

| `mode` | 含义 | `q` 的作用 | `tau` 的作用 | 摩擦补偿 |
| --- | --- | --- | --- | --- |
| `"position"` | 位置控制 | 位置目标 | 忽略，可省略 | 不使用本文的阻抗摩擦补偿 |
| `"impedance"` | 阻抗控制，不加摩擦 | 弹簧参考位置 | 忽略，可省略 | 强制为零 |
| `"impedance_friction"` | 阻抗控制，加摩擦 | 弹簧参考位置 | 忽略，可省略 | 乘 YAML 的 `friction_scale` |
| `"free"`（仅一问一答） | 零刚度、固定阻尼的 CST 零力模式 | 可省略；携带时校验但不跟踪 | 忽略，可省略 | 固定乘 0.6 |
| `"stop"` | 切换 CSP，MoveJ 到目标并保持 | MoveJ 目标，必填 | 忽略，可省略 | 不使用阻抗摩擦补偿 |

`position` 和 `stop` 使用 CSP，两个阻抗模式及 `free` 使用 CST。跨 CSP/CST 切换或进入新的 stop 目标时，先用原控制方式执行 Ruckig 减速，参考运动完成减速且实际速度低于 0.2 rad/s 后再交接。跨驱动模式时先预置当前位置或新模式的补偿力矩，再切换模式，过程中保持使能；10 秒内不能完成减速交接则故障退出。`free` 中无位置恢复力矩，切换到 CSP 时同样等待实际速度满足静止条件。

同属 CST 的两个阻抗模式及 `free` 直接切换参数，不改变驱动器模式，总输出仍经过力矩变化率限制。重复的相同 stop 目标不会重新启动 MoveJ；MoveJ 执行期间不同时调用 servoJ，收到的新目标（含 free）暂存，MoveJ 完成后处理最新一条。stop 到位后继续在 CSP 保持位置，可以再接收普通运动指令。

### 3.4 两种阻抗模式的区别

不加摩擦：

```text
tau_cmd = tau_rigid + Kp * (q_desired - q_actual) - Kd * dq_actual
```

加摩擦：

```text
tau_cmd = tau_rigid + friction_scale * friction_model(q_actual, dq_actual)
        + Kp * (q_desired - q_actual) - Kd * dq_actual
```

这里 `friction_model` 表示模型计算得到的全量摩擦需求；反馈中的 `tau_friction` 则是乘系数之后的分量。例如：

```yaml
friction_scale: 0.5
```

若某关节全量摩擦需求为 2 Nm，则 `impedance_friction` 模式采用 1 Nm，`impedance` 模式采用 0 Nm。

普通阻抗模式的 `friction_scale` 范围为 `[0,1]`，仅在程序启动时读取，修改 YAML 后需要重启。`impedance_friction` 在 YAML 系数为 0 时也不会加入摩擦补偿；`free` 独立使用固定的 0.6，不受该 YAML 系数影响。

普通阻抗模式的 Kp、Kd 及各模式共用的额定力矩、速度保护阈值、力矩限制和变化率限制仍由 YAML 配置，不放入本协议指令中。`free` 的增益与摩擦比例使用下述固定值。

### 3.5 零力模式 free（仅一问一答版本）

```yaml
kp: [0, 0, 0, 0, 0, 0, 0]
kd: [4, 4, 4, 3, 2, 1.5, 1]
friction_scale: 0.6
```

```text
tau_cmd = tau_rigid(q_actual, dq_actual)
        + 0.6 * friction_model(q_actual, dq_actual)
        - diag([4,4,4,3,2,1.5,1]) * dq_actual
```

不产生位置弹簧力矩，因此不会拉回进入模式时的位置或远端 `q`。零力在此表示零刚度及模型补偿，不表示向驱动器发送零力矩；刚体补偿、摩擦补偿和速度阻尼仍存在。实测速度来自 `getJointVelocity()`。输出仍受 YAML 力矩幅值、变化率、速度及位置保护约束。

启动时单独初始化 free 控制器，模型与保护配置来自同一 YAML，增益及摩擦比例固定覆盖；进入或退出 free 不修改普通阻抗参数。free 运行时规划参考随实际姿态更新，退出后从当前实测姿态接续位置参考。指令超时后继续运行 free；每条有效 free 指令仍只触发一帧最新状态回复。

回复是最新已采样状态，不是模式切换完成确认；进入 free 的即时回复可能仍包含上一模式的状态快照。

## 4. 指令示例

### 4.1 位置模式

```json
{
  "frame_header": "0xAA56",
  "timestamp_us": 1792216800000000,
  "mode": "position",
  "q": [0.1, -0.2, 0.3, -0.4, 0.2, -0.1, 0.0],
  "tau": [0, 0, 0, 0, 0, 0, 0]
}
```

上层提供目标位置，`tau` 不参与位置控制。指令本身不包含速度、加速度或运动时长，目标点之间的插值、连续性和限速应由上层轨迹或控制端运动接口处理。

### 4.2 阻抗模式，不加摩擦

```json
{
  "frame_header": "0xAA56",
  "timestamp_us": 1792216800001000,
  "mode": "impedance",
  "q": [0.1, -0.2, 0.3, -0.4, 0.2, -0.1, 0.0],
  "tau": [0, 0, 0, 0, 0, 0, 0]
}
```

采用刚体补偿、位置弹簧和速度阻尼；摩擦补偿为零，指令 tau 被忽略。

### 4.3 阻抗模式，加摩擦

```json
{
  "frame_header": "0xAA56",
  "timestamp_us": 1792216800002000,
  "mode": "impedance_friction",
  "q": [0.1, -0.2, 0.3, -0.4, 0.2, -0.1, 0.0],
  "tau": [0, 0, 0, 0, 0, 0, 0]
}
```

在上一种模式的基础上，叠加乘过 `friction_scale` 的摩擦补偿。

### 4.4 阻抗模式，兼容旧消息中的 tau

```json
{
  "frame_header": "0xAA56",
  "timestamp_us": 1792216800003000,
  "mode": "impedance_friction",
  "q": [0.1, -0.2, 0.3, -0.4, 0.2, -0.1, 0.0],
  "tau": [0.1, -0.2, 0, 0, 0, 0, 0]
}
```

这里的 `tau` 完全被忽略。该指令与移除 `tau` 字段后的指令具有相同控制效果；底层只使用模型补偿、位置弹簧、速度阻尼以及所选模式的摩擦补偿。

### 4.5 stop 模式：省略 tau

```json
{
  "frame_header": "0xAA56",
  "timestamp_us": 1792216800004000,
  "mode": "stop",
  "q": [0.1, -0.2, 0.3, -0.4, 0.2, -0.1, 0.0]
}
```

该指令退出在线目标跟踪或阻抗控制，在 CSP 下通过 MoveJ 运动到 `q`，到位后保持。它是一个位置运动请求，不是紧急停机或下使能指令；断线、超时及速度故障才走故障下使能退出流程。

### 4.6 停止模式：统一字段格式

如果上层固定发送相同字段，可以使用：

```json
{
  "frame_header": "0xAA56",
  "timestamp_us": 1792216800004000,
  "mode": "stop",
  "q": [0.1, -0.2, 0.3, -0.4, 0.2, -0.1, 0.0],
  "tau": [0, 0, 0, 0, 0, 0, 0]
}
```

`q` 是实际执行的 MoveJ 目标；`tau` 被忽略。若把 `q` 填为七轴零向量，就表示明确请求 MoveJ 回到零位置。

### 4.7 free 零力模式（仅一问一答版本）

```json
{
  "frame_header": "0xAA56",
  "timestamp_us": 1792216800005000,
  "mode": "free"
}
```

收到后进入固定零刚度、固定阻尼和 0.6 倍摩擦补偿的 CST 模式，回复一帧实际状态。也可携带合法的七轴 `q`，但 free 不产生到该位置的恢复力矩。

## 5. 单关节力矩计算示例

下面仅演示公式，系数不代表当前 YAML 的实际设置：

| 项目 | 示例值 |
| --- | --- |
| 刚体补偿 | 2.0 Nm |
| 全量摩擦需求 | 0.5 Nm |
| `friction_scale` | 0.5 |
| 位置误差 `q_desired - q_actual` | 0.02 rad |
| Kp | 20 Nm/rad |
| 实际速度 | 0.03 rad/s |
| Kd | 4 Nm·s/rad |
| 指令 `tau` | 0.1（忽略，不参与计算） |

加摩擦模式：

```text
tau_friction   = 0.5 × 0.5 = 0.25 Nm
tau_calculated = 2.0 + 0.25 + 20 × 0.02 - 4 × 0.03
               = 2.53 Nm
```

不加摩擦模式：

```text
tau_friction   = 0 Nm
tau_calculated = 2.0 + 20 × 0.02 - 4 × 0.03
               = 2.28 Nm
```

实际下发值可能因力矩限幅和变化率限制而与上述请求值不同；实际测量的 `tau_actual` 也不要求等于请求值。

## 6. 接收检查与消息边界

### 6.1 字段检查

接收端应检查：

1. 消息能解析为 JSON 对象，且没有重复字段。
2. 帧头与消息方向匹配，时间戳为非负整数。
3. 控制指令的 `mode` 属于上述取值之一；原 100 Hz 版本支持四种，一问一答版本额外支持 free，不将未知模式默认解释成运动模式。
4. 原四种指令都包含长度为 7 的有限数值数组 `q`；free 可省略 q，若提供仍须校验；兼容字段 `tau` 不读取、不校验其数组格式。
5. 模式使用的目标位置满足当前 URDF 限位；控制器计算的力矩统一经过幅值和变化率限制。

非法消息被拒绝且不刷新指令超时计时；控制端继续执行上一有效指令。原 100 Hz 版本在首条有效指令到达后，默认连续 1 秒没有新的有效指令即故障下使能退出，阈值由 `--command_timeout` 设置；反馈发送持续阻塞 1 秒也会故障退出。一问一答版本的指令及发送超时仅告警并继续运行（含 free），实际 TCP 断开仍故障退出，不自动重连或重新使能。时间戳只用于记录，超时依据本地单调时钟的接收时间；一问一答版本每条有效指令回复一帧状态，不依据发送端时间戳拒绝指令。

`TCP feedback send timeout` 表示 TCP 已连接，但一帧反馈在 1 秒内未能写完。优先检查上位机是否在**同一个已接受的连接**上持续 `recv()`：仅 `accept()`、只发送控制指令，或等整条连接关闭后才读取，都会使接收缓冲区逐渐填满并阻塞客户端发送。也需排查网络丢包或服务端卡顿；不能仅凭此报错确定是哪一种。反馈一行一个 JSON、以实际换行字节 `0x0A` 结束，服务端应边接收边按行拆帧，不等待 EOF。该报错会附带当前帧剩余字节、等待时间和已写入 socket 的帧数/字节数；写入成功不代表服务端应用已读取。`--command_timeout` 只影响接收指令超时，不影响这个发送保护。

例如，上位机地址为 `192.168.10.52`、端口为 `6666` 时，可先停止原来占用该端口的程序，再在上位机运行仓库测试服务端进行对照：

```bash
python3 tools/joint_tcp_test_server.py --tcp-host=0.0.0.0 --tcp-port=6666
```

客户端在仓库根目录运行（显式仿真，不连接真机硬件）：

```bash
./build/bin/jointTcpClientMain --config=config/identified_impedance.yaml \
  --host=192.168.10.52 --port=6666 --sim=true
```

不必启动运动即可验证反馈：连接后测试服务端持续读取状态，上位机本地网页 `http://127.0.0.1:8000` 应显示约 100 Hz 接收频率。若该服务端收发正常而原服务端超时，重点修正原服务端的接收循环；若仍超时，继续检查两端网络和服务端负载。默认 `--sim=true`；真机模式需显式传 `--nosim` 或 `--sim=false`。

### 6.2 传输边界

本客户端采用 TCP，帧头本身不能解决 TCP 拆包和粘包问题。

使用一行一个完整 JSON 对象、以换行符结束的 NDJSON 格式，支持分包、粘包和部分发送；单条指令最多 16 KiB。本文的多行示例用于阅读，发送时需压缩到单行，例如：

```text
{"frame_header":"0xAA56","timestamp_us":1792216800004000,"mode":"stop","q":[0.1,-0.2,0.3,-0.4,0.2,-0.1,0]}\n
```

上面末尾的 `\n` 表示一个实际换行字节 `0x0A`，不是发送反斜杠和字母 n 两个字符。接收端应按完整行组帧，不能假定一次 `recv()` 恰好得到一条完整消息。

## 7. 对应代码与当前边界

| 内容 | 对应位置 |
| --- | --- |
| 刚体补偿、摩擦补偿、PD 力矩计算 | [src/joint_impedance.cc](../src/joint_impedance.cc) |
| TCP 客户端、JSON、Ruckig 插值和四种模式 | [src/jointTcpClientMain.cc](../src/jointTcpClientMain.cc) |
| 位置差分、CST 启动、力矩限制与下发 | [src/jointImpedanceMain.cc](../src/jointImpedanceMain.cc) |
| 增益、额定力矩、摩擦比例及保护阈值 | [config/identified_impedance.yaml](../config/identified_impedance.yaml) |
| 模型适配和运行指令 | [docs/dynamics-id.md](dynamics-id.md) |

## 8. 编译与运行

在仓库根目录执行：

```bash
cmake --build build --target jointTcpClientMain -j2

# 仅检查模型和基础参数，不连接 TCP 或硬件。
./build/bin/jointTcpClientMain --config=config/identified_impedance.yaml --check_only

# 上位机须先启动 TCP 服务端，以下为连接本机 9000 端口的仿真示例。
./build/bin/jointTcpClientMain --config=config/identified_impedance.yaml \
  --host=127.0.0.1 --port=9000 --sim

# 真机：将地址替换为实际 TCP 服务端 IP。
sudo ./build/bin/jointTcpClientMain --config=config/identified_impedance.yaml \
  --host=192.168.1.100 --port=9000 --nosim
```

启动后在首条有效指令到来前保持下使能，已能发送实际状态；首条指令到来后才选择驱动模式并上使能。Ctrl+C 或进程故障会下使能退出，与普通 stop 指令不同。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--host` | `127.0.0.1` | 服务端主机名或 IP |
| `--port` | `9000` | 服务端端口 |
| `--config` | `config/identified_impedance.yaml` | 辨识模型、增益、摩擦比例、额定力矩及保护配置 |
| `--command_timeout` | `1.0` | 首条有效指令之后的接收超时，秒；原 100 Hz 版本超时退出，一问一答版本仅告警（含 free） |
| `--movej_speed` | `0.15` | stop MoveJ 的最大关节速度，rad/s |
| `--movej_acceleration` | `0.3` | stop MoveJ 的最大关节加速度，rad/s² |
| `--sim` / `--nosim` | 默认仿真 | HardwareSim / 真机 |
| `--base` / `--tip` | `base_link` / `link_7` | Robot 运动链 |
| `--id` | `0` | 真机硬件实例编号 |

Ruckig 的速度上限取 YAML `velocity_limit` 和硬件速度限制的较小值，加速度和 jerk 取 URDF 的 hardware 限制。每个新目标的终端速度、加速度为零；同一模式内更换目标时保留当前插值位置、速度和加速度，不重新从静止起步。目标不保证恰好在 100 ms 内到达，大跨度目标可能需要更长时间；接收邮箱只保留最新目标，不排队积压轨迹。

位置和两个阻抗模式均使用 Ruckig 生成的参考位置；free 无位置目标，规划参考随实际姿态更新，退出后从当前姿态接续规划。阻抗和 free 控制反馈项使用真实位置，以及一问一答版本的驱动器速度或原 100 Hz 版本的位置差分速度。上位机指令 `tau` 始终被忽略，模型与阻抗控制输出经过力矩变化率限制。stop 的轨迹由 Robot 的 MoveJ 自行规划。

CSP 与 CST 之间在线切换时，先在原模式减速至静止，整个交接过程不下使能。进入 CSP 前先把实际关节角写入目标位置，设置 CSP 时同步 Robot 内部规划起点；进入 CST 前以当前实际姿态计算补偿（`qd=q`；一问一答版本读取驱动器实际速度，原 100 Hz 版本预置时使用 `dq=0`），增益及摩擦补偿按新模式选择，通过限值检查后预置，不叠加远端目标的位置误差。切换后，位置目标经 Ruckig 插值，free 则随实际姿态更新参考；上位机指令 `tau` 在预置和正常运行时都不使用。七轴目标全部写入并等待一个总线周期后，才设置新模式；新模式写入后再经过总线同步。仅首次启动在预置及设置模式之后调用上使能；故障和退出仍下使能。CST 预置使用 `setJointTorque()`，因为向量接口 `setTorque()` 要求已经处于 CST；切换后使用 `setTorque()` 写入相同初值，运行期间也继续使用该接口。两个阻抗模式及 free 之间切换不改变驱动器的 CST 模式。

1 ms 是参考轨迹的算法步长，实际运行通过 servoJ/setTorque 内的总线同步推进，不额外 sleep。100 Hz 反馈由网络线程按单调时钟调度；驱动模式切换期间可能重复最近一次快照，采样时间戳保持原值。操作系统调度、总线和网络会影响实际周期，不能视为硬实时保证。HardwareSim 只能验证接口和流程，不模拟真实力矩动力学。

离线测试：

```bash
cmake --build build --target joint_tcp_protocol_test -j2
./build/bin/joint_tcp_protocol_test
python3 test/joint_tcp_client_test.py \
  --binary=build/bin/jointTcpClientMain --config=config/identified_impedance.yaml
```

## 9. Python 服务端与网页测试

[tools/joint_tcp_test_server.py](../tools/joint_tcp_test_server.py) 使用 Python 标准库提供 TCP 上位机和本地网页，无需 pip 安装。网页文件为 [tools/joint_tcp_test.html](../tools/joint_tcp_test.html)。

在仓库根目录，终端 1 启动服务端：

```bash
python3 tools/joint_tcp_test_server.py
```

终端 2 启动仿真客户端：

```bash
./build/bin/jointTcpClientMain --config=config/identified_impedance.yaml \
  --host=127.0.0.1 --port=9000 --sim
```

浏览器打开 **http://127.0.0.1:8000**。等待显示“客户端已连接”，选择 `position`，点击“开始 / 应用正弦”。连接本身不会发送运动指令。

- 仅第 4 关节 `joint_4`（数组下标 3）执行 `q₄ = sin(2πt/T + φ)`，绝对角度范围为 **−1～1 rad**，其余六轴保持首次启动时的反馈位置。
- 默认周期 `T = 10 s`，页面可设 4～120 s，点击“开始 / 应用正弦”生效。起始相位由现有目标位置确定，首次启动取实际位置；joint_4 起始角度超出 −1～1 rad 时拒绝启动。
- 默认周期下目标峰值速度约 0.628 rad/s、峰值加速度约 0.395 rad/s²。实际执行还受客户端 Ruckig 和 YAML/URDF 限制，不保证精确复现每个正弦目标点。
- “暂停正弦，保持目标”冻结最近发送的目标，在当前模式下持续发送 10 Hz 指令。点击开始可继续。修改模式也需点击开始才生效。
- “stop · CSP 停靠当前姿态”抓取点击时实际七轴位置，持续发送同一条 `stop` 目标；客户端切 CSP 并用 MoveJ 停靠，不会下使能。
- “断开连接”关闭 TCP，客户端按断线故障下使能并退出。重新启动 C++ 客户端后，页面等待手动开始，不沿用旧目标。

两个阻抗模式也发送同一正弦目标，不发送已忽略的 `tau` 字段。阻抗模式的位置恢复力矩取决于 YAML 的 `kp`；设为 0 时，目标角度变化不会产生位置恢复力矩，需要跟随测试时按实际机器人设置增益。HardwareSim 不模拟力矩动力学，因此只在 `position` / `stop` 中能验证位置跟随。

后台独立以 10 Hz 发送指令并接收约 100 Hz 状态，页面每 100 ms 刷新；浏览器标签页置于后台不影响 Python 发送。页面显示最近一秒实际收发帧数、七轴状态、第四轴角度曲线及最近的完整 JSON。接收频率统计按到达帧数计算，切换驱动模式期间的重复快照也计入。

如上位机与客户端不在同一台电脑，在上位机执行：

```bash
python3 tools/joint_tcp_test_server.py --tcp-host=0.0.0.0 --tcp-port=9000
# 客户端的 --host 改为这台上位机的 IP；网页仍在上位机本地 8000 端口。
```

真机客户端使用 `--nosim`；网页自身不判断关节限位和力矩安全范围，仍由 C++ 客户端保护。启动 Python 后停止它或按 Ctrl+C 会断开客户端。`--web-host` / `--web-port` 可修改网页监听地址/端口，默认只允许本机访问网页。

服务端与 C++ 客户端的纯仿真联调测试（覆盖正弦全周期、四模式、频率、暂停心跳和重新连接）：

```bash
python3 test/joint_tcp_test_server_test.py \
  --binary=build/bin/jointTcpClientMain --config=config/identified_impedance.yaml
```
