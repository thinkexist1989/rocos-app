# 3D 模型配置规范

本地加载和 gRPC 下载后的模型均由同一个 YAML 解析器处理。参考配置：[`models/talon/config.yaml`](../models/talon/config.yaml)。

## 基本结构与兼容性

```yaml
robot:
  - name: base
    type: fixed
    mesh: base_link.STL
  - name: joint1
    type: revolute
    translate: [0, 0, 0.348]
    angleAxis: [0, 0, 1]
    mesh: link_1.STL
```

`robot` 必须是非空列表。每一项代表一个关节坐标系及其关联的可视网格，包括基座、固定连接件和工具。模型项数不限于 7，**主动关节最多 7 个**；其余项必须是固定关节、mimic 关节或显式设置 `active: false` 的被动关节。

沿用现有的**串联链**结构：列表第一项相对于世界坐标系，之后每一项相对于前一项的关节坐标系。按列表顺序计算变换，不根据 `order` 排序。不支持树状分支、多自由度 floating/planar 关节或闭链约束求解。

为兼容仓库旧配置，省略 `type` 时，列表第一项按 `fixed` 处理，其余项按 `revolute` 处理。新配置建议显式写出每项的 `type`，尤其是中间和末端的固定关节。旧的 `unknown` 类型按 `fixed` 处理。

配置不支持 `${...}` 表达式求值，网格读取器仅支持 STL。仓库各模型的配置（包括 IIWA 备份）均已显式声明固定基座和旋转关节。

UR3 的数值、关节原点和网格原点依据 [ROS-Industrial UR3 定义](https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_description/urdf/ur3.urdf.xacro) 修正：移除误用的惯性原点偏移，腕部第二关节的静态旋转为零。其 STL 由同目录原有 DAE 转换，保留米制坐标和几何变换；由于原有 `upperarm.dae` 被截断，`upperarm.stl` 使用[同版本上游完整网格](https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_description/meshes/ur3/visual/upperarm.dae)转换。原 DAE 仅作为历史资料保留，不再被配置引用。

## 字段

| 字段 | 类型／默认值 | 含义 |
| --- | --- | --- |
| `name` | 必填字符串 | 本配置内唯一且非空的名称，也是 `mimic.joint` 的引用目标。 |
| `order` | 可选整数，默认列表下标 | 保留的描述字段，不决定变换顺序或状态索引。 |
| `type` | 字符串，兼容默认规则见上文 | `fixed`、`revolute`、`continuous`、`prismatic`；兼容 `unknown`。 |
| `active` | 布尔值 | 普通运动关节默认 `true`；固定和 mimic 关节默认 `false`，且不能设为 `true`。 |
| `position` | 数值，默认 `0` | 仅用于 `active: false` 且无 mimic 的运动关节，表示恒定的被动位置。 |
| `mimic` | 可选映射 | 从另一关节计算位置，见下文。 |
| `translate` | 三个数，默认 `[0, 0, 0]` | 关节原点相对于前一关节坐标系的平移，单位 m。 |
| `rotate` | 三个数，默认 `[0, 0, 0]` | 关节原点的 `[roll, pitch, yaw]`，单位 rad。 |
| `angleAxis` | 三个数，默认 `[0, 0, 1]` | 关节局部坐标系中的旋转轴或平移轴，允许小数和负数；运动关节的轴必须非零，解析时自动归一化。 |
| `translateLink` | 三个数，默认 `[0, 0, 0]` | STL 网格相对于当前关节坐标系的平移，单位 m。 |
| `rotateLink` | 三个数，默认 `[0, 0, 0]` | STL 网格相对于当前关节坐标系的 `[roll, pitch, yaw]`，单位 rad。 |
| `mesh` | 可选字符串 | STL 文件路径，相对于配置文件所在目录；也可使用绝对路径。省略、空字符串或 `null` 表示无网格。STL 坐标应使用 m。 |

所有变换、轴、位置及 mimic 系数必须是有限数值。`position` 不能与主动关节、固定关节或 mimic 混用，以免出现两个位置来源。

## 状态映射和关节运动

输入状态只包含主动关节，按照它们在 `robot` 中的出现顺序分配下标，从 `0` 开始。固定、被动和 mimic 关节均不消耗下标。例如：

```text
robot: 基座(fixed), J1(revolute), 转接件(fixed), J2(prismatic), 联动件(mimic)
输入:  [J1 的角度(rad), J2 的位移(m)]
```

- `revolute` / `continuous`：沿归一化后的 `angleAxis` 旋转，状态单位 rad。
- `prismatic`：沿归一化后的 `angleAxis` 平移，状态单位 m。
- `fixed` / `unknown`：只应用配置中的静态变换，不应用关节运动。
- `active: false` 且无 mimic：使用 `position`，不会从实时状态中读取位置，也不会自动求解受力或闭链运动。

加载时主动位置初始化为 `0`，同时计算被动位置和 mimic；无需等待服务器状态即可显示初始模型。更新时，若输入长度小于主动关节数或所需值包含 NaN/Inf，整次更新被忽略，保留上一姿态；多余的输入值被忽略。

设 `R(rpy) = Rz(yaw) * Ry(pitch) * Rx(roll)`，每项的变换为：

```text
T_joint = T_previous_joint * Translate(translate) * R(rotate) * Motion(q)
T_mesh  = T_joint * Translate(translateLink) * R(rotateLink)
```

其中 `Motion(q)` 对旋转关节为轴角旋转，对直线关节为 `Translate(angleAxis * q)`，对固定关节为单位变换。网格偏移不传递给后续关节。轨迹记录列表最后一项的关节坐标系原点。

## mimic 联动

```yaml
mimic:
  joint: joint1
  multiplier: -1.0
  offset: 0.2
```

位置公式：`q = multiplier * q_source + offset`。

- `joint` 必填，引用同一配置中的 `name`，源必须是运动关节。
- `multiplier` 默认 `1.0`，`offset` 默认 `0.0`。
- mimic 项保留实际运动类型，如 `revolute` 或 `prismatic`；没有名为 `mimic` 的 `type`。
- 支持引用后面的项以及多级 mimic，按依赖顺序计算。未知名称、自引用和循环依赖会拒绝加载。
- `offset` 使用目标关节单位（rad 或 m）；`multiplier` 表示目标单位与源单位之比。例如旋转驱动直线运动时单位为 m/rad，不做额外的单位换算。
- mimic 只描述数值依赖，不改变串联链的几何连接关系。

## 超过 7 个关节的完整示例

以下有 13 个模型项、7 个主动关节，其余为固定或被动关节。示例未指定网格，可通过显示坐标轴检查位置；实际使用时添加对应 `mesh`。

```yaml
robot:
  - name: base
    type: fixed
  - name: mount
    type: fixed
    translate: [0, 0, 0.1]
  - name: joint1
    type: revolute
  - name: joint2
    type: revolute
    translate: [0, 0, 0.3]
    angleAxis: [0, 1, 0]
  - name: spacer
    type: fixed
    translate: [0, 0, 0.1]
  - name: joint3
    type: revolute
    angleAxis: [0, 1, 0]
  - name: joint4
    type: revolute
  - name: joint5
    type: revolute
    angleAxis: [0, 1, 0]
  - name: joint6
    type: continuous
  - name: slide
    type: prismatic
    angleAxis: [1, 0, 0]
  - name: linked_slide
    type: prismatic
    angleAxis: [1, 0, 0]
    mimic:
      joint: slide
      multiplier: -0.5
      offset: 0.02
  - name: passive_hinge
    type: revolute
    active: false
    position: 0.1
  - name: tool
    type: fixed
    translate: [0, 0, 0.1]
```

对应输入为 `[joint1, joint2, joint3, joint4, joint5, joint6, slide]`，前六项单位 rad，最后一项单位 m。

## gRPC 模型描述

`protos/robot_info.proto` 的 `Link` 保留原有字段和编号，追加 `active`（10）、`position`（11）、`mimic`（12）。其中 `active` 和 `position` 使用 optional 保留“未指定”的语义；`JointMimic.multiplier` 同样使用 optional，以区分默认倍率 `1` 和显式倍率 `0`。下载器会将这些字段写入 YAML，再使用同一解析器加载。

旧服务器不发送新增字段时，客户端沿用默认规则：固定关节不占输入，普通运动关节占一个输入。要从服务器下载被动／mimic 模型，服务端需要更新对应 proto，并填充这些信息；客户端无法从网格或普通关节描述推断联动关系。也可以使用本地 YAML 配置这些信息。

配置结构错误、重复名称、不支持的类型、无效轴、非法 mimic 或超过 7 个主动关节时，界面显示加载错误，保留此前已加载的模型。
