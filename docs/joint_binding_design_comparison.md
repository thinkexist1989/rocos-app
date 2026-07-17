# JointBinding 绑定方案对比

## 背景

URDF/KDL chain 中的 joint 是控制算法使用的逻辑关节顺序，hardware config 中的 drive id 是真实硬件轴编号。当前 `Hardware::GetPosition()` / `SetPosition()` 等批量接口隐含使用 `config_.drives` 顺序：

```cpp
q(i) <-> config_.drives[i].id
```

这会把 URDF joint 顺序和 hardware 配置顺序隐式绑定。一旦 URDF joint 顺序、硬件 drive id、配置文件顺序不一致，反馈、控制、状态上报和可视化都会出现错轴风险。

目标是引入显式绑定：

```text
model joint name / model index <-> hardware drive id
```

基本规则：

- URDF 控制 joint 数量大于 hardware drive 数量时必须报错。
- URDF 控制 joint 数量小于或等于 hardware drive 数量时允许，但每个控制 joint 必须绑定一个真实 drive id。
- 绑定必须支持 YAML 加载和函数运行时修改。
- 运动中不允许修改绑定。
- 反馈链路和下发链路必须使用同一套绑定规则。

## 共同模块：JointBinding

两个方案都建议保留一个独立的 `JointBinding`，负责语义层面的绑定和校验，不直接读写硬件。

建议职责：

- 从 YAML 读取 `joint_name -> drive_id`。
- 支持函数修改绑定。
- 根据 model joint names 生成 `model_index -> drive_id` 映射。
- 校验 joint 是否存在、drive 是否存在、是否重复绑定、数量是否匹配。
- 保存 YAML。

建议 YAML：

```yaml
binding_policy: explicit

joint_bindings:
  - joint_name: joint_1
    drive_id: 2
  - joint_name: joint_2
    drive_id: 0
  - joint_name: joint_3
    drive_id: 3
```

建议新增文件：

```text
src/joint_binding.hpp
src/joint_binding.cpp
```

预计代码量：

```text
JointBinding header: 80-120 行
JointBinding implementation: 220-350 行
YAML 示例配置: 20-60 行
单元测试: 150-250 行
```

## 方案一：JointBinding + Hardware 内部映射表

### 思路

`JointBinding` 生成最终的：

```cpp
std::vector<int> model_index_to_drive_id;
```

然后传给 `Hardware`：

```cpp
Result Hardware::SetJointBinding(const std::vector<int>& model_index_to_drive_id);
```

`Hardware` 不理解 URDF，也不处理 joint name，只保存这张映射表。所有批量接口从原来的 `config_.drives[i].id` 改为：

```cpp
DriveIdByModelIndex(i)
```

### 数据流

反馈：

```text
raw drive feedback -> Hardware 根据 model_index_to_drive_id 重排 -> q_model
```

下发：

```text
q_model -> Hardware 根据 model_index_to_drive_id 找 drive_id -> raw drive command
```

### 需要修改的地方

`src/hardware.hpp`

- 增加绑定映射成员：

```cpp
std::vector<int32_t> model_index_to_drive_id_;
```

- 增加接口：

```cpp
Result SetJointBinding(const std::vector<int32_t>& model_index_to_drive_id);
void ClearJointBinding();
std::vector<int32_t> GetJointBinding() const;
int32_t DriveIdByModelIndex(size_t model_index) const;
```

`src/hardware.cpp`

需要统一改这些批量接口：

```cpp
GetPosition()
GetVelocity()
GetTorque()
GetLoadTorque()
SetPosition()
SetVelocity()
SetTorque()
```

当前相关位置集中在 `src/hardware.cpp` 约 393-439 行。

需要新增校验：

- binding 不能为空。
- drive id 必须存在于 `config_.drives`。
- drive id 不能重复。
- binding 数量不能大于 hardware drive 数量。

`src/robot.cpp`

- 初始化时创建 `JointBinding`。
- 从 Model 取得 model joint names。
- 从 Hardware 取得真实 drive ids。
- 加载 YAML。
- 调用 `hardware->SetJointBinding(binding.GetModelIndexToDriveIds())`。
- 上位机运行时修改绑定后，重新调用 `SetJointBinding()`。

`src/robot.hpp`

- 增加 `JointBinding` 成员。
- 增加绑定管理 API：

```cpp
Result LoadJointBinding(const std::string& path);
Result SaveJointBinding(const std::string& path) const;
Result BindJointToDrive(const std::string& joint_name, int32_t drive_id);
```

`CMakeLists.txt`

- `robot` target 增加 `src/joint_binding.cpp`。

### 预计代码量

```text
新增 JointBinding: 300-470 行
修改 Hardware: 120-220 行
修改 Robot 初始化和 API: 80-160 行
修改 CMake: 1-3 行
测试: 180-320 行
总计: 680-1170 行
```

### 优点

- 改动比适配器方案小。
- Controller / Executor 基本不用动。
- Robot 仍可只持有一个主要 `hardware` 指针。
- 所有现有 `hardware->GetPosition()` / `SetPosition()` 调用天然变为 model order。

### 缺点

- `Hardware` 承担了 model index 映射职责，硬件层概念变重。
- raw drive order 和 model order 混在同一个对象中，调试时需要额外 raw 接口。
- 如果未来有多个硬件后端，每个后端都需要类似映射逻辑。
- 如果只改部分 Get/Set，会出现反馈和下发顺序不一致的高风险问题。

### 适用场景

- 当前优先低成本上线。
- 硬件后端短期只有一个 `Hardware` 实现。
- 希望尽量少改 Robot / Controller / Executor。

## 方案二：JointBinding + BoundHardwareInterface

### 思路

保留 raw `Hardware`，新增一个实现 `HardwareInterface` 的适配器：

```cpp
class BoundHardwareInterface : public HardwareInterface {
 public:
  BoundHardwareInterface(HardwareInterface* raw_hardware, JointBinding* binding);
};
```

Robot 内持有两个硬件视角：

```text
raw_hardware: 真实硬件，drive id / raw order
hardware: bound 硬件，model joint order，给 controller / executor 使用
```

Controller 和 Executor 仍然接收 `HardwareInterface*`，但传入的是 `BoundHardwareInterface`。

### 数据流

反馈：

```text
raw Hardware drive feedback
  -> BoundHardwareInterface 根据 JointBinding 重排
  -> q_model
  -> Controller / Model / Robot state / HTTP / 可视化
```

下发：

```text
Controller 输出 q_model
  -> BoundHardwareInterface 根据 JointBinding 转 drive_id
  -> raw Hardware 下发真实 drive command
```

### 需要修改的地方

新增文件：

```text
src/joint_binding.hpp
src/joint_binding.cpp
src/bound_hardware_interface.hpp
src/bound_hardware_interface.cpp
```

`src/bound_hardware_interface.hpp/cpp`

至少覆盖这些接口：

```cpp
GetPosition()
GetVelocity()
GetTorque()
GetLoadTorque()
SetPosition()
SetVelocity()
SetTorque()
getJointName()
Reset()
WaitForSignal()
GetDt()
SetEnabled()
SetDisabled()
Is/状态相关接口按现有 HardwareInterface/DriveInterface 转发
```

批量 Get/Set 使用 `JointBinding` 重排；硬件状态、使能、周期等待等直接转发给 raw hardware。

`src/robot.hpp`

建议保留现有 `hardware` 作为 bound/model order 视角，新增 raw 成员：

```cpp
std::unique_ptr<HardwareInterface> raw_hardware;
std::unique_ptr<JointBinding> joint_binding;
std::unique_ptr<HardwareInterface> hardware; // BoundHardwareInterface
```

或者命名更清晰：

```cpp
std::unique_ptr<HardwareInterface> raw_hardware_;
std::unique_ptr<HardwareInterface> hardware_; // bound/model order
```

`src/robot.cpp`

当前构造逻辑：

```cpp
hardware = std::make_unique<Hardware>("hardware_talon_config.yaml", 0);
controller->SetHardware(hardware.get());
executor = std::make_unique<Executor>(motion.get(), controller.get(), hardware.get());
```

改为：

```cpp
raw_hardware = std::make_unique<Hardware>("hardware_talon_config.yaml", 0);
joint_binding = std::make_unique<JointBinding>();
joint_binding->Configure(...);
joint_binding->LoadFromYaml(...);
joint_binding->Validate();

hardware = std::make_unique<BoundHardwareInterface>(
    raw_hardware.get(),
    joint_binding.get());

controller->SetHardware(hardware.get());
executor = std::make_unique<Executor>(motion.get(), controller.get(), hardware.get());
```

Robot 中控制链路继续使用 `hardware`。底层调试、drive 原始状态、EtherCAT 诊断使用 `raw_hardware`。

`CMakeLists.txt`

- `robot` target 增加：

```text
src/joint_binding.cpp
src/bound_hardware_interface.cpp
```

### 预计代码量

```text
新增 JointBinding: 300-470 行
新增 BoundHardwareInterface: 180-320 行
修改 Robot 初始化和成员: 80-180 行
修改 CMake: 2-4 行
测试: 250-450 行
总计: 812-1424 行
```

### 优点

- 架构边界最清楚。
- raw hardware 仍保持 drive id / drive order，不被 model 概念污染。
- Controller / Executor 不需要理解绑定，只继续使用 `HardwareInterface`。
- 未来仿真硬件、真实硬件、不同品牌硬件都能复用同一个 bound adapter。
- 可同时支持可视化 model joint state 和底层 raw drive debug。

### 缺点

- 新增类更多。
- Robot 会同时持有 raw 和 bound 两个硬件视角，需要命名和所有权设计清楚。
- `BoundHardwareInterface` 需要转发较多 `HardwareInterface` / `DriveInterface` 方法，初期代码量高于方案一。

### 适用场景

- 希望长期架构稳定。
- 后续可能有仿真、不同真实硬件、多机器人型号。
- 需要清晰区分 model joint state 和 raw drive state。
- 上位机既要看机器人关节，也要看真实驱动器诊断信息。

## 综合对比

| 维度 | 方案一：Hardware 内部映射表 | 方案二：BoundHardwareInterface |
| --- | --- | --- |
| 核心思路 | Hardware 保存 `model_index -> drive_id` | Adapter 包装 raw Hardware |
| Controller/Executor 改动 | 很小 | 很小 |
| Robot 改动 | 小 | 中等 |
| Hardware 改动 | 中等，直接修改批量 Get/Set | 小，raw Hardware 基本不动 |
| 新增文件 | 少 | 多 |
| raw drive 调试 | 容易混淆，需要额外接口 | 清晰，raw_hardware 专门负责 |
| 架构边界 | 中等，Hardware 带 model index 概念 | 清晰，职责分离 |
| 多硬件后端复用 | 较差，各后端可能重复做映射 | 好，adapter 可复用 |
| 短期上线成本 | 低 | 中 |
| 长期维护成本 | 中高 | 低 |
| 错轴风险控制 | 取决于是否完整改全部 Get/Set | 更集中，更安全 |
| 预计总代码量 | 680-1170 行 | 812-1424 行 |

## 推荐结论

如果目标是低成本快速落地，可以先采用方案一：

```text
JointBinding + Hardware::SetJointBinding(vector<int>)
```

但必须保证所有批量反馈和下发接口统一使用映射：

```text
GetPosition / GetVelocity / GetTorque / GetLoadTorque
SetPosition / SetVelocity / SetTorque
```

如果目标是上线后的长期稳定和可扩展，推荐方案二：

```text
JointBinding + BoundHardwareInterface
```

它的代码量略高，但可以把职责边界一次性理顺：

```text
raw Hardware: drive id / raw order / EtherCAT 细节
JointBinding: model joint <-> drive id 绑定关系
BoundHardwareInterface: model order 硬件视图
Controller/Executor/Model: 永远使用 model order
```

## 建议落地路径

推荐分两步：

1. 先实现 `JointBinding`，把 YAML、函数绑定、校验、`model_index_to_drive_id` 生成做扎实。
2. 如果当前版本时间紧，先走方案一；如果还有时间，直接走方案二。

不要只改 `Hardware::SetPosition()`。只改下发不改反馈，会导致闭环控制使用错误反馈，风险最高。

