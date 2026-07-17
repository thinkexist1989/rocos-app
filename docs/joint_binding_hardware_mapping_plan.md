# JointBinding + Hardware 映射表修改大纲

## 目标

引入显式的 `URDF model joint -> hardware drive_id` 绑定能力，但保持 `joint_binding.yaml` 可选。

最终目标：

- 控制器、规划器、运动学、HTTP 状态上报统一使用 URDF/KDL 的 model joint order。
- Hardware 内部通过映射表把 model index 转成真实 hardware drive id。
- 没有 `joint_binding.yaml` 时默认按顺序绑定，但要求 model joint 数量和 hardware drive 数量完全一致。
- 有 `joint_binding.yaml` 时允许 model joint 数量小于或等于 hardware drive 数量，但 YAML 必须显式绑定每个 model joint。
- 运动中禁止修改绑定。
- `Robot::getJointNum()` 从 `Model` 读取，不再从 `Hardware::GetPosition().rows()` 读取。

## 总体规则

### 无 joint_binding.yaml

```text
model_joint_count == hardware_drive_count:
  允许启动
  自动生成默认绑定:
    model_index i -> config_.drives[i].id

model_joint_count != hardware_drive_count:
  Robot 初始化失败
```

无 YAML 时不允许“硬件多轴但默认取前 N 个”。这样可以避免真实硬件多出来的轴被系统默默忽略。

### 有 joint_binding.yaml

```text
model_joint_count <= hardware_drive_count:
  允许进入显式绑定校验

model_joint_count > hardware_drive_count:
  Robot 初始化失败
```

显式绑定必须满足：

- 每个 model joint 都有绑定。
- `joint_name` 必须存在于 URDF controllable joints。
- `drive_id` 必须存在于 hardware drives。
- `joint_name` 不能重复。
- `drive_id` 不能重复。
- 绑定数量必须等于 model joint 数量。

## 建议 YAML 格式

新增：

```text
config/joint_binding.yaml
```

内容：

```yaml
binding_policy: explicit

joint_bindings:
  - joint_name: joint_1
    drive_id: 0
  - joint_name: joint_2
    drive_id: 1
  - joint_name: joint_3
    drive_id: 2
```

第一阶段只支持：

```yaml
binding_policy: explicit
```

`by_order` 不需要写入 YAML，因为无 YAML 时就是默认顺序绑定。

## 修改大纲

### 1. ModelInterface 增加 joint 查询接口

文件：

```text
src/model_interface.hpp
```

新增虚函数：

```cpp
virtual int GetJointNum() const = 0;
virtual std::vector<std::string> GetJointNames() const = 0;
```

需要 include：

```cpp
#include <string>
#include <vector>
```

### 2. Model 实现 joint 查询

文件：

```text
src/model.hpp
src/model.cpp
```

新增成员：

```cpp
std::vector<std::string> joint_names_;
```

新增接口：

```cpp
int GetJointNum() const override;
std::vector<std::string> GetJointNames() const override;
```

实现逻辑：

- `GetJointNum()` 返回 `chain_.getNrOfJoints()`。
- `GetJointNames()` 返回 KDL chain 中 active joints 的名称，顺序必须和 KDL joint array 顺序一致。
- 在 `SetChain()` 或 `ParseUrdf()` 后刷新 `joint_names_`。

注意：

KDL chain 中 segment 可能包含 fixed joint。只统计 `segment.getJoint().getType() != KDL::Joint::None` 的关节。

伪代码：

```cpp
joint_names_.clear();
for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    const auto& joint = chain_.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::None) {
        joint_names_.push_back(joint.getName());
    }
}
```

校验：

```cpp
joint_names_.size() == chain_.getNrOfJoints()
```

不一致时记录错误，初始化失败或返回空。

### 3. 新增 JointBinding 模块

新增文件：

```text
src/joint_binding.hpp
src/joint_binding.cpp
```

职责：

- 保存 model joint names。
- 保存 hardware drive ids。
- 加载/保存 YAML。
- 支持函数绑定修改。
- 校验绑定。
- 生成 `model_index_to_drive_id`。

建议接口：

```cpp
class JointBinding {
public:
    Result Configure(const std::vector<std::string>& model_joint_names,
                     const std::vector<int32_t>& hardware_drive_ids);

    Result LoadFromYaml(const std::string& path);
    Result SaveToYaml(const std::string& path) const;

    Result Bind(const std::string& joint_name, int32_t drive_id);
    Result UnbindJoint(const std::string& joint_name);
    Result Clear();

    Result Validate() const;

    std::vector<int32_t> GetModelIndexToDriveIds() const;
    int32_t DriveIdByModelIndex(size_t model_index) const;

    bool HasBindingForJoint(const std::string& joint_name) const;
    bool IsComplete() const;
};
```

内部成员建议：

```cpp
std::vector<std::string> model_joint_names_;
std::vector<int32_t> hardware_drive_ids_;
std::map<std::string, int32_t> joint_to_drive_id_;
```

`Configure()` 校验：

- `model_joint_names` 不能为空。
- `hardware_drive_ids` 不能为空。
- `model_joint_names.size() <= hardware_drive_ids.size()`。
- model joint name 不能重复。
- hardware drive id 不能重复。

`Validate()` 校验：

- 每个 model joint 都存在绑定。
- 每个绑定的 joint name 都在 model joint names 中。
- 每个绑定的 drive id 都在 hardware drive ids 中。
- drive id 不重复。
- 绑定数量等于 model joint 数量。

错误码建议：

```text
Result::IllegalParameter      配置格式非法、重复、数量不匹配
Result::ResourceUnavailable   YAML 文件不存在或无法读取
Result::NoError               成功
```

### 4. HardwareInterface 增加可选绑定虚函数

文件：

```text
src/hardware_interface.hpp
```

新增：

```cpp
virtual Result SetJointBinding(const std::vector<int32_t>& model_index_to_drive_id) {
    return Result::FunctionNotSupported;
}

virtual void ClearJointBinding() {}

virtual std::vector<int32_t> GetJointBinding() const {
    return {};
}

virtual int GetDriveNum() const {
    return 0;
}

virtual std::vector<int32_t> GetDriveIds() const {
    return {};
}
```

需要 include：

```cpp
#include <vector>
#include "result.hpp"
```

说明：

- `SetJointBinding()` 接收的是最终映射表，不接收 joint name。
- HardwareInterface 不理解 URDF，只理解 model index 到 drive id。
- 如果某个硬件后端不支持绑定，默认返回 `FunctionNotSupported`。

### 5. Hardware 增加映射表支持

文件：

```text
src/hardware.hpp
src/hardware.cpp
```

新增成员：

```cpp
std::vector<int32_t> model_index_to_drive_id_;
```

新增接口实现：

```cpp
Result SetJointBinding(const std::vector<int32_t>& model_index_to_drive_id) override;
void ClearJointBinding() override;
std::vector<int32_t> GetJointBinding() const override;
int GetDriveNum() const override;
std::vector<int32_t> GetDriveIds() const override;
```

新增内部辅助函数：

```cpp
bool HasJointBinding() const;
size_t ModelJointCount() const;
int32_t DriveIdByModelIndex(size_t model_index) const;
```

行为：

```cpp
bool Hardware::HasJointBinding() const {
    return !model_index_to_drive_id_.empty();
}

size_t Hardware::ModelJointCount() const {
    return HasJointBinding() ? model_index_to_drive_id_.size() : config_.drives.size();
}

int32_t Hardware::DriveIdByModelIndex(size_t model_index) const {
    if (HasJointBinding()) {
        return model_index_to_drive_id_[model_index];
    }
    return config_.drives[model_index].id;
}
```

`SetJointBinding()` 校验：

- 映射表不能为空。
- 映射表数量不能大于 `config_.drives.size()`。
- 每个 drive id 必须存在。
- drive id 不能重复。

### 6. Hardware 批量 Get/Set 统一改成 model order

文件：

```text
src/hardware.cpp
```

当前需要改的函数集中在 `DriveInterface — 批量操作` 段：

```cpp
JntArray Hardware::GetPosition()
JntArray Hardware::GetVelocity()
JntArray Hardware::GetTorque()
JntArray Hardware::GetLoadTorque()
void Hardware::SetPosition(const JntArray& q)
void Hardware::SetVelocity(const JntArray& q_dot)
void Hardware::SetTorque(const JntArray& tau)
```

修改前语义：

```cpp
q(i) <-> config_.drives[i].id
```

修改后语义：

```cpp
q(i) <-> DriveIdByModelIndex(i)
```

示例：

```cpp
JntArray Hardware::GetPosition() {
    JntArray q(static_cast<unsigned int>(ModelJointCount()));
    for (size_t i = 0; i < ModelJointCount(); ++i) {
        q(i) = GetJointPosition(DriveIdByModelIndex(i));
    }
    return q;
}
```

下发同理：

```cpp
void Hardware::SetPosition(const JntArray& q) {
    const size_t n = std::min(ModelJointCount(), static_cast<size_t>(q.rows()));
    for (size_t i = 0; i < n; ++i) {
        SetJointPosition(DriveIdByModelIndex(i), q(i));
    }
}
```

### 7. Hardware::getJointName 语义调整

文件：

```text
src/hardware.cpp
```

当前 `Robot::getJointName(int id)` 把参数当 model index 使用，因此 `Hardware::getJointName(int32_t id)` 建议调整为 model index 语义：

```cpp
std::string Hardware::getJointName(int32_t model_index) {
    if (model_index < 0 || static_cast<size_t>(model_index) >= ModelJointCount()) {
        return "";
    }
    const int32_t drive_id = DriveIdByModelIndex(static_cast<size_t>(model_index));
    auto idx = getDriveIdx(drive_id);
    if (idx < 0) return "";
    return config_.drives[static_cast<size_t>(idx)].joint_name;
}
```

如果未来需要 raw drive name，另加：

```cpp
std::string getDriveJointName(int32_t drive_id);
```

### 8. Robot 增加 JointBinding 成员和初始化流程

文件：

```text
src/robot.hpp
src/robot.cpp
```

新增成员：

```cpp
std::unique_ptr<JointBinding> joint_binding_;
std::string joint_binding_path_{"joint_binding.yaml"};
```

构造函数流程调整：

```text
1. 创建 Hardware
2. 创建 Model
3. 从 Model 读取 model_joint_names / model_joint_count
4. 从 Hardware 读取 hardware_drive_ids / hardware_drive_count
5. 判断 joint_binding.yaml 是否存在
6. 若不存在:
     要求 model_joint_count == hardware_drive_count
     生成默认顺序绑定
7. 若存在:
     要求 model_joint_count <= hardware_drive_count
     加载 YAML
     Validate()
8. 调用 hardware->SetJointBinding(model_index_to_drive_id)
9. 再创建 controller，并 controller->SetHardware(hardware.get())
10. 创建 executor
```

关键点：

- `SetJointBinding()` 必须在 `controller->SetHardware()` 之前完成。
- 初始化失败直接 throw，防止机器人在错轴风险下启动。
- 如果 YAML 不存在但数量不一致，直接 throw。

伪代码：

```cpp
auto model_joint_names = model->GetJointNames();
auto hardware_drive_ids = hardware->GetDriveIds();

joint_binding_ = std::make_unique<JointBinding>();
Result rc = joint_binding_->Configure(model_joint_names, hardware_drive_ids);
if (rc != Result::NoError) throw std::runtime_error("JointBinding Configure failed");

if (std::filesystem::exists(joint_binding_path_)) {
    rc = joint_binding_->LoadFromYaml(joint_binding_path_);
    if (rc != Result::NoError) throw std::runtime_error("JointBinding Load failed");
} else {
    if (model_joint_names.size() != hardware_drive_ids.size()) {
        throw std::runtime_error("joint_binding.yaml missing and joint/drive count mismatch");
    }
    for (size_t i = 0; i < model_joint_names.size(); ++i) {
        joint_binding_->Bind(model_joint_names[i], hardware_drive_ids[i]);
    }
}

rc = joint_binding_->Validate();
if (rc != Result::NoError) throw std::runtime_error("JointBinding Validate failed");

rc = hardware->SetJointBinding(joint_binding_->GetModelIndexToDriveIds());
if (rc != Result::NoError) throw std::runtime_error("Hardware SetJointBinding failed");
```

### 9. Robot::getJointNum 改为从 Model 读取

文件：

```text
src/robot.hpp
```

当前：

```cpp
inline int getJointNum() const {
    return hardware ? static_cast<int>(hardware->GetPosition().rows()) : 0;
}
```

改为：

```cpp
inline int getJointNum() const {
    return model ? model->GetJointNum() : 0;
}
```

目的：

- joint 数量来自 URDF/KDL chain。
- Hardware 只负责真实驱动和映射，不再作为 model joint 数量来源。

### 10. Robot 绑定管理 API

文件：

```text
src/robot.hpp
src/robot.cpp
```

后续给上位机使用，可分第二阶段实现：

```cpp
Result LoadJointBinding(const std::string& path);
Result SaveJointBinding(const std::string& path) const;
Result BindJointToDrive(const std::string& joint_name, int32_t drive_id);
std::vector<int32_t> GetJointBinding() const;
```

运行时修改规则：

```text
如果 IsMotionBusy() == true:
  返回 Result::CallingConflictError

否则:
  修改 JointBinding
  Validate()
  hardware->SetJointBinding(...)
  SaveToYaml(...)
```

第一阶段可以只实现初始化加载，不暴露运行时修改 API。

### 11. CMake 修改

文件：

```text
CMakeLists.txt
```

在 `add_library(robot SHARED ...)` 加入：

```text
src/joint_binding.cpp
```

### 12. 测试计划

#### JointBinding 单元测试

新增：

```text
test/joint_binding_test.cc
```

测试用例：

- 正常显式绑定生成 `model_index_to_drive_id`。
- model joint 数量大于 hardware drive 数量时报错。
- unknown joint name 报错。
- unknown drive id 报错。
- duplicate joint name 报错。
- duplicate drive id 报错。
- 绑定不完整时报错。
- YAML load/save round trip。

#### Model 测试

修改：

```text
test/model_test.cc
```

增加：

- `GetJointNum()` 返回 7。
- `GetJointNames()` 数量为 7。
- `GetJointNames()` 顺序和 KDL chain active joint 顺序一致。

#### Hardware 测试

现有 `Hardware` 构造依赖共享内存，直接测运行态 Get/Set 成本较高。

第一阶段可先测试：

- `Hardware::LoadConfigFromYAML()` 保持不变。
- 如果可以安全构造 fake 或测试专用 Hardware，再补：
  - `SetJointBinding()` 校验非法 drive id。
  - `SetJointBinding()` 校验重复 drive id。
  - `GetJointBinding()` 返回设置结果。

#### Robot 初始化测试

后续可加：

- 无 YAML 且数量相等：启动通过。
- 无 YAML 且数量不等：初始化失败。
- 有 YAML 且 model joint 小于 hardware drive：启动通过。
- 有 YAML 但绑定不完整：初始化失败。

当前 `Robot` 构造依赖真实共享内存，建议等硬件构造可 mock 后再做自动化。

## 预计代码量

```text
ModelInterface / Model: 40-90 行
JointBinding: 300-470 行
HardwareInterface: 25-60 行
Hardware: 150-260 行
Robot 初始化与 getJointNum: 100-180 行
CMake: 1 行
config/joint_binding.yaml 示例: 10-30 行
JointBinding/Model 测试: 220-400 行
```

总计预计：

```text
846-1491 行
```

如果第一阶段不做 Robot 运行时绑定 API、不做 Robot 自动化测试，预计：

```text
550-900 行
```

## 推荐实施顺序

1. `ModelInterface/Model` 增加 `GetJointNum()` / `GetJointNames()`。
2. 新增并测试 `JointBinding`。
3. `HardwareInterface/Hardware` 增加映射表接口。
4. 修改 Hardware 批量 Get/Set 和 `getJointName()`。
5. `Robot::getJointNum()` 改为从 Model 读取。
6. Robot 构造函数接入 binding 初始化规则。
7. 补 `config/joint_binding.yaml` 示例。
8. 编译 `robot`、运行 `joint_binding_test` 和 `model_test`。

## 风险点

- 只改 `SetPosition()` 不改反馈函数会导致闭环错轴，禁止这样做。
- 无 YAML 但 model/hardware 数量不一致时必须失败，不能默认取前 N 个。
- `getJointName()` 参数语义要统一成 model index，否则上位机显示会错。
- 绑定修改必须禁止在运动中执行。
- 如果 `JointBinding` 和 Hardware 内部映射表不一致，控制链路会错轴，因此 Robot 初始化和运行时修改都必须按同一条路径刷新。

