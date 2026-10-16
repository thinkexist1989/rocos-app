# Dynamics ID Studio 采集入口

`src/dynamicsIdMain.cc` 只负责读取轨迹、匹配关节、调用 Robot 运动、采集反馈和保存 CSV。

`main()` 参照 `rocosAppMain.cc`，按仿真/真机分支创建硬件，再直接构造 `Robot robot(hw, ...)`。缓冲区预分配、采集及保存统一放在 `runExperiment()` 中。

执行流程：`loadTrajectory()` → 创建硬件和 Robot → `runExperiment()` 中调用 `MoveJ()`、`record()` → `setDisabled()` → `saveCsv()`。

## 输入与输出

推荐输入 Studio 导出的 **trajectory.json**，其中已经包含关节名称、起始位置、傅里叶系数和时长。程序在运动前按同一 `sin⁴` 包络生成 1 ms 参考位置，并检查硬件位置、速度、加速度和 jerk 限制。`servoJ()` 内部负责等待控制周期，采集端不再 sleep。

`--time_scale` 控制激励函数时间的推进速度，范围为 `0 < time_scale ≤ 1`，默认 `1`。控制周期始终为 1 ms：

| time_scale | 每个控制周期的函数时间步进 | 激励回合耗时 |
| --- | --- | --- |
| 1 | 1 ms | 原时长 |
| 0.5 | 0.5 ms | 约 2 倍 |
| 0.1 | 0.1 ms | 约 10 倍 |

例如原时长 20 s 的轨迹，`--time_scale=0.1` 约运行 200 s。路径与运动幅度保持相同，速度、加速度和 jerk 分别按系数的一次、二次和三次方缩放，预检也按缩放后的值检查。参数仅作用于激励段，接近起点仍使用下文的 MoveJ 参数。较慢运动方便人工观察周围干涉，不具备自动碰撞检测功能。

采集 CSV 的 `time` 始终是实际经过秒数，实测 `q/dq/tau` 不做时间缩放。导入 Studio 时，时间单位换算系数保持 `1`，不要再次填写本程序的 `time_scale`。低速预览会减弱惯性激励，不能据此判断原速下的力矩、跟踪和动态表现。

也兼容 `--trajectory=/path/trajectory.csv` 写法，此时读取该目录的 `trajectory.json`，不再解析或逐点核对 CSV。两份机器人模型的关节名称、几何、零点和正方向需要保持一致；精简版不做模型哈希校验。

输出按 Studio 的 `joint_names` 排列：

```csv
time,loop,q_1,dq_1,tau_1,q_2,dq_2,tau_2,...
```

每个采样点直接读取：

```cpp
sample.q[j] = robot.getJointPosition(joint.robotIndex);
sample.dq[j] = robot.getJointVelocity(joint.robotIndex);
sample.tau[j] = robot.getJointLoadTorque(joint.robotIndex);
```

`time` 是实际经过秒数，循环间不归零。`loop` 从 1 开始。位置和速度沿用 Robot 的 rad、rad/s 单位转换；力矩原样保存，不做二次标定。Studio 需要关节输出端 Nm，但当前 `Drive::getLoadTorque()` 直接转发底层计数，实际单位需与该接口和硬件反馈约定一致。

数据使用 `Measurement` 结构保存在预分配内存中，运动结束或 Ctrl+C/SIGTERM 中断后先下使能，再保存已采集的完整行。CSV 和 `.metadata.yaml` 都要求新文件名；侧文件记录仿真标记、关节映射、完成状态和失败原因。断电、SIGKILL 或崩溃无法保存尚未写盘的数据。

## 使用

```bash
cmake --build build --target dynamicsIdMain -j4

# 只检查轨迹，不连接硬件
build/bin/dynamicsIdMain --check_only \
  --urdf=/absolute/path/robot-control.urdf \
  --trajectory=/absolute/path/trajectory.json

# 仿真核对执行与导出流程
build/bin/dynamicsIdMain --sim=true \
  --urdf=/absolute/path/robot-control.urdf \
  --trajectory=/absolute/path/trajectory.json \
  --loops=2 --output=/absolute/path/measurements.sim.csv

# 真机采集：对应 EtherCAT 主站应已启动，避免其他程序同时控制
build/bin/dynamicsIdMain --sim=false --id=0 \
  --urdf=/absolute/path/robot-control.urdf \
  --base=base_link --tip=link_7 \
  --trajectory=/absolute/path/trajectory.json \
  --time_scale=0.1 \
  --loops=3 --output=/absolute/path/measurements.csv
```

目前 Robot 会初始化七轴 SRS 求解器，因此此入口面向七轴机器人。`MoveJ` 接近速度固定为 0.15 rad/s、加速度 0.3 rad/s²，首尾停留 0.5 s；接近和停留数据不写入辨识 CSV。

`HardwareSim` 没有真实速度、力矩与动力学，只能核对接口。真机 CSV 可在 Studio 中自动映射列；加速度由 Studio 从实测速度估计。程序没有温度输入，辨识时关闭温度摩擦项。

Studio 不支持 `continuous` 关节。导入其模型时应使用带标准 `<limit>` 的 `revolute` 关节；控制器 URDF 保留原有 `<hardware>` 节点，并保持两边的关节坐标一致。

程序沿用现有 Robot 的进程生命周期：显式下使能、关闭输出文件后调用 `std::exit()`，保持 Robot/Hardware 存活至进程退出，因为当前析构函数没有停止后台线程。

## 离线测试

在具有 Studio Python 依赖的环境执行：

```bash
python test/dynamics_id_compatibility_test.py \
  --binary=build/bin/dynamicsIdMain \
  --studio=/home/rocos/Documents/GitHub/dynamics-id-studio
```

测试使用 Studio 的原始轨迹函数与预处理器，验证七轴仿真、多循环、关节顺序、直接力矩反馈、数据导入、越限拒绝和中断保存，不连接真机。

## 单关节正反转摩擦采集

独立入口 `src/frictionIdMain.cc` 使用 Studio 导出的 `friction_protocol.json`，按相同的五次速度过渡函数，在 Robot 的 1 ms 周期计算往返位置。`friction_trajectory.csv` 路径也可传入，此时读取同目录协议，不直接逐行执行 CSV。`friction_schedule.csv` 只作摘要，不用于判定连续 ramp 区间。

此入口核对协议与 URDF 的 SHA256。因此 `--urdf` 必须是生成轨迹时的同一份七轴 URDF，既包含标准的 revolute 限位，也包含 ROCOS 的 `<hardware>`、rad 单位和标定参数。关节通过名称和硬件 ID 匹配，不依赖文件顺序。

```bash
cmake --build build --target frictionIdMain -j2

# 先检查协议、URDF 和硬件位置/速度/加速度/jerk 限制，不连接硬件
build/bin/frictionIdMain --check_only \
  --urdf=/absolute/path/robot.urdf \
  --trajectory=/absolute/path/friction_protocol.json

# EtherCAT 主站已运行时，执行协议中全部 repeats，保存一份新文件
build/bin/frictionIdMain --sim=false --id=0 \
  --urdf=/absolute/path/robot.urdf \
  --trajectory=/absolute/path/friction_protocol.json \
  --output=/absolute/path/joint_2_train_session01.csv
```

默认 `--sim=true`。仿真只核对接口，不会生成真实速度、摩擦或温度。`--time_scale=0.1` 将往返函数放慢十倍，并把 CSV 的 `target_velocity` 同步乘以 0.1；实测 `time/q/dq/tau` 不缩放。正式采集应按计划速度执行；导入 Studio 时的时间单位倍率仍设为 1。

程序先通过 `MoveJ()` 以 0.15 rad/s、0.3 rad/s² 到达协议 `initial_pose`，保持 0.5 s 后开始采集；最后一趟回到下限并保持 0.5 s，再下使能。接近与额外保持段不写入 CSV。所有参考阶段均保留，稳态筛选交给 Studio。

```csv
time,reference_time,repeat,pair,sweep,phase,target_velocity,q_1,dq_1,tau_1,...
```

`time` 为采集的实际经过秒数；`reference_time` 是协议函数时间。`repeat/pair/sweep` 从零开始，含义与 Studio 一致。`q/dq/tau` 分别直接读取 `getJointPosition()`、`getJointVelocity()` 和 `getJointLoadTorque()`；没有温度与电流接口，因而不输出这些列。力矩原值、内存缓冲、独占新文件、中断保存和进程生命周期约定与上文相同；输出侧 Nm 和信号是否包含目标传动摩擦仍需根据硬件核对。长时间实验的内存需求随总时长和采样数增加，运行前分配缓冲，期间不写盘。

采完后在 Studio 的“摩擦辨识”页面选择协议中的关节，确认其他关节静止，导入 CSV。列序号沿用协议 `joint_names`，不是 Robot 索引；无实测温度时使用无温度项模型。

离线兼容测试使用 Studio 的真实轨迹生成函数和清洗配对函数；另建的合成反馈仅用于检查配对，不冒充 HardwareSim 的实测数据：

```bash
python3 test/friction_id_compatibility_test.py \
  --binary=build/bin/frictionIdMain \
  --studio=/home/rocos/Documents/GitHub/dynamics-id-studio
```
