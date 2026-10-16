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
sample.tau[j] = robot.getJointTorque(joint.robotIndex) * (joint.ratedTorque / 1000.0);
```

`time` 是实际经过秒数，循环间不归零。`loop` 从 1 开始。位置和速度沿用 Robot 的 rad、rad/s 单位转换。当前 `getJointTorque()` 返回额定力矩千分比计数，采集端按 `tau_Nm = feedback * rated_torque_Nm / 1000` 换算后保存，CSV 的 `tau_*` 单位为 Nm。

两份采集入口共用 `--torque_config=config/joint_rated_torque.yaml`，其中按关节名配置 J1–J7 的额定力矩 `[48, 48, 40, 40, 7, 7, 7]` Nm，支持任意 Studio 列顺序和硬件 ID 映射。自定义关节名时同步修改 YAML 键名。启动预检会拒绝缺失、非正数或非有限的额定力矩；元数据记录反馈源、Nm 单位、换算公式和每关节额定值。导入 Studio 时力矩单位倍率使用 1，避免重复换算。

数据使用 `Measurement` 结构保存在预分配内存中，运动结束或 Ctrl+C/SIGTERM 中断后先下使能，再保存已采集的完整行。CSV 和 `.metadata.yaml` 都要求新文件名；侧文件记录仿真标记、关节映射、完成状态和失败原因。断电、SIGKILL 或崩溃无法保存尚未写盘的数据。

## 使用

```bash
cmake --build build --target dynamicsIdMain -j4

# 只检查轨迹，不连接硬件
build/bin/dynamicsIdMain --check_only \
  --urdf=/absolute/path/robot-control.urdf \
  --torque_config=/absolute/path/joint_rated_torque.yaml \
  --trajectory=/absolute/path/trajectory.json

# 仿真核对执行与导出流程
build/bin/dynamicsIdMain --sim=true \
  --urdf=/absolute/path/robot-control.urdf \
  --torque_config=/absolute/path/joint_rated_torque.yaml \
  --trajectory=/absolute/path/trajectory.json \
  --loops=2 --output=/absolute/path/measurements.sim.csv

# 真机采集：对应 EtherCAT 主站应已启动，避免其他程序同时控制
build/bin/dynamicsIdMain --sim=false --id=0 \
  --urdf=/absolute/path/robot-control.urdf \
  --torque_config=/absolute/path/joint_rated_torque.yaml \
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
  --torque_config=/absolute/path/joint_rated_torque.yaml \
  --trajectory=/absolute/path/friction_protocol.json

# EtherCAT 主站已运行时，执行协议中全部 repeats，保存一份新文件
build/bin/frictionIdMain --sim=false --id=0 \
  --urdf=/absolute/path/robot.urdf \
  --torque_config=/absolute/path/joint_rated_torque.yaml \
  --trajectory=/absolute/path/friction_protocol.json \
  --output=/absolute/path/joint_2_train_session01.csv
```

默认 `--sim=true`。仿真只核对接口，不会生成真实速度、摩擦或温度。`--time_scale=0.1` 将往返函数放慢十倍，并把 CSV 的 `target_velocity` 同步乘以 0.1；实测 `time/q/dq/tau` 不缩放。正式采集应按计划速度执行；导入 Studio 时的时间单位倍率仍设为 1。

程序先通过 `MoveJ()` 以 0.15 rad/s、0.3 rad/s² 到达协议 `initial_pose`，保持 0.5 s 后开始采集；最后一趟回到下限并保持 0.5 s，再下使能。接近与额外保持段不写入 CSV。所有参考阶段均保留，稳态筛选交给 Studio。

```csv
time,reference_time,repeat,pair,sweep,phase,target_velocity,q_1,dq_1,tau_1,...
```

`time` 为采集的实际经过秒数；`reference_time` 是协议函数时间。`repeat/pair/sweep` 从零开始，含义与 Studio 一致。`q/dq` 分别直接读取 `getJointPosition()`、`getJointVelocity()`；`tau` 读取 `getJointTorque()` 后乘该关节额定力矩并除以 1000，以 Nm 保存。额定力矩通过同一 `--torque_config` 按关节名匹配。没有温度与电流接口，因而不输出这些列。内存缓冲、独占新文件、中断保存和进程生命周期约定与上文相同。长时间实验的内存需求随总时长和采样数增加，运行前分配缓冲，期间不写盘。

采完后在 Studio 的“摩擦辨识”页面选择协议中的关节，确认其他关节静止，导入 CSV。列序号沿用协议 `joint_names`，不是 Robot 索引；无实测温度时使用无温度项模型。

离线兼容测试使用 Studio 的真实轨迹生成函数和清洗配对函数；另建的合成反馈仅用于检查配对，不冒充 HardwareSim 的实测数据：

```bash
python3 test/friction_id_compatibility_test.py \
  --binary=build/bin/frictionIdMain \
  --studio=/home/rocos/Documents/GitHub/dynamics-id-studio
```

## 阻抗控制的速度保护与调试

`jointImpedanceMain` 默认以启动位置为目标，启动前检查静止并预置补偿力矩，再上使能。
在 `config/identified_impedance.yaml` 中设置必填的 `velocity_limit`，单位 rad/s，数组按刚体模型的 `joint_names` 排列。
这是自定义实测速度阈值，不从 URDF 推导；每项必须为正且不高于 URDF 的硬件速度限制。
任一控制周期发现 `abs(dq[j]) > velocity_limit[j]` 就抛出异常，主程序先调用 `robot.setDisabled()`，再打印关节名、实测速度和阈值，以失败状态退出；人工确认后重新启动，不自动恢复使能。
没有增加相对目标的位置偏差限制，原有绝对位置限位、力矩幅值和变化率限制继续保留。
下使能后的减速及抱闸取决于驱动器配置，软件速度检查不是独立的硬件看门狗。

启动时打印每关节硬件 ID、起始位置、Kp/Kd、速度阈值、力矩限制及预置力矩。
运行中默认每 0.5 s 打印一组调试快照，可以用 `--debug_interval=0.1` 提高到 10 Hz，或 `--debug_interval=0` 关闭周期打印。
快照包含经过时间、当前循环间隔、实际/目标位置、位置误差、实际速度、刚体/摩擦补偿、弹簧/阻尼项、限幅前请求力矩及下发力矩。
`load(raw)` 直接读取 `robot.getJointLoadTorque()`，当前接口未做 Nm 换算，其单位不能直接与 Nm 力矩列混用。
周期打印由独立线程处理，控制线程只尝试投递最新快照，打印繁忙时丢弃快照；这些低频输出用于观察状态，不是完整的数据采集记录。

## 直接使用 URDF 的阻抗入口

`src/jointImpedanceUrdfMain.cc` 是独立副本，使用 URDF 的质量、质心和惯量计算
`C(q,dq)·dq + g(q) + Kp·(qd-q) - Kd·dq`，没有辨识参数或摩擦模型依赖。
原辨识入口保持不变。新入口沿用 CST 切换、当前位置保持、自定义速度保护、力矩限制和调试打印。
在 `config/urdf_impedance.yaml` 中设置 URDF 路径及参数，数组按 URDF 动力学模型的关节顺序排列，启动时打印关节名称。

```bash
cmake --build build/impedance --target jointImpedanceUrdfMain -j2
build/impedance/bin/jointImpedanceUrdfMain --config=config/urdf_impedance.yaml --check_only
# 仿真只验证接口，不模拟动力学；真机沿用 --sim=false。
build/impedance/bin/jointImpedanceUrdfMain --config=config/urdf_impedance.yaml --sim=true --duration=1
```
