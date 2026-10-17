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

每个采样点读取实际位置和力矩，速度由相邻两次实际位置差分计算：

```cpp
sample.q[j] = robot.getJointPosition(joint.robotIndex);
sample.dq[j] = (sample.q[j] - previousPositions[j]) / cycle;
sample.tau[j] = robot.getJointTorque(joint.robotIndex) * (joint.ratedTorque / 1000.0);
```

`time` 是实际经过秒数，循环间不归零。`loop` 从 1 开始。实际位置由 Robot 返回，单位为 rad；`cycle` 为相邻位置采样的实际时间间隔（秒），差分速度单位为 rad/s，不受 `time_scale` 影响。首条数据以激励开始前读取的位置为差分基准，回合之间连续计算。当前 `getJointTorque()` 返回额定力矩千分比计数，采集端按 `tau_Nm = feedback * rated_torque_Nm / 1000` 换算后保存，CSV 的 `tau_*` 单位为 Nm。

动力学采集的 CSV 速度和超速检查共用上述位置差分速度，不读取 `getJointVelocity()`，不做在线滤波。MoveJ 前及保持结束后的静止检查使用 20 ms 窗口的位置差分，减小编码器量化的影响；激励采集仍逐周期执行，不增加 sleep。元数据记录速度来源和换算公式。位置差分会放大量化噪声，辨识前应按 Studio 的预处理流程处理数据。

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

采集时默认启动 ROCOS-Viz 的 HTTP 接口，监听 `0.0.0.0:9090`，通过同一个 Robot
提供实时状态、URDF 和网格。可视化客户端连接采集电脑 IP 和端口 `9090` 即可，
无需另外运行 `rocosAppMain`。服务在后台运行，采集结束后随程序关闭。
使用 `--http_host`、`--http_port` 修改地址和端口，或用 `--http=false` 关闭；
`--check_only` 不连接硬件，也不启动 HTTP 服务。采集期间仅查看状态，避免通过可视化界面发送运动命令。

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

状态反馈和控制指令的 TCP 协议见 [joint-control-json.md](joint-control-json.md)，由新增的 `jointTcpClientMain` 实现，包含四种模式、Ruckig 插值及运行指令。

`jointImpedanceMain` 默认以启动位置为目标，启动前检查静止并预置补偿力矩，再上使能。
`friction_scale: 0.7` 将整个摩擦补偿乘以 0.7，刚体补偿、弹簧和阻尼项不缩放。范围为 `[0,1]`，0 关闭摩擦补偿，1 为全量补偿；仅启动时读取，修改 YAML 后需重启，旧配置未指定时保持系数 1。调试输出 `friction(Nm)` 为缩放后的补偿分量。降低比例不保证系统稳定，原有速度和力矩保护继续生效。
该入口不读取驱动器速度反馈：启动静止检查使用 20 ms 窗口的位置差分，控制循环使用相邻两次实际位置之差除以实际采样时间间隔。使能后先保持预置力矩一个总线周期，再计算首周期速度。动力学、摩擦补偿、阻尼项、调试打印和超速检查共用此速度，不做在线滤波；位置量化仍会造成差分速度抖动。
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

## 当前辨识结果：5d082d97

`config/identified_impedance.yaml` 已指向 `models/identified_5d082d97/`。结果来自 Studio 项目
`acc058eeeb2f4f428c06c31769afa7de` 的任务 `5d082d97a3f74883bd67ebe6105ebf1e`（2026-10-17）。
目录中保存辨识时的原始 URDF、由 `tools/export_impedance_model.py` 导出的 `Pβ` 和同次任务的 `friction_model.json`。
43 个最小惯性参数经 `70×43` 的 P 映射为 70 个回归参数；这不要求重建物理惯量，也没有改用 URDF 名义惯量。
URDF SHA-256 与结果一致，七轴顺序为 `joint_1` 至 `joint_7`，硬件 ID 为 0 至 6。
摩擦只启用了库仑、黏滞项；Stribeck、负载和温度项均为零，不需要外部温度输入。
额定力矩保持 `[48,48,40,40,7,7,7]` Nm，下发仍按 `Nm / rated_torque * 1000` 换算。

已完成配置检查、50 组 Studio/C++ 力矩对照（误差小于 `1e-9` Nm）及 HardwareSim 启动测试；没有运行真机。
这些检查证明接口和计算一致，不证明真实机械臂稳定。本次使用 `damped` 求解器且已收敛，
验证集来自同一采集实验的留出数据；各轴力矩 RMSE 约为 `[2.55,1.37,1.76,1.52,0.27,0.18,0.18]` Nm。
此外，1 ms 内一个位置计数对应约 0.048 rad/s，全量摩擦补偿（`friction_scale=1`）会产生约
`[5.74,4.11,3.59,3.57,0.45,0.29,0.32]` Nm 的正向请求；当前配置再乘 0.5。
需留意保持时的抖动；请求仍经过原有力矩限幅和变化率限制。
在此前日志的姿态 `q=[0.930455,0,-0.0000959,1.57094,0.0000479,0.0035953,0.0000479]`，
辨识重力项的 joint_2/joint_4 为 `-9.39/-9.49` Nm，该 URDF 名义值为 `-8.08/-8.08` Nm。
旧版模型的方向差异已不适用于本次结果；真机补偿效果仍需验证。

从仓库根目录执行：

```bash
cmake --build build --target jointImpedanceMain -j2
build/bin/jointImpedanceMain --config=config/identified_impedance.yaml --check_only
build/bin/jointImpedanceMain --config=config/identified_impedance.yaml --sim --duration=1

# 真机会切换 CST、上使能并保持启动位置；5 秒后下使能退出，Ctrl+C 也会退出。
sudo ./build/bin/jointImpedanceMain --config=config/identified_impedance.yaml \
  --nosim --duration=5 --debug_interval=0.5
```

持续保持时使用 `--duration=0`。Kp/Kd、`velocity_limit`、力矩及变化率限制仍在同一 YAML 中修改。

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
