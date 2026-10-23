# 六点法标定与工具管理

Viz 的“IO 与设备 → 末端工具”提供真实工具列表、新增、编辑、重命名、删除和当前工具切换；“手动示教”也可进入工具管理。工具列表中的位姿表示固定的 **法兰到 TCP** 变换。页面按全局长度/角度单位显示，HTTP 使用米和四元数（x、y、z、w），关节角为弧度。

## 六点采集顺序

1. 输入唯一工具名称，设置误差上限（默认 1 mm）。保持同一物理工具及模型配置，确保工具尖端具有稳定、可重复的接触点。
2. P1–P4：工具尖端对准同一固定参考点，以四种明显不同的法兰姿态逐点采集。姿态过于接近时不能可靠求解 TCP，会拒绝计算。
3. P4 是方向标定的原点姿态。保持 P4 的法兰方向，沿期望工具 **+X** 移动后采集 P5；再从 P4 位置沿期望工具 **+Z** 移动后采集 P6。两个方向应接近垂直，位移应明显大于 1 mm。P5/P6 相对 P4 的姿态差必须不超过 1°。
4. 每次调整后停止运动再点击采集。后端读取真实关节与法兰 FK，检查关节速度；不接受浏览器上传的测量位姿。可以返回示教页调整姿态，草稿会保留。
5. 六点齐全后计算。先对四点固定枢轴方程作 SVD 最小二乘，检查秩、条件数、RMS 和最大残差；再以 P5−P4 定义 X，P6−P4 确定 XZ 平面，通过叉积生成右手正交坐标系。两方向夹角的正弦需至少 sin(10°)，即避免接近平行或反平行。
6. 检查结果后明确点击“保存为工具”。计算本身不写入工具表，也不自动启用工具；需要使用时再“设为当前工具”。重采集任一点会撤销旧结果，重新计算后才能保存。

控制权改变、断开或切换控制器会清理本地草稿。控制器会话仅驻留内存，重启失效；最多保留最近 16 个会话。重新进入工具页会回读服务端会话，避免使用过期结果。标定结果必须结合真实工具和机器人精度验证；离线数学测试不能代替现场验收。

## 法兰与实时 TCP 位姿

`getFlange()` 保持返回基座到法兰的位姿。新增 C++ `Result GetTcpPose(Frame& tcp, const std::string& tool_name = "") const`，按 `T_base_tcp = T_base_flange * T_flange_tool` 合成，长度为米。省略或传空名称时使用当前活动工具；无活动工具时 TCP 就是法兰；指定名称不存在返回 ResourceUnavailable，不改变输出和活动工具。FK/反馈失败时明确返回错误，不将单位变换当作有效 TCP。

`GET /api/robot/state` 新增 `tcp` 字段（基座到当前 TCP 的 Pose，失败为 null）；原 `flange` 及 `active_tool_frame` 的含义保持不变。快照中的关节位置、法兰和 TCP 使用同一关节采样，工具名称、偏置和 TCP 使用同一注册表快照。`GET /api/robot/tcp?name=tool1` 查询指定工具的实时 TCP，省略 name 使用当前工具，返回 `{pose, reference_frame: "base"}`；只读、不要求控制权、不续期，也不切换活动工具，运动时仍可查询。

Lua 使用 `robot.GetTcpPose()` 或 `robot.GetTcpPose("tool1")` 取得同一计算结果，失败按现有 binding 规则抛出错误。`robot.GetFlange()` 保持原含义。

Viz 顶部“显示坐标系”选择“工具”后显示 `tcp`；默认法兰显示保持不变。新版 App 的 `tcp: null` 会显示不可用，只有旧 App 完全缺少该字段时才在 Viz 兼容合成。工具轨迹和工具参考同样使用统一 TCP；显示切换不会激活工具或发送运动命令。运动 API 的既有目标位姿及 tool_name 语义保持不变。

## 保存与启动

`config/config.yaml` 的 `robot.frames` 指定坐标系文件，默认 `frames.yaml`；相对路径以启动 YAML 的目录为基准。启动时读取已有文件，不存在则以空工具表启动；文件内容无效时启动失败并说明原因，防止默默使用错误 TCP。

Viz 的新增、编辑、重命名、删除和激活均发送 `persist: true`，成功后已保存到配置文件。“从文件重新加载”会替换当前工具和工件表以及活动选择。手动 API 调用省略 `persist` 时只更新内存，可调用 `/api/robot/frames/save` 保存。显式传入其他 path 仅操作该文件，不改变启动默认路径。

写文件采用同目录临时文件、flush/fsync 和原子替换；写入失败保留旧文件及内存。重命名同时更新活动名称；删除当前工具或激活空名称会回到法兰。工具及工件在同一 YAML 中，保存工具时保留工件数据。

工具变更、加载、保存和采集只允许机器人处于 IDLE 或 STOPPED，并要求控制权的两项凭据。状态检查与完整提交持有命令门闩，防止运动启动、使能等外部命令穿插；文件保存不占用 FSM 状态锁，实时状态读取和故障事件不等待磁盘。采集按命令门闩 → FSM → Robot 的顺序加锁。坐标系读取使用不可变快照。

工具负载在页面中读取和保存控制器全局 `load_params`，质量单位 kg、质心单位米；它与命名 TCP 分开管理。切换工具不会自动切换质量或质心。

## 接口衔接

完整协议以 [Apifox 项目 8387612](https://app.apifox.com/project/8387612) 为准。API 版本与 App 发布版本独立。

| 操作 | 方法与路径 | 主要参数 |
| --- | --- | --- |
| 工具列表 | GET `/api/robot/tool_frames` | 返回 names、active、file_path |
| 实时 TCP | GET `/api/robot/tcp?name=TCP` | name 可省略；返回基座到 TCP 的实时位姿，不改变活动工具 |
| 查询工具 | GET `/api/robot/tool_frame?name=TCP` | 返回 name、frame |
| 创建/编辑/重命名 | POST `/api/robot/tool_frame` | name、frame、persist；重命名传 previous_name，目标不能重名 |
| 删除 | DELETE `/api/robot/tool_frame?name=TCP&persist=true` | 删除活动工具时清除活动选择 |
| 启用/回到法兰 | POST `/api/robot/active_tool_frame` | name（空字符串为法兰）、persist |
| 重新加载/保存 | POST `/api/robot/frames/load`、`/save` | path 可省略或为空，使用 robot.frames |
| 采集一点 | POST `/api/calibration/pose` | name、index（1–6）、session_id；首次必须 P1，不传 session_id |
| 计算六点法 | POST `/api/calibration/run` | session_id、max_error（米，默认 .001，范围 (0,.05]） |
| 回读会话 | GET `/api/calibration/result?session_id=…` | 返回 samples、captured、result（未计算时 null） |

写操作要求 `X-Rocos-Control-Token` 与 `X-Rocos-Client-Id` 同时存在；会话采集/计算绑定创建者令牌。业务错误仍使用 JSON 标准封装：1001 参数无效，4001 点数不全或首点顺序错误，4002 求解退化或误差超限，4004 会话失效/不属于当前控制会话，4005 状态不允许写入/无法采集静止位姿。其他文件/注册表错误沿用 Result 错误码。

## 验证和部署

新增 `tool_calibration_test`、`frame_registry_test`、`tooling_http_test` 验证数学、退化、原子持久化、鉴权、会话以及计算不自动应用。Viz 的 `test/browser/tooling.spec.ts` 使用模拟控制器覆盖工具操作、六点向导、跨页保留、只读和错误路径。

部署时重建并重启 App，更新 Viz；旧 App 不提供 file_path 能力标记，Viz 会提示升级而不开放持久化工具写操作。本次自动验证不启动真实机器人、不执行运动。
