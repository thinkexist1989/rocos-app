# 工具包、安装覆盖与模型可视化

工具名称同时标识 TCP 与可选的三维模型。普通工具继续使用现有 `frames.yaml` 中的法兰到 TCP 变换；包工具将其拆成“法兰到工具基座”的安装变换 M 和“工具基座到 TCP”的局部变换 C。模型使用 `T_base_flange × M`，实时 TCP 使用 `T_base_flange × M × C`。`active_tool_frame` 和所有既有运动接口仍使用合成后的 `M × C`，含义不变。

## 文件与导入

启动配置增加 `robot.tool_library`（默认 `../tools`，相对启动 YAML）和 `robot.id`（默认 `default`）。不同机器人应配置不同的稳定 id，不以 IP 地址区分。

```text
gripper_01/
  tool.yaml
  gripper.urdf
  meshes/
    base.stl
    finger.stl
```

```yaml
schema_version: 1
name: gripper_01
package_version: 1.0.0
model:
  urdf: gripper.urdf
mount:
  position: [0, 0, 0]
  orientation: [0, 0, 0, 1]
tcp:
  position: [0, 0, 0.15]
  orientation: [0, 0, 0, 1]
```

位置单位为米，四元数顺序 xyzw；省略 mount 使用单位阵。model 可以省略，包中仍可携带可复用 TCP。工具名称沿用 ASCII 字母、数字、点、下划线、短横，1–64 字符；包名称不得以点开头。工具包与普通工具共用名称空间。

两种导入方式：

1. 将命名文件夹放到工具库根目录，在 Viz“IO 与设备 → 末端工具 → 工具包”点击“扫描工具目录”。启动时也会加入尚未注册的目录。
2. 将完整命名目录压成 ZIP，通过 Viz“导入工具 ZIP”上传。

导入会注册工具，不自动激活。运行时使用工具库下 `.objects/名称/SHA256/` 的内容快照，源文件后续变化不影响当前工具，旧快照也保留供在途资源请求使用。摘要覆盖包内全部文件，修改资源时应更新 package_version。重复导入完全相同的 ZIP 保留本机安装和标定；不同内容的同名包必须通过“更新工具 ZIP”，携带原摘要且使用不同版本。更新会恢复新包默认安装/TCP并清除旧标定。手工修改源目录不会自动替换已注册版本，重启仍恢复原版本；扫描遇到同名变更会报错，需使用明确的 ZIP 更新。

ZIP 必须只有一个与清单名称一致的顶层目录；压缩数据与展开数据均最多 64 MiB，最多 512 个文件。拒绝路径越界、链接和重复文件。第一版支持 URDF（.urdf/.xml）与 STL、URDF 纯色材质，不支持外部 URI、贴图或 DAE。mesh 文件支持相对 URDF 文件的路径，或 `package://工具名称/包内路径`。STL 缩放在 URDF 中设置。示例见 `examples/tool_packages/demo_gripper/`，可直接压缩整个目录导入。

## 本机配置与标定

工具包原件不被安装编辑或标定修改。为了使工具偏置、模型安装信息与活动选择同时生效，普通工具、工件、活动名称和 `package_tools` 绑定统一在 `robot.frames` 中原子保存。包绑定只保存摘要、版本、安装/局部 TCP 覆盖、revision 和可选标定记录，不将合成偏置重复存成普通工具。含包绑定的文件同时保存 robot_id，加载时必须与当前控制器一致；不同机器人使用各自的 frames 文件。写入失败不发布内存变更。

旧 `GET tool_frame` 仍返回法兰到 TCP 的 `frame`；包工具额外返回 `package` 分量信息。旧 POST tool_frame 不允许编辑或重命名包工具，防止把合成位姿当成局部 TCP；包工具使用专用覆盖接口，携带 digest/revision 防止覆盖过期编辑。删除包工具解除本机绑定并在必要时清除活动工具，不删除不可变资源。全局质量/质心仍独立管理，不随包切换。

六点法仍采集真实关节与法兰数据，求出法兰到 TCP 的 F。包工具会话固定包摘要及安装 revision，期间工具更新、删除或安装编辑使会话失效（4004）。计算只返回结果，不应用；明确保存后，App 用 `C = M⁻¹ × F` 写入本机局部 TCP，并记录 Unix 时间、robot_id、包摘要、当时安装变换、法兰到 TCP 结果、RMS、最大误差和条件数。手动修改覆盖会清除旧标定记录。

写操作要求控制权两项凭据且机器人为 IDLE/STOPPED；导入、安装编辑和保存不会自动激活工具或发送运动命令。只读列表、资源与导出不要求控制权、不续期。新版 `state.active_tool_package` 为当前包元信息（普通工具/无工具为 null），与 TCP 使用同一注册表快照；Viz 据此显示模型，TCP 轴继续直接读取 state.tcp。

## 导出与恢复

| 入口 | 内容 | 使用范围 |
| --- | --- | --- |
| 导出原始工具包 | 原始模型、默认安装与默认局部 TCP，内容摘要不变 | 工具库备份与跨机器人复用 |
| 导出含本机 TCP 的工具包 | 当前局部 TCP、新 package_version、原包默认安装；不携带本机安装覆盖和标定记录 | 人工确认后的可复用工具版本 |
| 导出安装配置 | robot_id、包版本/摘要、本机安装与局部 TCP、标定记录 | 当前机器人装配备份 |

“导入安装配置”接收 YAML，要求 robot_id 和已安装包的摘要/版本都一致；仅恢复覆盖值，不切换当前工具。跨机器人先导入工具 ZIP，再在目标机器人编辑安装变换、重新检查 TCP。单独备份 frames.yaml 不包含模型资源，应同时备份工具包或整个工具库。

## 接口

权威协议见 Apifox 项目 8387612，以下为接入索引。文件端点成功返回原始文件，失败返回标准 JSON（HTTP 200，检查 success/code）。

| 方法与路径 | 功能 |
| --- | --- |
| GET `/api/robot/tool_packages` | 本机已注册包、工具库路径、robot_id |
| POST `/api/robot/tool_packages/import` | 原始 ZIP；更新时 query expected_digest 指定当前摘要 |
| POST `/api/robot/tool_packages/reload` | 扫描工具库命名目录，完整验证后一次提交 |
| POST `/api/robot/tool_package/overrides` | name、digest、revision、mount、tcp（JSON Pose） |
| GET `/api/robot/tool_package/asset` | name、digest、path，返回不可变 URDF/STL |
| GET `/api/robot/tool_package/export` | name；可选 calibrated=true、package_version，返回 ZIP |
| GET `/api/robot/tool_package/installation` | name，返回安装 YAML |
| POST `/api/robot/tool_package/installation` | 原始安装 YAML |
| POST `/api/calibration/save` | session_id，保存服务端计算结果；普通工具也可使用 |

新增构建依赖 libarchive 与 OpenSSL Crypto（Debian/Ubuntu 包名 libarchive-dev、libssl-dev）。回归包括 tool_package_test、tooling_http_test 和 Viz 的 tool-packages 浏览器测试；均使用临时目录和模拟控制器，不需要硬件。
