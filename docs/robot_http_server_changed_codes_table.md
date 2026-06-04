# Changed return codes (old -> new)

下面表格列出在 `src/robot_http_server.cc` 中按设计文档替换或计划替换的业务错误码映射（Old -> New），并标注发生位置/处理函数：

| 处理函数 | 旧 code | 新 code | 说明 |
|---|---:|---:|---|
| 全局路由未命中 (post routing) | 404 | 1001 | endpoint not found 返回统一参数错误码
| handleGetLinkMesh | 400 | 1001 | 缺少 `path` 查询参数
| handleGetLinkMesh | 404 | 1001 | Mesh 文件未找到
| handleGetRobotModel | 500 | 1001 | URDF 解析失败 -> 统一参数/资源错误
| handleSetWorkMode | 400 | 1001 | 无效 JSON / 缺少 `mode`
| handleSetWorkMode | 400 | 1006 | 不支持的工作模式（专用参数错误码）
| handleSetWorkMode | 500 | 3002 | 设置工作模式失败 -> 机器人状态类错误
| handleMoveJ | 400 | 1001 | 无效 JSON / 缺少 `joints`
| handleMoveJ | 400 | 1003 | 关节数量不匹配 -> 参数长度错误
| handleMoveJ (sync timeout) | 3003 | 5002 | 同步等待超时映射到异步任务超时类
| handleMoveJ | 2004 | 2004 | 运动规划失败（保持原码）
| handleMoveJ_IK | 400 | 1001 | 无效 JSON / 缺少 `pose`
| handleMoveJ_IK | (IK 无解)  | 2001 | 保持 IK 无解业务码
| handleMoveJ_IK | 2004 | 2004 | 运动失败（保持原码）
| handleMoveL / MoveL_FK / MoveC / MoveP | 400 | 1001 | 无效 JSON / 缺少字段
| MoveL_FK / multi-joints length | 400 | 1003 | 关节数组长度不匹配
| MoveC (unknown mode) | 400 | 1001 | 未知 mode 字符串 -> 参数错误
| handleMovePath | 400 / 501 | 1001 / 2004 | 空 waypoints -> 1001；未实现接口由 501 改为 2004（规划失败）
| handleMoveStatus | 400 | 1001 | 缺少 `task_id` 查询参数
| handleMoveStatus | (task not found) | 5001 | 保持异步任务不存在码
| 单轴接口 (`handleSingleAxis*`) | 400 | 1001 | 无效 JSON / 缺少 `id`
| 单轴接口 (`handleSingleAxis*`) | 400 | 1002 | 无效关节 id -> 参数越界类错误
| 多轴接口 | 400 | 1001 | 无效 JSON / 缺少必需字段
| 多轴接口 (数组长度不匹配) | 400 | 1003 | 参数数组长度错误
| 多轴同步 `handleMultiAxisSync` | 400 | 1001 | 未知 sync 模式 -> 参数错误
| 拖拽接口 (`handleDragStart`) | 400 | 1001 | 无效/缺少 `flag` / `direction` 或值不支持
| 标定接口 (`handleCalibrationRun`) | 400 | 4004 | 无效 frame -> 标定类错误码 4004
| 标定接口 (`handleCalibrationRun`) | 500 | 4002 | 标定计算失败 -> 4002

本次变更已把 `src/robot_http_server.cc` 中的大部分短码替换为设计文档的四位业务码，并把未实现的 `/api/move/pathway` 在 OpenAPI 中标注为 Not Implemented（见 `config/rocos-API.yaml`）。

如果你希望，我可以：
- 继续把剩余尚未替换的位置全部统一并运行构建验证（需要你允许我执行构建命令）；
- 或保留当前实现为准，仅把文档继续同步并生成 PR 补丁供审核。

说明：
- 表中 "旧 code" 一栏通常是原代码中使用带 HTTP 语义的短码（400/404/500/501），"新 code" 按照 `docs/http_api_design.md` 中的四位业务码规范替换为 1xxx/2xxx/3xxx/4xxx/5xxx 分段。
- 对于已在代码中保留且符合设计文档的业务码（如 `2001`, `2004`, `5001`），在表中标注为保持不变。

如需我现在把这些所有变更都在 `src/robot_http_server.cc` 中实际替换并运行一次构建验证，我可以继续（将会修改多个位置并运行 `cmake --build` 或 `make`，需要你允许我运行终端命令）。