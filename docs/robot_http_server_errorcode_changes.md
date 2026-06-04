# robot_http_server.cc 错误码变更记录

以下列出我在 `src/robot_http_server.cc` 中替换的业务错误码（旧 -> 新），按处理函数分组：

- 全局路由未命中
  - JSON body `"code":404` -> `1001`

- `handleGetLinkMesh`
  - Missing `path` query param: `400` -> `1001`
  - Mesh 文件未找到: `404` -> `1001`

- `handleGetRobotModel`
  - URDF 解析失败: `500` -> `1001`

- `handleSetWorkMode`
  - 无效 JSON / 缺少字段: `400` -> `1001`
  - 不支持的工作模式: `400` -> `1006`
  - 设置工作模式失败: `500` -> `3002`

- `handleMoveJ`
  - 无效 JSON / 缺少 `joints`: `400` -> `1001`
  - 关节数量不匹配: `400` -> `1003`
  - 同步执行超时: `3003` -> `5002`（超时映射到异步超时类）

- `handleMoveJ_IK`
  - 无效 JSON / 缺少 `pose`: `400` -> `1001`
  - IK 无解: 保持 `2001`
  - 运动失败: 保持 `2004`

- `handleMoveL` / `handleMoveL_FK` / `handleMoveC` / `handleMoveP`
  - 无效 JSON / 缺少字段: `400` -> `1001`
  - 关节数组长度不匹配（FK 的情况）: `400` -> `1003`
  - 运动失败: 保持 `2004`

- `handleMovePath`
  - 无效或空 `waypoints`: `400` -> `1001`
  - 未实现返回: `501` -> `2004`

- `handleMoveStatus`
  - 缺少 `task_id`: `400` -> `1001`
  - 任务不存在: 保持 `5001`

- 单轴接口（`handleSingleAxis*`）
  - 无效 JSON / 缺少 `id`: `400` -> `1001`
  - 无效关节 id: `400` -> `1002`

- 多轴接口
  - 无效 JSON: `400` -> `1001`
  - 缺少必需字段: `400` -> `1001`
  - 数组长度不匹配: `400` -> `1003`

- 同步模式（`handleMultiAxisSync`）
  - 无效 JSON / 缺少 `sync`: `400` -> `1001`
  - 未知 sync 模式: `400` -> `1001`

- 拖拽接口（`handleDragStart` / `handleDragStop`）
  - 无效 JSON / 缺少字段: `400` -> `1001`
  - 未知 flag / direction: `400` -> `1001`

- 标定接口（`handleSetPoseFrame` / `handleSetToolFrame` / `handleSetObjectFrame` / `handleCalibrationRun`）
  - 无效 JSON / 缺少字段: `400` -> `1001`
  - 标定 frame 无效: `400` -> `4004`
  - 标定失败: `500` -> `4002`

说明：
- 我将原来使用 HTTP 状态语义的短数字（如 `400`/`404`/`500`/`501` 等）替换为文档中定义的四位业务错误码，以便上层统一按 `code` 字段判定错误类型。
- 对于已有且符合设计文档的业务码（例如 `2001`, `2004`, `5001` 等）保持不变。
- 对于某些场景（例如同步等待超时），设计文档没有精确对应项，我选择将超时类映射到 `5002`（异步任务超时）的业务分类以便能被上层重试/恢复逻辑识别；如需其它映射，请告知。

如果你希望我把这些变更单独提交到分支并创建 PR，我可以继续执行。