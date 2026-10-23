# 日志系统接入与本地验证

## 启动与配置

`rocosAppMain --config=...` 解析 YAML 后、建立 Robot/Hardware/Model 前配置 `LogManager` 并启动独立诊断 HTTP 服务。诊断端口绑定失败会使启动失败，避免报告不存在的服务。配置解析失败保留 stderr 后备输出。SIGINT/SIGTERM 处理器只设置退出标志，清理在正常线程中进行。

配置示例见 `config/config.yaml`：

- `logging.directory` 相对于启动 YAML 目录解析；默认 `../logs`。
- `file_level` 默认 `info`，允许 `trace/debug/info/warn`，确保文件门槛保留告警。`console_level` 独立允许六种级别。
- `max_file_size_mib=50`，`max_files=20`（包含当前文件），`flush_interval_ms=1000`。达到大小或跨本地日历日时创建新的 `.log` 文件；只清理 `rocos-app_*.log`。文件名包含毫秒、时区、递增号，并使用独占创建，禁止原地截断复用。
- `diagnostics.host=0.0.0.0`、`port=8082`；端口必须与控制 HTTP 端口不同。
- `max_sse_clients=4`、`max_downloads=1`；工作线程数为二者之和加 4，请求等待队列最多 64，写超时 2 秒。诊断服务使用自己的 cpp-httplib 实例、线程池和请求队列。

`LogStorage` 的专属文件线程通过 stdio 写入 spdlog 风格的可读文本，并管理独占创建、轮换、刷新与 fd 快照；spdlog 保留为原有非实时调用点的兼容入口。文件和终端均按 `[YYYY-MM-DD HH:mm:ss.SSS] [Module] [level] message` 输出，交互终端仅对级别标签着色，文件不写入 ANSI 颜色转义。HTTP 查询和 SSE 的事件仍使用 JSON。文件切点与读取句柄由同一个线程协调，文件和终端 I/O 仍与告警发布、实时生产者隔离。

诊断接口不申请、占用或续期控制权，也不使用控制服务的独占令牌。当前实现允许只读访问并允许跨域；部署时应在受控网络或反向代理下提供访问权限控制。

## 接口

三个接口位于诊断服务（默认端口 8082）。本文件是接入说明，完整协议仍以 Apifox 为维护源。

2026-10-23 已将这三个接口和负载参数的读取、设置、保存接口补入 [Apifox ROCOS API](https://app.apifox.com/project/8387612) 的主分支、模块 `7912103`。API 版本保持 `3.0.0`，当前为 67 条路径、72 个操作、38 个模型。MCP 刷新与逐项回读确认了诊断端口、参数、JSON/SSE/文本响应类型、错误码和文件响应头；两个负载写接口的鉴权为 `ControlToken AND ClientId`。原有 66 个操作与 28 个模型的回读结果保持一致；保留四个仅在线存在的旧操作。下方验证记录中的“尚未同步”是历史状态，以本段为准。

Viz 在“监控诊断 → 日志”中提供当前文件的查看和下载入口。预览显示文件名、快照时间和前 256 KiB 文本，仅级别标签着色；下载保存完整原文件。日志列表仍通过查询和 SSE 显示警告与错误，文件预览可查看 `info` 等通过文件门槛的记录。

### `GET /api/logs?after=<id>&limit=100`

只返回 `warn/error/critical`。`limit` 范围 1～500；未给 `after` 返回最近记录，有游标时按顺序分页。游标为 `<session>:<sequence>`，会话不同返回 `server_restarted` 缺口；格式错误或本会话未来游标返回 HTTP 400、业务码 1001。

```json
{"success":true,"code":0,"message":"ok","data":{"events":[],"next_id":"session:0","has_more":false,"gap":null}}
```

每个事件始终包含 `id/timestamp/level/module/code/message` 六个字段。`code` 为原有整数错误码或 null，时间为含毫秒和时区的 RFC 3339；文件使用可读文本，不包含事件 ID 或单独的 JSON 字段。没有 motion_id/context 字段。英文说明在后台生成，位置、速度事件区分指令值并包含实际数值和单位。

缓存和会话共享不可变的已序列化帧，不重复保留 JSON 树。缓存最多 4096 个条目且最多 16 MiB（按字符串实际容量、帧结构和每帧额外容器额度计费，含缺口记录），超过任一限制淘汰最旧条目。单条对外事件的序列化 JSON 最多 8192 字节，超长说明带 `[truncated]`，非法 UTF-8 统一替换。ID 序号也为内部缺口留号，客户端应按游标去重，不假设所有连续序号都是告警。

### `GET /api/logs/stream?after=<id>`

`Last-Event-ID` 请求头优先于查询参数。有游标先补发可用缓存；没有游标只订阅新事件。响应为 `text/event-stream`，包含 `Cache-Control: no-cache`、`X-Accel-Buffering: no`、`X-Rocos-SSE-Limit`。

```text
id: session:12
event: log
data: {"id":"session:12","timestamp":"2026-10-22T08:00:00.123+08:00","level":"warn","module":"Robot","code":null,"message":"Example warning."}

```

建立连接先发送 `: connected` 注释；每 15 秒发送 `: heartbeat` 注释。`event: gap` 不设置 SSE id，正文包含 `reason`、`earliest_id`，已知生产丢弃数时含 `count`。原因包括 `buffer_overrun/server_restarted/producer_overflow`。首次补发也受有界会话缓存限制；若补发期间继续产生的流量超过容量，会明确报告缺口。

三个日志路径均支持 OPTIONS 预检，返回 204，允许 GET 与 `Last-Event-ID`，供跨端口的浏览器或 Fetch 客户端重连。`X-Rocos-SSE-Limit` 同时出现在成功和被拒绝的 SSE 响应中。

每个会话的独立队列最多 4097 个帧和 17 MiB，能容纳完整历史缓存及缺口通知。网络写入在释放公共缓存锁后进行；慢会话溢出会清空本会话旧帧并插入缺口，不阻塞其他会话。配额满时在流开始前返回普通 JSON：HTTP 429、业务码 -3001、`Retry-After: 5`。关闭、发送失败和服务停止均释放配额。

### `GET /api/logs/file/latest`

成功为原始 `text/plain; charset=utf-8`，并带 `Content-Disposition`、固定 `Content-Length`、`Cache-Control: no-store`、`X-Rocos-Log-Snapshot-Time`。跨域响应暴露附件名称、快照时间、SSE 限额和重试头。

文件为可读文本格式，扩展名 `.log`。下载包含该段文件中所有通过文件门槛的级别，包括默认开启的 `info`，不合并其他轮转文件。请求不能指定文件系统路径。文件线程完成完整记录、刷新后，在同一次操作中选择当前文件、打开读取 fd、确定名称与长度。HTTP 每次最多读取 16 KiB；后续写入不进入本响应。POSIX 下旧目录项被轮换清理后，已打开 fd 仍然可读。

快照只包含切点前已写入的条目，不等待上游队列排空。快照请求邮箱最多 4 个，默认等待 250 ms。文件不存在返回 404；配额满返回 429；暂不可用或快照超时返回 503；正文开始前探测到读取错误返回 500。错误为普通 JSON，资源错误使用业务码 -3001。正文开始后读取/发送失败会终止响应，不向文件正文追加 JSON。

`fflush` 只刷新用户态缓冲，不是 `fsync`，不承诺断电持久化。清理在 POSIX 下删除目录项；下载期间文件实际占用空间可能暂时超过名义历史容量。

## 实时入口与隔离

`log_event.hpp/.cpp` 是独立的实时实现单元：固定大小 `RtLogEvent`（不超过 64 字节），每个已注册线程独立预分配 1024 槽 SPSC（可用 1023 条），最多 8 个生产者。编译期检查索引和丢弃计数原子始终无锁。入队满立即返回 false，原子累计丢弃，不重试、不打印、不回退。

在进入周期前申请生产者、绑定 TLS 并采样日历/单调时钟基准。周期内只复用已有 PerformanceProfiler 的时间采样，复制数值；后台转换为日历时间。时间分辨率由已有采样点决定，不代表日志调用指令的精确硬件时间。实时入口不格式化、不分配、不加等待锁、不读硬件、不执行文件/网络/时钟系统调用。

`emitLogEvent` 在已绑定实时线程上仅调用上述固定入口；其他线程使用允许格式化的后备路径。Robot 周期结果和可由周期触发的状态日志、Executor 阶段失败、PositionController 位置/速度超限、Model 计算失败、MoveJoint 逐周期曲线失败、Hardware PDO 大小与消费者限额日志已迁移。旧 `Logger::getInstance` 和 `Logger::logCode` 仅供非实时路径使用；同名实例并发复用。Lua、启动、参数与离线规划日志补充英文说明和可用错误码。

普通入口最多 1024 条、单条消息约 8 KiB，竞争或满时丢弃；事件线程到文件线程使用独立 2048 槽 SPSC（可用 2047 条）。文件/终端卡住时只影响文件线程；事件线程仍可发布告警。生产丢弃通过 gap 报告，文件丢弃/写入失败不递归打印错误。`LogManager::metrics()` 提供 RT/普通/文件丢弃、文件失败、缓存淘汰、缓存条数/字节和客户端缺口计数；这些指标当前是 C++ 诊断入口，未新增 HTTP 字段。

退出顺序应为停止控制生产者、关闭诊断会话、排出事件和文件队列、关闭存储。生产者租约在 Robot 构造失败和销毁时均会释放。持续卡死的内核磁盘 I/O 仍可能延长文件线程退出，线程隔离不等于操作系统资源隔离。

## 验证与限制

离线测试不建立 Robot，不连接或使能硬件：

```bash
cmake -S . -B build/logging -DCMAKE_BUILD_TYPE=Debug
cmake --build build/logging -j2
ctest --test-dir build/logging -R '^(logging_system_test|diagnostics_http_memory_test|app_config_test|position_controller_test)$' --output-on-failure
# 在允许本地 socket 的开发环境单独执行：
ctest --test-dir build/logging -R '^diagnostics_http_test$' --output-on-failure
```

`logging_system_test` 覆盖预分配队列、分配计数、并发顺序、生产溢出、缓存双上限、查询、序列化、文件轮换与固定句柄、受控文件线程停顿、写入失败及实例复用。`diagnostics_http_memory_test` 使用实际 cpp-httplib HTTP 解析、路由和响应生成，经内存 Stream 验证 SSE、下载和错误分支，不创建监听 socket。`diagnostics_http_test` 为真实回环 TCP 测试，包含并发配额、心跳及慢下载。

首次后台任务执行环境禁止创建 socket（`EPERM`）；后续交互会话已解除该限制并运行真实回环 TCP 测试，见文末续修记录。没有运行需要 Robot、真实 Hardware、模拟器的 FSM、Lua 运动及运动集成测试。目标硬件单次入队最大耗时、控制抖动、局域网 P95 <100 ms、真实磁盘满和内核 I/O 挂起均需准备好的环境实测，不能从本地测试推断硬实时达标。既有控制路径本身仍包含锁、分配和硬件等待，本次保证范围仅为已迁移的日志入口。

## Apifox 状态

目标为项目 8387612、模块 7912103。首次后台任务的 MCP 刷新与读取被自动审批拒绝；2026-10-22 的后续交互会话已成功刷新并读取当前连接协议，API 版本为 3.0.0，索引中没有三个拟新增日志路径；已核对现有 schema 索引和 StandardResponse。当前读取工具没有提供可确认的目标分支标识。

没有进行线上写入。可审阅的三个接口/模型局部草案位于 `/tmp/rocos-logging-apifox-delta.json`，仅用于审核新增差异，不应直接全量导入。实际同步仍需确认目标分支及只读 MCP 之外的写入授权；写入后必须重新 MCP 刷新和回读。已删除的旧 OpenAPI 副本不恢复。

## 首次后台验证记录（2026-10-22 08:00 任务）

- `cmake -S . -B build/logging -DCMAKE_BUILD_TYPE=Debug`：通过。
- `cmake --build build/logging -j2`：完整项目构建通过，版本保持原来的 `3.0.1-rc.5`。仅构建了硬件/运动例程，没有运行它们。
- `git diff --check`：通过。当前环境没有 clang-format 可执行文件；新模块使用四空格，已有文件保持周边风格。
- `logging_system_test`：16 个用例通过；`diagnostics_http_memory_test`：8 个用例通过，包含实际等待的 15 秒心跳。
- 扩展离线回归执行 18 项，14 项通过、4 项失败；完整输出 `/tmp/rocos-logging-final-tests.log`。

扩展回归实际命令：

```bash
ctest --test-dir build/logging -R '^(model_test|hardware_test|position_controller_test|joint_admittance_controller_test|joint_impedance_controller_test|cartesian_impedance_controller_test|unit_velocity_profile_test|performance_profiler_test|move_joint_test|move_servo_test|move_line_test|move_line_offline_test|move_circle_test|move_circle_three_point_test|virtual_wall_test|app_config_test|logging_system_test|diagnostics_http_memory_test)$' --output-on-failure
```

失败项与本次日志修改无关的证据：将 HEAD 中的对应控制器源文件单独编译并优先链接到相同测试，复现相同失败；未建立或切换空白工作树，也未恢复工作区源文件。临时源码、对象、测试程序和输出位于 `/tmp/rocos-logging-baseline-symbols/`。

| 失败测试 | 结果与原因 |
| --- | --- |
| joint_admittance_controller_test | SIGABRT / pure virtual method；夹具先销毁 FakeHardware，控制器析构随后调用硬件 |
| joint_impedance_controller_test | 同上 |
| cartesian_impedance_controller_test | 同上 |
| virtual_wall_test | 包装 PositionController 用例缺少限位数组维度，返回既有 `UnmatchedJointsNumber=-2332`，两条断言失败 |

没有为获得绿灯而修改这些无关控制器行为或跳过其中的失败断言。`hardware_test` 仅运行 YAML 解析用例；未运行会建立真实 Robot 的 FSM/Lua 测试，也未运行真实控制主程序。

Sanitizer 命令与结果：

```bash
cmake -S . -B build/logging-sanitized -DCMAKE_BUILD_TYPE=Debug \
  '-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer' \
  '-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=address,undefined' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined'
cmake --build build/logging-sanitized --target logging_system_test diagnostics_http_memory_test -j2
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir build/logging-sanitized -R '^(logging_system_test|diagnostics_http_memory_test)$' --output-on-failure
```

两项均通过，输出 `/tmp/rocos-logging-sanitizer-tests.log`。启用 LeakSanitizer 的首轮中，24 个用例的断言通过，但退出扫描报告受限环境不支持所需操作，命令失败；因此内存泄漏扫描未验证，不能据此宣称没有泄漏。

`nm -u build/logging/CMakeFiles/logger.dir/src/log_event.cpp.o` 仅列出 ELF/TLS 与栈保护符号（`_GLOBAL_OFFSET_TABLE_`、`__tls_get_addr`、`__stack_chk_fail`），没有分配、格式化、文件、网络、时钟或互斥锁依赖。结合绑定 TLS 后的实时路径分配计数测试，支持此实现单元的源码/本机构建边界；不替代目标平台 page fault、TLS、缓存一致性和调度延迟实测。

真实回环测试 `ctest --test-dir build/logging -R '^diagnostics_http_test$' --output-on-failure` 因环境禁止 socket 创建而失败；独立最小 socket 程序也返回 `PermissionError: EPERM`。没有提升权限或将该失败记为通过。受控文件线程停顿和 `RLIMIT_FSIZE` 写入/刷新失败已测；实际 ENOSPC、内核不可中断 I/O 挂起与真实 TCP 慢客户端仍未测。

本次保持旧 OpenAPI 副本的删除、未跟踪计划和 API 技能；没有 Git 提交、版本递增、标签、推送或发布。Apifox 刷新/读取受到自动审批拒绝，未确认当前版本或分支，未发生线上写入或同步。

## 续修验证记录（2026-10-22 交互会话）

本节更新上述首次后台记录的当前结论：本会话可以创建本地监听 socket，也可以执行 LeakSanitizer；Apifox MCP 刷新和读取已经恢复，线上写入仍未执行。

修复了四项既有测试夹具：三个阻抗/导纳测试将非拥有的 Hardware、Model 放在控制器之前构造，确保控制器析构时依赖仍然存在；虚拟墙测试补齐 FakeFKModel 的关节限位数组，并让包装控制器用例的模型和硬件均为六轴。控制器实现及原有断言未因这些修复改变。

真实 TCP 回归新增断言先复现了两个协议缺口：跨域 `OPTIONS /api/logs/stream` 返回 404，SSE 配额耗尽的 429 响应缺少 `X-Rocos-SSE-Limit`。修复后三个只读路径均支持预检，SSE 入口在拒绝连接前也设置限额头。独立只读代码复查未发现阻塞性问题。

```bash
cmake --build build/logging -j2
ctest --test-dir build/logging -E '^(robot_fsm_test|lua_script_engine_test)$' --output-on-failure -j2

cmake --build build/logging-sanitized --target logging_system_test diagnostics_http_memory_test diagnostics_http_test -j2
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir build/logging-sanitized -R '^(logging_system_test|diagnostics_http_memory_test|diagnostics_http_test)$' --output-on-failure -j2
git diff --check
```

- 完整项目构建通过；本轮 CTest 注册并运行的 71 项全部通过，包含第三方测试和真实回环 TCP 日志接口测试，耗时 17.77 秒。
- 更新后日志相关的三个 Sanitizer 测试目标全部通过，耗时 18.53 秒；本轮启用了 ASan、UBSan 和 LeakSanitizer，没有报告对应问题。此结果仅覆盖这些测试执行到的路径。
- `git diff --check` 通过。CTest 输出可查对应构建目录的 `Testing/Temporary/LastTest.log`。
- 三个 Apifox 接口与三个新增模型的局部草案已完成本地引用检查；当前在线协议 API 版本 3.0.0，尚无这三个日志接口。目标分支及替代写入授权仍待确认，不能声称线上同步完成。

未启动真实控制主程序、使能硬件或运行运动集成测试；目标机实时性、真实局域网延迟与磁盘耗尽测试仍需专用环境。没有提交、递增版本或推送；保留工作区原有的旧 OpenAPI 文件删除。
