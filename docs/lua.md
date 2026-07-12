# Lua 脚本编程功能实施计划

## 问题与目标

为 ROCOS-App 增加一个独立的 `LuaScriptEngine` 类，通过 sol2 + Lua 5.4 执行机器人程序。Lua 脚本可使用顺序、分支、循环和函数组织运动逻辑；`MoveJ`、`MoveL`、`MoveC` 默认阻塞到当前运动完成。类同时提供脚本位置查询、断点、暂停/继续、停止和逐 Lua source line 单步，并通过现有 `RobotHttpServer` 暴露 REST API。

首版明确边界：

- C++17，vendoring 固定版本的 sol2 与 Lua 5.4 到 `3rdparty/`，构建过程不联网。
- 默认 Sandbox，仅开放 `base`、`math`、`string`、`table` 等安全 library；不开放 `os`、`io`、`package`、`debug`。
- HTTP 通过 `source + filename` 加载；C++ `LoadFile` 仅允许读取配置的 scripts 根目录。
- 单步粒度为 Lua source line，不实现 `step into/over/out` 和局部变量查看。
- 同一 engine 同时只允许一个 loaded/running script，避免脚本间争用唯一的机器人运动执行通道。
- 脚本线程不进入 1 kHz realtime control loop，只调用 `Robot` 的线程安全公开运动 API。

## 类设计

新增公开类：

```cpp
namespace rocos {

class LuaScriptEngine final {
public:
    enum class State {
        Empty,
        Loaded,
        Running,
        Paused,
        Stopping,
        Completed,
        Failed,
        Stopped
    };

    struct SourceLocation {
        std::string filename;
        int line{0};
    };

    struct Status {
        State state{State::Empty};
        std::string script_id;
        SourceLocation location;
        std::string error;
        bool motion_active{false};
        std::vector<SourceLocation> breakpoints;
    };

    LuaScriptEngine(Robot& robot, std::filesystem::path scripts_root);
    ~LuaScriptEngine();

    LuaScriptEngine(const LuaScriptEngine&) = delete;
    LuaScriptEngine& operator=(const LuaScriptEngine&) = delete;

    Result LoadSource(const std::string& source,
                      const std::string& filename);
    Result LoadFile(const std::filesystem::path& relative_path);
    Result Run();
    Result Pause();
    Result Resume();
    Result Stop();
    Result Step();

    Result AddBreakpoint(const std::string& filename, int line);
    Result RemoveBreakpoint(const std::string& filename, int line);
    Result ClearBreakpoints();
    [[nodiscard]] Status GetStatus() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rocos
```

`Impl` 使用 PImpl 隐藏 sol2/Lua 头文件、worker thread、mutex、condition variable、breakpoint set 和执行细节，使公开头文件保持稳定。`Robot&` 是 non-owning reference，要求 `Robot` 生命周期长于 engine；析构时先请求停止、停止活动运动并 join worker。

### 内部状态与并发

- `LoadSource` 在 worker 未运行时使用 `sol::load` 只编译不执行，保存 protected function、source、规范化 filename 和递增 `script_id`；语法错误进入 `Failed` 并保留错误与行号。
- `Run` 从 `Loaded` 启动唯一 worker；`Paused` 只能走 `Resume`。重复 `Run`、运行中重新加载、无脚本 `Step` 等均返回明确状态错误。
- Lua VM 仅由 worker thread 访问；HTTP/C++ 控制方法只修改 mutex 保护的 control flags，并通过 condition variable 唤醒 worker。
- `Status` 返回 immutable snapshot，不把 sol object 或 Lua stack 暴露给其他线程。
- 允许的状态迁移：
  - `Empty -> Loaded`
  - `Loaded -> Running -> Completed | Failed | Stopped`
  - `Loaded -> Running(one-line budget) -> Paused`
  - `Running <-> Paused`
  - `Paused -> Running(one-line budget) -> Paused`
  - `Running | Paused -> Stopping -> Stopped`
  - terminal state 可由下一次成功 load 返回 `Loaded`

### 断点、位置与单步

- 使用 Lua C API `lua_sethook`，启用 `LUA_MASKLINE | LUA_MASKCOUNT`。line event 更新 `filename + currentline`；count event 保证同一 source line 内的 tight loop 仍能及时响应 `Stop`。
- breakpoint key 为规范化 chunk filename 与 1-based line。line hook 在执行该行之前暂停。
- `Step` 设置 one-line budget：允许当前行继续执行，在下一个有效 line event 前重新进入 `Paused`。在 `Loaded` 上调用 `Step` 会启动 worker 并在执行一个 source line 后暂停。
- breakpoint 命中后设置 `Paused` 并在 condition variable 上等待，不阻塞 HTTP server thread。
- `Stop` 唤醒所有等待；hook 通过受保护的 Lua error 中止 chunk，由 worker 统一映射为 `Stopped`，不把用户停止误报为 runtime error。

### Lua bindings

在全局 `robot` table 中注册小而明确的 API：

```lua
robot.MoveJ({0.0, -0.5, 0.8, 0.0, 0.5, 0.0}, 1.0, 2.0, 10.0)
robot.MoveL({
    x = 0.4, y = 0.0, z = 0.3,
    qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0
}, "", 0.2, 0.5, 2.0)

if robot.GetState() == "STOPPED" then
    local q1 = robot.GetJointPosition(1) -- Lua index 使用 1-based
end
```

首版 bindings：

- `MoveJ(joints, velocity, acceleration, jerk)`
- `MoveL(pose, tool_name, velocity, acceleration, jerk)`
- 两个命名清晰的圆弧接口，避免 Lua overload 歧义：
  - `MoveCByCenter(start_pose, center_pose, theta, velocity, acceleration, jerk)`
  - `MoveCByPoints(start_pose, via_pose, goal_pose, velocity, acceleration, jerk)`
- `GetState()`、`IsEnabled()`、`GetJointCount()`、`GetJointPosition(index)`
- `Sleep(milliseconds)`，使用可中断 condition variable，不使用不可取消的裸 `sleep_for`

所有 table、维度、finite number、quaternion、positive limit 和 1-based joint index 在进入 `Robot` 前校验。C++ `Result` 非成功时抛出带业务错误码的 Lua runtime error，worker 将 filename、line 和消息保存到 `Status::error`。

### 阻塞运动语义

- binding 调用 `Robot::MoveJ/MoveL/MoveC` 提交运动，然后在 script worker 内等待 Robot FSM 离开 `RUNNING/PAUSING/PAUSED/RESUMING/STOPPING`。
- `STOPPED` 表示该段正常完成；`ERROR_STATE`、意外 `IDLE` 或提交错误使脚本进入 `Failed`。
- engine 的 `Pause` 在普通 Lua 代码中只关闭脚本执行 gate；若 `motion_active`，同时调用 `Robot::PauseMotion()`，待 FSM 进入 `PAUSED` 后保持脚本暂停。
- `Resume` 对活动运动先调用 `Robot::ResumeMotion()`，再放开脚本 gate。
- `Stop` 对活动运动调用 `Robot::StopMotion()`，等待控制侧进入安全停止状态，同时终止 Lua chunk。
- 不在 Lua 层复制轨迹规划；bindings 只负责 typed conversion、调用现有 Robot API 和等待结果。

## HTTP REST API

`RobotHttpServer` 接收 non-owning `LuaScriptEngine*`，仅做 JSON validation、业务码映射和序列化，不保存重复的脚本状态。

新增路由：

| Method | Path | 作用 |
|---|---|---|
| `POST` | `/api/script/load` | body: `{filename, source}`，编译并加载 |
| `POST` | `/api/script/load-file` | body: `{path}`，只允许 scripts root 内相对路径 |
| `POST` | `/api/script/run` | 异步启动，立即返回 `script_id` |
| `POST` | `/api/script/pause` | 暂停脚本，并在运动中协调 Robot pause |
| `POST` | `/api/script/resume` | 继续脚本/活动运动 |
| `POST` | `/api/script/stop` | 安全停止活动运动并终止脚本 |
| `POST` | `/api/script/step` | 执行一个 Lua source line |
| `GET` | `/api/script/status` | 返回 state、script_id、filename、line、error、motion_active、breakpoints |
| `POST` | `/api/script/breakpoint/add` | body: `{filename, line}` |
| `POST` | `/api/script/breakpoint/remove` | body: `{filename, line}` |
| `POST` | `/api/script/breakpoint/clear` | 清空断点 |

扩展业务码文档，使用 `-6xxx` 表示脚本模块：

- `-6001` script state conflict
- `-6002` Lua syntax/runtime error
- `-6003` invalid breakpoint
- `-6004` script path outside allowed root / file unavailable
- `-6005` script stopped

通用 JSON/字段错误继续使用 `1xxx`，Robot/motion 错误继续映射现有 `2xxx/3xxx`，不把 HTTP status 当业务码。

## 文件与构建变更

- `3rdparty/lua/`：固定 Lua 5.4 release source，提供 CMake target `lua::lua`。
- `3rdparty/sol2/`：固定 sol2 release source，使用其 header target。
- `CMakeLists.txt`：加入两项依赖、新 engine source、include/link 设置、测试 target，并复制 `config/scripts/*.lua` 到 runtime config。
- `include/rocos_app/lua_script_engine.h`：稳定公开接口、状态与快照类型。
- `src/lua_script_engine.cpp`：PImpl、Sandbox、bindings、worker、hook、blocking motion 和错误转换。
- `src/robot_http_server.hpp/.cpp`：注入 engine、注册 routes、JSON/status 映射。
- `src/rocosAppMain.cpp`：在 Robot 后构造 engine，再注入 HTTP server；确保销毁顺序为 server -> engine -> Robot。
- `config/scripts/example.lua`：最小顺序运动/条件控制示例。
- `docs/http_api_design.md`、`docs/http_api_return_codes.md`：脚本 API、request/response、状态和 `-6xxx` 业务码。
- `test/lua_script_engine_test.cc`：doctest 单元测试。
- `test/lua_script_http_test.cc`：HTTP handler/controller integration test；若现有 server 不便注入 client，则先覆盖序列化与 engine 调用边界，不引入新测试框架。

## 实施步骤与验证标准

1. **集成依赖与构建 target**
   - 固定 sol2/Lua 5.4 版本，关闭不需要的 Lua CLI/test target。
   - 验证：全新 CMake configure 不联网，`robot` 与最小 Lua smoke test 可链接。

2. **实现 `LuaScriptEngine` 基础生命周期**
   - 完成 PImpl、Sandbox、source/file load、run/stop、状态快照和 protected error。
   - 验证：合法脚本完成；语法/运行错误携带 filename + line；禁止 `os/io/package/debug`；scripts root path traversal 被拒绝；析构不遗留线程。

3. **实现 debugger control**
   - 完成 source hook、当前位置、断点、pause/resume、line step、tight-loop stop。
   - 验证：断点停在执行前的正确行；连续 step 每次只跨一个有效 source line；运行中状态查询无 data race；单行无限循环可停止。

4. **绑定 Robot 运动与查询 API**
   - 完成参数转换和 `MoveJ/MoveL/MoveC` 阻塞等待，协调 pause/resume/stop。
   - 验证：非法维度/NaN/Inf/错误状态不会提交运动；多段运动严格顺序执行；脚本 pause/stop 会同步影响活动运动；Robot error 使脚本失败。

5. **接入 HTTP 与应用生命周期**
   - 增加 script routes、业务码映射和 main wiring。
   - 验证：HTTP load -> run/step -> status -> breakpoint -> stop 全链路；并发冲突返回 `6001`；status 始终对应当前 `script_id`。

6. **补齐测试、示例和文档**
   - 覆盖状态迁移、Sandbox、path restriction、hook/debug、参数校验、HTTP JSON contract 和 shutdown。
   - 验证：项目已有 build 与 ctest 通过，新 example 可在 simulation 环境加载执行。

## 注意事项

- 当前 `RobotHttpServer` 多个 motion handler 仍是注释 stub，`rocosAppMain` 的启动逻辑也被注释；Lua 功能只接通自身 routes 和必要生命周期，不顺带重写无关 HTTP motion endpoints。
- 当前运动完成信息主要来自 Robot FSM，而非独立 command result future。首版按现有公开状态实现等待；若实施时确认 FSM 无法区分“正常完成”和“外部停止”，只新增最小 completion snapshot/condition variable 到 `Robot`，不引入第二套 executor。
- breakpoint 只对当前 loaded chunk 的规范化 filename 生效；首版 Sandbox 禁止 `require/dofile/loadfile`，因此不处理多文件 call stack。
- Lua VM 不提供 realtime guarantee；Lua 只负责 orchestration，所有实时轨迹与硬件写入仍由现有 Robot/Executor/control thread 完成。

## 当前 C++ 使用方式

脚本引擎不依赖 HTTP，由调用方显式保证 `Robot` 生命周期长于
`LuaScriptEngine`：

```cpp
#include <rocos_app/lua_script_engine.h>

#include "robot.hpp"

rocos::Robot robot;
rocos::LuaScriptEngine engine(robot, "scripts");

const rocos::Result load_result = engine.LoadFile("example.lua");
if (load_result == rocos::Result::NoError) {
    const rocos::Result run_result = engine.Run();
}
```

`Run()` 异步启动脚本 worker。调用方通过 `GetStatus()` 查询
`RUNNING/PAUSING/PAUSED/COMPLETED/FAILED/STOPPED`，退出前可调用 `Stop()`；析构函数也会停止并
join worker。`LoadFile()` 只接受 scripts root 内的相对路径，并拒绝 `..` 或
symlink 指向根目录之外的文件。
