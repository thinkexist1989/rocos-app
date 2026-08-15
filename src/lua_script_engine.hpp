#ifndef ROCOS_APP_LUA_SCRIPT_ENGINE_H
#define ROCOS_APP_LUA_SCRIPT_ENGINE_H

#include "result.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace rocos {

class Robot;

class LuaScriptEngine final {
public:
    enum class State {
        Empty,
        Loaded,
        Running,
        Pausing,
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

    Result LoadSource(const std::string& source, const std::string& filename);
    Result LoadFile(const std::filesystem::path& relative_path);
    /// @brief 只读脚本文件内容（不加载进 Lua VM），供脚本下载接口使用
    Result ReadScriptFile(const std::filesystem::path& relative_path,
                          std::string& source) const;
    /// @brief 列出 scripts 根目录下可用的 Lua 脚本（相对路径）
    [[nodiscard]] std::vector<std::string> ListScripts() const;
    Result Run();
    Result Pause();
    Result Resume();
    Result Stop();
    Result Step();

    Result AddBreakpoint(const std::string& filename, int line);
    Result RemoveBreakpoint(const std::string& filename, int line);
    Result ClearBreakpoints();

    [[nodiscard]] Status GetStatus() const;
    [[nodiscard]] static const char* ToString(State state) noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rocos

#endif  // ROCOS_APP_LUA_SCRIPT_ENGINE_H
