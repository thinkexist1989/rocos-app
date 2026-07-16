/**
 * @file lua_runner.cpp
 * @brief Lua 脚本命令行启动器例程
 *
 * 用法: lua_runner <脚本文件名>
 * 示例: lua_runner example.lua
 *
 * 通过命令行参数传入脚本文件名，启动 Robot 并加载执行 Lua 脚本，
 * 等待脚本运行完毕后打印状态，最后安全退出。
 */

#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>

#include "../src/lua_script_engine.hpp"
#include "src/robot.hpp"

namespace {

rocos::Robot* g_robot = nullptr;
rocos::LuaScriptEngine* g_engine = nullptr;

/// SIGINT 信号处理 —— 停止脚本运行并清理退出
void signalHandler(int signo) {
    if (signo == SIGINT) {
        std::cout << "\n\033[1;33m[SIGNAL] 收到中断信号，正在停止...\033[0m\n";
        if (g_engine != nullptr) {
            g_engine->Stop();
        }
        if (g_robot != nullptr) {
            g_robot->setDisabled();
        }
        std::exit(0);
    }
}

/// 将 LuaScriptEngine::State 转为可读字符串
const char* stateToString(rocos::LuaScriptEngine::State s) {
    return rocos::LuaScriptEngine::ToString(s);
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <脚本文件名>\n"
                  << "示例: " << argv[0] << " example.lua\n";
        return 1;
    }

    const std::string script_file = argv[1];

    // 注册信号处理
    if (signal(SIGINT, signalHandler) == SIG_ERR) {
        std::cerr << "无法注册 SIGINT 信号处理器\n";
        return 1;
    }

    using namespace rocos;

    // 初始化机器人（内部自动加载硬件配置与运动学模型）
    std::cout << "[INFO] 正在初始化 Robot ...\n";
    Robot robot;
    g_robot = &robot;

    // 构造 Lua 脚本引擎，以 config/scripts 为脚本根目录
    LuaScriptEngine engine(robot, "scripts");
    g_engine = &engine;

    // 加载脚本文件
    std::cout << "[INFO] 加载脚本: " << script_file << "\n";
    Result rc = engine.LoadFile(script_file);
    if (rc != Result::NoError) {
        std::cerr << "[ERROR] 加载脚本失败, 错误码: " << static_cast<int>(rc) << "\n";
        return 1;
    }

    // 运行脚本
    std::cout << "[INFO] 开始执行脚本 ...\n";
    rc = engine.Run();
    if (rc != Result::NoError) {
        std::cerr << "[ERROR] 启动脚本失败, 错误码: " << static_cast<int>(rc) << "\n";
        return 1;
    }

    // 轮询等待脚本结束
    while (true) {
        auto status = engine.GetStatus();
        if (status.state == LuaScriptEngine::State::Completed ||
            status.state == LuaScriptEngine::State::Failed ||
            status.state == LuaScriptEngine::State::Stopped) {
            std::cout << "[INFO] 脚本结束, 状态: " << stateToString(status.state) << "\n";
            if (!status.error.empty()) {
                std::cerr << "[ERROR] 错误信息: " << status.error << "\n";
            }
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    auto final_status = engine.GetStatus();
    return final_status.state == LuaScriptEngine::State::Completed ? 0 : 1;
}
