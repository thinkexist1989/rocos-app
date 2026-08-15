/**
 * @file lua_runner.cpp
 * @brief Lua 脚本命令行客户端 —— 通过 HTTP API 上传并运行 Lua 脚本
 *
 * 用法: lua_runner <脚本文件名> [host] [port]
 * 示例: lua_runner example.lua                 # 连接本机默认端口 8080
 *       lua_runner example.lua 127.0.0.1 8080
 *
 * 工作流程:
 *   1. 读取本地 Lua 脚本源码
 *   2. 使用 debug 令牌直接控制（X-Rocos-Control-Token: debug-token），无需 acquire/release
 *   3. POST /api/script/upload  上传并编译脚本（filename + source）
 *   4. POST /api/script/run     异步执行脚本
 *   5. 轮询 GET /api/script/status 直到 COMPLETED / FAILED / STOPPED
 *
 * 前置条件:
 *   - rocosAppMain 已启动（默认 --http_port=8080），且服务端未注释掉 debug 令牌旁路
 *   - 接口定义见 docs/ROCOS  API.openapi.yaml 的 "Lua 脚本" 与 "控制权" 分组
 */

#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <httplib.h>
#include <json.hpp>

namespace {

constexpr const char* kControlTokenHeader = "X-Rocos-Control-Token";
constexpr const char* kClientIdHeader    = "X-Rocos-Client-Id";
constexpr const char* kContentTypeJson   = "application/json";
// 与服务端 RobotHttpServer 的 DEBUG_TOKEN / DEBUG_CLIENT_ID 保持一致
constexpr const char* kDebugToken        = "debug-token";
constexpr const char* kDebugClientId     = "debug-client";

volatile sig_atomic_t g_stop_requested = 0;

// SIGINT 只置位标志，真正的停止/释放逻辑放到主循环里做（避免在信号处理器里调用非安全函数）
void signalHandler(int signo) {
    if (signo == SIGINT) {
        g_stop_requested = 1;
    }
}

// 解析响应体；连接失败或响应非 JSON 对象时返回 false
bool parseResponse(const httplib::Result& res, nlohmann::json& out) {
    if (!res) {
        return false;
    }
    out = nlohmann::json::parse(res->body, nullptr, false);
    return out.is_object();
}

// 从响应 JSON 安全提取 message/code（连接失败或非法响应时给通用提示）
std::string responseError(const nlohmann::json& j) {
    if (!j.is_object()) {
        return "连接失败或响应非法";
    }
    return j.value("message", std::string("unknown")) +
           " (code=" + std::to_string(j.value("code", -1)) + ")";
}

// 脚本名仅取 basename，作为上传时的逻辑文件名
std::string baseName(const std::string& path) {
    return std::filesystem::path(path).filename().generic_string();
}

// 解析脚本路径：优先按原样打开；裸文件名（无目录分隔符）找不到时回退到 scripts/ 目录，
// 保持与旧版 lua_runner（以 scripts 为根目录）的兼容用法 `lua_runner example.lua`。
std::string resolveScriptPath(const std::string& path) {
    std::ifstream probe(path);
    if (probe.is_open()) {
        return path;
    }
    if (path.find('/') == std::string::npos) {
        const std::string fallback = "scripts/" + path;
        probe.open(fallback);
        if (probe.is_open()) {
            return fallback;
        }
    }
    return path;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <脚本文件名> [host] [port]\n"
                  << "示例: " << argv[0] << " example.lua 127.0.0.1 8080\n";
        return 1;
    }

    const std::string script_file = argv[1];
    const std::string host        = (argc >= 3) ? argv[2] : "127.0.0.1";
    const int         port        = (argc >= 4) ? std::stoi(argv[3]) : 8080;

    // 1. 读取本地脚本源码
    const std::string resolved_path = resolveScriptPath(script_file);
    std::ifstream in(resolved_path);
    if (!in.is_open()) {
        std::cerr << "[ERROR] 无法打开脚本文件: " << script_file << "\n";
        return 1;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    const std::string source = buffer.str();

    std::cout << "[INFO] 目标服务器: " << host << ":" << port << "\n";
    std::cout << "[INFO] 脚本文件: " << resolved_path << "\n";

    if (signal(SIGINT, signalHandler) == SIG_ERR) {
        std::cerr << "[ERROR] 无法注册 SIGINT 信号处理器\n";
        return 1;
    }

    httplib::Client cli(host, port);
    cli.set_connection_timeout(5);

    nlohmann::json j;

    // 2. 使用 debug 令牌（绕过 acquire/release，仅用于本地测试）
    const httplib::Headers headers = {
        {kControlTokenHeader, kDebugToken},
        {kClientIdHeader, kDebugClientId},
    };
    std::cout << "[INFO] 使用 debug 令牌（" << kDebugToken << "/" << kDebugClientId << "）\n";

    // 3. 上传脚本（仅加载编译到内存，不写文件系统）
    {
        nlohmann::json body;
        body["filename"] = baseName(resolved_path);
        body["source"]   = source;
        auto res = cli.Post("/api/script/upload", headers, body.dump(), kContentTypeJson);
        if (!parseResponse(res, j) || !j.value("success", false)) {
            std::cerr << "[ERROR] 上传脚本失败: " << responseError(j) << "\n";
            return 1;
        }
    }
    std::cout << "[INFO] 脚本上传成功\n";

    // 4. 运行脚本（异步，立即返回）
    {
        auto res = cli.Post("/api/script/run", headers, "", kContentTypeJson);
        if (!parseResponse(res, j) || !j.value("success", false)) {
            std::cerr << "[ERROR] 启动脚本失败: " << responseError(j) << "\n";
            return 1;
        }
    }
    std::cout << "[INFO] 开始执行脚本 ...\n";

    // 5. 轮询状态直到结束
    std::string state;
    std::string error;
    int consecutive_failures = 0;
    while (!g_stop_requested) {
        auto res = cli.Get("/api/script/status");
        if (!parseResponse(res, j) || !j.value("success", false)) {
            if (++consecutive_failures > 10) {
                std::cerr << "[ERROR] 查询脚本状态失败\n";
                break;
            }
        } else {
            consecutive_failures = 0;
            const auto& data = j["data"];
            state = data.value("state", "");
            error = data.value("error", "");
            if (state == "COMPLETED" || state == "FAILED" || state == "STOPPED") {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 收到 SIGINT：请求停止脚本并读取最终状态
    if (g_stop_requested) {
        std::cout << "\n[SIGNAL] 收到中断，正在停止脚本 ...\n";
        cli.Post("/api/script/stop", headers, "", kContentTypeJson);
        auto res = cli.Get("/api/script/status");
        if (parseResponse(res, j) && j.value("success", false)) {
            state = j["data"].value("state", "");
            error = j["data"].value("error", "");
        }
    }

    std::cout << "[INFO] 脚本结束, 状态: " << (state.empty() ? "UNKNOWN" : state) << "\n";
    if (!error.empty()) {
        std::cerr << "[ERROR] 错误信息: " << error << "\n";
    }

    return state == "COMPLETED" ? 0 : 1;
}
