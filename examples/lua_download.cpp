/**
 * @file lua_download.cpp
 * @brief Lua 脚本下载客户端 —— 测试脚本下载接口
 *
 * 用法: lua_download [选项] [文件名]
 *   无文件名              列出 scripts/ 下所有脚本
 *   <文件名>               下载指定脚本到当前目录
 *   --all                  下载全部脚本到 downloaded_scripts/
 *   --host HOST            服务器地址（默认 127.0.0.1）
 *   --port PORT            端口（默认 8080）
 *
 * 对应接口:
 *   GET /api/script/list               列出可用脚本
 *   GET /api/script/download?filename= 下载单个脚本源码（text/plain）
 *
 * 前置条件: rocosAppMain 已启动（默认 --http_port=8080）
 */

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <httplib.h>
#include <json.hpp>

namespace {

constexpr const char* kDefaultHost = "127.0.0.1";
constexpr int         kDefaultPort = 8080;

// 解析响应 JSON；连接失败或响应非对象返回 false
bool parseResponse(const httplib::Result& res, nlohmann::json& out) {
    if (!res) {
        return false;
    }
    out = nlohmann::json::parse(res->body, nullptr, false);
    return out.is_object();
}

std::string responseError(const nlohmann::json& j) {
    if (!j.is_object()) {
        return "连接失败或响应非法";
    }
    return j.value("message", std::string("unknown")) +
           " (code=" + std::to_string(j.value("code", -1)) + ")";
}

bool writeFile(const std::string& path, const std::string& content) {
    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) {
        return false;
    }
    out << content;
    return out.good();
}

void printUsage(const char* prog) {
    std::cout << "用法: " << prog << " [选项] [文件名]\n"
              << "  无文件名              列出 scripts/ 下所有脚本\n"
              << "  <文件名>               下载指定脚本到当前目录\n"
              << "  --all                  下载全部脚本到 downloaded_scripts/\n"
              << "  --host HOST            服务器地址（默认 " << kDefaultHost << "）\n"
              << "  --port PORT            端口（默认 " << kDefaultPort << "）\n"
              << "示例: " << prog << " example.lua --host 127.0.0.1 --port 8080\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string host = kDefaultHost;
    int         port = kDefaultPort;
    std::string filename;
    bool        download_all = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--host" && i + 1 < argc) {
            host = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = std::stoi(argv[++i]);
        } else if (arg == "--all") {
            download_all = true;
        } else if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else if (!arg.empty() && arg[0] == '-') {
            std::cerr << "[ERROR] 未知选项: " << arg << "\n";
            printUsage(argv[0]);
            return 1;
        } else {
            filename = arg;
        }
    }

    httplib::Client cli(host, port);
    cli.set_connection_timeout(5);

    std::cout << "[INFO] 目标服务器: " << host << ":" << port << "\n";

    nlohmann::json j;

    // 1. 列出可用脚本
    std::vector<std::string> scripts;
    {
        auto res = cli.Get("/api/script/list");
        if (!parseResponse(res, j) || !j.value("success", false)) {
            std::cerr << "[ERROR] 获取脚本列表失败: " << responseError(j) << "\n";
            return 1;
        }
        const auto& data = j["data"];
        if (data.contains("scripts") && data["scripts"].is_array()) {
            for (const auto& item : data["scripts"]) {
                scripts.push_back(item.get<std::string>());
            }
        }
    }

    if (scripts.empty()) {
        std::cout << "[INFO] scripts/ 目录下没有脚本\n";
        return 0;
    }

    std::cout << "[INFO] 可用脚本（共 " << scripts.size() << " 个）:\n";
    for (const auto& s : scripts) {
        std::cout << "  - " << s << "\n";
    }

    // 2. 决定要下载哪些脚本
    std::vector<std::string> to_download;
    if (download_all) {
        to_download = scripts;
    } else if (!filename.empty()) {
        to_download = {filename};
    } else {
        return 0;  // 仅列出
    }

    // 3. 逐个下载并保存
    int ok = 0;
    for (const auto& name : to_download) {
        auto res = cli.Get("/api/script/download?filename=" + name);
        if (!res || res->status != 200) {
            std::cerr << "[ERROR] 下载失败: " << name
                      << (res ? " (HTTP " + std::to_string(res->status) + ")"
                              : "（连接失败）")
                      << "\n";
            continue;
        }

        // 成功返回 text/plain；失败返回 application/json（{success:false,...}）
        if (res->get_header_value("Content-Type").find("application/json") !=
            std::string::npos) {
            nlohmann::json err = nlohmann::json::parse(res->body, nullptr, false);
            std::cerr << "[ERROR] 下载失败 " << name << ": " << responseError(err)
                      << "\n";
            continue;
        }

        std::filesystem::path dest =
            download_all ? std::filesystem::path("downloaded_scripts") / name
                         : std::filesystem::path(name);

        std::error_code ec;
        if (dest.has_parent_path()) {
            std::filesystem::create_directories(dest.parent_path(), ec);
        }
        if (ec) {
            std::cerr << "[ERROR] 创建目录失败: " << dest.parent_path() << "\n";
            continue;
        }
        if (!writeFile(dest.string(), res->body)) {
            std::cerr << "[ERROR] 写入文件失败: " << dest << "\n";
            continue;
        }

        std::cout << "[INFO] 已下载: " << name << " -> " << dest << "\n";
        ++ok;
    }

    std::cout << "[INFO] 完成: 成功 " << ok << "/" << to_download.size() << "\n";
    return ok == static_cast<int>(to_download.size()) ? 0 : 1;
}
