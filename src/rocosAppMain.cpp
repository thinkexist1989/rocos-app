#include <gflags/gflags.h>

#include <csignal>
#include <filesystem>
#include <iostream>
#include <string>

#include "app_config.hpp"
#include "lua_script_engine.hpp"
#include "robot.hpp"
#include "robot_http_server.hpp"

DEFINE_string(config, ROCOS_DEFAULT_CONFIG_FILE, "Path to the startup YAML configuration");
DEFINE_string(http_host, "0.0.0.0", "Override HTTP server listen host from config");
DEFINE_int32(http_port, 8080, "Override HTTP server listen port from config");
DEFINE_int32(http_control_ttl_seconds, 300, "Override HTTP control rights TTL from config (positive)");

bool isRuning = true;
rocos::Robot* robot_ptr = nullptr;

void signalHandler(int signo) {
    if (signo == SIGINT) {
        std::cout << "\033[1;31m"
                  << "[!!SIGNAL!!]"
                  << "INTERRUPT by CTRL-C"
                  << "\033[0m" << std::endl;
        isRuning = false;
        if (robot_ptr != nullptr) robot_ptr->setDisabled();
        exit(0);
    }
}

int main(int argc, char* argv[]) {
    using namespace rocos;
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    try {
        AppConfig config = AppConfig::load(FLAGS_config);
        // 仅显式传入的命令行参数覆盖 YAML，未传入时使用文件配置。
        if (!gflags::GetCommandLineFlagInfoOrDie("http_host").is_default) config.http_host = FLAGS_http_host;
        if (!gflags::GetCommandLineFlagInfoOrDie("http_port").is_default) config.http_port = FLAGS_http_port;
        if (!gflags::GetCommandLineFlagInfoOrDie("http_control_ttl_seconds").is_default) {
            config.http_control_ttl_seconds = FLAGS_http_control_ttl_seconds;
        }
        config.validate();

        // 在初始化硬件前检查必需资源，避免错误路径引起部分启动。
        for (const auto& path : {config.robot.hardware_config, config.robot.urdf}) {
            if (!std::filesystem::is_regular_file(path)) {
                throw std::runtime_error("Configuration resource not found: " + path);
            }
        }
        if (!std::filesystem::is_directory(config.scripts)) {
            throw std::runtime_error("Scripts directory not found: " + config.scripts);
        }
        std::cout << "[Config] " << std::filesystem::absolute(FLAGS_config).lexically_normal() << std::endl;

        Robot robot(config.robot);
        robot_ptr = &robot;
        if (signal(SIGINT, signalHandler) == SIG_ERR) {
            std::cerr << "Cannot catch SIGINT" << std::endl;
        }
        LuaScriptEngine script_engine(robot, config.scripts);
        RobotHttpServer httpServer(&robot, &script_engine, config.http_control_ttl_seconds, config.web_root);
        httpServer.runAsync(config.http_host, config.http_port);

        std::cout << "\033[1;32m[HTTP Server] Running on " << config.http_host << ":" << config.http_port << "\033[0m"
                  << std::endl;
        while (isRuning) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        robot_ptr = nullptr;
    } catch (const std::exception& error) {
        robot_ptr = nullptr;
        std::cerr << "Startup failed (config=" << FLAGS_config << "): " << error.what() << std::endl;
        return 1;
    }
    return 0;
}
