#include <gflags/gflags.h>
// #include "hardware2.hpp"
// #include <rocos_app/ethercat/hardware_sim.h>

#include <csignal>
#include <iostream>
#include <string>

// #include "drive.hpp"
#include "robot.hpp"
#include "robot_http_server.hpp"

DEFINE_string(http_host, "0.0.0.0", "HTTP server listen host");
DEFINE_int32(http_port, 8080, "HTTP server listen port");

bool isRuning = true;

rocos::Robot *robot_ptr = nullptr;

void signalHandler(int signo) {
    if (signo == SIGINT) {
        std::cout << "\033[1;31m"
                  << "[!!SIGNAL!!]"
                  << "INTERRUPT by CTRL-C"
                  << "\033[0m" << std::endl;

        isRuning = false;

        robot_ptr->setDisabled();

        exit(0);
    }
}


int main(int argc, char *argv[]) {
    if (signal(SIGINT, signalHandler) == SIG_ERR) {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

    using namespace rocos;

    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // 初始化机器人（内部自动加载硬件与运动学模型）
    Robot robot;
    robot_ptr = &robot;

    // 启动 HTTP REST API 服务器（非阻塞）
    RobotHttpServer httpServer(&robot);
    httpServer.runAsync(FLAGS_http_host, FLAGS_http_port);

    std::cout << "\033[1;32m"
              << "[HTTP Server] Running on "
              << FLAGS_http_host << ":" << FLAGS_http_port
              << "\033[0m" << std::endl;

    // 主线程保活，等待 SIGINT
    while (isRuning) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
