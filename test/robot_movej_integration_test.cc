// Robot HTTP integration example.
//
// Run from the project root after the hardware/EtherCAT runtime is ready:
//   ./build/bin/robot_movej_integration_test

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "src/robot.hpp"
#include "src/robot_http_server.hpp"

namespace {

volatile std::sig_atomic_t is_running = 1;
rocos::Robot* robot_ptr = nullptr;

void signalHandler(int signo) {
    if (signo == SIGINT) {
        std::cout << "\033[1;31m"
                  << "[!!SIGNAL!!] INTERRUPT by CTRL-C"
                  << "\033[0m" << std::endl;
        is_running = 0;
    }
}

}  // namespace

int main() {
    if (std::signal(SIGINT, signalHandler) == SIG_ERR) {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

    rocos::Robot robot;
    robot_ptr = &robot;

    rocos::RobotHttpServer http_server(&robot);
    http_server.runAsync("0.0.0.0", 12345);

    std::cout << "\033[1;32m"
              << "[HTTP Server] Running on 0.0.0.0:12345"
              << "\033[0m" << std::endl;

    while (is_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    http_server.stop();
    if (robot_ptr != nullptr) {
        robot_ptr->SetDisabled();
    }

    return 0;
}
