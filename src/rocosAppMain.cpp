#include <gflags/gflags.h>
// #include "hardware2.hpp"
// #include <rocos_app/ethercat/hardware_sim.h>

#include <csignal>
#include <iostream>
#include <string>

// #include "drive.hpp"
#include "robot.hpp"
#include "robot_http_server.hpp"

DEFINE_string(urdf, "robot.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");
DEFINE_bool(sim, true, "Sim or not");
DEFINE_int32(id, 0, "hardware id, only work for real hardware");
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
    //**-------------------------------**//

    // //**-------------启动admittance_joint-----------**//
    // // 初始化类
    // HardwareInterface* hw;
    // if (FLAGS_sim)
    //     hw = new HardwareSim(20);  // 仿真
    // else
    //     hw = new Hardware(FLAGS_urdf, FLAGS_id); // 真实机械臂
    //
    // Robot executor{};
    //
    // robot_ptr = &executor;
    //
    // rocos::RobotHttpServer httpServer(&executor);
    //
    // //------------------------wait----------------------------------
    // httpServer.runAsync(FLAGS_http_host, FLAGS_http_port);
    // // Keep main thread alive
    // std::cout << "\033[1;32m" << "[HTTP Server] Running on "
    //       << FLAGS_http_host << ":" << FLAGS_http_port << "\033[0m" << std::endl;
    // while (isRuning) {
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    // }

    return 0;
}
