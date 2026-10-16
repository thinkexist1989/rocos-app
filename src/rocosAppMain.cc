// Copyright 2021, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

#include <csignal>
#include <cstdio>
#include <cstdlib>

#include <rocos_app/drive.h>
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <fstream>
#include <iostream>
#include <rocos_app/robot.h>
#include <rocos_app/robot_service.h>
#include <rocos_app/robot_http_server.hpp>
#include <string>
#include <gflags/gflags.h>

DEFINE_string(urdf, "robot.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");
DEFINE_bool(sim, true, "Sim or not");
DEFINE_int32(id, 0, "hardware id, only work for real hardware");
DEFINE_bool(http, false, "Start the HTTP API server");
DEFINE_string(http_host, "0.0.0.0", "HTTP listening address");
DEFINE_int32(http_port, 8080, "HTTP listening port");
DEFINE_int32(http_control_ttl_seconds, 300, "HTTP control lease timeout in seconds");
DEFINE_bool(grpc, true, "Start the gRPC API server");
DEFINE_string(grpc_address, "0.0.0.0:30001", "gRPC listening address");

bool isRuning = true;

rocos::Robot *robot_ptr = nullptr;

void signalHandler(int signo) {
    if (signo == SIGINT || signo == SIGTERM) {
        std::cout << "\033[1;31m"
                  << "[!!SIGNAL!!]"
                  << "INTERRUPT by CTRL-C"
                  << "\033[0m" << std::endl;

        isRuning = false;

        if (robot_ptr) robot_ptr->setDisabled();

        exit(0);
    }
}


int main(int argc, char *argv[]) {
    if (signal(SIGINT, signalHandler) == SIG_ERR ||
        signal(SIGTERM, signalHandler) == SIG_ERR) {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

    using namespace rocos;

    gflags::ParseCommandLineFlags(&argc, &argv, true);
    if ((!FLAGS_http && !FLAGS_grpc) ||
        (FLAGS_http && (FLAGS_http_port < 1 || FLAGS_http_port > 65535 ||
                        FLAGS_http_control_ttl_seconds <= 0))) {
        std::cerr << "Enable HTTP or gRPC, use a valid HTTP port and a positive control TTL.\n";
        return EXIT_FAILURE;
    }
    if (!std::ifstream(FLAGS_urdf)) {
        std::cerr << "Cannot open URDF: " << FLAGS_urdf << '\n';
        return EXIT_FAILURE;
    }
    //**-------------------------------**//

    //**-------------启动admittance_joint-----------**//
    // 初始化类
    HardwareInterface* hw;
    if (FLAGS_sim)
        hw = new HardwareSim(20);  // 仿真
    else
        hw = new Hardware(FLAGS_urdf, FLAGS_id); // 真实机械臂

    Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);

    robot_ptr = &robot;

    try {
        std::unique_ptr<RobotHttpServer> httpServer;
        if (FLAGS_http) {
            httpServer = std::make_unique<RobotHttpServer>(&robot, nullptr, FLAGS_http_control_ttl_seconds);
            if (FLAGS_grpc) {
                httpServer->runAsync(FLAGS_http_host, FLAGS_http_port);
            } else {
                httpServer->run(FLAGS_http_host, FLAGS_http_port);
            }
        }
        if (FLAGS_grpc) {
            auto robotService = RobotServiceImpl::getInstance(&robot);
            robotService->runServer(FLAGS_grpc_address);
        }
    } catch (const std::exception& e) {
        std::cerr << "Server startup failed: " << e.what() << '\n';
        // The legacy Robot/DriveGuard background threads are process-scoped.
        // Match the signal shutdown path: do not destroy their live Robot here.
        std::exit(EXIT_FAILURE);
    }

    return 0;
}
