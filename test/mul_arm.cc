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
#include <string>
#include <gflags/gflags.h>

DEFINE_string(urdf, "robot.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");
DEFINE_bool(sim, true, "Sim or not");

bool isRuning = true;

rocos::Robot *robot_ptr = nullptr;
rocos::Robot *robot_ptr1 = nullptr;
void signalHandler(int signo) {
    if (signo == SIGINT) {
        std::cout << "\033[1;31m"
                  << "[!!SIGNAL!!]"
                  << "INTERRUPT by CTRL-C"
                  << "\033[0m" << std::endl;

        isRuning = false;

        robot_ptr->setDisabled();
        robot_ptr1->setDisabled();
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

    //**-------------启动admittance_joint-----------**//
    // 初始化类
    // boost::shared_ptr<HardwareInterface> hw;
    boost::shared_ptr<HardwareInterface> hw1;
    int id=0;
    int id1=1;
    // hw = boost::make_shared<Hardware>(FLAGS_urdf,id); // 真实机械臂
    hw1 = boost::make_shared<Hardware>(FLAGS_urdf,id1); // 真实机械臂

    // Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);
    Robot robot1(hw1, FLAGS_urdf, FLAGS_base, FLAGS_tip);

    // robot_ptr = &robot;
    robot_ptr1 = &robot1;
    
    // auto robotService = RobotServiceImpl::getInstance(&robot);
    auto robotService1=RobotServiceImpl::getInstance(&robot1);
    //------------------------wait----------------------------------
    // robotService->runServer("127.0.0.1",true);
     robotService1->runServer("170.170.170.170:12345",false);
    
    

    return 0;
}
