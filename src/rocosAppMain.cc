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
#include <rocos_app/ethercat/ft_sensor_ecat.h>
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
DEFINE_string(tip, "link_8", "Tip link name");

bool isRuning = true;

#pragma region

namespace rocos
{

    // boost::shared_ptr< HardwareInterface > hw = boost::make_shared< HardwareSim >( _joint_num );  // 仿真
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>(); // 真实机械臂
    FtSensorEcat ftsensor;

    Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);

    void signalHandler(int signo)
    {
        if (signo == SIGINT)
        {
            std::cout << "\033[1;31m"
                      << "[!!SIGNAL!!]"
                      << "INTERRUPT by CTRL-C"
                      << "\033[0m" << std::endl;

            isRuning = false;
            robot.stop_joint_admittance_teaching();
            std::cout << "stop teaching" << std::endl;
            robot.setDisabled();

            exit(0);
        }
    }

    void Robot::test()
    {
        //**变量初始化 **//

        JC_helper::TCP_server my_server;

        //**-------------------------------**//

        my_server.init();
        boost::thread(&JC_helper::TCP_server::RunServer, &my_server).detach(); // 开启服务器

        //**-------------------------------**//

#pragma region //*电机使能检查

        setEnabled();
#pragma endregion
        int ret = robot.joint_admittance_teaching(false);
        std::cout << "ret: " << ret << std::endl;
        //**-------------------------------**//

    } // namespace rocos

#pragma endregion

}

int main(int argc, char *argv[])
{
    if (signal(SIGINT, rocos::signalHandler) == SIG_ERR)
    {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

    using namespace rocos;

    gflags::ParseCommandLineFlags(&argc, &argv, true);
    //**-------------------------------**//

    //**-------------启动admittance_joint-----------**//
    // 初始化类

        float fx = ftsensor.getFx();
    float fy = ftsensor.getFy();
    float fz = ftsensor.getFz();
    float mx=ftsensor.getMx();
    float my=ftsensor.getMy();
    float mz=ftsensor.getMz();
    std::cout<<"Force:"<<fx<<","<<fy<<","<<fz<<","<<mx<<","<<my<<","<<mz<<std::endl;

    
    
    auto robotService = RobotServiceImpl::getInstance(&robot);
    // std::thread thread_test{ &rocos::Robot::test, &robot };
    //------------------------wait----------------------------------
    robotService->runServer();
    // thread_test.join();

    return 0;
}
