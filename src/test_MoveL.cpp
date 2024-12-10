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

DEFINE_string(l_urdf, "/home/sun/Documents/GitHub/rocos-app/config/HEBUT_DOUBLE/l_robot.urdf", "Urdf file path");
DEFINE_string(l_base, "base_link", "Base link name");
DEFINE_string(l_tip, "l_link_7", "Tip link name");
DEFINE_bool(l_sim, true, "Sim or not");
DEFINE_int32(l_id, 0, "hardware id, only work for real hardware");

DEFINE_string(r_urdf, "/home/sun/Documents/GitHub/rocos-app/config/HEBUT_DOUBLE/r_robot.urdf", "Urdf file path");
DEFINE_string(r_base, "base_link", "Base link name");
DEFINE_string(r_tip, "r_link_7", "Tip link name");
DEFINE_bool(r_sim, true, "Sim or not");
DEFINE_int32(r_id, 1, "hardware id, only work for real hardware");
std::ofstream outfile;
bool isRuning = true;

rocos::Robot *l_robot_ptr = nullptr;
rocos::Robot *r_robot_ptr = nullptr;

void signalHandler(int signo)
{
    if (signo == SIGINT)
    {
        std::cout << "\033[1;31m"
                  << "[!!SIGNAL!!]"
                  << "INTERRUPT by CTRL-C"
                  << "\033[0m" << std::endl;

        isRuning = false;

        l_robot_ptr->setDisabled();
        r_robot_ptr->setDisabled();
        outfile.close();

        exit(0);
    }
}
void threadsave()
{
//    建立csv存储文件

    outfile.open("MoveLSync.csv");
    outfile << "time, x, y, z, r, p, y" << std::endl;

    while (isRuning)
    {
        KDL::Frame l_currentFrame = l_robot_ptr->getFlange();
        KDL::Frame r_currentFrame = r_robot_ptr->getFlange();
        double r, p, y;
        l_currentFrame.M.GetRPY(r, p, y);


        outfile <<"left:"<< l_currentFrame.p.x() << "," << l_currentFrame.p.y() << "," << l_currentFrame.p.z() << "," << r << "," << p << "," << y << std::endl;
        r_currentFrame.M.GetRPY(r, p, y);
        outfile <<"right:"<< r_currentFrame.p.x() << "," << r_currentFrame.p.y() << "," << r_currentFrame.p.z() << "," << r << "," << p << "," << y << std::endl;
        usleep(1000000);
    }

}
void waitForMoveLCompletion(rocos::Robot *l_robot_ptr, rocos::Robot *r_robot_ptr)
{

    while(l_robot_ptr->getRunState()!=rocos::Robot::RunState::Stopped || r_robot_ptr->getRunState()!=rocos::Robot::RunState::Stopped)
    {

        if(l_robot_ptr->isThreadWaiting()&&r_robot_ptr->isThreadWaiting())
        {
            std::cout<<"both are waiting"<<std::endl;
            l_robot_ptr->triggerSync();//按道理静态类条件变量，可以触发多个等待线程
            usleep(10000);
            // r_robot_ptr->triggerSync();
        }
        //左臂报错退出，右臂在等待，触发右臂
        else if (l_robot_ptr->getRunState()==rocos::Robot::RunState::Stopped && r_robot_ptr->isThreadWaiting())
        {
            std::cout<<"left is stopped, right is waiting"<<std::endl;
            r_robot_ptr->triggerSync();
            usleep(10000);
        }
        //右臂报错退出，左臂在等待，触发左臂
        else if (r_robot_ptr->getRunState()==rocos::Robot::RunState::Stopped && l_robot_ptr->isThreadWaiting())
        {
            std::cout<<"left is waiting, right is stopped"<<std::endl;
            l_robot_ptr->triggerSync();
            usleep(10000);
        }
        else
        {
            usleep(1000000);
        }


    }

    std::cout<<"MoveL is completed"<<std::endl;



}

int main(int argc, char *argv[])
{
    if (signal(SIGINT, signalHandler) == SIG_ERR)
    {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

    using namespace rocos;

    gflags::ParseCommandLineFlags(&argc, &argv, true);
    //**-------------------------------**//

    // 初始化类(left)
    HardwareInterface *l_hw;
    if (FLAGS_l_sim)
        l_hw = new HardwareSim(20); // 仿真
    else
        l_hw = new Hardware(FLAGS_l_urdf, FLAGS_l_id); // 真实机械臂
    Robot l_robot(l_hw, FLAGS_l_urdf, FLAGS_l_base, FLAGS_l_tip);

    l_robot_ptr = &l_robot;

    auto l_robotService = RobotServiceImpl::getInstance(&l_robot);

    // 初始化类(right)
    HardwareInterface *r_hw;
    if (FLAGS_r_sim)
        r_hw = new HardwareSim(20); // 仿真
    else
        r_hw = new Hardware(FLAGS_r_urdf, FLAGS_r_id); // 真实机械臂
    Robot r_robot(r_hw, FLAGS_r_urdf, FLAGS_r_base, FLAGS_r_tip);

    r_robot_ptr = &r_robot;
    auto r_robotService = RobotServiceImpl::getInstance(&r_robot);


    l_robot_ptr->setEnabled();
    r_robot_ptr->setEnabled();
    JntArray q(7);
    JntArray l_q(7);
    JntArray r_q(7);
    double p = 0;
    q(0) = p / 180.0 * M_PI;
    std::cout << "q(0) = " << q(0) << std::endl;
    q(1) = p / 180.0 * M_PI;
    q(2) = p / 180.0 * M_PI;
    q(3) = p / 180.0 * M_PI;
    q(4) = p / 180.0 * M_PI;
    q(5) = p / 180.0 * M_PI;
    q(6) = p / 180.0 * M_PI;
    l_robot_ptr->MoveJ(q, 0.15, 0.15, 0.0, 0.0, true);
    r_robot_ptr->MoveJ(q, 0.15, 0.15, 0.0, 0.0, false);


    sleep(2);

    l_q(0) = 0.00 / 180.0 * M_PI;
    std::cout << "q(0) = " << q(0) << std::endl;
    l_q(1) = 45.00 / 180.0 * M_PI;
    l_q(2) = 0.00 / 180.0 * M_PI;
    l_q(3) = 90.00 / 180.0 * M_PI;
    l_q(4) = 0.00 / 180.0 * M_PI;
    l_q(5) = 45.00 / 180.0 * M_PI;
    l_q(6) = 0.00 / 180.0 * M_PI;
    l_robot_ptr->MoveJ(l_q, 0.15, 0.15, 0.0, 0.0, true);

    r_q(0) = 0.00 / 180.0 * M_PI;
    std::cout << "q(0) = " << q(0) << std::endl;
    r_q(1) = -45.00 / 180.0 * M_PI;
    r_q(2) = 0.00 / 180.0 * M_PI;
    r_q(3) = -90.00 / 180.0 * M_PI;
    r_q(4) = 0.00 / 180.0 * M_PI;
    r_q(5) = -45.00 / 180.0 * M_PI;
    r_q(6) = 0.00 / 180.0 * M_PI;
    r_robot_ptr->MoveJ(r_q, 0.15, 0.15, 0.0, 0.0, false);



    KDL::Frame currentFrame = l_robot_ptr->getFlange();
    std::cout << "currentFrame" << currentFrame.p.x() << "," << currentFrame.p.y() << "," << currentFrame.p.z() << "," << std::endl;
    double r, p1, y;
    currentFrame.M.GetRPY(r, p1, y);

    std::cout << "rpy" << r << "," << p << "," << y << std::endl;
    //开启线程
    std::thread th(threadsave);
    // initial Position
    KDL::Frame l_pose = KDL::Frame(KDL::Rotation::RPY(3.1415926, 0.00, 3.1415926), KDL::Vector(-0.560736, 0.00, 0.172071));
    KDL::Frame r_pose = KDL::Frame(KDL::Rotation::RPY(3.1415926, 0.00, 3.1415926), KDL::Vector(0.560736, 0.00, 0.172071));
    l_robot_ptr->MoveLSync(l_pose, 0.05, 0.05, 0.0, 0.0, true);
    r_robot_ptr->MoveLSync(r_pose, 0.05, 0.05, 0.0, 0.0, true);
    waitForMoveLCompletion(l_robot_ptr, r_robot_ptr);

    // start
    l_pose = KDL::Frame(KDL::Rotation::RPY(3.1415926, 0.00, 3.1415926), KDL::Vector(-0.560736, -0.20, 0.172071));
    r_pose = KDL::Frame(KDL::Rotation::RPY(3.1415926, 0.00, 3.1415926), KDL::Vector(0.560736, -0.20, 0.172071));
    l_robot_ptr->MoveLSync(l_pose, 0.05, 0.05, 0.0, 0.0, true);
    r_robot_ptr->MoveLSync(r_pose, 0.05, 0.05, 0.0, 0.0, true);
    waitForMoveLCompletion(l_robot_ptr, r_robot_ptr);

    // down
    l_pose = KDL::Frame(KDL::Rotation::RPY(3.1415926, 0.00, 3.1415926), KDL::Vector(-0.560736, -0.40, 0.172071));
    r_pose = KDL::Frame(KDL::Rotation::RPY(3.1415926, 0.00, 3.1415926), KDL::Vector(0.560736, -0.40, 0.172071));
    l_robot_ptr->MoveLSync(l_pose, 0.05, 0.05, 0.0, 0.0, true);
    r_robot_ptr->MoveLSync(r_pose, 0.05, 0.05, 0.0, 0.0, true);
    waitForMoveLCompletion(l_robot_ptr, r_robot_ptr);


    // up
    l_pose = KDL::Frame(KDL::Rotation::RPY(3.1415926, 0.00, 3.1415926), KDL::Vector(-0.560736, 0.30, 0.172071));
    r_pose = KDL::Frame(KDL::Rotation::RPY(3.1415926, 0.00, 3.1415926), KDL::Vector(0.560736, 0.30, 0.172071));
    l_robot_ptr->MoveLSync(l_pose, 0.05, 0.05, 0.0, 0.0, true);
    r_robot_ptr->MoveLSync(r_pose, 0.05, 0.05, 0.0, 0.0, true);
    waitForMoveLCompletion(l_robot_ptr, r_robot_ptr);


    // start
    l_pose = KDL::Frame(KDL::Rotation::RPY(3.1415926, 0.00, 3.1415926), KDL::Vector(-0.560736, -0.20, 0.172071));
    r_pose = KDL::Frame(KDL::Rotation::RPY(3.1415926, 0.00, 3.1415926), KDL::Vector(0.560736, -0.20, 0.172071));
    l_robot_ptr->MoveLSync(l_pose, 0.05, 0.05, 0.0, 0.0, true);
    r_robot_ptr->MoveLSync(r_pose, 0.05, 0.05, 0.0, 0.0, true);
    waitForMoveLCompletion(l_robot_ptr, r_robot_ptr);

    l_robotService->runServer("0.0.0.0:50001", false);

    l_robot_ptr->setDisabled();
    r_robot_ptr->setDisabled();
    return 0;
}
