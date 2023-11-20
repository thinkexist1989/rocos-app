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
#include <yaml-cpp/yaml.h>
DEFINE_string(urdf, "robot_sun_new.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");

bool isRuning = true;

#pragma region

namespace rocos
{

    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(7); // 仿真
                                                                                  // boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>(); // 真实机械臂

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
            robot.setDisabled();

            exit(0);
        }
    }
    void readPoseFromYAML(const YAML::Node &yamlNode, const std::string &poseName, KDL::Frame &pose)
    {

        if (yamlNode[poseName])
        {
            const YAML::Node &poseNode = yamlNode[poseName];
            double x = poseNode[0].as<double>();
            double y = poseNode[1].as<double>();
            double z = poseNode[2].as<double>();
            double roll = poseNode[3].as<double>() * M_PI / 180.0;
            double pitch = poseNode[4].as<double>() * M_PI / 180.0;
            double yaw = poseNode[5].as<double>() * M_PI / 180.0;

            pose.p = KDL::Vector(x, y, z);
            // RPY
            pose.M = KDL::Rotation::RPY(roll, pitch, yaw);
        }
    }
    void readJointFromYAML(const YAML::Node &yamlNode, const std::string &poseName, KDL::JntArray &q)
    {

        if (yamlNode[poseName])
        {
            const YAML::Node &poseNode = yamlNode[poseName];
            for (int i = 0; i < 7; i++)
            {
                q(i) = poseNode[i].as<double>() / 180.0 * M_PI;
            }
        }
    }
    void Robot::test()
    {
        //**变量初始化 **//

        JC_helper::TCP_server my_server;
        my_server.init();
        boost::thread(&JC_helper::TCP_server::RunServer, &my_server).detach(); // 开启服务器

        
        std::string yaml_path = "Violence.yaml";
        YAML::Node yaml_node;
        yaml_node = YAML::LoadFile(yaml_path);
        KDL::Frame pose;
        KDL::JntArray q_target(_joint_num);
        int num_joint;
        int pose_joint;
        num_joint = yaml_node["num_joints"].as<int>();
        pose_joint = yaml_node["num_poses"].as<int>();
        double Vel_joint =  yaml_node["Vel_joint"].as<double>();
        double Acc_joint =  yaml_node["Acc_joint"].as<double>();
        double Vel_pose=  yaml_node["Vel_pose"].as<double>();
        double Acc_pose=  yaml_node["Acc_pose"].as<double>();
        double Vel_circle=  yaml_node["Vel_circle"].as<double>();
        double Acc_circle=  yaml_node["Acc_circle"].as<double>();
        double length=  yaml_node["length"].as<double>();

        KDL::Frame f_p0;
        KDL::Frame f_p1;
        KDL::Frame f_p2;
        KDL::Frame f_p3;
        KDL::Frame f_p4;
        KDL::Frame f_p5;
        KDL::Frame f_p6;
        //**-------------------------------**//

#pragma region //*电机使能检查

        setEnabled();
#pragma endregion

        //**-------------------------------**//

        while (isRuning)
        {
            //**关节空间的运动 **//
            // 每个循环读取一次yaml文件，其中joint是目标关节角，pose是目标位姿，joint的名字从joint1到num_joint每次读取的名字都是累加1的
            // pose的名字从pose1到pose_joint每次读取的名字都是累加1的
            // for (int i = 1; i < num_joint; i++)
            // {
            //     std::string jointName = "joint" + std::to_string(i);
            //     // std::string poseName="pose"+std::to_string(i);
            //     readJointFromYAML(yaml_node, jointName, q_target);
            //     // readPoseFromYAML(yaml_node,poseName,pose);
            //     robot.MoveJ(q_target, Vel_joint, Acc_joint);
            // }

            // **笛卡尔空间的运动 **//
            // for(int i=1;i<pose_joint;i++)
            // {
            //     std::string poseName="pose"+std::to_string(i);
            //     readPoseFromYAML(yaml_node,poseName,pose);
            //     robot.MoVel_joint(pose,Vel_joint,Acc_joint);
            // }
            //**笛卡尔空间的运动 **//
            // 正方形
            q_target(1)=M_PI_4;
            q_target(3)=M_PI_2;
            q_target(5)=-M_PI_4;
            MoveJ( q_target, Vel_joint, Acc_joint, 0, 0, false);
            kinematics_.JntToCart(q_target, f_p0);

            f_p1 = f_p0 * KDL::Frame{KDL::Vector{sqrt(length)/2, sqrt(length)/2, 0.0}};
            MoveJ_IK(f_p1,Vel_joint,Acc_joint);
           
            

            
            f_p2 = f_p0 * KDL::Frame{KDL::Vector{sqrt(length)/2, -sqrt(length)/2, 0.0}};
            MoveL(f_p2, Vel_pose, Acc_pose, 0, 0, false);
            f_p3 = f_p0 * KDL::Frame{KDL::Vector{-sqrt(length)/2, -sqrt(length)/2, 0.0}};
            MoveL(f_p3, Vel_pose, Acc_pose, 0, 0, false);
            f_p4 = f_p0 * KDL::Frame{KDL::Vector{-sqrt(length)/2, sqrt(length)/2, 0.0}};
            MoveL(f_p4, Vel_pose, Acc_pose, 0, 0, false);
            MoveL(f_p1, Vel_pose, Acc_pose, 0, 0, false);

            // 画圆
            MoveJ(q_target,Vel_joint,Acc_joint);

            kinematics_.JntToCart(q_target, f_p0);

            KDL::Frame f_c_p1 = f_p0 * KDL::Frame{KDL::Vector{-length, 0.0, 0.0}};
            MoveL(f_c_p1,Vel_circle,Acc_circle);
            KDL::Frame f_c_p2 = f_p0 * KDL::Frame{KDL::Vector{0.0, length, 0.0}};
            KDL::Frame f_c_p3 = f_p0 * KDL::Frame{KDL::Vector{length, 0, 0.0}};
            MoveC(f_c_p2, f_c_p3,Vel_circle,Acc_circle, 0, 0, Robot::OrientationMode::FIXED, false);
        }

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
    //     boost::shared_ptr< HardwareInterface > hw = boost::make_shared< HardwareSim >( 7 );  // 仿真
    //    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>(); // 真实机械臂

    //    Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);

    auto robotService = RobotServiceImpl::getInstance(&robot);
    std::thread thread_test{&rocos::Robot::test, &robot};
    //------------------------wait----------------------------------
    robotService->runServer();
    thread_test.join();

    return 0;
}
