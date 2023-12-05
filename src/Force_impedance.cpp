// 代码功能
// 笛卡尔空间下的力控制，以力打磨为例
// 作者：孙锦程
// 时间：2023年12月5日
// 版本：第一版
// 参考论文：HYBRID FORCE-IMPEDANCE CONTROL FOR FAST END-EFFECTOR MOTION

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
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <Eigen/Core>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames_io.hpp>
#include <Eigen/Dense>
#include "kdl/chainiksolvervel_pinv_nso.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
DEFINE_string(urdf, "robot_sun_new.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");
bool isRunning = true;
KDL::Tree kdl_tree;
KDL::Chain robotChain;

namespace rocos
{
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(_joint_num); // 仿真
    // boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>(); // 真实机械臂

    Robot *robot_ptr = new Robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);
    void signalHandler(int signo)
    {
        if (signo == SIGINT)
        {
            std::cout << "\033[1;31m"
                      << "[!!SIGNAL!!]"
                      << "INTERRUPT by CTRL-C"
                      << "\033[0m" << std::endl;

            isRunning = false;
            robot_ptr->setDisabled();
            // robot.setDisabled();
            exit(0);
        }
    }
    void Robot::test()
    {
        // 1.初始化
        KDL::JntArray q(_joint_num);
        KDL::JntArray qd(_joint_num);
        KDL::JntArray qdd(_joint_num);
        KDL::JntArray joint_torques(_joint_num);
        if (!kdl_parser::treeFromFile(FLAGS_urdf, kdl_tree)) // 建立tree
        {
            // 加载URDF文件失败，处理错误
            std::cout << "加载URDF文件失败" << std::endl;
            exit(0);
        }
        if (!kdl_tree.getChain(FLAGS_base, FLAGS_tip, robotChain)) // 建立运动链
        {
            // 获取机器人链失败，处理错误
            std::cout << "获取机器人链失败" << std::endl;
            exit(0);
        }

        // 1.移动到初始位置
        for (int i = 0; i < 1; i++)
        {
            q(0) = 0 * M_PI / 180.0;
            q(1) = 45 * M_PI / 180.0;
            q(3) = 90 * M_PI / 180.0;
            q(5) = -45 * M_PI / 180.0;
            q(6) = 0 * M_PI / 180.0;
        }

        robot_ptr->MoveJ(q, 1, 3);
        // 2. 通过理论力矩计算笛卡尔空间的力（test）
        // 2.1 计算理论力矩1.44137e-16 60.1491 -0.215822 21.8786 -0.0923832 3.71445 0
        KDL::Wrenches external_forces(robotChain.getNrOfSegments());

        KDL::Vector gravity(0.0, 0.0, 9.81); // 重力加速度
        KDL::ChainIdSolver_RNE rne_solver(robotChain, gravity);
        rne_solver.CartToJnt(q, qd, qdd, external_forces, joint_torques);
        // 打印理论力矩joint_torques
        PLOG_INFO << "理论力矩joint_torques: " << joint_torques(0) << " " << joint_torques(1) << " " << joint_torques(2) << " " << joint_torques(3) << " " << joint_torques(4) << " " << joint_torques(5) << " " << joint_torques(6) << " " << std::endl;
        // 2.2 求解雅可比矩阵（test）
        KDL::Jacobian jacobian(robotChain.getNrOfJoints());
        KDL::ChainJntToJacSolver jnt_to_jac_solver(robotChain);
        jnt_to_jac_solver.JntToJac(q, jacobian);
        // 打印雅可比矩阵
        PLOG_INFO << "雅可比矩阵:  " << jacobian.data << std::endl;
        // 2.3笛卡尔空间下的力映射到关节空间下的力（test）
        Eigen::VectorXd cartesian_force(6), cartesian_force1(6); // 6维笛卡尔空间力
        cartesian_force << -10, 0, 0, 0, 0, 0;                    // 6维笛卡尔空间力
        cartesian_force1 << 0, 0, -10, 0, 0, 0;                  // 6维笛卡尔空间力
        Eigen::MatrixXd jacobian_transpose = jacobian.data.transpose();
        // 将笛卡尔空间力映射到关节空间
        Eigen::VectorXd joint_space_force = jacobian_transpose * cartesian_force;
        Eigen::VectorXd joint_space_force1 = jacobian_transpose * cartesian_force1;
        // 打印映射得到的关节空间力
        std::cout << "映射得到的关节空间力:\n"
                  << joint_space_force << std::endl;
        std::cout << "映射得到的关节空间力:\n"
                  << joint_space_force1 << std::endl;

        // 阻抗控制模拟
        // 设置期望外力5N->cartesian_force
        JC_helper::spring_mass_dump smd{};
        int max_count = 100000000;
        int traj_count{0};
        smd.set_force(0, 0, 1);
    }
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

    // boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(_joint_num); // 仿真

    // auto robotService = RobotServiceImpl::getInstance(&robot);

    //------------------------wait----------------------------------
    std::thread thread_test{&rocos::Robot::test, robot_ptr};

    //------------------------wait----------------------------------
    // robotService->runServer();

    thread_test.join();

    return 0;
}
