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
#include <Eigen/QR>
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
    // boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(_joint_num); // 仿真
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>(); // 真实机械臂

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
        #pragma region  //*电机使能检查
        string str;
        for ( int i{ 0 }; i < jnt_num_; i++ )
        {
            if ( joints_[ i ]->getDriveState( ) != DriveState::OperationEnabled )
            {
                for ( int j{ 0 }; j < 1; j++ )
                {
                    PLOG_ERROR << "电机[" << i << "] 未使能，确定主站已初始化完成了？,输入y确认";
                    std::cin >> str;
                    if ( str != std::string_view{ "y" } )
                    {
                        PLOG_ERROR << "未输入y, 判断主站 {未} 初始化完成,程序关闭";

                        exit( 0 );
                    }
                }
            }
        }

        setEnabled( );
#pragma endregion




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

        robot_ptr->MoveJ(q, 0.2, 0.2);
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
        cartesian_force << -5, 0, 0, 0, 0, 0;                    // 6维笛卡尔空间力
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
        Eigen::MatrixXd jacobian_pinv(6, 7);
        // *(joint_space_force+joint_space_force1);
        std::cout << jacobian.data.rows() << std::endl;
        std::cout << jacobian.data.cols() << std::endl;
        Eigen::VectorXd tau(7);
        std::cout << (joint_space_force + joint_space_force1).rows() << std::endl;
        std::cout << (joint_space_force + joint_space_force1).cols() << std::endl;
        std::cout << jacobian_transpose.completeOrthogonalDecomposition().pseudoInverse().rows() << std::endl;
        std::cout << jacobian_transpose.completeOrthogonalDecomposition().pseudoInverse().cols() << std::endl;
        std::cout << jacobian_transpose.completeOrthogonalDecomposition().pseudoInverse() << std::endl;
        std::cout << (joint_space_force + joint_space_force1) << std::endl;
        jacobian_pinv = jacobian_transpose.completeOrthogonalDecomposition().pseudoInverse();
        tau = (joint_space_force + joint_space_force1);
        std::cout << "映射得到的笛卡尔空间力:\n"
                  << jacobian_pinv * tau << std::endl;
        // 阻抗控制模拟
        // 设置期望外力5N->cartesian_force
        JC_helper::spring_mass_dump smd{};

        JC_helper::admittance_joint admittance_joint(robot_ptr);

        int max_count = 100000000;
        int traj_count{0};
        KDL::Frame frame_offset{};
        KDL::Twist admittance_vel;
        KDL::JntArray _q_init{_joint_num};
        KDL::JntArray joints_vel(_joint_num);
        KDL::JntArray _q_target(_joint_num);
        KDL::JntArray current_pos(_joint_num);
        KDL::JntArray last_pos(_joint_num);
        KDL::JntArray last_last_pos(_joint_num);
        KDL::ChainIkSolverVel_pinv _ik_vel{robot_ptr->kinematics_.getChain()};
        KDL::JntArray opt_joint(7);
        KDL::JntArray weight_joint(7);

        PLOG_DEBUG << "开始";

        for (int i = 0; i < 7; i++)
        {
            opt_joint(i) = q(i);
            weight_joint(i) = 1e7;
        }
        usleep(4000000);
        admittance_joint.get_actual_torques(robot_ptr, q, qd, qdd);
            PLOG_DEBUG << "外力：" << admittance_joint.Actual_torques[0] << " " << admittance_joint.Actual_torques[1] << " " << admittance_joint.Actual_torques[2] << " " << admittance_joint.Actual_torques[3] << " " << admittance_joint.Actual_torques[4] << " " << admittance_joint.Actual_torques[5] << " " << admittance_joint.Actual_torques[6] << " " << std::endl;
            jnt_to_jac_solver.JntToJac(q, jacobian);
            jacobian_transpose = jacobian.data.transpose();
            jacobian_pinv = jacobian_transpose.completeOrthogonalDecomposition().pseudoInverse();
            cartesian_force1=jacobian_pinv * admittance_joint.Actual_torques;
            std::cout << "映射得到的笛卡尔空间力:\n"
                      << cartesian_force1 << std::endl;

        //** 程序初始化 **//

        for (int i = 0; i < 7; i++)
        {
            _q_init(i) = q(i);

            _q_target(i) = _q_init(i);
            current_pos(i) = q(i);

            last_pos(i) = current_pos(i);
            last_last_pos(i) = current_pos(i);
        }

        smd.set_k(0);
        smd.set_m(30);
        smd.set_damp(100);

        double force_z = 0;
        double last_force_z = 0;
        double force_z_target = 1.0;
        double force_z_dot = 0.0;
        double force_z_command = 0.0;
        std::atomic<bool> on_stop_trajectory{false};
        for (; traj_count < max_count; traj_count++)
        {
            admittance_joint.get_actual_torques(robot_ptr, current_pos, qd, qdd);
            std::cout<< "外力：" << admittance_joint.Actual_torques[0] << " " << admittance_joint.Actual_torques[1] << " " << admittance_joint.Actual_torques[2] << " " << admittance_joint.Actual_torques[3] << " " << admittance_joint.Actual_torques[4] << " " << admittance_joint.Actual_torques[5] << " " << admittance_joint.Actual_torques[6] << " " << std::endl;
            jnt_to_jac_solver.JntToJac(current_pos, jacobian);
            jacobian_transpose = jacobian.data.transpose();
            jacobian_pinv = jacobian_transpose.completeOrthogonalDecomposition().pseudoInverse();
            cartesian_force=jacobian_pinv * admittance_joint.Actual_torques-cartesian_force1;
             std::cout << "映射得到的笛卡尔空间力:\n"
                      << cartesian_force[2] << std::endl;
        
          
        //         // force_z从1到0,中间间隔0.0001
     
               force_z_command=cartesian_force[2];
            //    if(abs(force_z_command) <0.3)
            //    force_z_command=0;
            //    if(force_z_command>5)
            //    {
               
            //     force_z_command=5.0;
            //    }
            //    else if(force_z_command<-5)
            //    {
            //         force_z_command=-5.0;
            //    }
                // std::cout<<"force_z_command: "<<force_z_command<<std::endl;
                smd.set_force(0, 0, force_z_command);
                smd.calculate(frame_offset, admittance_vel);
              
                // PLOG_INFO << "admittance_vel: " << admittance_vel(0) << " " << admittance_vel(1) << " " << admittance_vel(2) << " " << admittance_vel(3) << " " << admittance_vel(4) << " " << admittance_vel(5) << " " << std::endl;

                // 打印admittance_vel
                //** 笛卡尔速度求解 **//
                if (_ik_vel.CartToJnt(_q_init, admittance_vel, joints_vel) != 0)
                {
                    PLOG_ERROR << "ik_vel.CartToJnt  ERROR";
                    break;
                }
                KDL::Multiply(joints_vel, 0.001, joints_vel);
                KDL::Add(_q_init, joints_vel, _q_target);
                //**-------------------------------**//

                //** 速度和加速度保护 **//

                if (on_stop_trajectory)
                    break;

                if (JC_helper::check_vel_acc(_q_target, current_pos, last_pos, 5, 50) < 0)
                {
                    on_stop_trajectory = true;
                    break;
                }

                last_last_pos = last_pos;
                last_pos = current_pos;
                current_pos = _q_target;

                //** 位置伺服 **//
                //! 提供位置保护，防止越过关节限位

                JC_helper::safety_servo(robot_ptr, _q_target);

                //**-------------------------------**//

                _q_init = _q_target;

                on_stop_trajectory = false; // 由外界调用者决定什么时候停止
            }

            if (on_stop_trajectory)
            {
                PLOG_ERROR << "IK 触发急停";
                JC_helper::Joint_stop(robot_ptr, current_pos, last_pos, last_last_pos);
                isRunning = true; // 告知调用者，因为速度太大而线程已停止
            }
            else
                PLOG_INFO << "IK结束";
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

    auto robotService = RobotServiceImpl::getInstance(robot_ptr);

    //------------------------wait----------------------------------
    std::thread thread_test{&rocos::Robot::test, robot_ptr};

    //------------------------wait----------------------------------
    robotService->runServer();

    thread_test.join();

    return 0;
}
