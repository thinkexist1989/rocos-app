

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
using namespace std::chrono;
DEFINE_string(urdf, "meshes/GSR.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_8", "Tip link name");
DEFINE_bool(sim, true, "Sim or not");
DEFINE_int32(id, 0, "hardware id, only work for real hardware");
KDL::Tree kdl_tree;
KDL::Chain robotChain;
bool isRuning = true;
std::ofstream outputFile_246("joint_torques_246joint.csv");
std::ofstream outputFile_3("joint_torques_3joint.csv");
std::ofstream outputFile_5("joint_torques_5joint.csv");

const float samplingrate = 1000; // Hz
const float cutoff_frequency = 3;
#pragma region //*测试9  完整上电保护程序
namespace rocos
{
    /**
     * @brief 字符串切割函数
     *
     * @param str 待切割字符串
     * @param tokens 结果存储
     * @param delim 切割符
     */
    // HardwareInterface*  hw = new HardwareSim(20);  // 仿真
    HardwareInterface* hw=new Hardware(FLAGS_urdf, FLAGS_id);; // 真实机械臂

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
            outputFile_246.close();
            outputFile_3.close();
            outputFile_5.close();
            std::cout << "Data saved to joint_torques.csv" << std::endl;
            exit(0);
        }
    }
    

    void Robot::test()
    {

        int _joint_num = 7;                                  /* Your joint number */
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

#pragma region // RNE
        KDL::JntArray joints(_joint_num);
        KDL::JntArray last_joints(_joint_num);
        KDL::JntArray joint_velocities(_joint_num);
        KDL::JntArray joint_accelerations(_joint_num);
        KDL::JntArray joint_real_torques(_joint_num);
        KDL::JntArray joint_cmd_torques(_joint_num);
        KDL::JntArray q_target1(_joint_num);
        KDL::JntArray q_target2(_joint_num);
        KDL::JntArray q_target3(_joint_num);
        KDL::JntArray q_target4(_joint_num);
        KDL::JntArray q_target5(_joint_num);
        KDL::JntArray q_target_mid(_joint_num);
        // Define external forces (Wrenches) acting on each segment (link)
        KDL::Wrenches external_forces(robotChain.getNrOfSegments());
        std::cout << "the num of robot is :" << robotChain.getNrOfSegments() << std::endl;
        // Set external forces (e.g., due to end-effector contact or other external factors)
        // Define gravity vector
        KDL::Vector gravity(0.0, 0.0, 9.81); // Assuming gravity in the negative Z direction
        // Create the ChainIdSolver_RNE object
        KDL::ChainIdSolver_RNE rne_solver(robotChain, gravity);
        // Create a KDL JntArray to store computed joint torques
        KDL::JntArray joint_torques(_joint_num);
        int row_index = 1;
        
#pragma endregion

#pragma region //*电机使能检查

        setEnabled();

#pragma endregion

#pragma region //*滤波器

#pragma endregion
#pragma region // Open a file for writing
        double joint_tor[_joint_num];
        q_target1(0) =-M_PI/2.0;

        // if (outputFile_246.is_open())
        // {
            
        //     for (double angle = 90.0; angle >= -90.0; angle -= 2.0)
        //     {
        //         q_target1(1) = angle / 180.0 * M_PI;
        //         MoveJ(q_target1, 0.1, 0.1);
        //         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        //         for (int i = 0; i < _joint_num; ++i)
        //         {
        //             // Set initial joint positions
        //             joints(i) = getJointPosition(i);
        //             joint_velocities(i) = 0;    // Set initial joint velocities
        //             joint_accelerations(i) = 0; // Set initial joint accelerations
        //             std::cout<<q_target1(i)<<",";
        //         }
        //         std::cout<<std::endl;
        //         rne_solver.CartToJnt(joints, joint_velocities, joint_accelerations, external_forces, joint_torques);
        //         for (int j = 0; j < 2001; j++)
        //         {
                   
        //                 joint_tor[1] = getJointTorqueFilter(1); // 电压
        //                 joint_tor[3] = getJointTorqueFilter(3); // 电压
        //                 joint_tor[5] = getJointTorqueFilter(5); // 电压
                    

        //             if (j > 1000)
        //             {
        //                 outputFile_246 << joint_tor[1] << "," << joint_tor[3] << "," << joint_tor[5] << "," << joint_torques(1) << "," << joint_torques(3)<< "," << joint_torques(5)<< std::endl;
        //             }
        //             robot.hw_interface_->waitForSignal(0);
        //         }
        //     }

        //     // Close the file
        //     outputFile_246.close();
        //     std::cout << "Data saved to joint_torques.csv" << std::endl;
        // }
        // else
        // {
        //     std::cout << "Unable to open the file." << std::endl;
        // }

        // if (outputFile_3.is_open())
        // {
            

        //     for (double angle = 90.0; angle >= -90.0; angle -= 2.0)
        //     {
        //         q_target1(1) = -90.0 / 180.0 * M_PI;
        //         q_target1(3) = 90.0 / 180.0 * M_PI;
        //         q_target1(2) = angle / 180.0 * M_PI;
        //         MoveJ(q_target1, 0.1, 0.1);
        //         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        //         for (int i = 0; i < _joint_num; ++i)
        //         {
        //             // Set initial joint positions
        //             joints(i) = getJointPosition(i);
        //             joint_velocities(i) = 0;    // Set initial joint velocities
        //             joint_accelerations(i) = 0; // Set initial joint accelerations
        //         }
        //         rne_solver.CartToJnt(joints, joint_velocities, joint_accelerations, external_forces, joint_torques);
        //         for (int j = 0; j < 2001; j++)
        //         {
                  

        //                 joint_tor[2] = getJointTorqueFilter(2); // 电压
                    

        //             if (j > 1000)
        //             {
        //                 outputFile_3 << joint_tor[2] << "," << joint_torques(2) << std::endl;
        //             }
        //             robot.hw_interface_->waitForSignal(0);
        //         }
        //     }

        //     // Close the file
        //     outputFile_3.close();
        //     std::cout << "Data saved to joint_torques.csv" << std::endl;
        // }
        // else
        // {
        //     std::cout << "Unable to open the file." << std::endl;
        // }
    
     if (outputFile_5.is_open())
        {
            

            for (double angle = 90.0; angle >= -90.0; angle -= 2.0)
            {
                
                q_target1(1) = -90.0 / 180.0 * M_PI;
                q_target1(2) = 0.0 / 180.0 * M_PI;
                q_target1(3) = 0.0 / 180.0 * M_PI;
                q_target1(5) = 0.0 / 180.0 * M_PI;
                q_target1(4) = angle / 180.0 * M_PI;
                MoveJ(q_target1, 0.1, 0.1);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                for (int i = 0; i < _joint_num; ++i)
                {
                    // Set initial joint positions
                    joints(i) = getJointPosition(i);
                    joint_velocities(i) = 0;    // Set initial joint velocities
                    joint_accelerations(i) = 0; // Set initial joint accelerations
                }
                rne_solver.CartToJnt(joints, joint_velocities, joint_accelerations, external_forces, joint_torques);
                for (int j = 0; j < 2001; j++)
                {
                    

                        joint_tor[4] = getJointTorqueFilter(4); // 电压
                    
                    if (j > 1000)
                    {
                        outputFile_5 << joint_tor[4] << "," << joint_torques(4) << std::endl;
                    }
                    robot.hw_interface_->waitForSignal(0);
                }
            }

            // Close the file
            outputFile_5.close();
            std::cout << "Data saved to joint_torques.csv" << std::endl;
        }
        else
        {
            std::cout << "Unable to open the file." << std::endl;
        }
    }

#pragma endregion
    // PLOG_INFO << "全部测试结束,goodbye!";
}
// namespace rocos
#pragma endregion

/// \brief 处理终端的Ctrl-C信号
/// \param signo

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

    
    auto robotService = RobotServiceImpl::getInstance(&robot);
    // while (true)
    // {
    //     std::cout<<"torque"<<robot.getJointTorqueFilter(4)*0.001<<std::endl;
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //     /* code */
    // }
    
    //------------------------wait----------------------------------
    std::thread thread_test{&rocos::Robot::test, &robot};

    //------------------------wait----------------------------------
    // robotService->runServer();

    thread_test.join();

    return 0;
}
