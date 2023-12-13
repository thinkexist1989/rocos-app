#ifndef JC_HELPER_DYNAMICS_H
#define JC_HELPER_DYNAMICS_H

#include "highspeed_udp.hpp"
#include "kdl/frames.hpp"
#include <atomic>
#include <fstream>
#include "interpolate.h"
#include <iostream>
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include <mutex>
#include "plog/Appenders/ColorConsoleAppender.h"
#include "plog/Initializers/RollingFileInitializer.h"
#include "plog/Log.h"
#include "ruckig/ruckig.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>
namespace rocos
{
    class Robot;

} // namespace rocos

// enum class rocos::Robot::DRAGGING_DIRRECTION;

namespace JC_helper
{

    class ft_sensor
    {
    private:
        std::string _ip_dress{};
        Response res{};
        SOCKET_HANDLE socketHandle{};
        KDL::Wrench init_force_torque{};

    public:
        KDL::Wrench force_torque{};

    public:
        ft_sensor(const char *ip_dress = "192.168.1.105");
        ~ft_sensor();

        /**
         * @brief 初始化设置，获取初始位置下力信息并转换到base系下
         *
         * @param flange_pos 法兰盘的位姿
         * @return int
         */
        int init(KDL::Frame flange_pos);

        /**
         * @brief 刷新传感器数据,内置重力补偿
         *
         * @param flange_pos flange的位姿
         */
        void getting_data(KDL::Frame flange_pos);

        int debug(KDL::Frame flange_pos);
    };

    class spring_mass_dump
    {
    private:
        //! 不要用array,实测发现会吞掉后面的数据
        std::vector<double> TCP_force{0, 0, 0};
        std::vector<double> force_pos_offset{0, 0, 0};
        std::vector<double> force_vel_offset{0, 0, 0};
        std::vector<double> force_last_vel_offset{0, 0, 0};
        std::vector<double> force_acc_offset{0, 0, 0};
        std::vector<double> force_last_acc_offset{0, 0, 0};

        std::vector<double> TCP_torque{0, 0, 0};
        std::vector<double> torque_pos_offset{0, 0, 0};
        std::vector<double> torque_vel_offset{0, 0, 0};
        std::vector<double> torque_last_vel_offset{0, 0, 0};
        std::vector<double> torque_acc_offset{0, 0, 0};
        std::vector<double> torque_last_acc_offset{0, 0, 0};

        std::vector<double> M{60.0, 60., 60., 60., 60., 60.};
        std::vector<double> K{30., 30., 30., 30., 30., 30.};
        // std::vector< double > K{ 0., 0., 0., 100., 100., 100. };
        std::vector<double> B{50., 50., 50., 50., 50., 50.};
        double damp{1};

        double _dt{0.001};
        KDL::Twist _Cartesian_vel;

    public:
        spring_mass_dump();
        ~spring_mass_dump();

        void calculate_translate();

        KDL::Rotation calculate_rotation();

        int calculate(KDL::Frame &pos_offset, KDL::Twist &Cartesian_vel, double dt = 0.001);

        void set_force(double force_x, double force_y, double force_z);

        void set_torque(double tor_que_x, double tor_que_y, double tor_que_z);

        void set_damp(double value);
        void set_k(double vaule);
        void set_m(double vaule);
        void set_damp(vector<double> value);
        void set_k(vector<double> vaule);
        void set_m(vector<double> vaule);
    };

    class admittance_joint
    {
    private:
        KDL::Vector gravity{0, 0, 9.81};
        KDL::ChainIdSolver_RNE rne_solver;

        std::vector<double> Theory_torques;
        std::vector<double> Actual_torques;
        std::vector<double> a_sensor;
        std::vector<double> b_sensor;
        std::vector<double> K;
        std::vector<double> M;
        std::vector<double> B;
        std::vector<double> joint_K;
        std::vector<double> fext;
        std::vector<double> kesai;
        std::vector<double> pose_stop;
        std::vector<double> F_joint_stop;
        std::vector<double> ZeroOffset;
        std::vector<double> q_min;
        std::vector<double> q_max;
        // yaml文件的路径
        
        YAML::Node yaml_node;
        unsigned int  joint_num;
         KDL::Wrenches external_forces;

    public:
        admittance_joint(rocos::Robot *robot_ptr,std::string yaml_path = "joint_impedance_control.yaml");
        ~admittance_joint();
        // 初始化参数，从yaml文件中读取及初始化q_min,q_max,因为从yaml中读取，所以不需要各个参数的set函数
        int init(string yaml_path);
        // 获取当前关节位置下的理论力矩
        std::vector<double> get_theory_torques(rocos::Robot *robot_ptr, KDL::JntArray &joint, KDL::JntArray &joint_vel, KDL::JntArray &joint_acc);
        // 获取当前关节位置下的实际力矩从基础函数getLoadTorque中获取
        // 设置关节的模拟外力矩
        void set_torque(int id, double tor_que);
        // 设置力矩缩放的参数
        double set_K_sensor(double x, double K = 1.0, double amplitude = 40.0);
        void Initialize(int joint_num)
        {
            Theory_torques.resize(joint_num);
            Actual_torques.resize(joint_num);
            a_sensor.resize(joint_num);
            b_sensor.resize(joint_num);
            K.resize(joint_num);
            M.resize(joint_num);
            B.resize(joint_num);
            pose_stop.resize(joint_num);
            q_min.resize(joint_num);
            q_max.resize(joint_num);
            F_joint_stop.resize(joint_num);
            joint_K.resize(joint_num);
            kesai.resize(joint_num);
            fext.resize(joint_num);
            ZeroOffset.resize(joint_num);
        };
        //开始示教
         void Runteaching(rocos::Robot *robot_ptr,bool *flag_admittance_joint_turnoff);

    };

    class admittance
    {
    private:
        KDL::Frame frame_offset{};
        std::mutex mutex_traj_joint;
        std::atomic<bool> on_stop_trajectory{false};
        bool FinishRunPlanningIK{false};
        ft_sensor *my_ft_sensor_ptr{nullptr};
        KDL::ChainIkSolverVel_pinv _ik_vel;

        // std::ofstream out_joint_csv{ };

    public:
        spring_mass_dump smd{};
        admittance(rocos::Robot *robot_ptr, ft_sensor *ft_sensor_ptr);
        ~admittance();
        int init(KDL::Frame flange_pos);
        // void start( rocos::Robot* robot_ptr, const std::vector< KDL::Frame >& traj_target );
        void Runteaching(rocos::Robot *robot_ptr, const KDL::Frame traj_target, bool *flag_turn_off);
        void RunLink(rocos::Robot *robot_ptr, const KDL::Frame frame_taget, double max_path_v, double max_path_a);
        void sensor_update(rocos::Robot *robot_ptr);
    };
    /**
     * @brief  计算直线轨迹的线速度
     *
     * @param Cartesian_vel
     * @param t 时间点
     * @param start 起始位置
     * @param end 终止位置
     * @param max_path_v 直线轨迹最大速度
     * @param max_path_a 直线轨迹最大加速度
     * @return int
     */
    int moveL_vel(KDL::Twist &Cartesian_vel, double t, KDL::Frame start, KDL::Frame end, double max_path_v, double max_path_a);

} // namespace JC_helper

#endif