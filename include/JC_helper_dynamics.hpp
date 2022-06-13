#ifndef JC_HELPER_DYNAMICS_H
#define JC_HELPER_DYNAMICS_H

#include "highspeed_udp.hpp"
#include "kdl/frames.hpp"
#include <atomic>
#include <fstream>
#include <interpolate.h>
#include <iostream>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <mutex>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>
#include <ruckig/ruckig.hpp>
#include <vector>
#include "JC_helper_kinematics.hpp"

namespace rocos
{
    class Robot;

}  // namespace rocos

// enum class rocos::Robot::DRAGGING_DIRRECTION;

namespace JC_helper
{
    
class ft_sensor
{
private:
    std::string _ip_dress{ };
    Response res{ };
    SOCKET_HANDLE socketHandle{ };
    KDL::Wrench init_force_torque{ };

public:
    ruckig::OutputParameter< 3 > ft_output;
    ruckig::Ruckig< 3 > ft_otg{ 0.001 };
    ruckig::InputParameter< 3 > ft_input;
    ruckig::Result ft_res;
    KDL::Wrench force_torque{ };


public:
    ft_sensor( const char* ip_dress = "192.168.1.105" );
    ~ft_sensor( );

    /**
     * @brief 初始化设置，获取初始位置下力信息并转换到base系下
     *
     * @param flange_pos 法兰盘的位姿
     * @return int
     */
    int init( KDL::Frame flange_pos );

    /**
     * @brief 刷新传感器数据,内置重力补偿
     *
     * @param flange_pos flange的位姿
     */
    void getting_data( KDL::Frame flange_pos );

    int debug( KDL::Frame flange_pos );
};

class spring_mass_dump
{
private:
    //!不要用array,实测发现会吞掉后面的数据
    std::vector< double > TCP_force{ 0, 0, 0 };
    std::vector< double > force_pos_offset{ 0, 0, 0 };
    std::vector< double > force_vel_offset{ 0, 0, 0 };
    std::vector< double > force_last_vel_offset{ 0, 0, 0 };
    std::vector< double > force_acc_offset{ 0, 0, 0 };
    std::vector< double > force_last_acc_offset{ 0, 0, 0 };

    std::vector< double > TCP_torque{ 0, 0, 0 };
    std::vector< double > torque_pos_offset{ 0, 0, 0 };
    std::vector< double > torque_vel_offset{ 0, 0, 0 };
    std::vector< double > torque_last_vel_offset{ 0, 0, 0 };
    std::vector< double > torque_acc_offset{ 0, 0, 0 };
    std::vector< double > torque_last_acc_offset{ 0, 0, 0 };

    std::vector< double > M{ 50.0, 50., 50., 50., 50., 50. };
    // std::vector< double > K{ 50., 50., 50., 30., 30., 30. };
    std::vector< double > K{ 0., 0., 0., 100., 100., 100. };
    std::vector< double > B{ 50., 50., 50., 50., 50., 50. };

    double _dt{ 0.001 };
    KDL::Twist _Cartesian_vel;

    std::ofstream out_dat{ };

public:
    spring_mass_dump( );
    ~spring_mass_dump( );

    void calculate_translate( );

    KDL::Rotation calculate_rotation( );

    int calculate( KDL::Frame& pos_offset, KDL::Twist& Cartesian_vel, double dt = 0.001 );

    void set_force( double force_x, double force_y, double force_z );

    void set_torque( double tor_que_x, double tor_que_y, double tor_que_z );

    void set_damp( double value );
};

class admittance
{
private:
    KDL::Frame frame_offset{ };
    std::mutex mutex_traj_joint;
    std::atomic< bool > on_stop_trajectory{ false };
    std::vector< KDL::JntArray > traj_joint;
    bool FinishRunPlanningIK{ false };
    ft_sensor my_ft_sensor{ };
    KDL::ChainIkSolverVel_pinv _ik_vel;
    spring_mass_dump smd{ };

    std::ofstream out_joint_csv{ };
    std::ofstream ft_otg_csv{ };

public:
    admittance( rocos::Robot* robot_ptr );
    ~admittance( );
    int init( KDL::Frame flange_pos );
    void start( rocos::Robot* robot_ptr, const std::vector< KDL::Frame >& traj_target );
    void IK( rocos::Robot* robot_ptr, const std::vector< KDL::Frame >& traj_target );
    void motion( rocos::Robot* robot_ptr );
    void sensor_update( rocos::Robot* robot_ptr );
    void force_torque_otg( );

};

}  // namespace JC_helper

#endif