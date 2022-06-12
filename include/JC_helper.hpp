#ifndef JC_HELPER_H
#define JC_HELPER_H

#include "highspeed_udp.hpp"
#include "kdl/frames.hpp"
#include <atomic>
#include <interpolate.h>
#include <iostream>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <mutex>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>
#include <ruckig/ruckig.hpp>
#include <vector>
#include <kdl/chainiksolvervel_pinv.hpp>


#define RESET "\033[0m"

#define BLACK "\033[30m" /* Black */

#define RED "\033[31m" /* Red */

#define GREEN "\033[32m" /* Green */

#define YELLOW "\033[33m" /* Yellow */

#define BLUE "\033[34m" /* Blue */

#define MAGENTA "\033[35m" /* Magenta */

#define CYAN "\033[36m" /* Cyan */

#define WHITE "\033[37m" /* White */

#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */

#define BOLDRED "\033[1m\033[31m" /* Bold Red */

#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */

#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */

#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */

#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */

#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */

#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

//示例
// std::cout << BLUE << " hello world " << std::endl;

namespace rocos
{
    class Robot;

}  // namespace rocos

// enum class rocos::Robot::DRAGGING_DIRRECTION;

namespace JC_helper
{

    /**
     * @brief 姿态插值（轴角法角度线性插值）
     *
     * @param start 开始姿态
     * @param end 结束姿态
     * @param s 百分比
     * @return KDL::Rotation
     */
    KDL::Rotation RotAxisAngle( KDL::Rotation start, KDL::Rotation end, double s );

    /**
     * @brief 圆弧插值
     *
     * @param F_base_circlestart 起点
     * @param F_base_circleend 终点
     * @param F_base_circleCenter 圆心
     * @param s_p 位置百分比
     * @param s_r 位态百分比
     * @param alpha 夹角
     * @param success 计算是否成功标识位 (false 为不成功)
     * @return KDL::Frame
     */
    KDL::Frame circle( const KDL::Frame& F_base_circlestart, const KDL::Frame& F_base_circleend, const KDL::Frame& F_base_circleCenter, double s_p, double s_r, double alpha, double& success );

    /**
     * @brief 直线规划（使用doubleS速度曲线）
     *
     * @param start 起始位姿
     * @param end 终止位姿
     * @param success 计算是否成功标识位 (false 为不成功)
     * @return 指定s对应的位姿
     */
    KDL::Frame link( const KDL::Frame& start, const KDL::Frame& end, double s_p, double s_r, double& success );

    int link_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& start, const KDL::Frame& end, double v_start, double v_end, double max_path_v, double max_path_a );

    int link_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& start, const KDL::Frame& end, double max_path_v, double max_path_a );

    /**
     * @brief
     *
     * @param traj 存储的轨迹
     * @param f_start 应该开始运动的位姿
     * @param f_mid 中间的插值点
     * @param f_end 结束的插值点
     * @param next_f_start 下一段运动应该开始的位姿
     * @param current_path_start_v 应该开始运动的线速度
     * @param next_path_start_v 下一段运动应该开始的线速度
     * @param bound_dist 过渡半径
     * @param max_path_v 最大线速度
     * @param max_path_a 最大线加速度
     * @param next_max_path_v 下一段的最大线速度（如果无下一段了，此参数无意义）
     * @return int
     */
    int multilink_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& f_start, const KDL::Frame& f_mid, const KDL::Frame& f_end, KDL::Frame& next_f_start, double current_path_start_v, double& next_path_start_v, double bound_dist, double max_path_v, double max_path_a, double next_max_path_v = 1 );

    /**
     * @brief 给定三点，计算出圆心
     *
     * @param center
     * @param f_p1
     * @param f_p2
     * @param f_p3
     * @return int
     */
    int circle_center( KDL::Frame& center, const KDL::Frame& f_p1, const KDL::Frame& f_p2, const KDL::Frame& f_p3 );

    /**
     * @brief movec轨迹计算
     *
     * @param traj 输出计算结果
     * @param f_p1 第一点
     * @param f_p2 第二点
     * @param f_p3 第三点
     * @param max_path_v 最大速度
     * @param max_path_a 最大加速度
     * @param fixed_rotation 是否保持姿态：是->姿态不变；否->姿态指向圆心方向变换
     * @return int
     */
    int circle_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& f_p1, const KDL::Frame& f_p2, const KDL::Frame& f_p3, double max_path_v = 0.01, double max_path_a = 0.01, bool fixed_rotation = true );

    /**
     * @brief 位置保持，只旋转姿态
     *
     * @param traj 输出计算结果
     * @param f_p 位置保持
     * @param f_r1 起始姿态
     * @param f_r2 终止姿态
     * @param max_path_v 最大速度
     * @param max_path_a 最大加速度
     * @param equivalent_radius 等效半径，用于乘以角度得到等效弧度
     * @return int
     */
    int rotation_trajectory( std::vector< KDL::Frame >& traj, const KDL::Vector& f_p, const KDL::Rotation& f_r1, const KDL::Rotation& f_r2, double max_path_v = 0.01, double max_path_a = 0.01, double equivalent_radius = 0.01 );

    constexpr size_t _joint_num{ 7 };

    class SmartServo_Joint
    {
    private:
        //** 这里都是关节smart servo 所需的全部变量 **//
        std::mutex input_mutex;
        ruckig::Ruckig< _joint_num > otg{ 0.001 };
        ruckig::InputParameter< _joint_num > input;
        ruckig::OutputParameter< _joint_num > output;
        //**-------------------------------**//

        std::atomic< bool > on_stop_trajectory{ false };
        std::atomic< bool >* external_finished_flag_ptr;

    public:
        SmartServo_Joint( std::atomic< bool >* finished_flag_ptr );
        void init( std::vector< double > q_init, std::vector< double > v_init, std::vector< double > a_init, double max_v, double max_a, double max_j );
        void RunSmartServo( rocos::Robot* robot_ptr );
        void command( KDL::JntArray q_target );
    };

    class OnlineDoubleS
    {
    private:
        const double T_S{ 0.001 };

        ruckig::Ruckig< 1 > otg;
        ruckig::InputParameter< 1 > input;
        ruckig::OutputParameter< 1 > output;
        ruckig::Trajectory< 1 > _trajectory;
        std::array< double, 1 > position, velocity, acceleration;
        const double middle_eps{ 1E-4 };
        double s{ 0 };
        double sd{ 0 };
        double sdd{ 0 };
        int count{ 0 };

    public:
        void calculate(
            double q0,
            double q1,
            double v_s,
            double v_e,
            double a_s,
            double a_e,

            double v_max,
            double a_max,
            double j_max

        );
        void calculate(
            double q0,
            double q1,
            double v_s,
            double v_e,
            double a_s,
            double a_e,

            double v_max,
            double a_max,
            double j_max,
            double t );

        double get_duration( );
        void get_pos_vel_acc( int i, double& p, double& v, double& a );
        void get_pos_vel_acc( double t, double& p, double& v, double& a );
    };

    class SmartServo_Cartesian
    {
    private:
        std::atomic< bool > on_stop_trajectory{ false };
        std::atomic< bool >* external_finished_flag_ptr;

    private:
        double max_path_v{ 0.1 };
        double max_path_a{ 0.1 };
        double equivalent_radius{ 0.1 };
        KDL::Frame last_target;
        KDL::Frame last_last_target;
        KDL::Frame target;
        std::vector< KDL::Frame > traj_frame;
        std::vector< KDL::JntArray > traj_joint;
        std::mutex mutex_traj_frame;
        std::mutex mutex_traj_joint;

        atomic< int > command_flag{ 0 };

        OnlineDoubleS _OnlineDoubleS;
        OnlineDoubleS _rotaion_OnlineDoubleS;
        OnlineDoubleS _last_rotaion_OnlineDoubleS;

        bool FinishRunPlanningIK{ true };
        bool FinishPlanningFrame{ true };
        bool FinishRunMotion{ true };

        atomic< int > count_down{ 200 };
        KDL::Frame _new_target{ };

        const int rvie_max_count = 100;

        const double eps{ 1E-7 };
        const double small_eps{ 1E-6 };
        const double middle_eps{ 1E-4 };
        const double big_eps{ 1E-3 };

        KDL::JntArray _q_init;
        std::mutex target_mutex;

        long time_count{ 300 };

    public:
        SmartServo_Cartesian( std::atomic< bool >* finished_flag_ptr );
        ~SmartServo_Cartesian( );
        void init( KDL::JntArray q_init, KDL::Frame p_init, double v_init, double a_init, double max_v, double max_a, double max_j );

        void RunSmartServo_Plannig( );
        void RunSmartServo_Ik( rocos::Robot* );
        void RunSmartServo_Motion( rocos::Robot* );
        int command( const KDL::Vector& tem_v, const char* str = "BASE" );
        int command( const KDL::Rotation& tem_r, const char* str = "BASE" );

    private:
        int command( KDL::Frame p_target );
        KDL::Frame link_trajectory( const KDL::Frame& start, const KDL::Frame& end, double s_p );
        KDL::Frame link_trajectory( const KDL::Frame& start, const KDL::Frame& end, double s_p, double s_r );
        std::vector< double > UnitQuaternion_intep( const std::vector< double >& start, const std::vector< double >& end, double s, bool flag_big_angle = false );
        KDL::Frame cirlular_trajectory( const KDL::Frame& F_base_circlestart, const KDL::Frame& F_base_circleend, const KDL::Frame& F_base_circleCenter, double s_p, double alpha );
        KDL::Frame cirlular_trajectory( const KDL::Frame& F_base_circlestart, const KDL::Frame& F_base_circleend, const KDL::Frame& F_base_circleCenter, double s_p, double s_r, double alpha );
        KDL::Rotation ratation_trajectory( const KDL::Rotation& start, const KDL::Rotation& end, double s_r, bool flag_big_angle = false );
    };

    inline KDL::JntArray vector_2_JntArray( std::vector< double > pos )
    {
        KDL::JntArray _pos( pos.size( ) );
        for ( int i = 0; i < pos.size( ); i++ )
            _pos( i ) = pos[ i ];
        return _pos;
    }

    inline void print_JntArray( const char* str, KDL::JntArray joints )
    {
        PLOG_DEBUG << str << ":";
        for ( int i = 0; i < _joint_num; i++ )
            PLOG_DEBUG.printf( "[%d] = %f", i, joints( i ) * 180 / M_PI );
        PLOG_DEBUG;
    }

    /**
     * @brief 紧急停止，注意此函数不关心任何标志位，因此调用着需要保证函数执行期间，没有其他运动并行执行
     * @note joints_acc = (joints_vel - joints_last_vel) /dt
     * @param robot_ptr
     * @param joints_last_vel 当前时间-dt后的关节速度
     * @param joints_vel 当前时间的速度
     * @param dt 速度微分时间间隔
     */
    void motion_stop( rocos::Robot* robot_ptr, const std::vector< KDL::JntArray > &traj_joint  ,int traj_joint_count);




    class ft_sensor
    {
    private:
        std::string _ip_dress{ };
        Response res{ };
        SOCKET_HANDLE socketHandle{ };
        KDL::Wrench init_force_torque{ };
    public:
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

    public:
        admittance(rocos::Robot* robot_ptr);
        int init( KDL::Frame flange_pos );
        void start( rocos::Robot* robot_ptr, const std::vector< KDL::Frame >& traj_target );
        void IK( rocos::Robot* robot_ptr, const std::vector< KDL::Frame >& traj_target );
        void motion( rocos::Robot* robot_ptr );

        class spring_mass_dump
        {
        private:
            //!不要用array,实测发现会吞掉后面的数据
            std::vector<double> TCP_force{ 0, 0, 0 };
            std::vector<double> force_pos_offset{ 0, 0, 0 };
            std::vector<double> force_vel_offset{ 0, 0, 0 };
            std::vector<double> force_last_vel_offset{ 0, 0, 0 };
            std::vector<double> force_acc_offset{ 0, 0, 0 };
            std::vector<double> force_last_acc_offset{ 0, 0, 0 };

            std::vector<double> TCP_torque{ 0, 0, 0 };
            std::vector<double> torque_pos_offset{ 0, 0, 0 };
            std::vector<double> torque_vel_offset{ 0, 0, 0 };
            std::vector<double> torque_last_vel_offset{ 0, 0, 0 };
            std::vector<double> torque_acc_offset{ 0, 0, 0 };
            std::vector<double> torque_last_acc_offset{ 0, 0, 0 };

            std::vector<double> M{ 30.0, 30., 30., 30., 30., 30. };
            std::vector<double> K{ 150., 150., 150., 100., 100., 100. };
            std::vector<double> B{ 30., 30., 30., 30., 30., 30. };
          
            double _dt{ 0.001 };
            KDL::Twist _Cartesian_vel;

        public:
            spring_mass_dump( );

            void calculate_translate( );

            KDL::Rotation calculate_rotation( );

             int calculate( KDL::Frame& pos_offset  , double dt , KDL::Twist& Cartesian_vel );


            void set_force( double force_x, double force_y, double force_z );

            void set_torque( double tor_que_x, double tor_que_y, double tor_que_z );

            void set_damp( double value );
        } smd;

    };

}  // namespace JC_helper

#endif