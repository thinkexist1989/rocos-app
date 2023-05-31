#include <rocos_app/JC_helper_dynamics.hpp>
#include <rocos_app/robot.h>
/** 待处理问题：
 * 2. dragging 笛卡尔空间版 加入
 * 3. drggging 关节空间改进，允许给笛卡尔位姿
 */

namespace JC_helper
{
#pragma region  //* 6维力传感器

    ft_sensor::ft_sensor( const char* ip_dress ) : _ip_dress{ ip_dress }
    {
    }

    ft_sensor::~ft_sensor( )
    {
        UdpClose( &socketHandle );
    }

    int ft_sensor::init( KDL::Frame flange_pos )
    {
        if ( Connect( &socketHandle, _ip_dress.c_str( ), FT_PORT ) != 0 )
        {
            PLOG_ERROR << "Could not connect to device...";
            return -1;
        }
        SendCommand( &socketHandle, COMMAND_SPEED, FT_SPEED );
        SendCommand( &socketHandle, COMMAND_FILTER, FT_FILTER );
        // ! 爪子装上后，零漂消除不应该打开了，只有空载情况下才可以打开零漂消除
        SendCommand( &socketHandle, COMMAND_BIAS, FT_BIASING_OFF );

        std::this_thread::sleep_for( std::chrono::duration< double >{ 3 } );

        SendCommand( &socketHandle, COMMAND_START, 3 );
        for ( int i = 0; i < 3; i++ )
            res = Receive( &socketHandle );

        init_force_torque.force[ 0 ]  = res.fx / FORCE_DIV + 0.2;
        init_force_torque.force[ 1 ]  = res.fy / FORCE_DIV - 2.3;
        init_force_torque.force[ 2 ]  = res.fz / FORCE_DIV + 5.1;
        init_force_torque.torque[ 0 ] = res.tx / TORQUE_DIV + 0.05;
        init_force_torque.torque[ 1 ] = res.ty / TORQUE_DIV - 0.04;
        init_force_torque.torque[ 2 ] = res.tz / TORQUE_DIV - 0.09;

        //将起始收到的力信息转变到base坐标系下
        // TODO处理力矩
        init_force_torque.force = ( flange_pos * KDL::Frame{ KDL::Rotation::RPY( 0, 0, M_PI ), KDL::Vector( 0, 0, 0.035 ) } ) * init_force_torque.force;

        PLOG_INFO << "F/T sensor init success";
        return 0;
    }

    void ft_sensor::getting_data( KDL::Frame flange_pos )
    {
        SendCommand( &socketHandle, COMMAND_START, 2 );//请求2个数据

        for ( int i = 0; i < 2; i++ )
        {
            res = Receive( &socketHandle );
        }

            //收到的力信息转换到base系
            KDL::Vector  force_temp = ( flange_pos * KDL::Frame{ KDL::Rotation::RPY( 0, 0, M_PI ), KDL::Vector( 0, 0, 0.035 ) } ) * KDL::Vector{res.fx / FORCE_DIV + 0.2, res.fy / FORCE_DIV - 2.3,res.fz / FORCE_DIV + 5.1 };

            // 重力补偿
            force_temp = force_temp - init_force_torque.force;

            // 限制大小
            for ( int i{ 0 }; i < 3; i++ )
            if ( abs( force_temp( i ) ) < 3 || abs( force_temp( i ) ) > 6 )
                force_torque.force[ i ] = 0;
            else
                force_torque.force[ i ] = force_temp( i );

            // TODO 力矩未用上，屏蔽
            for ( int i{ 0 }; i < 3; i++ )
            force_torque.torque[ i ] = 0;
    }

    int ft_sensor::debug( KDL::Frame flange_pos )
    {
        getting_data( flange_pos );

        for ( int i{ 0 }; i < 3; i++ )
            PLOG_DEBUG.printf( "force[ %d ] = %f ", i, force_torque.force[ i ] );

        for ( int i{ 0 }; i < 3; i++ )
            PLOG_DEBUG.printf( "torque[ %d ] = %f ", i, force_torque.torque[ i ] );

        std::this_thread::sleep_for( std::chrono::duration< double >( 0.001 ) );

        return 0;
    }

#pragma endregion

#pragma region  //*弹簧阻尼质量系统
    spring_mass_dump::spring_mass_dump( )
    {
        for ( int i{ 0 }; i < _joint_num; i++ )
            B[ i ] = 2 * damp * sqrt( M[ i ] * K[ i ] );

        // out_dat.open( "/home/think/rocos-app/debug/admittance.csv" );
    }

    spring_mass_dump::~spring_mass_dump( )
    {
        out_dat.close( );
    }
    void spring_mass_dump::calculate_translate( )
    {
        for ( int i{ 0 }; i < 3; i++ )
        {
            force_acc_offset[ i ] = ( TCP_force[ i ] - B[ i ] * force_vel_offset[ i ] - K[ i ] * force_pos_offset[ i ] ) / M[ i ];
            force_vel_offset[ i ] = _dt * ( force_acc_offset[ i ] + force_last_acc_offset[ i ] ) / 2 + force_vel_offset[ i ];
            force_pos_offset[ i ] = _dt * ( force_vel_offset[ i ] + force_last_vel_offset[ i ] ) / 2 + force_pos_offset[ i ];

            force_last_acc_offset[ i ] = force_acc_offset[ i ];
            force_last_vel_offset[ i ] = force_vel_offset[ i ];

            _Cartesian_vel.vel[ i ] = force_vel_offset[ i ];
        }
    }

    KDL::Rotation spring_mass_dump::calculate_rotation( )
    {
        KDL::Vector delta_rot;
        KDL::Vector current_rot;
        KDL::Rotation template_rot;

        for ( int i{ 0 }; i < 3; i++ )
        {
            torque_acc_offset[ i ] = ( TCP_torque[ i ] - B[ i ] * torque_vel_offset[ i ] - K[ i ] * torque_pos_offset[ i ] ) / M[ i ];
            torque_vel_offset[ i ] = _dt * ( torque_acc_offset[ i ] + torque_last_acc_offset[ i ] ) / 2 + torque_vel_offset[ i ];

            delta_rot( i )   = _dt * ( torque_vel_offset[ i ] + torque_last_vel_offset[ i ] ) / 2;
            current_rot( i ) = torque_pos_offset[ i ];

            torque_last_acc_offset[ i ] = torque_acc_offset[ i ];
            torque_last_vel_offset[ i ] = torque_vel_offset[ i ];

            _Cartesian_vel.rot[ i ] = torque_vel_offset[ i ];
        }

        template_rot = KDL::Rotation::Rot( delta_rot, delta_rot.Norm( ) ) * KDL::Rotation::Rot( current_rot, current_rot.Norm( ) );

        current_rot = template_rot.GetRot( );

        for ( int i{ 0 }; i < 3; i++ )
            torque_pos_offset[ i ] = current_rot[ i ];

        return template_rot;
    }

    int spring_mass_dump::calculate( KDL::Frame& pos_offset, KDL::Twist& Cartesian_vel, double dt )
    {
        // static int dt_count{0};
        _dt = dt;

        calculate_translate( );

        for ( int i{ 0 }; i < 3; i++ )
        {
            pos_offset.p[ i ] = force_pos_offset[ i ];
        }
        // out_dat << std::to_string( dt_count++*0.001) << "\t,";
        // out_dat << std::to_string( pos_offset.p[ 0 ]) ;
        // out_dat << "\n,";

        // pos_offset.M = calculate_rotation( );

        //_Cartesian_vel代表仅仅由力引起的速度矢量
        Cartesian_vel = _Cartesian_vel;

        return 0;
    }

    void spring_mass_dump::set_force( double force_x, double force_y, double force_z )
    {
        TCP_force[ 0 ] = force_x;
        TCP_force[ 1 ] = force_y;
        TCP_force[ 2 ] = force_z;
        // PLOG_DEBUG.printf( "TCP_force  = %f %f %f", TCP_force[ 0 ], TCP_force[ 1 ], TCP_force[ 2 ] );
    }

    void spring_mass_dump::set_torque( double tor_que_x, double tor_que_y, double tor_que_z )
    {
        TCP_torque[ 0 ] = tor_que_x;
        TCP_torque[ 1 ] = tor_que_y;
        TCP_torque[ 2 ] = tor_que_z;
        // PLOG_DEBUG.printf( "TCP_torque  = %f %f %f", TCP_torque[ 0 ], TCP_torque[ 1 ], TCP_torque[ 2 ] );
    }

    void spring_mass_dump::set_damp( double value )
    {
        damp += value;

        for ( int i{ 0 }; i < _joint_num; i++ )
            B[ i ] = 2 * damp * sqrt( M[ i ] * K[ i ] );

        PLOG_DEBUG << "damp  = " << damp;
    }

    void spring_mass_dump::set_k( double value )
    {
        if ( abs( value ) < 1e-3 )
        {
            for ( int i{ 0 }; i < _joint_num; i++ )
            {
                B[ i ] = 50;
                K[ i ] = 0;
            }
        }
        else
        {
            for ( int i{ 0 }; i < _joint_num; i++ )
            {
                K[ i ] = value;
                B[ i ] = 2 * damp * sqrt( M[ i ] * K[ i ] );
            }
        }

        PLOG_DEBUG << "k  = " << value;
    }
#pragma endregion

#pragma region  //*导纳控制

    admittance::admittance( rocos::Robot* robot_ptr  , ft_sensor* ft_sensor_ptr ) : _ik_vel{ robot_ptr->kinematics_.getChain( ) }
    {
        my_ft_sensor_ptr = ft_sensor_ptr;
        // out_joint_csv.open( "/home/think/rocos-app/debug/admittance_joints.csv" );

    }

    admittance::~admittance( )
    {
        // out_joint_csv.close( );
    }

    int admittance::init( KDL::Frame flange_pos )
    {
        //** 6维力初始化 **//
        // if ( my_ft_sensor.init( flange_pos ) < 0 )
        // {
        //     PLOG_ERROR << " force-torque sensor init failed ";
        //     return -1;
        // }
        //**-------------------------------**//

        PLOG_INFO << " init success";
        return 0;
    }

    // void admittance::start( rocos::Robot* robot_ptr, const std::vector< KDL::Frame >& traj_target )
    // {
    //     std::shared_ptr< std::thread > _thread_IK{ nullptr };
    //     // std::shared_ptr< std::thread > _thread_motion{ nullptr };
    //     std::shared_ptr< std::thread > _thread_ft_sensor{ nullptr };

    //     _thread_ft_sensor.reset( new std::thread{ &JC_helper::admittance::sensor_update, this, robot_ptr } );
    //     _thread_IK.reset( new std::thread{ &JC_helper::admittance::IK, this, robot_ptr, std::ref( traj_target ) } );
    //     // _thread_motion.reset( new std::thread{ &JC_helper::admittance::motion, this, robot_ptr } );

    //     if ( traj_target.size( ) == 1 )  //示教模式
    //     {
    //         PLOG_INFO << " starting  teaching";

    //         std::string str;
    //         while ( str.compare( "break" ) != 0 )
    //         {
    //             PLOG_INFO << " enter 'break' to turnoff teaching function:";
    //             std::cin >> str;
    //         }
    //         on_stop_trajectory = true;
    //     }
    //     else  //运动导纳模式
    //     {
    //         PLOG_INFO << " starting  admittance motion";
    //     }

    //     // _thread_motion->join( );
    //     _thread_IK->join( );
    //     _thread_ft_sensor->join( );

    //     PLOG_INFO << "admittance  全部结束";
    // }


    void admittance::Runteaching( rocos::Robot* robot_ptr, const KDL::Frame traj_target ,bool * flag_turnoff )
    {
        //** 变量初始化 **//
        // std::unique_lock< std::mutex > lock_traj_joint( mutex_traj_joint, std::defer_lock );  //不上锁
        KDL::Frame frame_target;
        KDL::JntArray _q_target( _joint_num );
        KDL::JntArray _q_init( _joint_num );
        int_least64_t max_count{ 0 };
        KDL::Twist admittance_vel;
        KDL::JntArray joints_vel( _joint_num );

        KDL::JntArray current_pos( _joint_num );
        KDL::JntArray last_pos( _joint_num );
        KDL::JntArray last_last_pos( _joint_num );

        //**-------------------------------**//

        //** 程序初始化 **//

        for ( int i = 0; i < _joint_num; i++ )
        {
            _q_init( i )   = robot_ptr->pos_[ i ];
            _q_target( i ) = _q_init( i );
        }

        for ( int i = 0; i < _joint_num; i++ )
        {
            current_pos( i )   = robot_ptr->pos_[ i ];
            last_pos( i )      = current_pos( i );
            last_last_pos( i ) = current_pos( i );
        }
        //**-------------------------------**//

        //** 轨迹计算 **//

        max_count = numeric_limits< int_least64_t >::max( );


        int traj_count{ 0 };
        for ( ; traj_count < max_count; traj_count++ )
        {
            // TODO 导纳计算
            smd.set_force( my_ft_sensor_ptr->force_torque.force[ 0 ], my_ft_sensor_ptr->force_torque.force[ 1 ], my_ft_sensor_ptr->force_torque.force[ 2 ] );
            smd.calculate( frame_offset, admittance_vel );

            //** 读取最新Frame **//
            // frame_target = frame_offset * traj_target;  //示教模式
            //**-------------------------------**//

            //** IK求解 **//
            // if ( ( robot_ptr->kinematics_ ).CartToJnt( _q_init, frame_target, _q_target ) < 0 )
            // {
            //     PLOG_ERROR << " CartToJnt failed  frame_target =\n"
            //                << frame_target;
            //     //! on_stop_trajectory = true;
            //     //! break;

            //**-------------------------------**//

            //** 笛卡尔速度求解 **//

            _ik_vel.CartToJnt( _q_init, admittance_vel, joints_vel );
            KDL::Multiply( joints_vel, 0.001, joints_vel );
            KDL::Add( _q_init, joints_vel, _q_target );
            //**-------------------------------**//


            //** 速度和加速度保护 **//

            if ( on_stop_trajectory ) break;

            if ( check_vel_acc( _q_target, current_pos, last_pos, 1, 2) < 0 )
            {
                on_stop_trajectory = true;
                break;
            }

            last_last_pos = last_pos;
            last_pos      = current_pos;
            current_pos   = _q_target;

            //**-------------------------------**//

            //** csv打印 **//
            // out_joint_csv << std::to_string( traj_count * 0.001 ) << "\t,";
            // for ( int i = 0; i < _joint_num - 1; i++ )
            // {
            //     out_joint_csv << std::to_string( _q_target( i ) ) << "\t,";
            // }
            // out_joint_csv << std::to_string( _q_target( 6 ) );
            // out_joint_csv << "\n";
            //**-------------------------------**//

            //** 位置伺服 **//
            //!提供位置保护，防止越过关节限位
            safety_servo( robot_ptr, _q_target );

            //**-------------------------------**//

            _q_init = _q_target;

            on_stop_trajectory = *flag_turnoff;//由外界调用者决定什么时候停止

        }

        if ( on_stop_trajectory )
        {
            PLOG_ERROR << "IK 触发急停";
            Joint_stop( robot_ptr, current_pos, last_pos, last_last_pos );
            *flag_turnoff  = true ; //告知调用者，因为速度太大而线程已停止
        }
        else
            PLOG_INFO << "IK结束";

        FinishRunPlanningIK = true;
    }

    void admittance::RunLink( rocos::Robot* robot_ptr,  const KDL::Frame frame_target, double max_path_v, double max_path_a )
    {
        //** 变量初始化 **//
        // std::unique_lock< std::mutex > lock_traj_joint( mutex_traj_joint, std::defer_lock );  //不上锁
        KDL::Frame frame_intep;
        KDL::JntArray _q_target( _joint_num );
        KDL::JntArray _q_init( _joint_num );
        std::vector< double > max_step;
        KDL::Twist admittance_vel;
        KDL::JntArray joints_vel( _joint_num );
        KDL::Twist traj_vel;
        KDL::Twist Cartesian_vel;

        KDL::JntArray current_pos( _joint_num );
        KDL::JntArray last_pos( _joint_num );
        KDL::JntArray last_last_pos( _joint_num );
        
        //**-------------------------------**//

        //** 程序初始化 **//

        for ( int i = 0; i < _joint_num; i++ )
        {
            max_step.push_back( robot_ptr->max_vel_[ i ] * 0.001 );
            _q_init( i )   = robot_ptr->pos_[ i ];
            _q_target( i ) = _q_init( i );
        }

        for ( int i = 0; i < _joint_num; i++ )
        {
            current_pos( i )   = robot_ptr->pos_[ i ];
            last_pos( i )      = current_pos( i );
            last_last_pos( i ) = current_pos( i );
        }

        //**-------------------------------**//

        //** 直线轨迹规划 **//

        std::vector< KDL::Frame > traj_target;
        KDL::Frame frame_init = robot_ptr->flange_;

        if ( link_trajectory( traj_target, frame_init, frame_target, max_path_v, max_path_a ) < 0 )
        {
            PLOG_ERROR << "link trajectory planning fail ";
            FinishRunPlanningIK = true;

            return;
        }

        //**-------------------------------**//


        //** 轨迹计算 **//
        int traj_count{ 0 };
        for ( ; traj_count < traj_target.size( ); traj_count++ )
        {

            // TODO 导纳计算
            smd.set_force( my_ft_sensor_ptr->force_torque.force[0] ,my_ft_sensor_ptr->force_torque.force[ 1 ] , my_ft_sensor_ptr->force_torque.force[ 2 ] );
            // smd.set_force( 0, -20, 0 );
            // smd.set_torque( 0, 0, 0 );
            smd.calculate( frame_offset, admittance_vel );

            //** 读取最新Frame **//
          
                // frame_intep = frame_offset * traj_target[ traj_count ];  //导纳控制模式

            //**-------------------------------**//

            //** IK求解 **//
            // if ( ( robot_ptr->kinematics_ ).CartToJnt( _q_init, frame_intep, _q_target ) < 0 )
            // {
            //     PLOG_ERROR << " CartToJnt failed  frame_intep =\n"
            //                << frame_intep;
            //     //! on_stop_trajectory = true;
            //     //! break;


            //**-------------------------------**//

            //** 笛卡尔速度求解 **//
            if ( moveL_vel( traj_vel, traj_count * 0.001, traj_target.front( ), traj_target.back( ), max_path_v, max_path_a ) < 0 )
            {
                PLOG_ERROR << "笛卡尔速度转关节速度失败";
                on_stop_trajectory = true;
                break;
            }

            Cartesian_vel.vel = traj_vel.vel + admittance_vel.vel;
            Cartesian_vel.rot = traj_vel.rot + admittance_vel.rot;

            _ik_vel.CartToJnt( _q_init, Cartesian_vel, joints_vel );
            KDL::Multiply( joints_vel, 0.001, joints_vel );
            KDL::Add( _q_init, joints_vel, _q_target );
            //**-------------------------------**//

            //** 速度和加速度保护 **//

            if ( on_stop_trajectory ) break;

            if ( check_vel_acc( _q_target, current_pos, last_pos, 1, 2 ) < 0 )
            {
                on_stop_trajectory = true;
                break;
            }

            last_last_pos = last_pos;
            last_pos      = current_pos;
            current_pos   = _q_target;

            //**-------------------------------**//

            //** csv打印 **//
            // out_joint_csv << std::to_string( traj_count * 0.001 ) << "\t,";
            // for ( int i = 0; i < _joint_num - 1; i++ )
            // {
            //     out_joint_csv << std::to_string( _q_target( i ) ) << "\t,";
            // }
            // out_joint_csv << std::to_string( _q_target( 6 ) );
            // out_joint_csv << "\n";
            //**-------------------------------**//

            //** 位置伺服 **//

            //!提供位置保护，防止越过关节限位
            safety_servo( robot_ptr, _q_target );

            //**-------------------------------**//

            _q_init = _q_target;

        }

        if ( on_stop_trajectory )
        {
            PLOG_ERROR << "IK 触发急停";
            Joint_stop( robot_ptr, current_pos, last_pos, last_last_pos );
        }
        else
            PLOG_INFO << "IK结束";

        FinishRunPlanningIK = true;
        
    }

    void admittance::sensor_update( rocos::Robot* robot_ptr )
    {
        // 6维力信息刷新，频率设置1000hz(需根据传感器设置)
        while ( !FinishRunPlanningIK )
        {
            my_ft_sensor_ptr->getting_data( robot_ptr->flange_ );
            std::this_thread::sleep_for( std::chrono::duration< double >( 0.001 ) );
        }
    }

#pragma endregion

    int moveL_vel( KDL::Twist& Cartesian_vel, double t, KDL::Frame start, KDL::Frame end ,double max_path_v,double max_path_a)
    {
        //** 变量初始化 **//
        KDL::Vector vel   = end.p - start.p;
        double Plength = vel.Norm( );
        KDL::Rotation R_start_end = start.M.Inverse( ) * end.M;
        KDL::Vector ration_axis;
        double angle                   = R_start_end.GetRotAngle( ration_axis );
        const double equivalent_radius = 0.1;
        double Rlength                 = ( equivalent_radius * abs( angle ) );
        double Path_length             = std::max( Plength, Rlength );
        rocos::DoubleS doubleS;
        //**-------------------------------**//

        doubleS.planProfile( 0, 0.0, 1.0, 0, 0, max_path_v / Path_length, max_path_a / Path_length,
                             max_path_a * 2 / Path_length );

        if ( !doubleS.isValidMovement( ) || !( doubleS.getDuration( ) > 0 ) )
        {
            PLOG_ERROR << "link velocity compution failed";
            return -1;
        }

        if ( vel.Norm( ) < 1e-3 )
            Cartesian_vel.vel = KDL::Vector{ };
        else
        {
            vel.Normalize( );  //零向量单位化会变成【1，0，0】
            Cartesian_vel.vel = vel * doubleS.vel( t ) * Path_length;
        }

        if ( angle < 1e-3 )
            Cartesian_vel.rot = KDL::Vector{ };
        else
        {
            Cartesian_vel.rot = ration_axis * doubleS.vel( t ) * Path_length;
        }

        return 0;
    }

}  // namespace JC_helper