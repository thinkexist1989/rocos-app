#include "JC_helper_dynamics.hpp"
#include "robot.h"
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

        std::this_thread::sleep_for( std::chrono::duration< double >{ 1 } );

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
        SendCommand( &socketHandle, COMMAND_START, 1 );
        res = Receive( &socketHandle );

        force_torque.force[ 0 ]  = res.fx / FORCE_DIV + 0.2;
        force_torque.force[ 1 ]  = res.fy / FORCE_DIV - 2.3;
        force_torque.force[ 2 ]  = res.fz / FORCE_DIV + 5.1;
        force_torque.torque[ 0 ] = res.tx / TORQUE_DIV + 0.05;
        force_torque.torque[ 1 ] = res.ty / TORQUE_DIV - 0.04;
        force_torque.torque[ 2 ] = res.tz / TORQUE_DIV - 0.09;

        //收到的力信息转换到base系
        force_torque.force = ( flange_pos * KDL::Frame{ KDL::Rotation::RPY( 0, 0, M_PI ), KDL::Vector( 0, 0, 0.035 ) } ) * force_torque.force;

        // 重力补偿
        force_torque.force = force_torque.force - init_force_torque.force;

        // TODO 处理力矩
        for ( int i{ 0 }; i < 3; i++ )
            force_torque.torque[ i ] = 0;

        //去除毛刺，限制2N
        for ( int i{ 0 }; i < 3; i++ )
            if ( abs( force_torque.force[ i ] ) < 2 )
                force_torque.force[ i ] = 0;
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
        // for ( int i{ 0 }; i < _joint_num; i++ )
        //     B[ i ] = 2 * 1 * sqrt( M[ i ] * K[ i ] );

        out_dat.open( "/home/think/rocos-app/debug/admittance.csv" );
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
        static double damp = 1;
        damp += value;

        for ( int i{ 0 }; i < _joint_num; i++ )
            B[ i ] = 2 * damp * sqrt( M[ i ] * K[ i ] );

        PLOG_DEBUG << "damp  = " << damp;
    }
#pragma endregion

#pragma region  //*导纳控制

    admittance::admittance( rocos::Robot* robot_ptr ) : _ik_vel{ robot_ptr->kinematics_.getChain( ) }
    {
        out_joint_csv.open( "/home/think/rocos-app/debug/joints.csv" );
    }

    admittance::~admittance( )
    {
        out_joint_csv.close( );
    }

    int admittance::init( KDL::Frame flange_pos )
    {
        //** 6维力初始化 **//
        if ( my_ft_sensor.init( flange_pos ) < 0 )
        {
            PLOG_ERROR << " force-torque sensor init failed ";
            return -1;
        }
        //**-------------------------------**//

        PLOG_INFO << " init success";

        return 0;
    }

    void admittance::start( rocos::Robot* robot_ptr, const std::vector< KDL::Frame >& traj_target )
    {
        std::shared_ptr< std::thread > _thread_IK{ nullptr };
        // std::shared_ptr< std::thread > _thread_motion{ nullptr };
        std::shared_ptr< std::thread > _thread_ft_sensor{ nullptr };

        _thread_ft_sensor.reset( new std::thread{ &JC_helper::admittance::sensor_update, this, robot_ptr } );
        _thread_IK.reset( new std::thread{ &JC_helper::admittance::IK, this, robot_ptr, std::ref( traj_target ) } );
        // _thread_motion.reset( new std::thread{ &JC_helper::admittance::motion, this, robot_ptr } );

        if ( traj_target.size( ) == 1 )  //示教模式
        {
            PLOG_INFO << " starting  teaching";

            std::string str;
            while ( str.compare( "break" ) != 0 )
            {
                PLOG_INFO << " enter 'break' to turnoff teaching function:";
                std::cin >> str;
            }
            on_stop_trajectory = true;
        }
        else  //运动导纳模式
        {
            PLOG_INFO << " starting  admittance motion";
        }

        // _thread_motion->join( );
        _thread_IK->join( );
        _thread_ft_sensor->join( );

        PLOG_INFO << "admittance  全部结束";
    }

    void admittance::IK( rocos::Robot* robot_ptr, const std::vector< KDL::Frame >& traj_target )
    {
        //** 变量初始化 **//
        // std::unique_lock< std::mutex > lock_traj_joint( mutex_traj_joint, std::defer_lock );  //不上锁
        KDL::Frame frame_target;
        KDL::JntArray _q_target( _joint_num );
        KDL::JntArray _q_init( _joint_num );
        std::vector< double > max_step;
        int_least64_t max_count{ 0 };
        KDL::Twist admittance_vel;
        KDL::JntArray joints_vel( _joint_num );

        //**-------------------------------**//

        for ( int i = 0; i < _joint_num; i++ )
        {
            max_step.push_back( robot_ptr->max_vel_[ i ] * 0.001 );
            _q_init( i )   = robot_ptr->pos_[ i ];
            _q_target( i ) = _q_init( i );
        }

        //** 轨迹计算 **//

        if ( traj_target.size( ) == 1 )  //示教模式
            max_count = numeric_limits< int_least64_t >::max( );
        else
            max_count = traj_target.size( );

        int traj_count{ 0 };
        for ( ; traj_count < max_count; traj_count++ )
        {
            // auto t_start = std::chrono::steady_clock::now( );

            // TODO 导纳计算
            smd.set_force( my_ft_sensor.force_torque.force[ 0 ], my_ft_sensor.force_torque.force[ 1 ], my_ft_sensor.force_torque.force[ 2 ] );
            // smd.set_force( 0, -20, 0 );
            // smd.set_torque( 0, 0, 0 );
            smd.calculate( frame_offset, admittance_vel );

            //** 读取最新Frame **//
            if ( traj_target.size( ) == 1 )  //示教模式
                frame_target = frame_offset * traj_target[ 0 ];
            else
                frame_target = frame_offset * traj_target[ traj_count ];

            //**-------------------------------**//

            //** IK求解 **//
            if ( ( robot_ptr->kinematics_ ).CartToJnt( _q_init, frame_target, _q_target ) < 0 )
            {
                PLOG_ERROR << " CartToJnt failed  frame_target =\n"
                           << frame_target;
                on_stop_trajectory = true;
                break;
            }
            //**-------------------------------**//

            //** 笛卡尔速度求解 **//
            // _ik_vel.CartToJnt( _q_init, admittance_vel, joints_vel );
            // KDL::Multiply( joints_vel, 0.001, joints_vel );
            // KDL::Add( _q_init, joints_vel, _q_target );
            //**-------------------------------**//

            //** 速度保护**//
            for ( int i = 0; i < _joint_num; i++ )
            {
                if ( abs( _q_target( i ) - _q_init( i ) ) >  3*max_step[ i ] )
                {
                    PLOG_ERROR << "joint[" << i << "] speep is too  fast";
                    PLOG_ERROR << "target speed = " << abs( _q_target( i ) - _q_init( i ) )
                               << " and  max_step=" << max_step[ i ];
                    PLOG_ERROR << "_q_target( " << i << " )  = " << _q_target( i ) * 180 / M_PI;
                    PLOG_ERROR << "_q_init( " << i << " ) =" << _q_init( i ) * 180 / M_PI;
                    on_stop_trajectory = true;
                    break;
                }
            }

            if ( on_stop_trajectory ) break;
            //**-------------------------------**//

            //** 打印 **//
            //  out_joint_csv << std::to_string( traj_count*0.001) << "\t,";
            // for ( int i = 0; i < _joint_num - 1; i++ )
            // {
            //     out_joint_csv << std::to_string( _q_target( i ) ) << "\t,";
            // }
            // out_joint_csv << std::to_string( _q_target( 6 ) );
            // out_joint_csv << "\n";
            //**-------------------------------**//

            //** 位置伺服 **//
            for ( int i = 0; i < _joint_num; ++i )
            {
                robot_ptr->pos_[ i ] = _q_target( i );
                robot_ptr->joints_[ i ]->setPosition( _q_target( i ) );
            }
            robot_ptr->hw_interface_->waitForSignal( 0 );
            //**-------------------------------**//

            // lock_traj_joint.lock( );
            traj_joint.push_back( _q_target );
            // lock_traj_joint.unlock( );
            _q_init = _q_target;

            // auto t_stop     = std::chrono::steady_clock::now( );
            // auto t_duration = std::chrono::duration< double >( t_stop - t_start );

            // if ( t_duration.count( ) < 0.0008 )
            // {
            //     std::this_thread::sleep_for( std::chrono::duration< double >( 0.0008 - t_duration.count( ) ) );
            // }
        }

        if ( on_stop_trajectory )
        {
            PLOG_ERROR << "IK 触发急停";
            motion_stop( robot_ptr, std::ref( traj_joint ), traj_count );
        }
        else
            PLOG_INFO << "IK结束";

        FinishRunPlanningIK = true;
    }

    void admittance::motion( rocos::Robot* robot_ptr )
    {
        //** 变量初始化 **//
        std::unique_lock< std::mutex > lock_traj_joint( mutex_traj_joint, std::defer_lock );  //不上锁
        int traj_joint_count = 0;
        KDL::JntArray joint_command;
        //**-------------------------------**//

        //** 正常情况下，接收IK解算后的关节轨迹 **//
        while ( !on_stop_trajectory )
        {
            //** 读取最新命令 **//
            lock_traj_joint.lock( );
            if ( traj_joint_count < traj_joint.size( ) )
            {
                joint_command = traj_joint[ traj_joint_count ];
                lock_traj_joint.unlock( );
            }
            else
            {
                lock_traj_joint.unlock( );
                PLOG_DEBUG << "已超过最新命令";
                PLOG_DEBUG << traj_joint_count;
                PLOG_DEBUG << traj_joint.size( );
                if ( FinishRunPlanningIK )
                    break;  //全部frame已经取出
                else
                    continue;
            }

            traj_joint_count++;
            //**-------------------------------**//

            //** 位置伺服 **//
            for ( int i = 0; i < _joint_num; ++i )
            {
                robot_ptr->pos_[ i ] = joint_command( i );
                robot_ptr->joints_[ i ]->setPosition( joint_command( i ) );
            }
            robot_ptr->hw_interface_->waitForSignal( 0 );
            //**-------------------------------**//
        }
        //**--------------------------------------------------------------**//

        //**紧急停止 ，可能原因：1.轨迹规划失败、2.IK求解失败 3.心跳保持超时**//
        if ( on_stop_trajectory )
        {
            PLOG_ERROR << "motion触发紧急停止";
            // for ( int i = 0; i < _joint_num; ++i )
            // {
            //     PLOG_DEBUG << "joints_vel[ " << i << " ] = " << joints_vel[ i ];
            //     PLOG_DEBUG << "joints_acc[ " << i << " ] = " << ( joints_vel[ i ] - joints_last_vel[ i ] ) / 0.001;
            //     PLOG_DEBUG;
            // }
            //!紧急停止措施，对于仿真是立刻停止，实物才能看到效果
            motion_stop( robot_ptr, std::ref( traj_joint ), traj_joint_count );
        }
        else
            PLOG_INFO << "motion 结束";

        // robot_ptr->is_running_motion    = false;  //机械臂运动已结束，可以执行其他离线类运动
        // on_stop_trajectory              = false;
    }

    void admittance::sensor_update( rocos::Robot* robot_ptr )
    {
        // 6维力信息刷新
        while ( !FinishRunPlanningIK )
            my_ft_sensor.getting_data( robot_ptr->flange_ );
    }

#pragma endregion

}  // namespace JC_helper