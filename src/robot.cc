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

#include "robot.h"

namespace rocos
{
    Robot::Robot( boost::shared_ptr< HardwareInterface > hw ) : hw_interface_( hw )
    {
        addAllJoints( );

        target_positions_.resize( jnt_num_ );
        target_positions_prev_.resize( jnt_num_ );
        target_velocities_.resize( jnt_num_ );
        target_torques_.resize( jnt_num_ );
        pos_.resize( jnt_num_ );
        vel_.resize( jnt_num_ );
        acc_.resize( jnt_num_ );
        max_vel_.resize( jnt_num_ );
        max_acc_.resize( jnt_num_ );
        max_jerk_.resize( jnt_num_ );
        interp_.resize( jnt_num_ );

        for ( int i = 0; i < jnt_num_; ++i )
        {
            pos_[ i ]                   = joints_[ i ]->getPosition( );
            target_positions_[ i ]      = pos_[ i ];
            target_positions_prev_[ i ] = pos_[ i ];

            vel_[ i ]               = joints_[ i ]->getVelocity( );
            target_velocities_[ i ] = vel_[ i ];

            target_torques_[ i ] = joints_[ i ]->getTorque( );

            max_vel_[ i ]  = joints_[ i ]->getMaxVel( );
            max_acc_[ i ]  = joints_[ i ]->getMaxAcc( );
            max_jerk_[ i ] = joints_[ i ]->getMaxJerk( );

            if ( profile_type_ == trapezoid )
            {
                interp_[ i ] = new Trapezoid;
            }
            else if ( profile_type_ == doubleS )
            {
                interp_[ i ] = new DoubleS;
            }
        }

        startMotionThread( );  //现在只计算JntToCart
    }

    void Robot::addAllJoints( )
    {
        jnt_num_ = hw_interface_->getSlaveNumber( );
        joints_.clear( );
        for ( int i = 0; i < jnt_num_; i++ )
        {
            joints_.push_back( boost::make_shared< Drive >( hw_interface_, i ) );
            joints_[ i ]->setMode( ModeOfOperation::CyclicSynchronousPositionMode );
        }
    }

    // TODO: 切换HW指针
    bool Robot::switchHW( boost::shared_ptr< HardwareInterface > hw ) { return false; }

    // TODO: 测试用MoveJ，阻塞运行，需要改为private
    void Robot::moveJ( const std::vector< double >& pos,
                       const std::vector< double >& max_vel,
                       const std::vector< double >& max_acc,
                       const std::vector< double >& max_jerk,
                       Robot::Synchronization sync, ProfileType type )
    {
        if ( pos.size( ) != jnt_num_ || max_vel.size( ) != jnt_num_ ||
             max_acc.size( ) != jnt_num_ || max_jerk.size( ) != jnt_num_ )
        {
            std::cout << "[ERROR] MoveJ => Error Input Vector Size!" << std::endl;
            return;
        }

        std::vector< R_INTERP_BASE* > interp( jnt_num_ );

        double max_time = 0.0;

        // Start trajectory generation....
        for ( int i = 0; i < jnt_num_; i++ )
        {
            auto p0 = joints_[ i ]->getPosition( );
            switch ( type )
            {
                case trapezoid:
                    interp[ i ] = new Trapezoid;
                    reinterpret_cast< Trapezoid* >( interp[ i ] )->planTrapezoidProfile( 0, p0, pos[ i ], 0, 0, max_vel[ i ], max_acc[ i ] );
                    break;
                case doubleS:
                    interp[ i ] = new DoubleS;
                    reinterpret_cast< DoubleS* >( interp[ i ] )->planDoubleSProfile( 0, p0, pos[ i ], 0, 0, max_vel[ i ], max_acc[ i ], max_jerk[ i ] );
                    break;
                default:
                    std::cout << "Not Supported Profile Type" << std::endl;
                    return;
            }

            max_time = max( max_time, interp[ i ]->getDuration( ) );
        }

        // Sync scaling....
        if ( sync == SYNC_TIME )
        {
            for_each( interp.begin( ), interp.end( ),
                      [ = ]( R_INTERP_BASE* p ) { p->scaleToDuration( max_time ); } );
        }
        else if ( sync == SYNC_PHASE )
        {
            std::cout
                << "[WARNING] Phase sync has not implemented...instead of time sync."
                << std::endl;
            for_each( interp.begin( ), interp.end( ),
                      [ = ]( R_INTERP_BASE* p ) { p->scaleToDuration( max_time ); } );
        }

        // Start moving....
        double dt = 0.0;
        while ( dt <= max_time )
        {
            hw_interface_->waitForSignal( 9 );

            for ( int i = 0; i < jnt_num_; i++ )
            {
                if ( !interp[ i ]->isValidMovement( ) )
                {
                    continue;
                }
                switch ( joints_[ i ]->getMode( ) )
                {
                    case ModeOfOperation::CyclicSynchronousPositionMode:
                        joints_[ i ]->setPosition( interp[ i ]->pos( dt ) );
                        break;
                    case ModeOfOperation::CyclicSynchronousVelocityMode:
                        joints_[ i ]->setVelocity( interp[ i ]->vel( dt ) );
                        break;
                    default:
                        std::cout << "Only Supported CSP and CSV" << std::endl;
                }
            }

            dt += 0.001;
        }

        // delete pointer
        for ( auto& p : interp )
        {
            delete p;
        }
    }

    void Robot::setEnabled( )
    {
        for_each( joints_.begin( ), joints_.end( ),
                  [ = ]( boost::shared_ptr< Drive >& d ) { d->setEnabled( ); } );
    }

    void Robot::setDisabled( )
    {
        for_each( joints_.begin( ), joints_.end( ),
                  [ = ]( boost::shared_ptr< Drive >& d ) { d->setDisabled( ); } );
    }

    void Robot::startMotionThread( )
    {
        is_running_ = true;
        otg_motion_thread_ =
            boost::make_shared< boost::thread >( &Robot::motionThreadHandler, this );
        //        boost::thread(&Robot::motionThreadHandler, this);
    }

    void Robot::stopMotionThread( )
    {
        is_running_ = false;
        otg_motion_thread_->interrupt( );
        otg_motion_thread_->join( );  //等待运动线程结束
    }

    void Robot::motionThreadHandler( )
    {
        std::cout << "Motion thread is running on thread "
                  << boost::this_thread::get_id( ) << std::endl;
        //** vector 数组大小初始化 **//
        target_positions_.resize( jnt_num_ );
        target_positions_prev_.resize( jnt_num_ );
        target_velocities_.resize( jnt_num_ );
        target_torques_.resize( jnt_num_ );
        pos_.resize( jnt_num_ );
        vel_.resize( jnt_num_ );
        acc_.resize( jnt_num_ );
        max_vel_.resize( jnt_num_ );
        max_acc_.resize( jnt_num_ );
        max_jerk_.resize( jnt_num_ );
        interp_.resize( jnt_num_ );
        need_plan_.resize( jnt_num_, false );
        //**-------------------------------**//
        //** vector 数组数值初始化 **//
        for ( int i = 0; i < jnt_num_; ++i )
        {
            pos_[ i ]                   = joints_[ i ]->getPosition( );
            target_positions_[ i ]      = pos_[ i ];
            target_positions_prev_[ i ] = pos_[ i ];

            vel_[ i ]               = joints_[ i ]->getVelocity( );
            target_velocities_[ i ] = vel_[ i ];

            target_torques_[ i ] = joints_[ i ]->getTorque( );

            if ( profile_type_ == trapezoid )
            {
                interp_[ i ] = new Trapezoid;
            }
            else if ( profile_type_ == doubleS )
            {
                interp_[ i ] = new DoubleS;
            }
        }
        //**-------------------------------**//

        std::vector< double > dt( jnt_num_, 0.0 );  // delta T
        double max_time = 0.0;

        while ( is_running_ )
        {  // while start

            hw_interface_->waitForSignal( 9 );

            //!< Update Flange State
            updateCartesianInfo( );

            //     //! 屏蔽开始
            //    //!< Trajectory generating......
            //    max_time = 0.0;
            //    //** 轨迹生成 **//
            //    for (int i = 0; i < jnt_num_; ++i) {
            //      if (joints_[i]->getDriveState() != DriveState::OperationEnabled) {
            //        target_positions_[i] = joints_[i]->getPosition();
            //        //                    target_positions_prev_[i] = target_positions_[i];
            //
            //        continue;  // Disabled, ignore
            //      }
            //
            //      if (fabs(target_positions_[i] != target_positions_prev_[i]) ||
            //          need_plan_[i]) {  // need update
            //        target_positions_prev_[i] =
            //            target_positions_[i];  // assign to current target position
            //
            //        interp_[i]->planProfile(0,                          // t
            //                                pos_[i],                    // p0
            //                                target_positions_prev_[i],  // pf
            //                                vel_[i],                    // v0
            //                                target_velocities_[i],      // vf
            //                                max_vel_[i], max_acc_[i], max_jerk_[i]);
            //
            //        dt[i] = 0.0;  // once regenerate, dt = 0.0
            //        // consider at least execute time
            //        if (interp_[i]->getDuration() < least_motion_time_) {
            //          interp_[i]->scaleToDuration(least_motion_time_);
            //        }
            //        // record max_time
            //        max_time =
            //            max(max_time, interp_[i]->getDuration());  // get max duration time
            //
            //        need_plan_[i] = false;
            //      }
            //    }
            //    //**-------------------------------**//
            //
            //    //!< Sync scaling....
            //    //各关节时间同步
            //    if (sync_ == SYNC_TIME) {
            //      for_each(interp_.begin(), interp_.end(),
            //               [=](R_INTERP_BASE* p) { p->scaleToDuration(max_time); });
            //    }
            //    //各关节无需同步
            //    else if (sync_ == SYNC_NONE) {
            //    }
            //    //各关节相位同步
            //    else if (sync_ == SYNC_PHASE) {
            //      std::cout
            //          << "[WARNING] Phase sync has not implemented...instead of time sync."
            //          << std::endl;
            //      for_each(interp_.begin(), interp_.end(),
            //               [=](R_INTERP_BASE* p) { p->scaleToDuration(max_time); });
            //    }
            //
            //    //!< Start moving....
            //    for (int i = 0; i < jnt_num_; ++i) {
            //      if (joints_[i]->getDriveState() != DriveState::OperationEnabled) {
            //        continue;  // Disabled, ignore
            //      }
            //
            //      if (interp_[i]->isValidMovement()) {
            //        dt[i] += 0.001;
            //        pos_[i] = interp_[i]->pos(dt[i]);  //当前位置更新
            //        vel_[i] = interp_[i]->vel((dt[i]));
            //        acc_[i] = interp_[i]->acc(dt[i]);
            //
            //        switch (joints_[i]->getMode()) {
            //          case ModeOfOperation::CyclicSynchronousPositionMode:
            //            joints_[i]->setPosition(pos_[i]);
            //            break;
            //          case ModeOfOperation::CyclicSynchronousVelocityMode:
            //            joints_[i]->setVelocity(vel_[i]);
            //            break;
            //          default:
            //            std::cout << "Only Supported CSP and CSV" << std::endl;
            //        }
            //      } else {
            //        vel_[i] = 0.0;
            //      }
            ////      std::cout << " pos_[" << i << "]  = " << pos_[i] << std::endl;
            //    }
            //
            //    //! 屏蔽结束

            ////    std::cout << "----------------------" << std::endl;
        }

        // process before exit:
    }

    void Robot::moveJ( const vector< double >& target_pos,
                       const vector< double >& target_vel,
                       Robot::Synchronization sync )
    {
        if ( ( target_pos.size( ) != jnt_num_ ) || ( target_vel.size( ) != jnt_num_ ) )
        {
            std::cout << "[ERROR] MoveJ => Error Input Vector Size!" << std::endl;
            return;
        }

        sync_ = sync;

        target_positions_  = target_pos;
        target_velocities_ = target_vel;

        need_plan_.resize( jnt_num_, true );
    }

    /// \brief 停止单轴运动
    /// \param id 轴ID
    void Robot::stopSingleAxis( int id )
    {
        double dt = fabs( vel_[ id ] ) / max_acc_[ id ];  // 所需要的减速时间
        target_positions_[ id ] =
            pos_[ id ] +
            dt * vel_[ id ] / 2.0;  // TODO：这个减速段计算有问题
        //        target_positions_[id] = pos_[id];
        target_velocities_[ id ] = 0.0;
        least_motion_time_       = 0.0;

        auto sync = sync_;
        sync_     = SYNC_NONE;  //停止时候就不需要同步了

        std::cout << "max_acc: " << max_acc_[ id ] << "; pos: " << pos_[ id ]
                  << "; vel: " << vel_[ id ] << std::endl;
        std::cout << "dt: " << dt << "; target_positions: " << target_positions_[ id ]
                  << std::endl;

        need_plan_[ id ] = true;

        //        usleep(dt * 1000000);
        //        sync_ = sync;
    }

    void Robot::stopMultiAxis( )
    {
        //        auto sync = sync_;
        sync_ = SYNC_NONE;  //停止时候就不需要同步了

        double wait_time = 0.0;

        for ( int id = 0; id < jnt_num_; ++id )
        {
            double dt = fabs( vel_[ id ] ) / max_acc_[ id ];  // 所需要的减速时间
            target_positions_[ id ] =
                pos_[ id ] +
                2 * ( dt * vel_[ id ] / 2.0 );  // TODO：这个减速段计算有问题
            //        target_positions_[id] = pos_[id];
            target_velocities_[ id ] = 0.0;
            least_motion_time_       = 0.0;

            std::cout << "max_acc: " << max_acc_[ id ] << "; pos: " << pos_[ id ]
                      << "; vel: " << vel_[ id ] << std::endl;
            std::cout << "dt: " << dt << "; target_positions: " << target_positions_[ id ]
                      << std::endl;

            need_plan_[ id ] = true;

            wait_time = wait_time <= dt ? wait_time : dt;
        }
    }

    /// 设置单轴运动
    /// \param id 轴ID
    /// \param pos 目标位置
    /// \param vel 目标速度
    /// \param max_vel 最大速度
    /// \param max_acc 最大加速度
    /// \param max_jerk 最大加加速度
    /// \param least_time 最短运行时间
    void Robot::moveSingleAxis( int id, double pos, double vel, double max_vel,
                                double max_acc, double max_jerk, double least_time )
    {
        target_positions_[ id ]  = pos;
        target_velocities_[ id ] = vel;

        if ( max_vel != -1 ) max_vel_[ id ] = max_vel;
        if ( max_acc != -1 ) max_acc_[ id ] = max_acc;
        if ( max_jerk != -1 ) max_jerk_[ id ] = max_jerk;
        if ( least_time != -1 ) least_motion_time_ = least_time;

        need_plan_[ id ] = true;
    }

    /// 设置多轴运动
    /// \param target_pos 目标位置
    /// \param target_vel 目标速度
    /// \param max_vel 最大速度
    /// \param max_acc 最大加速度
    /// \param max_jerk 最大加加速度
    /// \param least_time 最短运行时间
    void Robot::moveMultiAxis( const vector< double >& target_pos,
                               const vector< double >& target_vel,
                               const vector< double >& max_vel,
                               const vector< double >& max_acc,
                               const vector< double >& max_jerk, double least_time )
    {
        if ( ( target_pos.size( ) != jnt_num_ ) || ( target_vel.size( ) != jnt_num_ ) ||
             ( max_vel.size( ) != jnt_num_ ) || ( max_acc.size( ) != jnt_num_ ) ||
             ( max_jerk.size( ) != jnt_num_ ) )
        {
            std::cout << "[ERROR] moveMultiAxis: wrong size!" << std::endl;
        }

        for ( int id = 0; id < jnt_num_; ++id )
        {
            target_positions_[ id ]  = target_pos[ id ];
            target_velocities_[ id ] = target_vel[ id ];

            if ( max_vel[ id ] != -1 ) max_vel_[ id ] = max_vel[ id ];
            if ( max_acc[ id ] != -1 ) max_acc_[ id ] = max_acc[ id ];
            if ( max_jerk[ id ] != -1 ) max_jerk_[ id ] = max_jerk[ id ];

            need_plan_[ id ] = true;
        }

        if ( least_time != -1 ) least_motion_time_ = least_time;
    }

    //TODO 心跳刷新
    void Robot::HeartKeepAlive( )
    {
        std::lock_guard< std::mutex > lck( tick_lock );
        tick_count++;
    }

    /////// Motion Command /////////////

    int Robot::MoveJ( JntArray q, double speed, double acceleration, double time,
                      double radius, bool asynchronous )
    {
        if ( radius )
        {
            std::cerr << RED << " radius not supported yet" << WHITE << std::endl;
            return -1;
        }
        if ( time )
        {
            std::cerr << RED << " time not supported yet" << WHITE << std::endl;
            return -1;
        }

        if ( CheckBeforeMove( q, speed, acceleration, time, radius ) < 0 )
        {
            std::cerr << RED << "MoveJ():given parameters is invalid" << WHITE << std::endl;
            return -1;
        }

        if ( is_running_motion )  //不是OTG规划，异步/同步都不能打断
        {
            std::cerr << RED << " Motion is still running and waiting for it to finish"
                      << WHITE << std::endl;
            motion_thread_->join( );
        }

        if ( asynchronous )  //异步执行
        {
            motion_thread_.reset( new boost::thread{ &Robot::RunMoveJ, this, q,
                                                     speed, acceleration, time,
                                                     radius } );
            is_running_motion = true;
        }
        else  //同步执行
        {
            motion_thread_.reset( new boost::thread{ &Robot::RunMoveJ, this, q,
                                                     speed, acceleration, time,
                                                     radius } );
            motion_thread_->join( );
            is_running_motion = false;
        }

        return 0;
    }

    int Robot::MoveJ_IK( Frame pose, double speed, double acceleration, double time,
                         double radius, bool asynchronous )
    {
        std::cout << "MoveJ_IK pose: " << pose << std::endl;
        JntArray q_init( jnt_num_ );
        JntArray q_target( jnt_num_ );
        for ( int i = 0; i < jnt_num_; i++ )
        {
            q_init.data[ i ] = pos_[ i ];
        }
        if ( kinematics_.CartToJnt( q_init, pose, q_target ) < 0 )
        {
            std::cerr << RED << " MoveJ_IK():CartToJnt failed" << WHITE << std::endl;
            return -1;
        }
        return MoveJ( q_target, speed, acceleration, time, radius, asynchronous );
    }

    int Robot::MoveL( Frame pose, double speed, double acceleration, double time,
                      double radius, bool asynchronous )
    {
        if ( radius )
        {
            std::cerr << RED << " radius not supported yet" << WHITE << std::endl;
            return -1;
        }
        if ( time )
        {
            std::cerr << RED << " time not supported yet" << WHITE << std::endl;
            return -1;
        }

        if ( CheckBeforeMove( pose, speed, acceleration, time, radius ) < 0 )
        {
            std::cerr << RED << "MoveL():given parameters is invalid" << WHITE << std::endl;
            return -1;
        }

        //** 变量初始化 **//
        KDL::Vector Pstart  = flange_.p;
        KDL::Vector Pend    = pose.p;
        KDL::Vector Plenght = Pend - Pstart;
        traj_.clear( );
        KDL::JntArray q_init( jnt_num_ );
        KDL::JntArray q_target( jnt_num_ );
        double s = 0;
        std::unique_ptr< R_INTERP_BASE > doubleS( new rocos::DoubleS );
        //**-------------------------------**//

        doubleS->planProfile(
            0, 0, 1, 0, 0, speed / Plenght.Norm( ), acceleration / Plenght.Norm( ),
            ( *std::min_element( std::begin( max_jerk_ ), std::end( max_jerk_ ) ) ) / Plenght.Norm( ) );

        if ( !doubleS->isValidMovement( ) || !( doubleS->getDuration( ) > 0 ) )
        {
            std::cerr << RED << "MoveL():moveL trajectory "
                      << "is infeasible " << WHITE << std::endl;
            return -1;
        }

        //** 变量初始化 **//
        double dt       = 0;
        double duration = doubleS->getDuration( );
        std::vector< double > Quaternion_start{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_end{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_interp{ 0, 0, 0, 0 };
        flange_.M.GetQuaternion( Quaternion_start.at( 0 ), Quaternion_start.at( 1 ),
                                 Quaternion_start.at( 2 ), Quaternion_start.at( 3 ) );
        pose.M.GetQuaternion( Quaternion_end.at( 0 ), Quaternion_end.at( 1 ),
                              Quaternion_end.at( 2 ), Quaternion_end.at( 3 ) );
        std::vector< double > max_step;

        //**-------------------------------**//

        for ( int i = 0; i < jnt_num_; i++ )
        {
            q_init( i ) = pos_[ i ];
            max_step.push_back( max_vel_[ i ] * 0.001 );
        }

        //** 轨迹计算 **//
        while ( dt <= duration )
        {
            s             = doubleS->pos( dt );
            KDL::Vector P = Pstart + Plenght * s;
            Quaternion_interp =
                JC_helper::UnitQuaternion_intep( Quaternion_start, Quaternion_end, s );
            KDL::Frame interp_frame(
                KDL::Rotation::Quaternion( Quaternion_interp[ 0 ], Quaternion_interp[ 1 ],
                                           Quaternion_interp[ 2 ], Quaternion_interp[ 3 ] ),
                P );
            if ( kinematics_.CartToJnt( q_init, interp_frame, q_target ) < 0 )
            {
                std::cerr << RED << "MoveL(): CartToJnt failed " << WHITE << std::endl;
                return -1;
            }
            //防止奇异位置速度激增
            for ( int i = 0; i < jnt_num_; i++ )
            {
                if ( abs( q_target( i ) - q_init( i ) ) > max_step[ i ] )
                {
                    std::cout << RED << "MoveL():joint[" << i << "] speep is too  fast" << WHITE << std::endl;
                    return -1;
                }
            }

            q_init = q_target;
            traj_.push_back( q_target );  //TODO: 未初始化
            dt += 0.001;
        }
        //**-------------------------------**//

        if ( is_running_motion )  //不是OTG规划，异步/同步都不能打断
        {
            std::cerr << RED << " Motion is still running and waiting for it to finish"
                      << WHITE << std::endl;
            motion_thread_->join( );
        }

        if ( asynchronous )  //异步执行
        {
            motion_thread_.reset( new boost::thread{ &Robot::RunMoveL, this, traj_ } );
            is_running_motion = true;
        }
        else  //同步执行
        {
            motion_thread_.reset( new boost::thread{ &Robot::RunMoveL, this, traj_ } );
            motion_thread_->join( );
            is_running_motion = false;
        }

        return 0;
    }

    int Robot::MoveL_FK( JntArray q, double speed, double acceleration, double time,
                         double radius, bool asynchronous )
    {
        KDL::Frame target;
        kinematics_.JntToCart( q, target );
        std::cout << "Target pose is: \n"
                  << target << std::endl;
        return MoveL( target, speed, acceleration, time, radius, asynchronous );
    }

    int Robot::MoveC( Frame pose_via, Frame pose_to, double speed,
                      double acceleration, double time, double radius,
                      Robot::OrientationMode mode, bool asynchronous )
    {
        if ( radius )
        {
            std::cerr << RED << " radius not supported yet" << WHITE << std::endl;
            return -1;
        }
        if ( time )
        {
            std::cerr << RED << " time not supported yet" << WHITE << std::endl;
            return -1;
        }

        if ( CheckBeforeMove( pose_via, speed, acceleration, time, radius ) < 0 )
        {
            std::cerr << RED << "MoveC():given parameters is invalid" << WHITE << std::endl;
            return -1;
        }

        if ( CheckBeforeMove( pose_to, speed, acceleration, time, radius ) < 0 )
        {
            std::cerr << RED << "MoveC():given parameters is invalid" << WHITE << std::endl;
            return -1;
        }

        //** 变量初始化 **//
        traj_.clear( );
        KDL::JntArray q_init( jnt_num_ );
        KDL::JntArray q_target( jnt_num_ );
        KDL::Frame f_flange = flange_;
        double dt           = 0;
        std::vector< double > max_step;
        bool orientation_fixed = mode == Robot::OrientationMode::FIXED ? true : false;
        std::vector< KDL::Frame > traj_target;
        //**-------------------------------**//

        for ( int i = 0; i < jnt_num_; i++ )
        {
            q_init( i ) = pos_[ i ];
            max_step.push_back( max_vel_[ i ] * 0.001 );
        }

        if ( JC_helper::circle_trajectory( traj_target, f_flange, pose_via, pose_to, speed, acceleration, orientation_fixed ) < 0 )
        {
            std::cerr << RED << "MoveC(): circle trajectory planning fail " << WHITE << std::endl;
            return -1;
        }

        //** 轨迹计算 **//
        for ( const auto& target : traj_target )
        {
            if ( kinematics_.CartToJnt( q_init, target, q_target ) < 0 )
            {
                std::cerr << RED << "MoveC(): CartToJnt failed " << WHITE << std::endl;
                return -1;
            }
            //防止奇异位置速度激增
            for ( int i = 0; i < jnt_num_; i++ )
            {
                if ( abs( q_target( i ) - q_init( i ) ) > max_step[ i ] )
                {
                    std::cout << RED << "MoveC():joint[" << i << "] speep is too  fast" << WHITE << std::endl;
                    return -1;
                }
            }

            q_init = q_target;
            traj_.push_back( q_target );
            dt += 0.001;
        }
        //**-------------------------------**//

        if ( is_running_motion )  //不是OTG规划，异步/同步都不能打断
        {
            std::cerr << RED << " Motion is still running and waiting for it to finish"
                      << WHITE << std::endl;
            motion_thread_->join( );
        }

        if ( asynchronous )  //异步执行
        {
            motion_thread_.reset( new boost::thread{ &Robot::RunMoveL, this, traj_ } );
            is_running_motion = true;
        }
        else  //同步执行
        {
            motion_thread_.reset( new boost::thread{ &Robot::RunMoveL, this, traj_ } );
            motion_thread_->join( );
            is_running_motion = false;
        }

        return 0;
    }

    int Robot::MoveR( Rotation rotation_to, double speed,
                      double acceleration, double time, bool asynchronous, double equivalent_radius )
    {
        if ( time )
        {
            std::cerr << RED << " MoveR():time not supported yet" << WHITE << std::endl;
            return -1;
        }

        if ( CheckBeforeMove( Frame{ rotation_to, flange_.p }, speed, acceleration, time, equivalent_radius ) < 0 )
        {
            std::cerr << RED << "MoveR():given parameters is invalid" << WHITE << std::endl;
            return -1;
        }
        if ( equivalent_radius < epsilon )
        {
            std::cerr << RED << " MoveR():equivalent_radius too small" << WHITE << std::endl;
            return -1;
        }
        //** 变量初始化 **//
        traj_.clear( );
        KDL::JntArray q_init( jnt_num_ );
        KDL::JntArray q_target( jnt_num_ );
        KDL::Frame f_flange = flange_;
        double dt           = 0;
        std::vector< double > max_step;
        std::vector< KDL::Frame > traj_target;
        //**-------------------------------**//

        for ( int i = 0; i < jnt_num_; i++ )
        {
            q_init( i ) = pos_[ i ];
            max_step.push_back( max_vel_[ i ] * 0.001 );
        }

        if ( JC_helper::rotation_trajectory( traj_target, f_flange.p, f_flange.M, rotation_to, speed, acceleration, equivalent_radius ) < 0 )
        {
            std::cerr << RED << "MoveR():  Rotation trajectory planning fail " << WHITE << std::endl;
            return -1;
        }

        //** 轨迹计算 **//
        for ( const auto& target : traj_target )
        {
            if ( kinematics_.CartToJnt( q_init, target, q_target ) < 0 )
            {
                std::cerr << RED << "MoveR(): CartToJnt failed " << WHITE << std::endl;
                return -1;
            }
            //防止奇异位置速度激增
            for ( int i = 0; i < jnt_num_; i++ )
            {
                if ( abs( q_target( i ) - q_init( i ) ) > max_step[ i ] )
                {
                    std::cout << RED << "MoveR():joint[" << i << "] speep is too  fast" << WHITE << std::endl;
                    return -1;
                }
            }

            q_init = q_target;
            traj_.push_back( q_target );
            dt += 0.001;
        }
        //**-------------------------------**//

        if ( is_running_motion )  //不是OTG规划，异步/同步都不能打断
        {
            std::cerr << RED << " Motion is still running and waiting for it to finish"
                      << WHITE << std::endl;
            motion_thread_->join( );
        }

        if ( asynchronous )  //异步执行
        {
            motion_thread_.reset( new boost::thread{ &Robot::RunMoveL, this, traj_ } );
            is_running_motion = true;
        }
        else  //同步执行
        {
            motion_thread_.reset( new boost::thread{ &Robot::RunMoveL, this, traj_ } );
            motion_thread_->join( );
            is_running_motion = false;
        }

        return 0;
    }

    int Robot::MoveP( Frame pose, double speed, double acceleration, double time,
                      double radius, bool asynchronous )
    {
        return 0;
    }

    int Robot::MovePath( const Path& path, bool asynchronous ) { return 0; }

    int Robot::MultiMoveL( const std::vector< KDL::Frame >& point, std::vector< double > bound_dist, std::vector< double > max_path_v, std::vector< double > max_path_a, bool asynchronous )
    {
        std::vector< KDL::Frame > traj_target;
        std::vector< int > traj_index;
        KDL::Frame Cart_point = flange_;

        if ( point.size( ) == 0 )
        {
            std::cout << RED << "MultiMoveL(): point size is at least one or more" << std::endl;
            return -1;
        }
        //一段轨迹不存在圆弧过渡处理
        else if ( point.size( ) == 1 )
        {
            std::cout << GREEN << "***************第1次规划***************" << WHITE << std::endl;
            if ( JC_helper::link_trajectory( traj_target, Cart_point, point[ 0 ], 0, 0, max_path_v[ 0 ], max_path_a[ 0 ] ) < 0 )
            {
                std::cerr << RED << "MultiMoveL(): given parameters is invalid in the 1th planning "
                          << WHITE << std::endl;
                return -1;
            }
            traj_index.push_back( traj_target.size( ) );
        }
        else
        {
            KDL::Frame Frame_motion_1;
            KDL::Frame Frame_motion_2;
            double motion_v_1;
            double motion_v_2;
            int success{ 0 };

            std::cout << GREEN << "***************第1次规划***************" << WHITE << std::endl;
            success = JC_helper::multilink_trajectory( traj_target, Cart_point, point[ 0 ], point[ 1 ], Frame_motion_1, 0, motion_v_1, bound_dist[ 0 ], max_path_v[ 0 ], max_path_a[ 0 ], max_path_v[ 1 ] );
            if ( success < 0 )
            {
                std::cerr << RED << "MultiMoveL(): given parameters is invalid in the 1th planning "
                          << WHITE << std::endl;
                return -1;
            }
            traj_index.push_back( traj_target.size( ) );

            for ( int i = 1; i < ( point.size( ) - 1 ); i++ )
            {
                std::cout << GREEN << "***************第" << i + 1 << "次规划***************" << WHITE << std::endl;
                success = JC_helper::multilink_trajectory( traj_target, Frame_motion_1, point[ i ], point[ i + 1 ], Frame_motion_2, motion_v_1, motion_v_2, bound_dist[ i ], max_path_v[ i ], max_path_a[ i ], max_path_v[ i + 1 ] );
                if ( success < 0 )
                {
                    std::cerr << RED << "MultiMoveL(): given parameters is invalid in the " << i + 1 << "th planning "
                              << WHITE << std::endl;
                    return -1;
                }
                Frame_motion_1 = Frame_motion_2;
                motion_v_1     = motion_v_2;
                traj_index.push_back( traj_target.size( ) );
            }

            std::cout << GREEN << "***************第" << point.size( ) << "次规划***************" << WHITE << std::endl;
            success = JC_helper::link_trajectory( traj_target, Frame_motion_1, point.back( ), motion_v_1, 0, max_path_v.back( ), max_path_a.back( ) );
            if ( success < 0 )
            {
                std::cerr << RED << "MultiMoveL(): given parameters is invalid in the last of planning "
                          << WHITE << std::endl;
                return -1;
            }
            traj_index.push_back( traj_target.size( ) );
        }
        std::cout << GREEN << "***************规划全部完成***************" << WHITE << std::endl;

        std::vector< double > max_step;
        KDL::JntArray q_init( jnt_num_ );
        KDL::JntArray q_target( jnt_num_ );
        int count{ 0 };
        int p = 0;  //表示当前正处理第几段轨迹

        traj_.clear( );

        for ( int i = 0; i < jnt_num_; i++ )
        {
            q_init( i )   = pos_[ i ];
            q_target( i ) = q_init( i );
            max_step.push_back( max_vel_[ i ] * 0.001 );
        }

        //** IK计算 **//
        for ( const auto& pos_goal : traj_target )
        {
            q_init = q_target;
            if ( kinematics_.CartToJnt( q_init, pos_goal, q_target ) < 0 )
            {
                for ( int i = 0; i < traj_index.size( ); i++ )
                    p = count < traj_index[ i ] ? i + 1 : p;  //找到当前是第几段轨迹
                std::cerr << RED << "MultiMoveL():CartToJnt failed on the " << p << "th trajectory，please chose other interpolate Points "
                          << WHITE << std::endl;
                return -1;
            }
            //防止奇异位置速度激增
            for ( int i = 0; i < jnt_num_; i++ )
            {
                if ( abs( q_target( i ) - q_init( i ) ) > max_step[ i ] )
                {
                    for ( int j = 0; j < traj_index.size( ); j++ )
                        p = count < traj_index[ j ] ? ( j + 1 ) : p;  //找到当前是第几段轨迹
                    std::cout << RED << "MultiMoveL():count =" << p << count << std::endl;
                    std::cout << RED << "MultiMoveL():joint[" << i << "] speep is too  fast on the " << p << "th trajectory" << std::endl;
                    std::cout << RED << "MultiMoveL():target speed = " << abs( q_target( i ) - q_init( i ) ) << "and  max_step=" << max_step[ i ] << std::endl;
                    std::cout << RED << "MultiMoveL():q_target( " << i << " )  = " << q_target( i ) << std::endl;
                    std::cout << RED << "MultiMoveL():q_init( " << i << " ) =" << q_init( i ) << WHITE << std::endl;
                    return -1;
                }
            }
            traj_.push_back( q_target );
            count++;
        }
        //**-------------------------------**//

        if ( is_running_motion )  //不是OTG规划，异步/同步都不能打断
        {
            std::cerr << RED << " Motion is still running and waiting for it to finish"
                      << WHITE << std::endl;
            motion_thread_->join( );
        }

        if ( asynchronous )  //异步执行
        {
            motion_thread_.reset( new boost::thread{ &Robot::RunMultiMoveL, this, traj_ } );
            is_running_motion = true;
        }
        else  //同步执行
        {
            motion_thread_.reset( new boost::thread{ &Robot::RunMultiMoveL, this, traj_ } );
            motion_thread_->join( );
            is_running_motion = false;
        }

        return 0;
    }

    int Robot::Dragging( Frame pose, double speed, double acceleration, double time,
                         double radius )
    {
        if ( radius )
        {
            std::cerr << RED << " dragging(): radius not supported yet" << WHITE << std::endl;
            return -1;
        }
        if ( time )
        {
            std::cerr << RED << "dragging(): time not supported yet" << WHITE << std::endl;
            return -1;
        }

        if ( CheckBeforeMove( pose, speed, acceleration, time, radius ) < 0 )
        {
            std::cerr << RED << "dragging():given parameters is invalid" << WHITE << std::endl;
            return -1;
        }

        if ( is_running_motion )
        {
            std::cerr << GREEN << "dragging(): Motion is still running and keep heard alive"
                      << WHITE << std::endl;
            HeartKeepAlive( );
            return 0;
        }

        //** 变量初始化 **//
        KDL::Vector Pstart  = flange_.p;
        KDL::Vector Pend    = pose.p;
        KDL::Vector Plenght = Pend - Pstart;
        traj_.clear( );
        KDL::JntArray q_init( jnt_num_ );
        KDL::JntArray q_target( jnt_num_ );
        double s = 0;
        std::unique_ptr< R_INTERP_BASE > doubleS( new rocos::DoubleS );
        //**-------------------------------**//

        //** 规划速度设置 **//
        //TODO 完善速度规划
        doubleS->planProfile(
            0, 0, 1, 0, 0, speed / Plenght.Norm( ), acceleration / Plenght.Norm( ),
            ( *std::min_element( std::begin( max_jerk_ ), std::end( max_jerk_ ) ) ) / Plenght.Norm( ) );

        if ( !doubleS->isValidMovement( ) || !( doubleS->getDuration( ) > 0 ) )
        {
            std::cerr << RED << "dragging():dragging trajectory "
                      << "is infeasible " << WHITE << std::endl;
            return -1;
        }
        //**-------------------------------**//

        //** 变量初始化 **//
        double dt       = 0;
        double duration = doubleS->getDuration( );
        std::vector< double > Quaternion_start{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_end{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_interp{ 0, 0, 0, 0 };
        flange_.M.GetQuaternion( Quaternion_start.at( 0 ), Quaternion_start.at( 1 ),
                                 Quaternion_start.at( 2 ), Quaternion_start.at( 3 ) );
        pose.M.GetQuaternion( Quaternion_end.at( 0 ), Quaternion_end.at( 1 ),
                              Quaternion_end.at( 2 ), Quaternion_end.at( 3 ) );
        std::vector< double > max_step;
        //**-------------------------------**//

        for ( int i = 0; i < jnt_num_; i++ )
        {
            q_init( i ) = pos_[ i ];
            max_step.push_back( max_vel_[ i ] * 0.001 );
        }

        //** 轨迹计算 **//
        while ( dt <= duration )
        {
            s             = doubleS->pos( dt );
            KDL::Vector P = Pstart + Plenght * s;
            Quaternion_interp =
                JC_helper::UnitQuaternion_intep( Quaternion_start, Quaternion_end, s );
            KDL::Frame interp_frame(
                KDL::Rotation::Quaternion( Quaternion_interp[ 0 ], Quaternion_interp[ 1 ],
                                           Quaternion_interp[ 2 ], Quaternion_interp[ 3 ] ),
                P );
            if ( kinematics_.CartToJnt( q_init, interp_frame, q_target ) < 0 )
            {
                std::cerr << RED << " dragging():CartToJnt failed " << WHITE << std::endl;
                return -1;
            }
            //防止奇异位置速度激增
            for ( int i = 0; i < jnt_num_; i++ )
            {
                if ( abs( q_target( i ) - q_init( i ) ) > max_step[ i ] )
                {
                    std::cout << RED << "dragging():joint[" << i << "] speep is too  fast" << WHITE << std::endl;
                    return -1;
                }
            }

            q_init = q_target;
            traj_.push_back( q_target );  //TODO: 未初始化
            dt += 0.001;
        }
        //**-------------------------------**//

        motion_thread_.reset( new boost::thread{ &Robot::RunDragging, this, traj_ } );
        is_running_motion = true;

        return 0;
    }

    int Robot::CheckBeforeMove( const JntArray& q, double speed, double acceleration,
                                double time, double radius )
    {
        //** 数据有效性检查  **//
        for ( int i = 0; i < jnt_num_; i++ )
        {  //TODO 这里的速度、加速度目前只针对关节空间进行检查
            //位置检查
            if ( q( i ) > joints_[ i ]->getMaxPosLimit( ) ||
                 q( i ) < joints_[ i ]->getMinPosLimit( ) )
            {
                std::cerr << RED << " CheckBeforeMove():  Pos command is out of range" << WHITE << std::endl;
                return -1;
            }
            //速度检查
            if ( speed > joints_[ i ]->getMaxVel( ) ||
                 speed < ( -1 ) * joints_[ i ]->getMaxVel( ) )
            {
                std::cerr << RED << "CheckBeforeMove():  Vel command is out of range" << WHITE << std::endl;
                return -1;
            }
            //加速度检查
            if ( acceleration > joints_[ i ]->getMaxAcc( ) ||
                 acceleration < ( -1 ) * joints_[ i ]->getMaxAcc( ) )
            {
                std::cerr << RED << "CheckBeforeMove(): Acc command is out of range" << WHITE << std::endl;
                return -1;
            }
            //使能检查
            if ( joints_[ i ]->getDriveState( ) != DriveState::OperationEnabled )
            {
                std::cerr << RED << "CheckBeforeMove():  joints[" << i << "]"
                          << "is in OperationDisabled " << WHITE << std::endl;
                return -1;
            }
        }
        if ( time < 0 )
        {
            std::cerr << RED << "CheckBeforeMove():  time is less than 0 invalidly" << WHITE << std::endl;
            return -1;
        }

        if ( radius < 0 )
        {
            std::cerr << RED << "CheckBeforeMove():  radius is less than 0 invalidly" << WHITE << std::endl;
            return -1;
        }

        return 0;
        //**-------------------------------**//
    }

    int Robot::CheckBeforeMove( const Frame& pos, double speed, double acceleration,
                                double time, double radius )
    {
        //** 数据有效性检查  **//
        KDL::JntArray q_init( jnt_num_ );
        KDL::JntArray q_target( jnt_num_ );
        for ( int i = 0; i < jnt_num_; i++ ) q_init( i ) = pos_[ i ];
        //位置检查
        if ( kinematics_.CartToJnt( q_init, pos, q_target ) < 0 )
        {
            std::cerr << RED << " CheckBeforeMove(): Pos command is infeasible " << WHITE << std::endl;
            return -1;
        }

        for ( int i = 0; i < jnt_num_; i++ )
        {  //TODO 这里的速度、加速度目前只针对关节空间进行检查
            //速度检查
            if ( speed > joints_[ i ]->getMaxVel( ) ||
                 speed < ( -1 ) * joints_[ i ]->getMaxVel( ) )
            {
                std::cerr << RED << "CheckBeforeMove(): Vel command is out of range" << WHITE << std::endl;
                return -1;
            }
            //加速度检查
            if ( acceleration > joints_[ i ]->getMaxAcc( ) ||
                 acceleration < ( -1 ) * joints_[ i ]->getMaxAcc( ) )
            {
                std::cerr << RED << "CheckBeforeMove(): Acc command is out of range" << WHITE << std::endl;
                return -1;
            }
            //使能检查
            if ( joints_[ i ]->getDriveState( ) != DriveState::OperationEnabled )
            {
                std::cerr << RED << "CheckBeforeMove(): joints[" << i << "]"
                          << "is in OperationDisabled " << WHITE << std::endl;
                return -1;
            }
        }
        //总时间检查
        if ( time < 0 )
        {
            std::cerr << RED << "CheckBeforeMove(): time is less than 0 invalidly" << WHITE << std::endl;
            return -1;
        }
        //过渡半径检查
        if ( radius < 0 )
        {
            std::cerr << RED << "CheckBeforeMove(): radius is less than 0 invalidly" << WHITE << std::endl;
            return -1;
        }

        //**-------------------------------**//
        return 0;
    }

    void Robot::RunMoveJ( JntArray q, double speed, double acceleration, double time, double radius )
    {
        double dt       = 0.0;
        double max_time = 0.0;
        std::vector< std::shared_ptr< DoubleS > > interp( jnt_num_ );
        for ( auto& i : interp )
            i.reset( new DoubleS{ } );

        std::cout << "Joint Pos: \n"
                  << GREEN << q.data << WHITE << std::endl;
        for ( int i = 0; i < jnt_num_; ++i )
        {
            if ( q( i ) == pos_[ i ] )
            {
                std::cerr << RED << " Target pos[" << i << "]"
                          << "is same as  pos_[" << i << "]" << WHITE << std::endl;
                need_plan_[ i ] = false;
                continue;
            }
            need_plan_[ i ] = true;

            interp[ i ]->planDoubleSProfile( 0,          // t
                                             pos_[ i ],  // p0
                                             q( i ),     // pf
                                             0,          // v0
                                             0,          // vf
                                             speed, acceleration, max_jerk_[ i ] );

            if ( !interp[ i ]->isValidMovement( ) || !( interp[ i ]->getDuration( ) > 0 ) )
            {
                std::cerr << RED << "RunMoveJ():movej trajectory "
                          << "is infeasible " << WHITE << std::endl;
                is_running_motion = false;
                return;
            }
            max_time = max( max_time, interp[ i ]->getDuration( ) );
        }

        for ( int i = 0; i < jnt_num_; i++ )
        {
            if ( need_plan_[ i ] )
                interp[ i ]->JC_scaleToDuration( max_time );
        }

        while ( dt <= max_time )
        {
            for ( int i = 0; i < jnt_num_; ++i )
            {
                if ( !need_plan_[ i ] )
                    continue;

                pos_[ i ] = interp[ i ]->pos( dt );  //! 需要更新一下实时位置
                joints_[ i ]->setPosition( pos_[ i ] );
            }
            dt += 0.001;

            hw_interface_->waitForSignal( 0 );
        }

        is_running_motion = false;
    }

    void Robot::RunMoveL( const std::vector< KDL::JntArray >& traj )
    {
        std::cout << "No. of waypoints: " << traj.size( ) << std::endl;

        for ( const auto& waypoints : traj )
        {
            for ( int i = 0; i < jnt_num_; ++i )
            {
                pos_[ i ] = waypoints( i );
                joints_[ i ]->setPosition( pos_[ i ] );
            }

            hw_interface_->waitForSignal( 0 );
        }

        is_running_motion = false;  //TODO: added by Yangluo
    }

    void Robot::RunMultiMoveL( const std::vector< KDL::JntArray >& traj )
    {
        for ( const auto& waypoints : traj )
        {
            for ( int i = 0; i < jnt_num_; ++i )
            {
                pos_[ i ] = waypoints( i );
                joints_[ i ]->setPosition( pos_[ i ] );
            }

            hw_interface_->waitForSignal( 0 );
        }

        is_running_motion = false;
    }

    void Robot::RunDragging( const std::vector< KDL::JntArray >& traj )
    {
        int count{ 0 };
        int tick_count_{ 0 };

        {
            std::lock_guard< std::mutex > lck( tick_lock );
            tick_count_ = tick_count;
        }

        for ( const auto& waypoints : traj )
        {
            for ( int i = 0; i < jnt_num_; ++i )
            {
                pos_[ i ] = waypoints( i );
                joints_[ i ]->setPosition( pos_[ i ] );
            }

            if ( ++count == 100 )
            {
                count = 0;
                {
                    std::unique_lock< std::mutex > lck( tick_lock );
                    if ( tick_count_ != tick_count )
                        tick_count_ = tick_count;
                    else
                    {
                        std::cout << RED << "RunDragging():Some errors such as disconnecting from the controller" << WHITE << std::endl;
                        lck.unlock( );
                        StopMotion( );
                        is_running_motion = false;
                        return;
                    }
                }
            }

            hw_interface_->waitForSignal( 0 );
        }

        is_running_motion = false;  //TODO: added by Yangluo
    }

    //TODO 紧急停止
    void Robot::StopMotion( ) {}

}  // namespace rocos
