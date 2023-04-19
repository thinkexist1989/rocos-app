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
#include <drive.h>
#include <ethercat/hardware.h>
#include <ethercat/hardware_sim.h>
#include <fstream>
#include <iostream>
#include <robot.h>
#include <robot_service.h>
#include <string>
bool isRuning = true;

/// \brief 处理终端的Ctrl-C信号
/// \param signo
void signalHandler( int signo )
{
    if ( signo == SIGINT )
    {
        std::cout << "\033[1;31m"
                  << "[!!SIGNAL!!]"
                  << "INTERRUPT by CTRL-C"
                  << "\033[0m" << std::endl;

        isRuning = false;
        exit( 0 );
    }
}

#pragma region  //*测试9  上电保护程序

namespace rocos
{

    void Robot::test( )
    {
        //**变量初始化 **//
        std::string str{ "" };
        KDL::JntArray q_target( JC_helper::_joint_num );
        //**-------------------------------**//

#pragma region  //*电机使能检查

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

        PLOG_INFO << "当前环境是否安全,如果是,输入run开始执行程序";
        std::cin >> str;

        if ( str == std::string_view{ "run" } )
        {
            //这里演示的是一个关节
            std::array< double, 1 >same_torque_and_pos{1};//! 力传感器大小方向与关节轴向相同为1 ，不同为-1[必填参数]
            std::array< double, 1 > ref_pos{ 0 };//! 竖直位置弧度[必填参数]

            
            std::array< double, 1 > gravity_torque{ 0 }; //!相当于重力乘长度（G*L）
            std::array< double, 1 > zero_drift{ 0};//!零漂
            size_t time_torque{ 0 };  // 计数器

#if 1  //** 方向测试 **//

            // 力矩传感器方向确认

            //** 滤波器初始化 **//

            std::array< Iir::Butterworth::LowPass< 3 >, 1 > filter_array{ };
            const float samplingrate     = 200;                            // Hz
            const float cutoff_frequency = 5;                              // Hz
            filter_array[ 0 ].setup( 3, samplingrate, cutoff_frequency );  // NOTE： here order should replaced by a int number!

            //**-------------------------------**//

            time_torque = 0;
            while ( time_torque < 60000 )
            {
                if ( time_torque % 200 == 0 )
                {
                    PLOG_INFO << "sensor_torque[0] =" << filter_array[ 0 ].filter( joints_[ 0 ]->getLoadTorque( ) );
                    PLOG_INFO << "-------------------------------------";
                }
                else
                {
                    filter_array[ 0 ].filter( joints_[ 0 ]->getLoadTorque( ) );
                }
                hw_interface_->waitForSignal( 0 );
                time_torque++;
            }

#endif

#if 1  //** 连杆质心处的重力计算 **//

            {
                //** 滤波器初始化 **//
                std::array< Iir::Butterworth::LowPass< 3 >, 1 > filter_array{ };
                const float samplingrate     = 200;                            // Hz
                const float cutoff_frequency = 5;                              // Hz
                filter_array[ 0 ].setup( 3, samplingrate, cutoff_frequency );  // NOTE： here order should replaced by a int number!

                //**-------------------------------**//

                q_target( 0 ) = ref_pos[0];
                MoveJ( q_target, 0.3, 1, 0, 0, false );

                time_torque = 0;
                while ( time_torque < 5000 )
                {
                    zero_drift[ 0 ] = filter_array[ 0 ].filter( joints_[ 0 ]->getLoadTorque( ) );  // 竖立情况下测量，得到零漂
                    hw_interface_->waitForSignal( 0 );
                    time_torque++;
                }

                PLOG_INFO << "零漂[0]  = " << zero_drift[0];

                time_torque                    = 0;
                double gravity_torque_test_pos = 30 * KDL::deg2rad;
                q_target( 0 )                  = ref_pos[ 0 ] + gravity_torque_test_pos;
                MoveJ( q_target, 0.3, 1, 0, 0, false );

                while ( time_torque < 3000 )
                {
                    gravity_torque[0] = filter_array[0].filter( joints_[ 0 ]->getLoadTorque( ) ) - zero_drift[0];  // 水平情况下测量,得到负载重力
                    gravity_torque[0] = same_torque_and_pos[0]*gravity_torque[0] / KDL::sign( gravity_torque_test_pos );
                    hw_interface_->waitForSignal( 0 );
                    time_torque++;
                }

                PLOG_INFO << "负载引起额外力矩[0]  = " << gravity_torque[0] ;


                // 零漂_1 =    -6.01295;
                // 零漂_2 =    26.6135;
                //!  gravity_torque_1 =-6.61275
                //!  gravity_torque_2 =-6.77272
            }
#endif

#if 1  //** 重力补偿检查 **//
            {
                double angle                    = 0;
                double gravity_torque_component = 0;



                //** 滤波器初始化 **//
                std::array< Iir::Butterworth::LowPass< 3 >, 1 > filter_array{ };
                const float samplingrate     = 200;                            // Hz
                const float cutoff_frequency = 5;                              // Hz
                filter_array[ 0 ].setup( 3, samplingrate, cutoff_frequency );  // NOTE： here order should replaced by a int number!
                //**-------------------------------**//


                for ( int i = 0; i < 51; i = i + 10 )
                {
                    q_target( 0 ) = ref_pos[0] + i * KDL::deg2rad;
                    MoveJ( q_target, 0.3, 1, 0, 0, false );

                    time_torque = 0;
                    while ( time_torque < 1200 )
                    {
                        filter_array[0].filter( joints_[ 0 ]->getLoadTorque( ) );
                        hw_interface_->waitForSignal( 0 );
                        time_torque++;
                    }

                    angle                    = joints_[ 0 ]->getPosition( ) - ref_pos[ 0 ];
                    gravity_torque_component = gravity_torque[ 0 ] * sin( angle );
                    PLOG_DEBUG << "  误差[0] =  " << filter_array[ 0 ].filter( joints_[ 0 ]->getLoadTorque( ) ) - zero_drift[ 0 ] - gravity_torque_component * same_torque_and_pos[ 0 ];
                }


            }

#endif

#if 1  //** 阻抗实验 **//

            {
                std::cout.setf( ios::scientific );


                std::array< double, 1 > pos{ 0 };
                std::array< double, 1 > last_pos{ 0 };
                std::array< double, 1 > vel{ 0 };
                std::array< double, 1 > target_torque{ 0 };
                std::array< double, 1 > sensor_torque{ 0 };
                std::array< double, 1 > offset_torque{ 0 };
                std::array< double, 1 > last_offset_torque{ 0 };
                std::array< double, 1 > vel_torque{ 0 };
                std::array< double, 1 > command_torque{ 0 };
                std::array< double, 1 > gravity_torque_component{ 0 };

                //** 滤波器初始化 **//
                std::array< Iir::Butterworth::LowPass< 3 >, 1 > filter_array{ };
                const float samplingrate     = 200;                            // Hz
                const float cutoff_frequency = 5;                              // Hz
                filter_array[ 0 ].setup( 3, samplingrate, cutoff_frequency );  // NOTE： here order should replaced by a int number!
                //**-------------------------------**//

                q_target( 0 ) = ref_pos[ 0 ];
                MoveJ( q_target, 0.3, 1, 0, 0, false );

                joints_[ 0 ]->setMode( ModeOfOperation::CyclicSynchronousTorqueMode );

                time_torque = 0;
                // while ( time_torque < 300000 )
                while ( 1 )
                {
                    for ( int i = 0; i < 1; i++ )
                    {
                        pos[ i ] = joints_[ i ]->getPosition( ) - ref_pos[ i ];
                        vel[ i ] = ( pos[ i ] - last_pos[ i ] ) / 0.001;

                        gravity_torque_component[ i ] = gravity_torque[ i ] * sin( pos[ i ] );

                        target_torque[ i ] = -( pos[ i ] * 120 + vel[ i ] * 5 );

                        sensor_torque[ i ] = filter_array[ i ].filter( joints_[ i ]->getLoadTorque( ) ) - zero_drift[ i ] - gravity_torque_component[ i ] * same_torque_and_pos[ i ];

                        offset_torque[ i ] = target_torque[ i ] - ( -1 * same_torque_and_pos[ i ] * sensor_torque[ i ] );  //-1代表将力矩传感器受到的力转换为关节实际输出的力

                        vel_torque[ i ] = ( offset_torque[ i ] - last_offset_torque[ i ] );

                        command_torque[ i ] = target_torque[ i ] + ( 10 * offset_torque[ i ] + 6.5 * vel_torque[ i ] );

                        if ( std::abs( command_torque[ i ] ) > 400 )
                            command_torque[ i ] = KDL::sign( command_torque[ i ] ) * 400;

                        joints_[ i ]->setTorque( command_torque[ i ] );

                        if ( time_torque % 200 == 0 )
                        {
                            PLOG_INFO << "target_torque[" << i << "] =" << target_torque[ i ];
                            PLOG_WARNING << "sensor_torque[" << i << "] =" << sensor_torque[ i ];
                            PLOG_DEBUG << "command_torque [" << i << "] =" << command_torque[ i ];
                            PLOG_DEBUG << "--------------------------------------";
                        }

                        last_pos[ i ]           = pos[ i ];
                        last_offset_torque[ i ] = offset_torque[ i ];
                    }

                    time_torque += 1;
                    hw_interface_->waitForSignal( 0 );
                }

                joints_[ 0 ]->setTorque( 0 );
                hw_interface_->waitForSignal( 0 );
            }

#endif
        }
        else
        {
            PLOG_ERROR << "不安全环境,电机抱闸";
            setDisabled( );
        }

        PLOG_INFO << "全部测试结束,goodbye!";
    }
}  // namespace rocos

#pragma endregion

int main( int argc, char* argv[] )
{
    if ( signal( SIGINT, signalHandler ) == SIG_ERR )
    {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

    using namespace rocos;

    //** 等待主站清除共享内存,25后再启动APP **//
    std::cerr << "\033[32m"
              << "等待主站清除共享内存" << std::endl;
    std::this_thread::sleep_for( std::chrono::duration< double >( 5 ) );
    //**-------------------------------**//

    // boost::shared_ptr< HardwareInterface > hw = boost::make_shared< HardwareSim >( 2 );  // 仿真
    boost::shared_ptr< HardwareInterface > hw = boost::make_shared< Hardware >( );  // 真实机械臂

    //** 判断主站ECM是否启动成功 **//
    //! 如果主站25S以内启动，既先主站清除内存，在hw与主站建立连接，那下面程序可以成功判断Ready 三次
    //! 如果主站25S以外启动，既先初始化HW，再主站清除内存，那么在hw与主站就建立不了连接，那下面程序三次判断Not Ready
    //! 主站不启动和25S以外一样，建立不了连接
    //! 小结：主站必需25s以内启动，并且连续三次判断主站处于Ready状态，其余情况统统退出程序

    // int Ready_count{ 0 };
    // for ( int i{ 0 }; i < 3; i++ )
    // {
    //     hw->setHardwareState( HardwareInterface::HWState::UNKNOWN );

    //     std::this_thread::sleep_for( std::chrono::duration< double >( 0.1 ) );

    //     if ( hw->getHardwareState( ) == HardwareInterface::HWState::READY )
    //     {
    //         Ready_count++;
    //         std::cerr << "\033[32m"
    //                   << "Ready" << std::endl;
    //     }
    //     else
    //     {
    //         Ready_count = 0;
    //         std::cerr << "\033[1;31m"
    //                   << "Not Ready" << std::endl;
    //     }
    // }

    // if ( Ready_count == 3 )
    // {
    //     std::cerr << "\033[32m"
    //               << "HardWare准备好,开始程序" << std::endl;
    // }
    // else
    // {
    //     std::cerr << "\033[1;31m"
    //               << "HardWare未准备好,程序退出" << std::endl;
    //     exit( 0 );
    // }
    //**-------------------------------**//

    Robot robot( hw );

    auto robotService = RobotServiceImpl::getInstance( &robot );

    std::thread thread_test{ &rocos::Robot::test, &robot };

    //------------------------wait----------------------------------
    robotService->runServer( );

    thread_test.join( );
    return 0;
}
