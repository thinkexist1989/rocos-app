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

//#include <QtCore>
//#include <QProcess>
//#include <QString>
//#include <QDebug>
//#include <QFile>

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

#pragma region  //*测试 1
// void test( )
// {
// using namespace KDL;
// Frame f_p1;
// Frame f_p2;
// Frame f_p3;
// Frame f_p4;
// Kinematics kinematics_;
// kinematics_.initChain7Dofs( );
// KDL::JntArray q( 7 );

// q( 0 ) = 60 * M_PI / 180;
// q( 1 ) = -115 * M_PI / 180;
// q( 2 ) = 0 * M_PI / 180;
// q( 3 ) = -110 * M_PI / 180;
// q( 4 ) = -45 * M_PI / 180;
// q( 5 ) = 90 * M_PI / 180;
// q( 6 ) = 0 * M_PI / 180;

// kinematics_.JntToCart( q, f_p1 );

// {  //测试moveC

//     std::cout << "----------------test moveC start---------------" << std::endl;
//     f_p2 = f_p1 * Frame{ KDL::Rotation::RotX( 90 * M_PI / 180 ), Vector{ 0.0, -0.1, -0.1 } };
//     f_p3 = f_p1 * Frame{ KDL::Rotation::RotY( 90 * M_PI / 180 ), Vector{ 0.0, 0.0, -0.2 } };

//     for ( int i = 0; i < 6; i++ )
//     {
//         robot.test_set_pos( i, q( i ) );                                //?pos_资源竞争
//         std::this_thread::sleep_for( std::chrono::milliseconds( 3 ) );  //?pos_资源竞争
//     }
//     robot.MoveC( f_p2, f_p3, 0.05, 0.05, 0, 0, Robot::OrientationMode::FIXED, false );

//     //    for ( int i = 0; i < 6; i++ )
//     //    {
//     //        robot.test_set_pos( i, q( i ) );
//     //        std::this_thread::sleep_for( std::chrono::milliseconds( 3 ) );
//     //    }
//     //    robot.MoveC( f_p2, f_p3, 0.01, 0.01, 0, 0, Robot::OrientationMode::UNCONSTRAINED, false );
//     std::cout << "----------------test moveC end---------------" << std::endl;
// }
// {  //测试moveL

//     std::cout << "----------------test moveL start---------------" << std::endl;
//     std::cout << "f_p1 = \n"
//               << f_p1 << std::endl;
//     f_p2 = f_p1 * Frame{ KDL::Rotation::RotX( 90 * M_PI / 180 ), Vector{ 0.1, 0.0, 0 } };
//     for ( int i = 0; i < 7; i++ )
//     {
//         robot.test_set_pos( i, q( i ) );                                //?pos_资源竞争
//         std::this_thread::sleep_for( std::chrono::milliseconds( 3 ) );  //?pos_资源竞争
//     }
//     robot.MoveL( f_p1, 0.01, 0.01, 0, 0, false );
//     std::cout << "----------------test moveL end---------------" << std::endl;
// }

// {  //测试moveL(只旋转，不移动)

//     std::cout << "----------------test moveL start---------------" << std::endl;
//     f_p2 = f_p1 * Frame{ KDL::Rotation::RotX( 90 * M_PI / 180 ) };

//     for ( int i = 0; i < 7; i++ )
//     {
//         robot.test_set_pos( i, q( i ) );                                //?pos_资源竞争
//         std::this_thread::sleep_for( std::chrono::milliseconds( 3 ) );  //?pos_资源竞争
//     }
//     robot.MoveL( f_p2, 0.01, 0.01, 0, 0, false );
//     std::cout << "----------------test moveL end---------------" << std::endl;
// }

// {  //测试MultiMoveL，
//     std::cout << "----------------test MultiMoveL start---------------" << std::endl;

//     for ( int i = 0; i < 6; i++ )
//     {
//         robot.test_set_pos( i, q( i ) );
//         std::this_thread::sleep_for( std::chrono::milliseconds( 3 ) );
//     }
//     Frame f_p5;
//     Frame f_p6;
//     Frame f_p7;
//     Frame f_p8;
//     Frame f_p9;
//     Frame f_p11;

//     f_p1  = f_p1 * Frame{ KDL::Rotation::RotX( 90 * M_PI / 180 ), Vector{ 0.3, 0.0, 0 } };
//     f_p2  = f_p1 * Frame{ KDL::Rotation::RotY( 90 * M_PI / 180 ), Vector{ 0.0, -0.3, -0.0 } };
//     f_p3  = f_p2 * Frame{ KDL::Rotation::RotX( -90 * M_PI / 180 ), Vector{ 0.0, 0.0, -0.3 } };
//     f_p4  = f_p3 * Frame{ KDL::Rotation::RotZ( -90 * M_PI / 180 ), Vector{ 0.0, 0.0, 0.3 } };
//     f_p5  = f_p4 * Frame{ KDL::Rotation::RotZ( 45 * M_PI / 180 ), Vector{ 0.0, 0.0, -0.15 } };  //180度调头
//     f_p6  = f_p5 * Frame{ KDL::Rotation::RotZ( 45 * M_PI / 180 ), Vector{ 0.0, 0.0, -0.15 } };  //0度平行
//     f_p7  = f_p6 * Frame{ KDL::Rotation::RotZ( -90 * M_PI / 180 ), Vector{ 0.0, 0.0, 0.3 } };   //180度调头
//     f_p8  = f_p7 * Frame{ KDL::Rotation::RotZ( -90 * M_PI / 180 ) };                            //只旋转
//     f_p9  = f_p8 * Frame{ KDL::Rotation::RotZ( 90 * M_PI / 180 ) };                             //只旋转
//     f_p11 = f_p1 * Frame{ KDL::Rotation::RotY( 5 * M_PI / 180 ) };                              //只旋转

//     std::vector< KDL::Frame > points{ f_p1, f_p2, f_p3, f_p4, f_p5, f_p6, f_p7, f_p8, f_p9, f_p1, f_p11 };
//     std::vector< double > max_path_v{ 0.06, 0.04, 0.04, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.01 };
//     std::vector< double > max_path_a{ 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.01 };
//     std::vector< double > bound_dist{ 0.05, 0.0, 0.1, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0 };

//     robot.MultiMoveL( points, bound_dist, max_path_v, max_path_a, true );
//     std::cout << "----------------test MultiMoveL end---------------" << std::endl;
// }

// {  //测试dragging
//     using namespace rocos;

//     PLOG_DEBUG << "======连发测试开始===========";
//     //!要求每100ms以内至少调用一次，否则触发急停
//     for ( int i = 0; i < 100; i++ )
//     {
//         robot.Dragging( Robot::DraggingFlag::J0, Robot::DraggingDirection::POSITION, 1, 1 );
//         std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
//     }

//     PLOG_DEBUG << "======连发测试切换===========";  //调用紧急停止
//     sleep( 2 );

//     for ( int i = 0; i < 100; i++ )  //第二套动作
//     {
//         robot.Dragging( Robot::DraggingFlag::J1, Robot::DraggingDirection::POSITION, 1, 1 );
//         std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
//     }

//     PLOG_DEBUG << "======连发测试结束===========";
//     sleep( 5 );
//     PLOG_DEBUG << "急停测试开始";
//     sleep( 7 );
//     robot.Dragging( Robot::DraggingFlag::J0, Robot::DraggingDirection::NEGATIVE, 1, 1 );  //第三套动作
//     sleep( 7 );
//     PLOG_DEBUG << "急停测试结束";
// }

// }

#pragma endregion

#pragma region  //*测试2
// namespace rocos
// {
//     void Robot::test( )
//     {
//         // KDL::JntArray q_init( 7 );
//         KDL::JntArray q_target( 7 );
//         KDL::Frame f_p1;
//         KDL::Frame f_p2;
//         KDL::Frame f_p3;
//         KDL::Frame f_p4;
//         KDL::Frame f_p5;
//         KDL::Frame f_p6;

//         KDL::Frame f_c_p1;
//         KDL::Frame f_c_p2;
//         KDL::Frame f_c_p3;
//         KDL::Frame f_c_p0;
//         std::string str{ "disable" };

//         //** 电机使能检查 **//
//         PLOG_INFO << "电机使能检查";

//         for ( int i{ 0 }; i < jnt_num_; i++ )
//         {
//             if ( joints_[ i ]->getDriveState( ) != DriveState::OperationEnabled )
//             {
//                 for ( int j{ 0 }; j < 1; j++ )
//                 {
//                     PLOG_ERROR << "电机[" << i << "] 未使能，确定主站已初始化完成了？,输入y确认";
//                     std::cin >> str;
//                     if ( str != std::string_view{ "y" } )
//                     {
//                         PLOG_ERROR << "未输入yes, 判断主站 {未} 初始化完成,程序关闭";
//                         exit( 0 );
//                     }
//                 }
//             }
//         }

//         setEnabled( );
//         //**-------------------------------**//

//         PLOG_INFO << "当前环境是否安全，如果是，输入run开始执行程序";
//         std::cin >> str;

//         for ( int i = 0; i < 3; i++ )
//         {
//             if ( str == std::string_view{ "run" } )
//             {
//                 q_target( 0 ) = 0 * M_PI / 180;
//                 q_target( 1 ) = -45 * M_PI / 180;
//                 q_target( 2 ) = 0 * M_PI / 180;
//                 q_target( 3 ) = -90 * M_PI / 180;
//                 q_target( 4 ) = 0 * M_PI / 180;
//                 q_target( 5 ) = 45 * M_PI / 180;
//                 q_target( 6 ) = 0 * M_PI / 180;

//                 MoveJ( q_target, 0.4, 0.2, 0, 0, false );

//                 kinematics_.JntToCart( q_target, f_p1 );
//                 f_p1 = f_p1 * KDL::Frame{ KDL::Vector{ 0.0, -0.15, 0.0 } };
//                 f_p2 = f_p1 * KDL::Frame{ KDL::Vector{ -0.3, 0.0, 0.0 } };
//                 f_p3 = f_p2 * KDL::Frame{ KDL::Vector{ 0.0, 0.3, 0.0 } };
//                 f_p4 = f_p3 * KDL::Frame{ KDL::Vector{ 0.3, 0.0, 0.0 } };
//                 f_p5 = f_p4 * KDL::Frame{ KDL::Vector{ 0.0, -0.15, 0.0 } };

//                 std::vector< KDL::Frame > points{ f_p1, f_p2, f_p3, f_p4, f_p5 };
//                 std::vector< double > max_path_v{ 0.30, 0.30, 0.30, 0.30, 0.30 };
//                 std::vector< double > max_path_a{ 0.2, 0.2, 0.2, 0.2, 0.2 };
//                 std::vector< double > bound_dist{ 0.05, 0.05, 0.05, 0.05, 0.0 };

//                 MultiMoveL( points, bound_dist, max_path_v, max_path_a, false );

//                 f_p6 = f_p5 * KDL::Frame{ KDL::Vector{ -0.3, 0.3, 0.0 } };
//                 MoveL( f_p6, 0.2, 0.2, 0, 0, false );

//                 for ( int i = 0; i < jnt_num_; i++ )
//                     q_target( i ) = 0;
//                 MoveJ( q_target, 0.2, 0.2, 0, 0, false );

//                 q_target( 0 ) = 0 * M_PI / 180;
//                 q_target( 1 ) = 45 * M_PI / 180;
//                 q_target( 2 ) = 0 * M_PI / 180;
//                 q_target( 3 ) = 90 * M_PI / 180;
//                 q_target( 4 ) = 0 * M_PI / 180;
//                 q_target( 5 ) = 45 * M_PI / 180;
//                 q_target( 6 ) = 0 * M_PI / 180;

//                 MoveJ( q_target, 0.2, 0.2, 0, 0, false );

//                 kinematics_.JntToCart( q_target, f_c_p0 );

//                 f_c_p1 = f_c_p0 * KDL::Frame{ KDL::Vector{ -0.1, -0.1, 0 } };
//                 f_c_p2 = f_c_p0 * KDL::Frame{ KDL::Vector{ -0.2, 0.0, 0 } };
//                 f_c_p3 = f_c_p0 * KDL::Frame{ KDL::Vector{ -0.1, 0.1, 0 } };

//                 MoveC( f_c_p1, f_c_p2, 0.2, 0.2, 0, 0, Robot::OrientationMode::FIXED, false );
//                 MoveC( f_c_p3, f_c_p0, 0.2, 0.2, 0, 0, Robot::OrientationMode::FIXED, false );

//                 for ( int i = 0; i < jnt_num_; i++ )
//                     q_target( i ) = 0;
//                 MoveJ( q_target, 0.2, 0.2, 0, 0, false );
//             }
//         }
//         PLOG_INFO << "全部测试结束，goodbye！";
//     }
// }  // namespace rocos

#pragma endregion

#pragma region  //*测试3
// namespace rocos
// {
//     void Robot::test( )
//     {
//         KDL::JntArray q_target( 7 );

//         setEnabled( );

//         PLOG_DEBUG << "执行 moveJ,输入run执行";

//         std::cin >> str;

//         while ( str != std::string{ "exit" } )
//         {

//             std::cout << "----------------test MultiMoveL start---------------" << std::endl;

//             if ( str == std::string{ "1" } )
//             {
//                 q_target( 0 ) = 32.139 * M_PI / 180;
//                 q_target( 1 ) = -56.752 * M_PI / 180;
//                 q_target( 2 ) = -49.227 * M_PI / 180;
//                 q_target( 3 ) = -90 * M_PI / 180;
//                 q_target( 4 ) = -0.336 * M_PI / 180;
//                 q_target( 5 ) = 45.085 * M_PI / 180;
//                 q_target( 6 ) = 39.535 * M_PI / 180;

//                 MoveJ( q_target, 0.4, 0.2, 0, 0, false );
//             }

//             else if ( str == std::string{ "2" } )
//             {
//                 q_target( 0 ) = 0 * M_PI / 180;
//                 q_target( 1 ) = -45 * M_PI / 180;
//                 q_target( 2 ) = 0 * M_PI / 180;
//                 q_target( 3 ) = -90 * M_PI / 180;
//                 q_target( 4 ) = 0 * M_PI / 180;
//                 q_target( 5 ) = -45 * M_PI / 180;
//                 q_target( 6 ) = 0 * M_PI / 180;

//                 MoveJ( q_target, 0.4, 0.2, 0, 0, false );
//             }

//             else if ( str == std::string{ "home" } )
//             {
//                 for ( int i = 0; i < 7; i++ )
//                     q_target( i ) = 0;
//                 MoveJ( q_target, 0.4, 0.2, 0, 0, false );
//             }

//             std::cout << "----------------test MultiMoveL end---------------" << std::endl;
//             std::cin >> str;
//         }

//     }
// }  // namespace rocos

#pragma endregion

#pragma region  //* 测试4
/**
 * @brief 测试4：机械臂标定的50个路点
 *
 */
// namespace rocos
// {
//     void Robot::test( )
//     {
//         // KDL::JntArray q_init( 7 );
//         KDL::JntArray q_target( 7 );
//         std::ifstream dat{ "/home/think/rocos-app/file_measpoint0.dat" };
//         std::ofstream invalid_Pos{ "/home/think/rocos-app/invalid_pos.dat" };

//         std::vector< KDL::JntArray > q_vector;
//         std::string str{ "" };
//         int count = 0;

//         while ( dat >> str )
//         {
//             q_target( count ) = stod( str ) * M_PI / 180;
//             count++;
//             if ( count == 7 )
//             {
//                 count = 0;
//                 q_vector.push_back( q_target );
//             }
//         }
//         for ( int i = 0; i < q_vector.size( ); i++ )
//         {
//             for ( int j = 0; j < 7; j++ )
//                 std::cout << " " << q_vector[ i ]( j );

//             std::cout << std::endl;
//         }

//         //** 电机使能检查 **//
//         PLOG_INFO << "电机使能检查";

//         for ( int i{ 0 }; i < jnt_num_; i++ )
//         {
//             if ( joints_[ i ]->getDriveState( ) != DriveState::OperationEnabled )
//             {
//                 for ( int j{ 0 }; j < 1; j++ )
//                 {
//                     PLOG_ERROR << "电机[" << i << "] 未使能，确定主站已初始化完成了？,输入y确认";
//                     std::cin >> str;
//                     if ( str != std::string_view{ "y" } )
//                     {
//                         PLOG_ERROR << "未输入yes, 判断主站 {未} 初始化完成,程序关闭";
//                         exit( 0 );
//                     }
//                 }
//             }
//         }

//         setEnabled( );
//         //**-------------------------------**//

//         PLOG_INFO << "当前环境是否安全,如果是,输入run开始执行程序";
//         std::cin >> str;

//         if ( str == std::string_view{ "run" } )
//         {
//             for ( int i = 0; i < q_vector.size( ); i++ )
//             {
//                 for ( int j = 0; j < 7; j++ )
//                 {
//                     q_target( j ) = q_vector[ i ]( j );
//                 }

//                 if ( MoveJ( q_target, 0.6, 0.3, 0, 0, false ) < 0 )
//                 {
//                     for ( int k = 0; k < 7; k++ )
//                         invalid_Pos << q_target( k ) << "\t";
//                 }
//                 invalid_Pos  << "\n";
//                 std::cin>>str;
//             }
//             invalid_Pos.close( );
//         }
//         PLOG_INFO << "全部测试结束，goodbye！";
//     }
// }  // namespace rocos

#pragma endregion

#pragma region  //* 测试5 
/**
 * @brief 测试5：导纳测试
 *
 */
// namespace rocos
// {
//     void Robot::test( )
//     {

//         std::string str{ "" };

//         //** 电机使能检查 **//
//         PLOG_INFO << "电机使能检查";

//         for ( int i{ 0 }; i < jnt_num_; i++ )
//         {
//             if ( joints_[ i ]->getDriveState( ) != DriveState::OperationEnabled )
//             {
//                 for ( int j{ 0 }; j < 1; j++ )
//                 {
//                     PLOG_ERROR << "电机[" << i << "] 未使能，确定主站已初始化完成了？,输入y确认";
//                     std::cin >> str;
//                     if ( str != std::string_view{ "y" } )
//                     {
//                         PLOG_ERROR << "未输入yes, 判断主站 {未} 初始化完成,程序关闭";
//                         exit( 0 );
//                     }
//                 }
//             }
//         }

//         setEnabled( );
//         //**-------------------------------**//

//         PLOG_INFO << "当前环境是否安全,如果是,输入run开始执行程序";
//         std::cin >> str;

//         if ( str == std::string_view{ "run" } )
//         {
//             KDL::JntArray q_target( 7 );

//             // q_target( 0 ) = (-19.378) * M_PI / 180;
//             // q_target( 1 ) = (21.62)* M_PI / 180;
//             // q_target( 2 ) = 0 * M_PI / 180;
//             // q_target( 3 ) = (-96.381) * M_PI / 180;
//             // q_target( 4 ) = -0 * M_PI / 180;
//             // q_target( 5 ) = (-42.71) * M_PI / 180;
//             // q_target( 6 ) = (-0.011) * M_PI / 180;

//             q_target( 0 ) = (-18.037) * M_PI / 180;
//             q_target( 1 ) = (-2.096)* M_PI / 180;
//             q_target( 2 ) = (-7.843) * M_PI / 180;
//             q_target( 3 ) = (-78.336) * M_PI / 180;
//             q_target( 4 ) = (9.855) * M_PI / 180;
//             q_target( 5 ) = (-40.732) * M_PI / 180;
//             q_target( 6 ) = (-10.692) * M_PI / 180;

//             MoveJ( q_target, 0.1, 0.5, 0, 0, false );

//             //** 导纳调试 **//
//             // admittance_teaching();
//             sleep( 2 );
//             KDL::Frame frame_init = flange_;
//             admittance_link( frame_init * KDL::Frame{ KDL::Vector{ 0, 0, 0.008 } }, 0.001, 0.1 );

//             // for(int i{0};i<3;i++)
//             // {
//             //     MoveL( frame_init * KDL::Frame{ KDL::Vector{ -0.1, 0,0 } }, 0.01, 1, 0, 0, false ,1);
//             //     MoveL( frame_init, 0.01, 1, 0, 0, false );
//             // }

//             // q_target( 0 ) = 5 * M_PI / 180;
//             // q_target( 1 ) = -40 * M_PI / 180;
//             // q_target( 2 ) = 5* M_PI / 180;
//             // q_target( 3 ) = -95* M_PI / 180;
//             // q_target( 4 ) = -5 * M_PI / 180;
//             // q_target( 5 ) = 50 * M_PI / 180;
//             // q_target( 6 ) = -5 * M_PI / 180;

//             // MoveJ( q_target, 0.001, 0.001, 0, 0, false );

//             //**-------------------------------**//

//             //** 6维力调试 **//
//             // JC_helper::ft_sensor my_sensor{ };
//             // my_sensor.init( flange_ );

//             // for ( int i{ 0 }; i < 5*60*1000; i++ )
//             // {
//             //     my_sensor.debug( flange_ );
//             // }

//             //**-------------------------------**//

//             //** 速度采集**//
//             // std::vector< double >
//             //     joints_last_vel( 7, 0 );
//             // std::vector< double >
//             //     joints_vel( 7, 0 );
//             // for ( int count{ 0 }; count < 5 * 60 ; count++ )
//             // {
//             //     for ( int i = 0; i < 7; ++i )
//             //     {
//             //         joints_last_vel[ i ] = joints_vel[ i ];
//             //         joints_vel[ i ]      = joints_[ i ]->getVelocity( );
//             //         PLOG_DEBUG << "joints_vel[ " << i << " ] = " << joints_vel[ i ];
//             //         PLOG_DEBUG << "joints_acc[ " << i << " ] = " << ( joints_vel[ i ] - joints_last_vel[ i ] ) / 1;
//             //         PLOG_DEBUG;
//             //     }
//             //     std::this_thread::sleep_for( std::chrono::milliseconds( 1000 ) );
//             // }
//             //**-------------------------------**//
//         }
//         PLOG_INFO << "全部测试结束,goodbye!";
//     }
// }  // namespace rocos

#pragma endregion

#pragma region  //* 测试6
/**
 * @brief 测试6 ：全部按键、提拉杆、旋钮测试
 *
 */
// namespace rocos
// {

// void split( const std::string& str,
//             std::vector< std::string >& tokens,
//             const std::string delim = " " )
// {
//     tokens.clear( );//!注意清除上次结果

//     auto start    = str.find_first_not_of( delim, 0 );  // 分割到的字符串的第一个字符
//     auto position = str.find_first_of( delim, start );  // 分隔符的位置
//     while ( position != std::string::npos || start != std::string::npos )
//     {
//         // [start, position) 为分割下来的字符串
//         tokens.emplace_back( std::move( str.substr( start, position - start ) ) );
//         start    = str.find_first_not_of( delim, position );
//         position = str.find_first_of( delim, start );
//     }
// }


//     void Robot::test( )
//     {

//         std::string str{ "" };

//         //** 电机使能检查 **//
//         PLOG_INFO << "电机使能检查";

//         for ( int i{ 0 }; i < jnt_num_; i++ )
//         {
//             if ( joints_[ i ]->getDriveState( ) != DriveState::OperationEnabled )
//             {
//                 for ( int j{ 0 }; j < 1; j++ )
//                 {
//                     PLOG_ERROR << "电机[" << i << "] 未使能，确定主站已初始化完成了？,输入y确认";
//                     std::cin >> str;
//                     if ( str != std::string_view{ "y" } )
//                     {
//                         PLOG_ERROR << "未输入yes, 判断主站 {未} 初始化完成,程序关闭";
//                         exit( 0 );
//                     }
//                 }
//             }
//         }

//         setEnabled( );
//         //**-------------------------------**//

//         PLOG_INFO << "当前环境是否安全,如果是,输入run开始执行程序";
//         std::cin >> str;

//         if ( str == std::string_view{ "run" } )
//         {
//             KDL::JntArray q_target( 7 );

//             q_target( 0 ) = 0 * M_PI / 180;
//             q_target( 1 ) = -45 * M_PI / 180;
//             q_target( 2 ) = 0 * M_PI / 180;
//             q_target( 3 ) = -90 * M_PI / 180;
//             q_target( 4 ) = -0 * M_PI / 180;
//             q_target( 5 ) = 45 * M_PI / 180;
//             q_target( 6 ) = 0 * M_PI / 180;

//             MoveJ( q_target, 0.4, 1.5, 0, 0, false );

//             std::ifstream button_knob_csv{ };
//             button_knob_csv.open("/home/think/rocos-app/debug/button_konb_pos.csv");
//             std::vector< std::string > tokens;

//             while ( button_knob_csv >> str )  //遇见/t,/n,空格停下
//             {
//                 split( str, tokens, "," );

//                 KDL::Frame frame_target = KDL::Frame{ KDL::Rotation::RotY(M_PI/2) ,KDL::Vector{ std::stod( tokens[ 0 ] )/1000, std::stod( tokens[ 1 ] )/1000, std::stod( tokens[ 2 ] )/1000 }};

//                 MoveJ_IK( frame_target, 0.4, 1.5, 0, 0, false );
//                 sleep(2);
//             }

//             button_knob_csv.close();
//         }
//         PLOG_INFO << "全部测试结束,goodbye!";
   
//     }
// }  // namespace rocos

#pragma endregion

#pragma region //*测试7 演示测试

namespace rocos
{

void split( const std::string& str,
            std::vector< std::string >& tokens,
            const std::string delim = " " )
{
    tokens.clear( );//!注意清除上次结果

    auto start    = str.find_first_not_of( delim, 0 );  // 分割到的字符串的第一个字符
    auto position = str.find_first_of( delim, start );  // 分隔符的位置
    while ( position != std::string::npos || start != std::string::npos )
    {
        // [start, position) 为分割下来的字符串
        tokens.emplace_back( std::move(str.substr( start, position - start )  ) );
        start    = str.find_first_not_of( delim, position );
        position = str.find_first_of( delim, start );
    }
}

void Robot::pos_bag(bool* flag_turnoff )
{
    std::ofstream out_joint_csv{ };
    std::string str;
    out_joint_csv.open( "/home/think/rocos-app/debug/joints.csv" );
    while ( !(*flag_turnoff) && str.compare( "exit" ) )
    {
        PLOG_DEBUG << " 输入“rem”并回车,记录当前的关节值和笛卡尔位置;输入“exit”退出记录";
        std::cin >> str;
        if ( str.compare( "rem" ) == 0 )
        {
            for ( int i{ 0 }; i < 7; i++ )
                out_joint_csv << pos_[ i ] << ",";

            for ( int i{ 0 }; i < 3; i++ )
                out_joint_csv << flange_.p[ i ] << ",";

            double rpy[ 3 ];
            flange_.M.GetRPY( rpy[ 0 ], rpy[ 1 ], rpy[ 2 ] );
            out_joint_csv << rpy[ 0 ] << ",";
            out_joint_csv << rpy[ 1 ] << ",";
            out_joint_csv << rpy[ 2 ] << "\n";
        }
    }
    out_joint_csv.close();
    PLOG_DEBUG<<"采集完毕";
}

    void Robot::test( )
    {

        std::string str{ "" };

        //** 电机使能检查 **//
        PLOG_INFO << "电机使能检查";

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
                        PLOG_ERROR << "未输入yes, 判断主站 {未} 初始化完成,程序关闭";
                        exit( 0 );
                    }
                }
            }
        }

        setEnabled( );
        //**-------------------------------**//

        PLOG_INFO << "当前环境是否安全,如果是,输入run开始执行程序";
        std::cin >> str;

        if ( str == std::string_view{ "run" } )
        {
            //** 开启记录 **//
            bool flag_turnoff{ false };
            std::thread thread_pos_bag{ &rocos::Robot::pos_bag, this, &flag_turnoff };

            //**-------------------------------**//

#pragma region  //* 第一个按键

            KDL::JntArray q_target( 7 );
            std::ifstream button_knob_csv{ };  //真实飞机舱里三种按键测试
            std::vector< std::string > tokens;

            // // //** 读取文件的全部关节值，并执行 **//
            // button_knob_csv.open( "/home/think/rocos-app/debug/demo_1.csv" );

            // while ( button_knob_csv >> str )  //遇见/t,/n,空格停下
            // {
            //     split( str, tokens, "," );

            //     for ( int i{ 0 }; i < 7; i++ )
            //     {
            //         q_target( i ) = ( std::stod( tokens[ i ] ) );
            //     }

            //     MoveJ( q_target, 0.2, 0.5, 0, 0, false );
            // }

            // button_knob_csv.close( );

            // // //**-------------------------------**//

            // // //** 导纳调试 **//
            // KDL::Frame frame_init = flange_;
            // // admittance_link( frame_init * KDL::Frame{ KDL::Vector{ 0, 0, 0.008 } }, 0.0022, 0.1 );
            // MoveL(frame_init * KDL::Frame{ KDL::Vector{ 0, 0, 0.0085 } }, 0.0022, 0.1 , 0, 0, false );
            // // // admittance_teaching();
            // // //**-------------------------------**//

            // //** 导纳调试 **//
            // frame_init = flange_;
            // // admittance_link( frame_init * KDL::Frame{ KDL::Vector{ 0, 0, -0.008 } }, 0.0022, 0.1 );
            // MoveL( frame_init * KDL::Frame{ KDL::Vector{ 0, 0, -0.0085 } }, 0.0022, 0.1, 0, 0, false );

            // //**-------------------------------**//

            // //** 读取文件的全部关节值，并执行 **//
            // button_knob_csv.open( "/home/think/rocos-app/debug/demo_1_inverset.csv" );

            // while ( button_knob_csv >> str )  //遇见/t,/n,空格停下
            // {
            //     split( str, tokens, "," );

            //     for ( int i{ 0 }; i < 7; i++ )
            //     {
            //         q_target( i ) = ( std::stod( tokens[ i ] ) );
            //     }

            //     MoveJ( q_target, 0.2, 0.5, 0, 0, false );
            // }

            // button_knob_csv.close( );

            // //**-------------------------------**//

#pragma endregion

#pragma region  //*第二个按键


         // //** 读取文件的全部关节值，并执行 **//
            button_knob_csv.open( "/home/think/rocos-app/debug/demo_2_inverst.csv" );

            while ( button_knob_csv >> str )  //遇见/t,/n,空格停下
            {
                split( str, tokens, "," );

                for ( int i{ 0 }; i < 7; i++ )
                {
                    q_target( i ) = ( std::stod( tokens[ i ] ) );
                }

                MoveJ( q_target, 0.1, 0.5, 0, 0, false );
            }

            button_knob_csv.close( );

            // //**-------------------------------**//



            // //** 导纳调试 **//
            // KDL::Frame frame_init = flange_;
            // MoveL(frame_init * KDL::Frame{ KDL::Vector{ 0, 0, 0.0085 } }, 0.0022, 0.1 , 0, 0, false );
            // admittance_teaching( );
            // //**-------------------------------**//




#pragma endregion

            flag_turnoff = false;
            thread_pos_bag.join( );
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
    // boost::shared_ptr< HardwareInterface > hw = boost::make_shared< HardwareSim >( 7 );  // 仿真
    boost::shared_ptr< HardwareInterface > hw = boost::make_shared< Hardware >( );  //真实机械臂

    Robot robot( hw );

    auto robotService = RobotServiceImpl::getInstance( &robot );

    std::thread thread_test{ &rocos::Robot::test, &robot };

    //------------------------wait----------------------------------
    robotService->runServer( );
    thread_test.join( );
    return 0;
}
