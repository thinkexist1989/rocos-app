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


#pragma region  //*测试9  完整上电保护程序

namespace rocos
{
    /**
     * @brief 字符串切割函数
     *
     * @param str 待切割字符串
     * @param tokens 结果存储
     * @param delim 切割符
     */
    void split( const std::string& str,
                std::vector< std::string >& tokens,
                const std::string delim = " " )
    {
        tokens.clear( );  //!注意清除上次结果

        auto start    = str.find_first_not_of( delim, 0 );  // 分割到的字符串的第一个字符
        auto position = str.find_first_of( delim, start );  // 分隔符的位置
        while ( position != std::string::npos || start != std::string::npos )
        {
            // [start, position) 为分割下来的字符串
            tokens.emplace_back( std::move( str.substr( start, position - start ) ) );
            start    = str.find_first_not_of( delim, position );
            position = str.find_first_of( delim, start );
        }
    }

    void Robot::test( )
    {
        //**变量初始化 **//
        std::string str{ "" };
        std::ifstream csv_null_motion;

        char tem[ 2048 ];
        std::vector< std::string > tokens;
        std::vector< KDL::JntArray > servo_data;
        KDL::JntArray joints( _joint_num );
        KDL::JntArray last_joints( _joint_num );

        int row_index = 1;

        //**-------------------------------**//

        //** 程序初始化 **//
        // csv_null_motion.open( "./debug/7dof_null_motion_data.csv" );
        // if ( !csv_null_motion.is_open( ) )
        // {
        //     PLOG_ERROR << "7dof_null_motion_data.csv 文件打开失败";
        //     return;
        // }

        // last_joints( 0 ) = 0.707233696463318;
        // last_joints( 1 ) = 1.92101781439896;
        // last_joints( 2 ) = -1.96069246176939;
        // last_joints( 3 ) = 1.5707963267949;
        // last_joints( 4 ) = -0.00800697316204158;
        // last_joints( 5 ) = -0.795252940741234;
        // last_joints( 6 ) = 2.09439372345304;

        // while ( csv_null_motion.getline( tem, 2048 ) )
        // {
        //     if ( strcmp( tem, "" ) == 0 )
        //     {
        //         row_index++;
        //         continue;
        //     }  //排除空字符串

        //     split( tem, tokens, "," );

        //     for ( int i{ 0 }; i < _joint_num; i++ )
        //     {
        //         joints( i ) = std::stod( tokens[ i ] );
        //         if ( abs( joints( i ) - last_joints( i ) ) > 0.001 )
        //         {
        //             PLOG_ERROR<<"行数："<<row_index;
        //             PLOG_ERROR << "joint [" << i << "] ::"
        //                        << "速度过快 ,命令速度偏差为: " << abs( joints( i ) - last_joints( i ) );
        //             return;
        //         }
        //         else
        //             last_joints( i ) = joints( i );
        //     }

        //     servo_data.push_back( joints );
        // }

        auto t_start = std::chrono::high_resolution_clock::now( ); //记录程序启动时间

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


        while ( isRuning )
        {
            str.clear();

            PLOG_INFO << "当前环境是否安全,如果是,输入run开始执行程序";
            std::cin >> str;

            auto t_stop     = std::chrono::high_resolution_clock::now( );
            auto t_duration = std::chrono::duration< double >( t_stop - t_start );
            PLOG_DEBUG << "当前已经运行了: " << t_duration.count( ) / 60 << "分钟";

            if(t_duration.count( ) / 60   >45)
            {
                PLOG_ERROR<< "运行时间已超过45分钟,程序关闭";
                exit(0);
            }


            if ( str == std::string_view{ "run" } )
            {
            
                using namespace KDL;
                KDL::JntArray q_target( _joint_num );


                //** 正方形+ 直线+moveJ+圆弧 **//

                // KDL::Frame f_p0;
                // KDL::Frame f_p1;
                // KDL::Frame f_p2;
                // KDL::Frame f_p3;
                // KDL::Frame f_p4;
                // KDL::Frame f_p5;
                // KDL::Frame f_p6;

                // for ( int i = 0; i < 7; i++ )
                //     joints_[ i ]->setMode( ModeOfOperation::CyclicSynchronousPositionMode );

                // q_target( 0 ) = 0 * M_PI / 180;
                // q_target( 1 ) = -45 * M_PI / 180;
                // q_target( 2 ) = 0 * M_PI / 180;
                // q_target( 3 ) = -90 * M_PI / 180;
                // q_target( 4 ) = 0 * M_PI / 180;
                // q_target( 5 ) = 45 * M_PI / 180;
                // q_target( 6 ) = 0 * M_PI / 180;

                // MoveJ( q_target, 0.3, 1, 0, 0, false );

                // kinematics_.JntToCart( q_target, f_p0 );


                // f_p1 = f_p0 * KDL::Frame{ KDL::Vector{ 0.0, -0.15, 0.0 } };
                // MoveL( f_p1, 0.07, 0.2, 0, 0, false );



                // KDL::Frame f_c_p1 = f_p0 * KDL::Frame{ KDL::Vector{ -0.15, 0.0, 0.0 } };
                // KDL::Frame f_c_p2 = f_p0 * KDL::Frame{ KDL::Vector{ 0.0, 0.15, 0.0 } };
                // MoveC( f_c_p1, f_c_p2, 0.04, 0.3, 0, 0, Robot::OrientationMode::UNCONSTRAINED, false );

                // MoveC( f_p0, M_PI, 2, 0.04, 0.3, 0, 0, Robot::OrientationMode::UNCONSTRAINED, false );

                // PLOG_INFO << "开始速度模式";
                // std::cin >> str;

                // for ( int i = 0; i < 7; i++ )
                //     joints_[ i ]->setMode( ModeOfOperation::CyclicSynchronousVelocityMode );

                // MoveJ( q_target, 0.1, 1, 0, 0, false );

                // MoveL( f_p1, 0.07, 0.2, 0, 0, false );

                // MoveC( f_c_p1, f_c_p2, 0.04, 0.3, 0, 0, Robot::OrientationMode::UNCONSTRAINED, false );

                // MoveC( f_p0, M_PI, 2, 0.04, 0.3, 0, 0, Robot::OrientationMode::UNCONSTRAINED, false );

                //**-------------------------------**//

                //** 回到起始位置 **//

                // for ( int i = 0; i < jnt_num_; i++ )
                //     q_target( i ) = 0;
                // MoveJ( q_target, 0.4, 0.6, 0, 0, false );

                //**-------------------------------**//

                q_target( 0 ) = 0 * M_PI / 180;
                q_target( 1 ) = -45 * M_PI / 180;
                q_target( 2 ) = 0 * M_PI / 180;
                q_target( 3 ) = -90 * M_PI / 180;
                q_target( 4 ) = 0 * M_PI / 180;
                q_target( 5 ) = 45 * M_PI / 180;
                q_target( 6 ) = 0 * M_PI / 180;

                for ( int i = 0; i < 7; i++ )
                    joints_[ i ]->setMode( ModeOfOperation::CyclicSynchronousPositionMode );

                MoveJ( q_target, 0.3, 1, 0, 0, false );

                PLOG_INFO << "切换速度模式";
                // std::cin >> str;

                for ( int i = 0; i < 7; i++ )
                    joints_[ i ]->setMode( ModeOfOperation::CyclicSynchronousVelocityMode );

                q_target( 5 ) = 0;

                MoveJ( q_target, 0.3, 1, 0, 0, false );

                PLOG_INFO << "切换位置模式，危险";
                // std::cin >> str;

                for ( int i = 0; i < 7; i++ )
                    joints_[ i ]->setMode( ModeOfOperation::CyclicSynchronousPositionMode );

                q_target( 5 ) = 45 * M_PI / 180;
                MoveJ( q_target, 0.3, 1, 0, 0, false );

                for ( int i = 0; i < 7; i++ )
                    joints_[ i ]->setMode( ModeOfOperation::CyclicSynchronousVelocityMode );

                for ( int i = 0; i < jnt_num_; i++ )
                    q_target( i ) = 0;
                MoveJ( q_target, 0.4, 0.6, 0, 0, false );
            }
            else
            {
                PLOG_ERROR << "不安全环境,电机抱闸";
                setDisabled( );
                return ;
            }
     
     
     
        }

        PLOG_INFO << "全部测试结束,goodbye!";
    }
}  // namespace rocos

#pragma endregion





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
    std::this_thread::sleep_for( std::chrono::duration< double >( 15 ) );
    //**-------------------------------**//

    boost::shared_ptr< HardwareInterface > hw = boost::make_shared< HardwareSim >( _joint_num );  // 仿真
    // boost::shared_ptr< HardwareInterface > hw = boost::make_shared< Hardware >( );  //真实机械臂

    //** 判断主站ECM是否启动成功 **//
    //! 如果主站25S以内启动，既先主站清除内存，在hw与主站建立连接，那下面程序可以成功判断Ready 三次
    //! 如果主站25S以外启动，既先初始化HW，再主站清除内存，那么在hw与主站就建立不了连接，那下面程序三次判断Not Ready
    //! 主站不启动和25S以外一样，建立不了连接
    //!小结：主站必需25s以内启动，并且连续三次判断主站处于Ready状态，其余情况统统退出程序

    int Ready_count{ 0 };
    for ( int i{ 0 }; i < 3; i++ )
    {
        hw->setHardwareState( HardwareInterface::HWState::UNKNOWN );

        std::this_thread::sleep_for( std::chrono::duration< double >( 0.1 ) );

        if ( hw->getHardwareState( ) == HardwareInterface::HWState::READY )
        {
            Ready_count++;
            std::cerr << "\033[32m"
                      << "Ready" << std::endl;
        }
        else
        {
            Ready_count = 0;
            std::cerr << "\033[1;31m"
                      << "Not Ready" << std::endl;
        }
    }

    if ( Ready_count == 3 )
    {
        std::cerr << "\033[32m"
                  << "HardWare准备好,开始程序" << std::endl;
    }
    else
    {
        std::cerr << "\033[1;31m"
                  << "HardWare未准备好,程序退出" << std::endl;
        exit( 0 );
    }
    //**-------------------------------**//

    Robot robot( hw );

    auto robotService = RobotServiceImpl::getInstance( &robot );

    std::thread thread_test{ &rocos::Robot::test, &robot };

    //------------------------wait----------------------------------
    robotService->runServer( );

    thread_test.join( );

    return 0;
}
