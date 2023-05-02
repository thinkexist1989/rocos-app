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

#pragma region  //*测试9  完整上电保护程序



/**
 * @brief 字符切割
 *
 * @param str 源字符串
 * @param tokens 结果存储
 * @param delim 切割字符
 * @example 1,2,3-> [1] [2] [3]
 */
void split( const std::string& str,
            std::vector< std::string >& tokens,
            const std::string& delim = " " )
{
    tokens.clear( );  //! 注意清除上次结果

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

namespace rocos
{

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

        JC_helper::TCP_server my_server;

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


        my_server.init( );
        boost::thread( &JC_helper::TCP_server::RunServer, &my_server ).detach( );  //开启服务器

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


            if ( str == std::string_view{ "run" } )
            {
            
                using namespace KDL;
                KDL::JntArray q_target( _joint_num );

                q_target( 0 ) = 0 * M_PI / 180;
                q_target( 1 ) = -45 * M_PI / 180;
                q_target( 2 ) = 0 * M_PI / 180;
                q_target( 3 ) = -90 * M_PI / 180;
                q_target( 4 ) = 0 * M_PI / 180;
                q_target( 5 ) = 45 * M_PI / 180;
                q_target( 6 ) = 0 * M_PI / 180;

                MoveJ( q_target, 0.8, 0.6, 0, 0, false );

                DRAGGING_FLAG flag      = DRAGGING_FLAG::NULLSPACE;
                DRAGGING_DIRRECTION dir = DRAGGING_DIRRECTION::NEGATIVE;
                double vel              = 1;
                double acc              = 1;

                int time_count = 0;

                while ( 1 )
                {
                    if ( my_server.flag_receive )
                    {
                        std::string receive_str{ &my_server.receive_buff[ 0 ] };
                        // PLOG_DEBUG << "Received=" << &my_server.receive_buff[ 0 ];
                        std::vector< std::string > tokens;  // 存储字符串分解后的结果

                        split( receive_str, tokens, "#" );

                        if ( tokens[ 0 ].compare( "pos" ) == 0 )
                        {
                            dir = DRAGGING_DIRRECTION::POSITION;
                        }
                        else if ( tokens[ 0 ].compare( "nvg" ) == 0 )
                        {
                            dir = DRAGGING_DIRRECTION::NEGATIVE;
                        }
                        Dragging( flag, dir, stod( tokens[ 1 ] ), stod( tokens[ 2 ] ) );
                        my_server.flag_receive = false;
                    }
                    time_count++;
                    std::this_thread::sleep_for( std::chrono::duration< double >( 0.002 ) );
                }
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

/**
 * @brief 设置指定线程为最高优先级
 *
 * @param this_thread 线程号
 * @return int -1失败，0成功
 */
int set_thread_priority_max( pthread_t this_thread )
{
    const int max_thread_priority = sched_get_priority_max( SCHED_FIFO );
    if ( max_thread_priority != -1 )
    {
        // We'll operate on the currently running thread.
        // pthread_t this_thread = pthread_self( );

        // struct sched_param is used to store the scheduling priority
        struct sched_param params;

        // We'll set the priority to the maximum.
        params.sched_priority = max_thread_priority;

        int ret = pthread_setschedparam( this_thread, SCHED_FIFO, &params );
        if ( ret != 0 )
        {
            std::cerr << RED << "Unsuccessful in setting main thread realtime priority. Error code: " << ret << GREEN << std::endl;
            return -1;
        }
        // Now verify the change in thread priority
        int policy = 0;
        ret        = pthread_getschedparam( this_thread, &policy, &params );
        if ( ret != 0 )
        {
            std::cerr << RED << "Couldn't retrieve real-time scheduling paramers" << GREEN << std::endl;
            return -1;
        }

        // Check the correct policy was applied
        if ( policy != SCHED_FIFO )
        {
            std::cerr << RED << "Main thread: Scheduling is NOT SCHED_FIFO!" << GREEN << std::endl;
            return -1;
        }
        else
        {
            std::cout << GREEN << "Main thread: SCHED_FIFO OK" << std::endl;
        }

        // Print thread scheduling priority
        std::cout << GREEN << "Main thread priority is " << params.sched_priority << std::endl;
        return 0;
    }
    else
    {
        std::cerr << RED << "Could not get maximum thread priority for main thread" << GREEN << std::endl;
        return -1;
    }
}

/**
 * @brief 设置指定进程为最高优先级
 *
 * @param this_thread 线程号
 * @return int -1失败，0成功
 */
int set_process_priority_max( pid_t pid )
{
    const int max_thread_priority = sched_get_priority_max( SCHED_FIFO );
    if ( max_thread_priority != -1 )
    {
        // We'll operate on the currently running thread.
        // pthread_t this_thread = pthread_self( );

        // struct sched_param is used to store the scheduling priority
        struct sched_param params;

        // We'll set the priority to the maximum.
        params.sched_priority = max_thread_priority;

        int ret = sched_setscheduler( pid, SCHED_FIFO, &params );
        if ( ret != 0 )
        {
            std::cerr << RED << "Unsuccessful in setting main process realtime priority. Error code: " << ret << GREEN << std::endl;
            return -1;
        }
        // Now verify the change in thread priority
        ret = sched_getparam( pid, &params );
        if ( ret != 0 )
        {
            std::cerr << RED << "Couldn't retrieve real-time scheduling paramers" << GREEN << std::endl;
            return -1;
        }

        // Print thread scheduling priority
        std::cout << GREEN << "Main process : [" << pid << "] priority is " << params.sched_priority << std::endl;
        return 0;
    }
    else
    {
        std::cerr << RED << "Could not get maximum thread priority for main process" << GREEN << std::endl;
        return -1;
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

    // if ( set_process_priority_max( getpid( ) ) < 0 )
    // {
    //     std::cerr << RED << "Can not set priority max for rocos-app process" << std::endl;
    //     exit( -1 );
    // }

    using namespace rocos;

    //** 等待主站清除共享内存,25后再启动APP **//
    std::cerr << GREEN
              << "等待主站清除共享内存" << std::endl;
    // std::this_thread::sleep_for( std::chrono::duration< double >( 10 ) );
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


