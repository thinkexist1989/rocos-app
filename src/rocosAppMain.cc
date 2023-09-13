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

#include <rocos_app/drive.h>
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <fstream>
#include <iostream>
#include <rocos_app/robot.h>
#include <rocos_app/robot_service.h>
#include <string>
#include <gflags/gflags.h>

DEFINE_string(urdf, "robot.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");

bool isRuning = true;


#pragma region  

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
            str.clear( );

            PLOG_INFO << "当前环境是否安全,如果是,输入run开始执行程序";
            std::cin >> str;

            if ( str == std::string_view{ "run" } )
            {
                using namespace KDL;

                //** movej **//
                //data_analy
                //Theroy:joint_torque=[0,0,0,0,0,0,0]
                //Get the zero_offset of torque_sensor
                KDL::JntArray q_target( _joint_num );
                double Torque[7];
                std::cout<<"joint:";
                for(unsigned int i = 0; i < _joint_num; ++i)
                {
                    Torque[i]=getJointTorque(i);
                    std::cout<<"["<<Torque[i]<<",";
                }
                std::cout<<"]"<<std::endl;

                // fifter-data
                


                //**-------------------------------**//
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

    if ( set_process_priority_max( getpid( ) ) < 0 )
    {
        std::cerr << RED << "Can not set priority max for rocos-app process" << std::endl;
        exit( -1 );
    }

    using namespace rocos;

    gflags::ParseCommandLineFlags(&argc, &argv, true);
    //**-------------------------------**//

//    boost::shared_ptr< HardwareInterface > hw = boost::make_shared< HardwareSim >( _joint_num );  // 仿真
     boost::shared_ptr< HardwareInterface > hw = boost::make_shared< Hardware >( );  //真实机械臂


    Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);

    auto robotService = RobotServiceImpl::getInstance( &robot );

    //------------------------wait----------------------------------
    robotService->runServer( );

    return 0;
}
