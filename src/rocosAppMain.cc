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
DEFINE_bool(sim, true, "Sim or not");
DEFINE_int32(id, 0, "hardware id, only work for real hardware");

bool isRuning = true;

rocos::Robot *robot_ptr = nullptr;

void signalHandler(int signo) {
    if (signo == SIGINT) {
        std::cout << "\033[1;31m"
                  << "[!!SIGNAL!!]"
                  << "INTERRUPT by CTRL-C"
                  << "\033[0m" << std::endl;

        isRuning = false;

        robot_ptr->setDisabled();

        exit(0);
    }
}

#pragma region  //*测试10  测试动态调速

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
        KDL::JntArray joints( 7 );
        KDL::JntArray last_joints( 7 );

        int row_index = 1;

        //**-------------------------------**//

       

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
                KDL::JntArray q_target( 7 );
                {//!速度缩放测速
                    // for ( int i = 0; i < 1;i ++)
                    // {
                    //     q_target( 0 ) = 0 * M_PI / 180;
                    //     q_target( 1 ) = -45 * M_PI / 180;
                    //     q_target( 2 ) = 0 * M_PI / 180;
                    //     q_target( 3 ) = -90 * M_PI / 180;
                    //     q_target( 4 ) = 0 * M_PI / 180;
                    //     q_target( 5 ) = 45 * M_PI / 180;
                    //     q_target( 6 ) = 0 * M_PI / 180;

                    //     moveJ_with_speed_scaling( q_target, 1, 3, 10 );

                    //     q_target( 1 ) = 45 * M_PI / 180;
                    //     q_target( 3 ) = 90 * M_PI / 180;
                    //     q_target( 5 ) = -45 * M_PI / 180;

                    //     moveJ_with_speed_scaling( q_target, 1, 3, 10 );
                    // }
                }

                    q_target( 0 ) = 0 * M_PI / 180;
                    q_target( 1 ) = -45 * M_PI / 180;
                    q_target( 2 ) = 30 * M_PI / 180;
                    q_target( 3 ) = -90 * M_PI / 180;
                    q_target( 4 ) = 0 * M_PI / 180;
                    q_target( 5 ) = 45 * M_PI / 180;
                    q_target( 6 ) = 0 * M_PI / 180;
                    MoveJ( q_target, 100 * KDL::deg2rad, 250 * KDL::deg2rad );
                    std::cin >> str;

                    {
                        //! runto movel测速
                        // 正常测速
                        // for (int i = 0; i < 2000; i++) {
                        //     Dragging(DRAGGING_FLAG::RUNTO_MOVEL, DRAGGING_DIRRECTION::POSITION, 0.1, 0.1);
                        //     std::this_thread::sleep_for(std::chrono::duration<double>{0.02});
                        // }
                        // 测速中间停止
                        // for ( int i = 0; i < 100; i++ )
                        // {
                        //     Dragging( DRAGGING_FLAG::RUNTO_MOVEL, DRAGGING_DIRRECTION::POSITION, 0.1, 0.1 );
                        //     std::this_thread::sleep_for( std::chrono::duration< double >{ 0.02 } );
                        // }
                        // PLOG_DEBUG << "已暂停";
                        // std::this_thread::sleep_for( std::chrono::duration< double >{ 2 } );

                        // for ( int i = 0; i < 100; i++ )
                        // {
                        //     Dragging( DRAGGING_FLAG::RUNTO_MOVEL, DRAGGING_DIRRECTION::POSITION, 0.1, 0.1 );
                        //     std::this_thread::sleep_for( std::chrono::duration< double >{ 0.02 } );
                        // }
                        // PLOG_DEBUG << "已暂停";
                        // std::this_thread::sleep_for( std::chrono::duration< double >{ 2 } );

                        // for ( int i = 0; i < 100; i++ )
                        // {
                        //     Dragging( DRAGGING_FLAG::RUNTO_MOVEL, DRAGGING_DIRRECTION::POSITION, 0.1, 0.1 );
                        //     std::this_thread::sleep_for( std::chrono::duration< double >{ 0.02 } );
                        // }
                    }

                    {  //! runto movej测速
                    // 正常测速
                    // for ( int i = 0; i < 2000; i++ )
                    // {
                    //     Dragging( DRAGGING_FLAG::RUNTO_MOVEJ, DRAGGING_DIRRECTION::POSITION, 0.1, 0.1 );
                    //     std::this_thread::sleep_for( std::chrono::duration< double >{ 0.02 } );
                    // }
                    // 测速中间停止
                    for ( int i = 0; i < 100; i++ )
                    {
                        Dragging( DRAGGING_FLAG::RUNTO_MOVEJ, DRAGGING_DIRRECTION::POSITION, 0.1, 0.1 );
                        std::this_thread::sleep_for( std::chrono::duration< double >{ 0.02 } );
                    }
                    PLOG_DEBUG << "已暂停";
                    std::this_thread::sleep_for( std::chrono::duration< double >{ 2 } );

                    for ( int i = 0; i < 100; i++ )
                    {
                        Dragging( DRAGGING_FLAG::RUNTO_MOVEJ, DRAGGING_DIRRECTION::POSITION, 0.1, 0.1 );
                        std::this_thread::sleep_for( std::chrono::duration< double >{ 0.02 } );
                    }
                    PLOG_DEBUG << "已暂停";
                    std::this_thread::sleep_for( std::chrono::duration< double >{ 2 } );

                    for ( int i = 0; i < 2000; i++ )
                    {
                        Dragging( DRAGGING_FLAG::RUNTO_MOVEJ, DRAGGING_DIRRECTION::POSITION, 0.1, 0.1 );
                        std::this_thread::sleep_for( std::chrono::duration< double >{ 0.02 } );
                    }
                    }
                }
                else
                {
                    PLOG_ERROR << "不安全环境,电机抱闸";
                    setDisabled( );
                    return;
                }
        }

        PLOG_INFO << "全部测试结束,goodbye!";
    }
}  // namespace rocos
#pragma endregion


int main(int argc, char *argv[]) {
    if (signal(SIGINT, signalHandler) == SIG_ERR) {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

    using namespace rocos;

    gflags::ParseCommandLineFlags(&argc, &argv, true);
    //**-------------------------------**//

    //**-------------启动admittance_joint-----------**//
    // 初始化类
    HardwareInterface* hw;
    if (FLAGS_sim)
        hw = new HardwareSim(20);  // 仿真
    else
        hw = new Hardware(FLAGS_urdf, FLAGS_id); // 真实机械臂

    Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);

    robot_ptr = &robot;

    auto robotService = RobotServiceImpl::getInstance(&robot);
    std::thread thread_test{ &rocos::Robot::test, &robot };
    //------------------------wait----------------------------------
    robotService->runServer();
    thread_test.join( );

    return 0;
}
