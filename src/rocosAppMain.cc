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

#pragma region  //*测试7 8个按钮标定
#if 0
namespace rocos
{

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

    /**
     * @brief csv文件解析，解析格式：rem#movej#vel#acc 或者 rem#gripper#pos
     *
     * @param path csv文件绝对路径
     * @param max_size csv一行最大字符
     * @return int
     */
    int Robot::csv_parse( const char* path, size_t max_size,int start )
    {
        std::ifstream input_csv;
        input_csv.open( path );
        if ( input_csv.is_open( ) )
            PLOG_INFO << "输入文件打开成功";
        else
        {
            PLOG_ERROR << "输入文件打开失败";
            return -1;
        }

        char tem[ max_size ];               //存储单行
        std::vector< std::string > tokens;  //存储单行分解后的结果
        KDL::JntArray q_target( 7 );
        int index{ 1 };  //指示当前正在执行的文件
        bool flag_invalid_status{ false };
        std::string str;

        while ( input_csv.getline( tem, max_size ) )  //直接读取一行，以\n结束
        {
            PLOG_DEBUG << index;
            // std::cin >> str;//为了记录线程方便精确定位到某一步
            if ( strcmp( tem, "" ) == 0 ) {
                index++;continue;} //!跳过csv文件\n行

            split( tem, tokens, "," );
            tokens.erase(tokens.begin());//!消去"go";

            if ( tokens[ 0 ].find( "movej" ) != std::string::npos )
            {
                for ( int i{ 0 }; i < 7; i++ )
                {
                    q_target( i ) = std::stod( tokens[ 3 + i ] );
                }
                if ( MoveJ( q_target, std::stod( tokens[ 1 ] ), std::stod( tokens[ 2 ] ), 0, 0, false ) < 0 )
                {
                    PLOG_ERROR << "第" + std::to_string( index ) + "行指令执行失败";
                    flag_invalid_status = true;
                    break;
                }
                index++;
            }
            else if ( tokens[ 0 ].find( "movel" ) != std::string::npos )
            {
                double xyz[ 3 ];
                double rpy[ 3 ];
                for ( int i{ 0 }; i < 3; i++ )
                {
                    xyz[ i ] = std::stod( tokens[ 3 + i ] );
                    rpy[ i ] = std::stod( tokens[ 6 + i ] );
                }
                KDL::Frame frame_target{ KDL::Rotation::RPY( rpy[ 0 ], rpy[ 1 ], rpy[ 2 ] ), KDL::Vector{ xyz[ 0 ], xyz[ 1 ], xyz[ 2 ] } };
                if ( MoveL( frame_target, std::stod( tokens[ 1 ] ), std::stod( tokens[ 2 ] ), 0, 0, false ) < 0 )
                {
                    PLOG_ERROR << "第" + std::to_string( index ) + "行指令执行失败";
                    flag_invalid_status = true;
                    break;
                }
                index++;
            }
            else if ( tokens[ 0 ].find( "gripper" ) != std::string::npos )
            {
                if ( my_gripper.send_command( tokens[ 1 ] + "#80#120" ) < 0 )
                {
                    PLOG_ERROR << "第" + std::to_string( index ) + "行指令执行失败";
                    flag_invalid_status = true;
                    break;
                }
                index++;
            }
            else
            {
                PLOG_ERROR << "未知类型 :" << tokens[ 0 ];
                flag_invalid_status = true;
                break;
            }
        }

        input_csv.close( );

        if ( flag_invalid_status )
        {
            PLOG_ERROR << "csv文件执行失败";
            return -1;
        }
        else
        {
            PLOG_INFO << "csv文件执行成功";
            return 0;
        }
    }

    /**
     * @brief 可执行命令，也可以记录命令，全能的调试工具
     *
     * @return int
     */
    void Robot::csv_helper( )
    {
        std::string cin_str{ };
        std::vector< std::string > tokens;  //存储单行分解后的结果
        KDL::JntArray q_target( 7 );
        std::ofstream out_joint_csv{ };
        out_joint_csv.open( "debug/joints_record.csv" );
        if ( out_joint_csv.is_open( ) )
        {
            PLOG_INFO << "记录文件打开成功";
        }
        else
        {
            PLOG_ERROR << "记录文件打开失败,退出";
            exit( 0 );
        }

        PLOG_INFO << "支持指令列表如下,输入exit退出:";
        PLOG_INFO << "go,movej,vel,acc,j0,j1,j2,j3,j4,j5,j6";
        PLOG_INFO << "go,movel,vel,acc,x,y,z,r,p,y";
        PLOG_INFO << "go,gripper,pos";
        PLOG_INFO << "rem,movej,vel,acc";
        PLOG_INFO << "rem,movel,vel,acc";
        PLOG_INFO << "rem,gripper,pos";
        while ( 1 )
        {
            std::getline( std::cin, cin_str );
            if ( cin_str.empty( ) )
            {   
                PLOG_WARNING << "输入为空，跳过";
                continue;
            }

            split( cin_str, tokens, "," );
   
            if ( tokens[ 0 ].find( "go" ) == 0 )
            {
                if ( tokens[ 1 ].find( "movej" ) != std::string::npos &&tokens.size() == 11  )
                {
                    PLOG_INFO<< "执行movej指令";
                    for ( int i{ 0 }; i < 7; i++ )
                    {
                        q_target( i ) = std::stod( tokens[ 4 + i ] );
                    }

                    if ( MoveJ( q_target, std::stod( tokens[ 2 ] ), std::stod( tokens[ 3 ] ), 0, 0, false ) < 0 )
                    {
                        PLOG_ERROR << "指令执行失败";
                    }
                }
                else if ( tokens[ 1 ].find( "movel" ) != std::string::npos  &&tokens.size() == 10 )
                {
                    PLOG_INFO<< "执行movel指令";

                    double xyz[ 3 ];
                    double rpy[ 3 ];
                    for ( int i{ 0 }; i < 3; i++ )
                    {
                        xyz[ i ] = std::stod( tokens[ 4 + i ] );
                        rpy[ i ] = std::stod( tokens[ 7 + i ] );
                    }
                    KDL::Frame frame_target{ KDL::Rotation::RPY( rpy[ 0 ], rpy[ 1 ], rpy[ 2 ] ), KDL::Vector{ xyz[ 0 ], xyz[ 1 ], xyz[ 2 ] } };
                    if ( MoveL( frame_target, std::stod( tokens[ 2 ] ), std::stod( tokens[ 3 ] ), 0, 0, false ) < 0 )
                    {
                        PLOG_ERROR << "指令执行失败";
                    }
                }
                else if ( tokens[ 1 ].find( "gripper" ) != std::string::npos &&tokens.size() == 3  )
                {
                    PLOG_INFO<< "执行gripper指令";

                    if ( my_gripper.send_command( tokens[ 2 ] + "#80#120" ) < 0 )
                    {
                        PLOG_ERROR << "指令执行失败";
                    }
                }
                else
                {
                    PLOG_ERROR << "未知类型 :" << tokens[ 1 ];
                }
            }
            else if ( tokens[ 0 ].find( "rem" ) == 0 )
            {
                if ( tokens[ 1 ].find( "movel" ) == 0 )
                {

                    PLOG_DEBUG << "记录movel ,速度=" << tokens[ 2 ] << " 加速度= " << tokens[ 3 ];
                    char char_buffer[2048];

                    out_joint_csv << "go  ,movel  ," << std::stod( tokens[ 2 ] ) << "  ," << std::stod( tokens[ 3 ] ) << "  ,";

                    snprintf( char_buffer, 2048, "%-14.7f,%-14.7f,%-14.7f," ,flange_.p[ 0 ] ,flange_.p[ 1 ] ,flange_.p[ 2]  );
                    out_joint_csv.write( char_buffer, strlen(char_buffer));

                    double rpy[ 3 ];
                    flange_.M.GetRPY( rpy[ 0 ], rpy[ 1 ], rpy[ 2 ] );

                    snprintf( char_buffer, 2048, "%-14.7lf,%-14.7lf,%-14.7lf" ,rpy[ 0 ], rpy[ 1 ], rpy[ 2 ] );
                    out_joint_csv.write( char_buffer, strlen(char_buffer));
                    out_joint_csv<<std::endl;
          
                }
                else if ( tokens[ 1 ].find( "movej" ) == 0 )
                {
                    PLOG_DEBUG << "记录movej ,速度=" << tokens[ 2 ] << " 加速度= " << tokens[ 3 ];

                    out_joint_csv << "go  ,movej  ," << std::stod( tokens[ 2 ] ) << "  ," << std::stod( tokens[ 3 ] ) << "  ,";

                    char char_buffer[2048];
                    double _pos[7];
                    for(int i{0};i<7;i++)
                    _pos[i] = pos_[i];

                    snprintf( char_buffer, 2048, "%-14.7lf,%-14.7lf,%-14.7lf,%-14.7lf,%-14.7lf,%-14.7lf,%-14.7lf", _pos[ 0 ], _pos[ 1 ], _pos[ 2 ], _pos[ 3 ], _pos[ 4 ], _pos[ 5 ], _pos[ 6 ] );
                    out_joint_csv.write( char_buffer, strlen( char_buffer ) );
                    out_joint_csv<<std::endl;
               
                }
                else if ( tokens[ 1 ].find( "gripper" ) == 0 )
                {
                    PLOG_DEBUG << "记录gripper ,位置 =" << tokens[ 2 ];

                    out_joint_csv << "gripper  ,";
                    out_joint_csv << tokens[ 2 ] << std::endl;
                }
            }
            else if ( tokens[ 0 ].find( "exit" ) == 0 )
            {
                PLOG_INFO << "输入eixt,采集结束";
                out_joint_csv.close( );
            }
            else
                PLOG_WARNING << "未知命令:" << cin_str;
        }
        PLOG_INFO << "csv_helper 全部结束";
    }

    void Robot::test( )
    {
        //** 变量初始化 **//
        std::string str{ "" };

        //**-------------------------------**//

        //** 程序初始化 **//

        if ( my_gripper.init( ) < 0 )  //夹抓初始化
            return;

        //**-------------------------------**//


#    pragma region  //*电机使能检查
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
#    pragma endregion

        PLOG_INFO << "当前环境是否安全,如果是,输入run开始执行程序";
        std::cin >> str;

        if ( str == std::string_view{ "run" } )
        {
            //** 开启记录线程 **//
            std::thread thread_pos_bag{ &rocos::Robot::csv_helper, this };
            thread_pos_bag.detach();
            //**-------------------------------**//

   
          
        }

        PLOG_INFO << "全部测试结束,goodbye!";
    }
}  // namespace rocos
#endif

#pragma endregion

#pragma region  //*测试8  验收程序
#if 1
namespace rocos
{

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

    /**
     * @brief csv文件解析，解析格式：rem#movej#vel#acc 或者 rem#gripper#pos
     *
     * @param path csv文件绝对路径
     * @param max_size csv一行最大字符
     * @param index 从第几行开始执行，默认第1行
     * @return int
     */
    int Robot::csv_parse( const char* path, size_t max_size, int start )
    {
        std::ifstream input_csv;
        input_csv.open( path );
        if ( input_csv.is_open( ) )
            PLOG_INFO << "输入文件打开成功";
        else
        {
            PLOG_ERROR << "输入文件打开失败";
            return -1;
        }

        //**变量初始化 **//
        char tem[ max_size ];               //存储单行
        std::vector< std::string > tokens;  //存储单行分解后的结果
        KDL::JntArray q_target( jnt_num_ );
        int index{ start };  //指示当前正在执行文件的第几行指令
        bool flag_invalid_status{ false };
        std::string str;
        //**-------------------------------**//

        //** 跳转到指定行执行 **//
        for ( int i{ 1 }; i < start; i++ )
        {
            input_csv.getline( tem, max_size );
        }
        //**-------------------------------**//

        while ( input_csv.getline( tem, max_size ) )  //直接读取一行，以\n结束
        {
           
            // std::cin >> str; //临时修改

            if ( strcmp( tem, "" ) == 0 )
            {
                index++;
                continue;
            }  //!跳过csv文件\n行

            PLOG_DEBUG <<"当前正在执行第 " <<index<<"行指令";
            split( tem, tokens, "," );
            tokens.erase(tokens.begin());//!消去"go";

            if ( tokens[ 0 ].find( "movej" ) != std::string::npos )
            {
                for ( int i{ 0 }; i < 7; i++ )
                {
                    q_target( i ) = std::stod( tokens[ 3 + i ] );
                }
                
                if ( MoveJ( q_target, 1.5 * std::stod( tokens[ 1 ] ), 1.5 * std::stod( tokens[ 2 ] ), 0, 0, false ) < 0 )
                {
                    PLOG_ERROR << "第" + std::to_string( index ) + "行指令执行失败";
                    flag_invalid_status = true;
                    break;
                }
                index++;
            }
            else if ( tokens[ 0 ].find( "movel" ) != std::string::npos )
            {
                double xyz[ 3 ];
                double rpy[ 3 ];
                for ( int i{ 0 }; i < 3; i++ )
                {
                    xyz[ i ] = std::stod( tokens[ 3 + i ] );
                    rpy[ i ] = std::stod( tokens[ 6 + i ] );
                }
                KDL::Frame frame_target{ KDL::Rotation::RPY( rpy[ 0 ], rpy[ 1 ], rpy[ 2 ] ), KDL::Vector{ xyz[ 0 ], xyz[ 1 ], xyz[ 2 ] } };

                if ( MoveL( frame_target, std::stod( tokens[ 1 ] ), std::stod( tokens[ 2 ] ), 0, 0, false ) < 0 )
                {
                    PLOG_ERROR << "第" + std::to_string( index ) + "行指令执行失败";
                    flag_invalid_status = true;
                    break;
                }
                index++;
            }
            else if ( tokens[ 0 ].find( "gripper" ) != std::string::npos )
            {
              
                if ( my_gripper.send_command( tokens[ 1 ] + "#80#120" ) < 0 )
                {
                    PLOG_ERROR << "第" + std::to_string( index ) + "行指令执行失败";
                    flag_invalid_status = true;
                    break;
                }
                index++;
            }
            else
            {
                PLOG_ERROR << "未知类型 :" << tokens[ 0 ];
                flag_invalid_status = true;
                break;
            }
        }

        input_csv.close( );

        if ( flag_invalid_status )
        {
            PLOG_ERROR << "csv文件执行失败";
            return -1;
        }
        else
        {
            PLOG_INFO << "csv文件执行成功";
            return 0;
        }
    }

    /**
     * @brief 检查上电起始位置，如果不在起始位置则报错
     *
     */
    int Robot::check_init_pos( )
    {
        bool is_in_initPos{ true };
       constexpr std::array< double, _joint_num > init_pos{ -0.3665191, -0.3719260, 0.0000000, 1.7631100, 0.0000000, 1.9602000, 0.0000000 };

       sleep( 2 );
       if ( abs( pos_[ 0 ] - init_pos[ 0 ] ) > 1e-2 )
       {
           PLOG_DEBUG << "1关节偏差:" << abs( pos_[ 0 ] - init_pos[ 0 ] );
           is_in_initPos = false;
       }
       if ( abs( pos_[ 1 ] - init_pos[ 1 ] ) > 1e-2 )
       {
           PLOG_DEBUG << "2关节偏差:" << abs( pos_[ 1 ] - init_pos[ 1 ] );
           is_in_initPos = false;
       }
       if ( abs( pos_[ 2 ] - init_pos[ 2 ] ) > 1e-2 )
       {
           PLOG_DEBUG << "3关节偏差:" << abs( pos_[ 2 ] - init_pos[ 2 ] );
           is_in_initPos = false;
       }
       if ( abs( pos_[ 3 ] - init_pos[ 3 ] ) > 1e-2 )
       {
           PLOG_DEBUG << "4关节偏差:" << abs( pos_[ 3 ] - init_pos[ 3 ] );
           is_in_initPos = false;
       }
       if ( abs( pos_[ 4 ] - init_pos[ 4 ] ) > 1e-2 )
       {
           PLOG_DEBUG << "5关节偏差:" << abs( pos_[ 4 ] - init_pos[ 4 ] );
           is_in_initPos = false;
       }
       if ( abs( pos_[ 5 ] - init_pos[ 5 ] ) > 1e-2 )
       {
           PLOG_DEBUG << "6关节偏差:" << abs( pos_[ 5 ] - init_pos[ 5 ] );
           is_in_initPos = false;
       }
       if ( abs( pos_[ 6 ] - init_pos[ 6 ] ) > 1e-2 )
       {
           PLOG_DEBUG << "7关节偏差:" << abs( pos_[ 6 ] - init_pos[ 6 ] );
           is_in_initPos = false;
       }

       if ( !is_in_initPos )
       {
           PLOG_ERROR << "当前位置不在预定的起始位置";

           return -1;
        }
        else
        {
            PLOG_INFO << "当前位置为起始位置";
            return 0;
        }
    }

    void Robot::waitting_for_exit( )
    {
        std::string cin_str{ };
        while ( 1 )
        {
            std::cin >> cin_str;
            if ( cin_str.compare( "exit" ) == 0 )
            {
                if ( is_running_motion )
                    PLOG_WARNING << "当前机械臂正在运动,不允许电机抱闸";
                else
                {
                    PLOG_INFO << "电机抱闸成功，程序即将关闭";
                    setDisabled( );
                    exit( 0 );
                }
            }
            else
                PLOG_WARNING << "不支持此指令：" << cin_str;
        }
    }

    void Robot::test( )
    {
        //**变量初始化 **//
        std::string str{ "" };
        //**-------------------------------**//

        //** 程序初始化 **//

        if ( my_gripper.init( ) < 0 )  //夹抓初始化
            return;

        if ( my_server.init( ) < 0 )  // TCP服务器初始化
            return;

        if ( my_ft_sensor.init( flange_ ) < 0 )  // 6维力初始化
            return;

        //**-------------------------------**//

        //** 开启TCP服务器线程 **//
        std::thread{ &JC_helper::TCP_server::RunServer, &my_server }.detach( );
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
            //** 上电起始位置检查，不在起始位置则程序关闭 **//
            if ( check_init_pos( ) < 0 )
            {
                PLOG_ERROR << "无法返回起始位置!,电机抱闸，程序退出";
                setDisabled( );
                exit(0);
            }
            //**-------------------------------**//

            //** 等待输入exit,然后电机抱闸**//
            std::thread{ &Robot::waitting_for_exit, this }.detach( );
            bool flag_display_disable{true};
            //**-------------------------------**//

            while ( isRuning )
            {
                std::this_thread::sleep_for( std::chrono::duration< double >( 0.1 ) );
                if ( flag_display_disable )
                {
                    PLOG_DEBUG << "静止状态下输入exit,可以电机抱闸，程序关闭";
                    flag_display_disable = false;
                }
                if ( my_server.flag_receive )
                {
                    flag_display_disable = true;

                    std::string receive_str{ &my_server.receive_buff[ 0 ] };

                    PLOG_DEBUG << "TCP 服务器 收到：" << receive_str;
                    if ( receive_str.find( "ROB" ) != std::string::npos && receive_str.find( "btn" ) != std::string::npos )  //!目前只能处理ROB#btn#1指令
                    {
                        std::vector< std::string > tokens;  //存储字符串分解后的结果
                        split( receive_str, tokens, "#" );

                        if ( std::stod( tokens[ 2 ] ) == 9 )
                        {
                            if ( csv_parse( ( std::string{ "debug/demo_" } + std::string{ "process_1" } + std::string{ ".csv.csv.csv" } ).c_str( ) ) < 0 )
                            {
                                PLOG_ERROR << "csv脚本执行失败";
                                return;
                            }
                        }
                        else if ( std::stod( tokens[ 2 ] ) >= 1 && std::stod( tokens[ 2 ] ) <= 8 )
                        {
                            if ( csv_parse( ( std::string{ "debug/demo_" } + tokens[ 2 ] + std::string{ ".csv.csv.csv" } ).c_str( ) ) < 0 )
                            {
                                PLOG_ERROR << "csv脚本执行失败";
                                return;
                            }
                        }
                        else  
                        {
                            PLOG_ERROR << "该命令无效";
                        }
                  
                    }
                    //力控模式
                    else if ( receive_str.find( "ROB" ) != std::string::npos && receive_str.find( "admittance" ) != std::string::npos )  //!目前只能处理ROB#btn#1指令
                    {
                        int res = admittance_teaching( );
                        
                        if ( res < 0 )
                        {
                            PLOG_ERROR << "导纳控制失败";
                            return;
                        }
                        else //导纳控制成功
                        {
                            constexpr std::array< double, _joint_num > init_pos{ -0.3665191, -0.3719260, 0.0000000, 1.7631100, 0.0000000, 1.9602000, 0.0000000 };
                            KDL::JntArray init_pos_jnt( _joint_num );

                            for ( int i{ 0 }; i < _joint_num; i++ )
                                init_pos_jnt( i ) = init_pos[ i ];

                            if ( MoveJ( init_pos_jnt, 0.1, 0.2, 0, 0, false ) < 0 )
                            {
                                PLOG_ERROR << "导纳控制后回归起始位置失败";
                                return;
                            }
                        }
                    }
                    else
                        PLOG_ERROR << "undifined  robot's command : " << receive_str;

                    my_server.receive_buff[ 0 ] = '\0';   //为下次消息接收做准备
                    my_server.flag_receive      = false;  //为下次消息接收做准备
                }
            }
        }
        else
        {
            PLOG_ERROR<<"不安全环境,电机抱闸,程序退出";
            setDisabled();
            exit(0);
        }
        PLOG_INFO << "全部测试结束,goodbye!";
    }
}  // namespace rocos

#endif
#pragma endregion

int main( int argc, char* argv[] )
{
    if ( signal( SIGINT, signalHandler ) == SIG_ERR )
    {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
                  exit(0);
    }

    using namespace rocos;


    //** 等待主站清除共享内存,25后再启动APP **//
    std::cerr << "\033[32m"
              << "等待主站清除共享内存" << std::endl;
    std::this_thread::sleep_for( std::chrono::duration< double >( 15 ) );
    //**-------------------------------**//

    //  boost::shared_ptr< HardwareInterface > hw = boost::make_shared< HardwareSim >( 7 );  // 仿真
    boost::shared_ptr< HardwareInterface > hw = boost::make_shared< Hardware >( );  //真实机械臂

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
