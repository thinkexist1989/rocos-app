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
void signalHandler(int signo)
{
    if (signo == SIGINT)
    {
        std::cout << "\033[1;31m"
                  << "[!!SIGNAL!!]"
                  << "INTERRUPT by CTRL-C"
                  << "\033[0m" << std::endl;

        isRuning = false;
        exit(0);
    }
}

#pragma region //*测试9  上电保护程序

namespace rocos
{

    void Robot::test()
    {
        //**变量初始化 **//
        std::string str{""};
        //**-------------------------------**//


#pragma region //*电机使能检查

        for (int i{0}; i < jnt_num_; i++)
        {
            if (joints_[i]->getDriveState() != DriveState::OperationEnabled)
            {
                for (int j{0}; j < 1; j++)
                {
                    PLOG_ERROR << "电机[" << i << "] 未使能，确定主站已初始化完成了？,输入y确认";
                    std::cin >> str;
                    if (str != std::string_view{"y"})
                    {
                        PLOG_ERROR << "未输入y, 判断主站 {未} 初始化完成,程序关闭";

                        exit(0);
                    }
                }
            }
        }

        setEnabled();
#pragma endregion

        PLOG_INFO << "当前环境是否安全,如果是,输入run开始执行程序";
        std::cin >> str;

        if ( str == std::string_view{ "run" } )
        {


#if 0  //** 连杆质心处的重力计算 **//

            double length  = 0.203;//连杆长度的一半
            double gravity = 0;

            size_t time_torque{ 0 };//计数器

            //** 滤波器初始化 **//
            Iir::Butterworth::LowPass< 3 > filter;              // NOTE： here order should replaced by a int number!
            const float samplingrate     = 1000;                // Hz
            const float cutoff_frequency = 5;                   // Hz
            filter.setup( 3, samplingrate, cutoff_frequency );  // NOTE： here order should replaced by a int number!
            //**-------------------------------**//



            while ( time_torque < 5000 )
            {
                // gravity = filter.filter( joints_[ 0 ]->getLoadTorque( ) ) / length; //水平情况下测量,得到负载重力
                gravity = filter.filter( joints_[ 0 ]->getLoadTorque( ) ) ;// 竖立情况下测量，得到零漂

                PLOG_INFO << "gravity = " << gravity;
                hw_interface_->waitForSignal( 0 );
                time_torque++;
            }

                joints_[ 0 ]->setTorque(0);

                //零漂 =    60;
                //!  gravity = −116.49

#endif

#if 0 //** 重力补偿检查 **//

            double angle = 0;

            double load_torque = 0;

            size_t time_torque{ 0 };

            double ref_pos = 9.79558;  //需要在0度情况

            double load_gravity = 116.49;

            double length = 0.203;

            // init filter
            Iir::Butterworth::LowPass< 3 > filter;              // NOTE： here order should replaced by a int number!
            const float samplingrate     = 200;                // Hz
            const float cutoff_frequency = 5;                   // Hz
            filter.setup( 3, samplingrate, cutoff_frequency );  // NOTE： here order should replaced by a int number!

            while ( time_torque < 50000 )
            {
                angle = joints_[ 0 ]->getPosition( ) - ref_pos;

                load_torque = load_gravity * sin( angle ) * length;

                if ( time_torque % 100 == 0 )
                    PLOG_DEBUG << "  误差 =  " << filter.filter( joints_[ 0 ]->getLoadTorque( ) ) - 60 - load_torque;

                hw_interface_->waitForSignal( 0 );

                time_torque++;
            }

#endif

#if 1 //** 阻抗实验 **//



            std::cout.setf( ios::scientific );
            double reffrent_pos{ 9.79558};

            double pos{ 0 };
            double last_pos{ 0 };
            double vel {0};
            double target_torque{0};
            double sensor_torque{0};
            double offset_torque{0};
            double last_offset_torque{0};
            double vel_torque{0};
            double command_torque{0};

            double load_torque = 0;
            double load_gravity = 116.49;
            double length = 0.203;


            size_t time_torque{ 0 };

            // init filter
            Iir::Butterworth::LowPass< 3 > filter_1;              // NOTE： here order should replaced by a int number!
            const float samplingrate     = 200;                 // Hz
            const float cutoff_frequency = 5;                   // Hz
            filter_1.setup( 3, samplingrate, cutoff_frequency );  // NOTE： here order should replaced by a int number!

            Iir::Butterworth::LowPass< 2 > filter_2;              // NOTE： here order should replaced by a int number!
            filter_2.setup( 3, samplingrate, cutoff_frequency );  // NOTE： here order should replaced by a int number!




            PLOG_DEBUG << "当前位置 = " << joints_[ 0 ]->getPosition( );

            while ( time_torque < 120000 )
            {
                pos = joints_[ 0 ]->getPosition( ) - reffrent_pos;
                vel = ( pos - last_pos ) / 0.001;

                if ( std::abs( pos ) > 2.09 )
                {
                    PLOG_ERROR << "位置超限";
                    break;
                }

                load_torque = load_gravity * sin( pos ) * length;

                target_torque = -( pos * 180 + vel * 10 );

                sensor_torque = filter_1.filter( joints_[ 0 ]->getLoadTorque( ) ) - 60 - load_torque;

                offset_torque = target_torque - ( -sensor_torque );

                vel_torque = ( offset_torque - last_offset_torque );

                command_torque = target_torque + ( 5 * offset_torque + 3.5* vel_torque );

                if ( std::abs( command_torque ) > 400 )
                    command_torque = KDL::sign(command_torque)*400;

                joints_[ 0 ]->setTorque( command_torque );



                if ( time_torque % 200 == 0 )
                {
                    PLOG_INFO << "target_torque =" << target_torque;
                    PLOG_WARNING << "sensor_torque = " << -sensor_torque;
                    PLOG_DEBUG << "command_torque = " << command_torque;
                }

                time_torque += 1;
                last_pos = pos;
                last_offset_torque = offset_torque;
                hw_interface_->waitForSignal( 0 );
            }

            joints_[ 0 ]->setTorque( 0 );
            hw_interface_->waitForSignal( 0 );

#endif

#if 0 //** 速度测试 **//
            joints_[ 0 ]->setMode( ModeOfOperation::CyclicSynchronousVelocityMode );

            PLOG_INFO << "输入数字，设置速度";
            std::cin >> str;
            PLOG_INFO << "收到速度为" << std::stod( str );
            std::this_thread::sleep_for( std::chrono::milliseconds( 1000 ) );

            if ( std::abs( std::stod( str ) ) > 0.08 )
            {
                PLOG_ERROR << "设置速度过大，为 " << std::stod( str );
                setDisabled( );
                return;
            }
            else
            {
                joints_[ 0 ]->setVelocity( std::stod( str ) );
                hw_interface_->waitForSignal( 0 );
            }

            std::cin >> str;

            joints_[ 0 ]->setVelocity( 0 );
            hw_interface_->waitForSignal( 0 );


#endif





        }
        else
        {
            PLOG_ERROR << "不安全环境,电机抱闸";
            setDisabled();
        }


        PLOG_INFO << "全部测试结束,goodbye!";
    }
} // namespace rocos

#pragma endregion

int main(int argc, char *argv[])
{
    if (signal(SIGINT, signalHandler) == SIG_ERR)
    {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

    using namespace rocos;

    //** 等待主站清除共享内存,25后再启动APP **//
    std::cerr << "\033[32m"
              << "等待主站清除共享内存" << std::endl;
    std::this_thread::sleep_for(std::chrono::duration<double>(5));
    //**-------------------------------**//

    // boost::shared_ptr< HardwareInterface > hw = boost::make_shared< HardwareSim >( 1 );  // 仿真
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>(); //真实机械臂

    //** 判断主站ECM是否启动成功 **//
    //! 如果主站25S以内启动，既先主站清除内存，在hw与主站建立连接，那下面程序可以成功判断Ready 三次
    //! 如果主站25S以外启动，既先初始化HW，再主站清除内存，那么在hw与主站就建立不了连接，那下面程序三次判断Not Ready
    //! 主站不启动和25S以外一样，建立不了连接
    //!小结：主站必需25s以内启动，并且连续三次判断主站处于Ready状态，其余情况统统退出程序

    int Ready_count{0};
    for (int i{0}; i < 3; i++)
    {
        hw->setHardwareState(HardwareInterface::HWState::UNKNOWN);

        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));

        if (hw->getHardwareState() == HardwareInterface::HWState::READY)
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

    if (Ready_count == 3)
    {
        std::cerr << "\033[32m"
                  << "HardWare准备好,开始程序" << std::endl;
    }
    else
    {
        std::cerr << "\033[1;31m"
                  << "HardWare未准备好,程序退出" << std::endl;
        exit(0);
    }
    //**-------------------------------**//

    Robot robot(hw);

    auto robotService = RobotServiceImpl::getInstance(&robot);

    std::thread thread_test{&rocos::Robot::test, &robot};

    //------------------------wait----------------------------------
    robotService->runServer();

    thread_test.join();
    return 0;
}
