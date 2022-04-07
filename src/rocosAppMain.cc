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
#include <iostream>
#include <robot.h>
#include <robot_service.h>

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

int main( int argc, char* argv[] )
{
    if ( signal( SIGINT, signalHandler ) == SIG_ERR )
    {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

    using namespace rocos;
    boost::shared_ptr< HardwareInterface > hw = boost::make_shared< HardwareSim >( 6 );  // 仿真
                                                                                         //    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>();

    Robot robot( hw );

    auto robotService = RobotServiceImpl::getInstance( &robot );

   {  //测试
       using namespace KDL;
       Frame f_p1;
       Frame f_p2;
       Frame f_p3;
       Frame f_p4;
       Kinematics kinematics_;
       kinematics_.initTechServo();
       KDL::JntArray q( 6 );

       q( 0 ) = 60 * M_PI / 180;
       q( 1 ) = -115 * M_PI / 180;
       q( 2 ) = -110 * M_PI / 180;
       q( 3 ) = -45 * M_PI / 180;
       q( 4 ) = 90 * M_PI / 180;
       q( 5 ) = 0 * M_PI / 180;

       kinematics_.JntToCart( q, f_p1 );

    //    { //测试moveC
          
    //        std::cout << "----------------test moveC start---------------" << std::endl;
    //        f_p2 = f_p1 * Frame{ KDL::Rotation::RotX( 90 * M_PI / 180 ), Vector{ 0.0, -0.1, -0.1 } };
    //        f_p3 = f_p1 * Frame{ KDL::Rotation::RotY( 90 * M_PI / 180 ), Vector{ 0.0, 0.0, -0.2 } };

    //        for ( int i = 0; i < 6; i++ )
    //        {
    //            robot.test_set_pos( i, q( i ) );                                //?pos_资源竞争
    //            std::this_thread::sleep_for( std::chrono::milliseconds( 3 ) );  //?pos_资源竞争
    //        }
    //        robot.MoveC( f_p2, f_p3, 0.05, 0.05, 0, 0, Robot::OrientationMode::FIXED, false );

    //     //    for ( int i = 0; i < 6; i++ )
    //     //    {
    //     //        robot.test_set_pos( i, q( i ) );
    //     //        std::this_thread::sleep_for( std::chrono::milliseconds( 3 ) );
    //     //    }
    //     //    robot.MoveC( f_p2, f_p3, 0.01, 0.01, 0, 0, Robot::OrientationMode::UNCONSTRAINED, false );
    //        std::cout << "----------------test moveC end---------------" << std::endl;
    //    }

    //    {  //测试moveL(只旋转，不移动)

    //        std::cout << "----------------test moveL start---------------" << std::endl;
    //        f_p2 = f_p1 * Frame{ KDL::Rotation::RotX( 90 * M_PI / 180 ) };

    //        for ( int i = 0; i < 6; i++ )
    //        {
    //            robot.test_set_pos( i, q( i ) );                                //?pos_资源竞争
    //            std::this_thread::sleep_for( std::chrono::milliseconds( 3 ) );  //?pos_资源竞争
    //        }
    //        robot.MoveL( f_p2, 0.01, 0.01, 0, 0, false );
    //        std::cout << "----------------test moveL end---------------" << std::endl;
    //    }

       {  //测试MultiMoveL，
           std::cout << "----------------test MultiMoveL start---------------" << std::endl;

           for ( int i = 0; i < 6; i++ )
           {
               robot.test_set_pos( i, q( i ) );
               std::this_thread::sleep_for( std::chrono::milliseconds( 3 ) );
           }
           Frame f_p5;
           Frame f_p6;
           Frame f_p7;
           Frame f_p8;
           Frame f_p9;
           Frame f_p11;

           f_p1 = f_p1 * Frame{ KDL::Rotation::RotX( 90 * M_PI / 180 ), Vector{ 0.3, 0.0, 0 } };
           f_p2 = f_p1 * Frame{ KDL::Rotation::RotY( 90 * M_PI / 180 ), Vector{ 0.0, -0.3, -0.0 } };
           f_p3 = f_p2 * Frame{ KDL::Rotation::RotX( -90 * M_PI / 180 ), Vector{ 0.0, 0.0, -0.3 } };
           f_p4 = f_p3 * Frame{ KDL::Rotation::RotZ( -90 * M_PI / 180 ), Vector{ 0.0, 0.0, 0.3 } };
           f_p5 = f_p4 * Frame{ KDL::Rotation::RotZ( 45 * M_PI / 180 ), Vector{ 0.0, 0.0, -0.15 } };  //180度调头
           f_p6 = f_p5 * Frame{ KDL::Rotation::RotZ( 45 * M_PI / 180 ), Vector{ 0.0, 0.0, -0.15 } };  //0度平行
           f_p7 = f_p6 * Frame{ KDL::Rotation::RotZ( -90 * M_PI / 180 ), Vector{ 0.0, 0.0, 0.3 } };   //180度调头
           f_p8 = f_p7 * Frame{ KDL::Rotation::RotZ( -90 * M_PI / 180 ) };   //只旋转
           f_p9 = f_p8 * Frame{ KDL::Rotation::RotZ( 90 * M_PI / 180 ) };   //只旋转
           f_p11 = f_p1 * Frame{ KDL::Rotation::RotY( 5 * M_PI / 180 ) };   //只旋转
           

           std::vector< KDL::Frame > points{ f_p1, f_p2, f_p3, f_p4, f_p5, f_p6, f_p7 ,f_p8,f_p9,f_p1 ,f_p11};
           std::vector< double > max_path_v{ 0.06, 0.04, 0.04, 0.06, 0.06, 0.06, 0.06 ,0.06 ,0.06 ,0.06,0.01};
           std::vector< double > max_path_a{ 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06 ,0.06,0.06  ,0.06 ,0.01  };
           std::vector< double > bound_dist{ 0.05, 0.0 , 0.1 ,  0.1, 0.1 , 0.1 , 0.0  ,0.0 ,0.0   ,0.0 ,0.0};

           robot.MultiMoveL( points, bound_dist, max_path_v, max_path_a, true );
           std::cout << "----------------test MultiMoveL end---------------" << std::endl;
       }
   }

    //------------------------wait----------------------------------
    robotService->runServer( );

    return 0;
}
