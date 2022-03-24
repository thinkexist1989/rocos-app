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

#include <cstdlib>
#include <cstdio>
#include <csignal>

//#include <QtCore>
//#include <QProcess>
//#include <QString>
//#include <QDebug>
//#include <QFile>

#include <iostream>
#include <ethercat/hardware.h>
#include <ethercat/hardware_sim.h>
#include <drive.h>
#include <robot.h>
#include <robot_service.h>

bool isRuning = true;

/// \brief 处理终端的Ctrl-C信号
/// \param signo
void signalHandler(int signo) {
    if (signo == SIGINT) {
        std::cout << "\033[1;31m" << "[!!SIGNAL!!]" << "INTERRUPT by CTRL-C" << "\033[0m" << std::endl;
        isRuning = false;
        exit(0);
    }

}

int main(int argc, char *argv[]) {

    if (signal(SIGINT, signalHandler) == SIG_ERR) {
        std::cout << "\033[1;31m" << "Can not catch SIGINT" << "\033[0m" << std::endl;
    }

    using namespace rocos;
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(6); // 仿真
//    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>();

    Robot robot(hw);

    auto robotService = RobotServiceImpl::getInstance(&robot);

{//测试MultiMoveL
            using namespace KDL;
            Frame f_p1;
            Frame f_p2;
            Frame f_p3;
            Frame f_p4;

            f_p1 = robot.getFlange() * Frame{ KDL::Rotation::RotX( 90 * M_PI / 180 ), Vector{ 0.3, 0.0, 0 } };
            f_p2 = f_p1 * Frame{ KDL::Rotation::RotY( 90 * M_PI / 180 ), Vector{ 0.0, -0.3, -0.0 } };
            f_p3 = f_p2 * Frame{ KDL::Rotation::RotX( -90 * M_PI / 180 ), Vector{ 0.0, 0.0, -0.3 } };
            f_p4 = f_p3 * Frame{ KDL::Rotation::RotZ( -90 * M_PI / 180 ), Vector{ 0.0, 0.0, 0.3 } };

            std::vector< KDL::Frame > points{ f_p1,f_p2, f_p3, f_p4 };
            std::vector< double > max_path_v{ 0.06, 0.12, 0.12, 0.24};
            std::vector< double > max_path_a{ 0.06, 0.06, 0.06, 0.06};
            std::vector< double > bound_dist{0.05,0.1,0.0,0.2};
           robot. MultiMoveL( points, bound_dist, max_path_v, max_path_a, false );
           //测试第二次
           robot. MultiMoveL( points, bound_dist, max_path_v, max_path_a, false );


}



    //------------------------wait----------------------------------
    robotService->runServer();

    return 0;
}




