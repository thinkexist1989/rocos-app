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

    //------------------------wait----------------------------------
    robotService->runServer();

    return 0;
}




