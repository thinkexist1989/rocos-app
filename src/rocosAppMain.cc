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
#include <rocos_app/robot_http_server.h>
#include <string>
#include <gflags/gflags.h>

DEFINE_string(urdf, "robot.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");
DEFINE_bool(sim, true, "Sim or not");
DEFINE_int32(id, 0, "hardware id, only work for real hardware");
DEFINE_string(http_host, "0.0.0.0", "HTTP server listen host");
DEFINE_int32(http_port, 8080, "HTTP server listen port");

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

    rocos::RobotHttpServer httpServer(&robot);


        int jnt_num_=robot.getJointNum();
        //参数可以直接flange读取Frame getFlange()，读取回来就是Frame类型
        KDL::Frame frame=robot.getFlange();//放求解的笛卡尔姿态
        
        // KDL::Frame frame(
        // KDL::Rotation::RPY(roll, pitch, yaw),  // 旋转矩阵
        // KDL::Vector(x, y, z)                   // 平移向量
        // );
        // // 2. 如果只有位置、没有旋转
        // KDL::Frame frame_pos_only(KDL::Vector(x, y, z));  // 默认 R = Identity
        // // 3. 如果只有旋转、没有位置
        // KDL::Frame frame_rot_only(KDL::Rotation::RPY(roll, pitch, yaw));  // p = 0
        // ========== 验证输出 ==========
        std::cout << frame << std::endl;
        

        KDL::JntArray q_init(jnt_num_);//当前初始关节角
        KDL::JntArray q_target(jnt_num_);//求解出来的关节角
        for (int i = 0; i < jnt_num_; i++) {
            q_init.data[i] = robot.getJointPosition(i);
            q_target.data[i] = robot.getJointPosition(i);
        }
        if (robot.CartToJnt(q_init, frame, q_target) < 0) {
            PLOG_ERROR << " CartToJnt failed";
            return -1;
        }

    //------------------------wait----------------------------------
    httpServer.runAsync(FLAGS_http_host, FLAGS_http_port);
    // Keep main thread alive
    std::cout << "\033[1;32m" << "[HTTP Server] Running on "
          << FLAGS_http_host << ":" << FLAGS_http_port << "\033[0m" << std::endl;
    while (isRuning) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
