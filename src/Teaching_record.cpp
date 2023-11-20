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
#include <yaml-cpp/yaml.h>
DEFINE_string(urdf, "robot_sun_new.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");

bool isRuning = true;

#pragma region

namespace rocos
{

    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(7); // 仿真
                                                                                  // boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>(); // 真实机械臂

    Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);

    void signalHandler(int signo)
    {
        if (signo == SIGINT)
        {
            std::cout << "\033[1;31m"
                      << "[!!SIGNAL!!]"
                      << "INTERRUPT by CTRL-C"
                      << "\033[0m" << std::endl;

            isRuning = false;
            robot.setDisabled();

            exit(0);
        }
    }

    //**---------------写入示教点到csv----------------**//
    // 写入格式joint1: [joint1_1, joint1_2, joint1_3, joint1_4, joint1_5, joint1_6, joint1_7]
    int SaveJointsToCSV(std::ofstream &outfile, std::vector<double> joints, int num)
    {
        // 写入文件
        std::string jointnum = "joint" + std::to_string(num) + ": [ ";
        outfile << jointnum;
        for (int i = 0; i < 7; i++)
        {
            outfile << joints[i] << " ";
        }
        outfile << "]" << std::endl;
        return 0;
    }
    // 读取csv的每一行joint1: [joint1_1, joint1_2, joint1_3, joint1_4, joint1_5, joint1_6, joint1_7]到joint
 void  ReadJointsFromCSV(std::string filename, KDL::JntArray &joint, int num)
    {
        std::ifstream infile;
        infile.open(filename, std::ios::in);
        if (!infile.is_open())
        {
            std::cout << "Open file failure" << std::endl;
            std::cout<<"请检查文件名是否存在"<<std::endl;
          
        }
        std::string str;
        std::string jointnum = "joint" + std::to_string(num) + ": [ ";
        while (getline(infile, str))
        {
            if (str.find(jointnum) != std::string::npos)
            {
                // std::cout << str << std::endl;
                std::string str1 = str.substr(10, str.length() - 11);
                // std::cout << str1 << std::endl;
                std::stringstream input(str1);
                for (int i = 0; i < 7; i++)
                {
                    input >> joint(i);
                }
            }
        }
       
    }

#pragma endregion
    void Robot::test()
    {
        // 初始化csv
        sleep(1);
        std::ofstream outfile;
        // 手动输入示教轨迹的文件名
        PLOG_INFO << "Please input the filename of teaching trajectory: ";//举例joint2.csv
        // std::cout << "Please input the filename of teaching trajectory" << std::endl;
        std::string filename;
        std::cin >> filename;
       
        // std::string filename = "joint_record.csv";
        std::string str;
        int num = 1;
        robot.setEnabled();
        outfile.open(filename, std::ios::out | std::ios::trunc);
        if (!outfile.is_open())
        {
            std::cout << "Open file failure" << std::endl;
            exit(0);
        }
        while (true)
        {
            // 输入yes进行读取，输入no退出
            std::cout << "Do you want to save the joints to csv file? yes/no" << std::endl;
            std::cin >> str;
            if (str == "no")
            {
                break;
            }
            else if (str == "yes")
            {
                // 读取关节角
                std::vector<double> joints;
                for (int i = 0; i < 7; i++)
                {
                    joints.push_back(getJointPosition(i));
                }
                // 写入csv
                SaveJointsToCSV(outfile, joints, num);
                num++;
            }
            else
            {
                std::cout << "Please input yes or no" << std::endl;
            }
        }
        outfile.close();
        // 读取csv测试



        std::cout << "-----------------test start-----------------" << std::endl;
        KDL::JntArray joint(_joint_num);
        PLOG_INFO << "Please input the filename of teaching trajectory: ";
        // std::cout << "Please input the filename of teaching trajectory" << std::endl;
        std::string filename_run;
        std::cin >> filename_run;
        std::ifstream file(filename_run);
        int rowCount = std::count(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>(), '\n');
        std::cout<<"文件中点的个数"<<rowCount<<std::endl;
        for (int i = 1; i <=rowCount; i++)
        {
            ReadJointsFromCSV(filename_run, joint, i);
            std::cout << "joint" << i << ":" << joint(0) << "," << joint(1) << "," << joint(2) << "," << joint(3) << "," << joint(4) << "," << joint(5) << "," << joint(6) << std::endl;
            robot.MoveL_FK(joint, 0.1, 0.1);
        }
        std::cout << "test finished" << std::endl;
        robot.setDisabled();
        exit(0);

    }

} // namespace rocos

int main(int argc, char *argv[])
{
    if (signal(SIGINT, rocos::signalHandler) == SIG_ERR)
    {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

    using namespace rocos;

    gflags::ParseCommandLineFlags(&argc, &argv, true);

    auto robotService = RobotServiceImpl::getInstance(&robot);
    std::thread thread_test{&rocos::Robot::test, &robot};
    //------------------------wait----------------------------------
    robotService->runServer();
    thread_test.join();

    return 0;
}
