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

//#define DOCTEST_CONFIG_IMPLEMENT
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>

#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <rocos_app/drive.h>
#include <rocos_app/robot.h>
#include <rocos_app/robot_service.h>
#include <rocos_app/kinematics.h>


TEST_CASE("Hello World") {
    std::cout << "hello world!" << std::endl;
}

TEST_CASE("Hardware") {
//    int id = 1;
    using namespace rocos;
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>();

    std::cout << "Hardware Type is: " << hw->getHardwareTypeString(hw->getHardwareType()) << std::endl;

    std::cout << "Slave number is: " << hw->getSlaveNumber() << std::endl;

    for (int i = 0; i < 3; i++) {
        std::cout << "Min: " << hw->getMinCycleTime() << "; Max: " << hw->getMaxCycleTime() << "; Avg: "
                  << hw->getAvgCycleTime() << "; Curr: " << hw->getCurrCycleTime() << std::endl;

        usleep(1000000);
    }

    for (int id = 0; id < 4; id++) {
        hw->setModeOfOperation(id, ModeOfOperation::CyclicSynchronousPositionMode);
        auto pos = hw->getActualPositionRaw(id);
        std::cout << "Curr pos: " << pos << std::endl;

        hw->setTargetPositionRaw(id, pos);
//    hw->setTargetVelocityRaw(1, 100000);
        hw->setControlwordRaw(id, 128);
//    std::cout << "Status word: " << hw->getStatuswordRaw(1) << std::endl;
//    std::cout << hw->getStatusword(1) << std::endl;
        usleep(10000);

        hw->setControlwordRaw(id, 6);
//    std::cout << "Status word: " << hw->getStatuswordRaw(1) << std::endl;
//    std::cout << hw->getStatusword(1) << std::endl;
        usleep(10000);

        hw->setControlwordRaw(id, 7);
//    std::cout << "Status word: " << hw->getStatuswordRaw(1) << std::endl;
//    std::cout << hw->getStatusword(1) << std::endl;
        usleep(10000);

        hw->setControlwordRaw(id, 15);
//    std::cout << "Status word: " << hw->getStatuswordRaw(1) << std::endl;
        usleep(100000);
        std::cout << hw->getStatusword(id) << std::endl;

        double i = 0.0;
        while (i <= 2 * M_PI) {
            hw->waitForSignal(0);
            int32_t p = pos + 80000.0 * sin(i);
            hw->setTargetPositionRaw(id, p);
            i += 0.0005;
//            std::cout << "Curr pos: " << p << std::endl;
        }

//    hw->setTargetVelocityRaw(1, 0);

//    hw->setControlwordRaw(1, 0);
////    std::cout << "Status word: " << hw->getStatuswordRaw(1) << std::endl;
        std::cout << hw->getStatusword(id) << std::endl;
//    usleep(10000);
    }

}

TEST_CASE("HardwareSim") {
    using namespace rocos;
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(4);

    std::cout << "Hardware Type is: " << hw->getHardwareTypeString(hw->getHardwareType()) << std::endl;

    std::cout << "Slave number is: " << hw->getSlaveNumber() << std::endl;

    for (int i = 0; i < 3; i++) {
        std::cout << "Min: " << hw->getMinCycleTime() << "; Max: " << hw->getMaxCycleTime() << "; Avg: "
                  << hw->getAvgCycleTime() << "; Curr: " << hw->getCurrCycleTime() << std::endl;

        usleep(1000000);
    }

    for (int id = 0; id < 1; id++) {
        hw->setModeOfOperation(id, ModeOfOperation::CyclicSynchronousPositionMode);
        auto pos = hw->getActualPositionRaw(id);
        std::cout << "Curr pos: " << pos << std::endl;

        hw->setTargetPositionRaw(id, pos);
//    hw->setTargetVelocityRaw(1, 100000);
        hw->setControlwordRaw(id, 128);
//    std::cout << "Status word: " << hw->getStatuswordRaw(1) << std::endl;
//    std::cout << hw->getStatusword(1) << std::endl;
        usleep(10000);

        hw->setControlwordRaw(id, 6);
//    std::cout << "Status word: " << hw->getStatuswordRaw(1) << std::endl;
//    std::cout << hw->getStatusword(1) << std::endl;
        usleep(10000);

        hw->setControlwordRaw(id, 7);
//    std::cout << "Status word: " << hw->getStatuswordRaw(1) << std::endl;
//    std::cout << hw->getStatusword(1) << std::endl;
        usleep(10000);

        hw->setControlwordRaw(id, 15);
//    std::cout << "Status word: " << hw->getStatuswordRaw(1) << std::endl;
        usleep(100000);
        std::cout << hw->getStatuswordRaw(id) << std::endl;

        double i = 0.0;
        while (i <= 2 * M_PI) {
            hw->waitForSignal(0);
            int32_t p = pos + 80000.0 * sin(i);
            hw->setTargetPositionRaw(id, p);
            i += 0.0005;
            std::cout << "Curr pos: " << p << std::endl;
        }

//    hw->setTargetVelocityRaw(1, 0);

//    hw->setControlwordRaw(1, 0);
////    std::cout << "Status word: " << hw->getStatuswordRaw(1) << std::endl;
        std::cout << hw->getStatuswordRaw(id) << std::endl;
//    usleep(10000);
    }
}

TEST_CASE("Drive") {

    using namespace rocos;
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>();

    Drive drive(hw, 1);

    usleep(1000000);
    drive.setEnabled();
    std::cout << "After Enabled: \n" << drive.getDriveState() << std::endl;

    usleep(1000000);
    drive.setDisabled();
    std::cout << "After Disabled: \n" << drive.getDriveState() << std::endl;


    usleep(1000000);
    drive.setMode(rocos::ModeOfOperation::CyclicSynchronousPositionMode);
//    drive.setVelocityInCnt(100000);
    drive.setEnabled();
    std::cout << drive.getPositionInCnt() << std::endl;

    drive.moveToPositionInCnt(500000, 100000, 100000);

    std::cout << "Curr State \n" << drive.getDriveState() << std::endl;

    usleep(1000000); // 10s
    drive.setDisabled();

}

TEST_CASE("Async motion") {
    using namespace rocos;
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>();
    std::vector<Drive *> drives;
    for (int i = 0; i < 4; i++) {
        drives.push_back(new Drive(hw, i));
        drives[i]->setMode(rocos::ModeOfOperation::CyclicSynchronousPositionMode);
    }

    usleep(1000000);
    for (auto &drive: drives) {
        drive->setEnabled();
        std::cout << "Drive " << drive->getId() << " After Enabled: \n" << drive->getDriveState() << std::endl;
    }

    drives[0]->moveToPositionInCnt(0, 100000, 100000);
    drives[1]->moveToPositionInCnt(0, 100000, 100000);
    drives[2]->moveToPositionInCnt(0, 100000, 100000);
    drives[3]->moveToPositionInCnt(0, 5000000, 5000000);

    drives[0]->moveToPositionInCnt(500000, 100000, 100000);
    drives[1]->moveToPositionInCnt(500000, 100000, 100000);
    drives[2]->moveToPositionInCnt(500000, 100000, 100000);
    drives[3]->moveToPositionInCnt(25000000, 5000000, 5000000);


}

TEST_CASE("Sync motion") {
    using namespace rocos;
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>();
    Robot robot(hw);

    robot.setEnabled();

    std::vector<double> max_vel{100000, 100000, 100000, 5000000};
    std::vector<double> max_acc{100000, 100000, 100000, 5000000};
    std::vector<double> max_jerk(4, std::numeric_limits<double>::max());

    for (int i = 0; i < 10; i++) {
        std::vector<double> pos{0, 0, 0, 0};

//        robot.moveJ(pos, max_vel, max_acc, max_jerk);

        pos = {500000, 500000, 500000, 25000000};

//        robot.moveJ(pos, max_vel, max_acc, max_jerk);
    }

    robot.setDisabled();

}

TEST_CASE("Robot Motion Thread") {
    using namespace rocos;
//    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(5);
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>();

    Robot robot(hw);

    robot.setEnabled();

    usleep(1000000);

    std::vector<double> pos(4, 0);
    std::vector<double> vel(4, 0);

    robot.moveJ(pos, vel);

    usleep(10000000);

    std::vector<double> pos2(4, 500000);
    robot.moveJ(pos2, vel);

    usleep(10000000);
}


TEST_CASE("gRPC communication") {
    using namespace rocos;
//    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(5);
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>();

    Robot robot(hw);

    auto robotService = RobotServiceImpl::getInstance(&robot);

    robotService->runServer();

}

TEST_CASE("Kinematics") {
    using namespace rocos;

    Kinematics kin;


    std::cout << "FK test: " << std::endl;
    JntArray q(6);
    q.data << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    Frame p;
    kin.JntToCart(q, p);
    std::cout << "pos 1: " << std::endl;

    std::cout << p.p << std::endl;

    q.data << 0.0, M_PI_2, 0.0, 0.0, 0.0, 0.0;
    kin.JntToCart(q, p);
    std::cout << "pos 2: " << std::endl;
    std::cout << p.p << std::endl;


    std::cout << "IK test: " << std::endl;
    JntArray q_out(6);
    q.data << 0.0, M_PI_4, 0.0, 0.0, 0.0, 0.0;

    int res = kin.CartToJnt(q, p, q_out);
    if (res < 0)
        std::cout << "ik solve failed" << std::endl;
    std::cout << "q_out: " << std::endl;
    std::cout << q_out.data << std::endl;

}

void printLink(const KDL::SegmentMap::const_iterator &link, const std::string &prefix,
               tinyxml2::XMLDocument *xmlDocument = nullptr) {
    std::cout << prefix << "- Segment " << GetTreeElementSegment(link->second).getName() <<
              " has " << GetTreeElementChildren(link->second).size() << " children and joint name is "
              << GetTreeElementSegment(link->second).getJoint().getName() << std::endl;
    for (unsigned int i = 0; i < GetTreeElementChildren(link->second).size(); i++) {
        printLink(GetTreeElementChildren(link->second)[i], prefix + "  ");
    }
}


TEST_CASE("URDF") {
    urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDFFile("robot.urdf");
    if (!robot_model) {
        std::cerr << "Could not generate robot model" << std::endl;
        return;
    }

    KDL::Tree my_tree;
//    bool bOk = kdl_parser::treeFromUrdfModel(*robot_model, my_tree);
    if (!kdl_parser::treeFromUrdfModel(*robot_model, my_tree)) {
        std::cerr << "Could not extract kdl tree" << std::endl;
        return;
    }

    KDL::Chain chain;
    my_tree.getChain("base_link", "link_6", chain);



    // walk through tree
    std::cout << " ======================================" << std::endl;
    std::cout << " Tree has " << my_tree.getNrOfSegments() << " link(s) and a root link" << std::endl;
    std::cout << " ======================================" << std::endl;
    KDL::SegmentMap::const_iterator root = my_tree.getRootSegment();

    printLink(root, "");


    tinyxml2::XMLDocument xml_doc;
    xml_doc.LoadFile("robot.urdf");

    auto robot = xml_doc.FirstChildElement("robot");
    for (auto element = robot->FirstChildElement("joint"); element; element = element->NextSiblingElement("joint")) {
        for (int i = 0; i < chain.getNrOfJoints(); ++i) {
            if (element->Attribute("name") == chain.getSegment(i).getJoint().getName()) {
                std::cout << "Joint" << std::endl
                          << "\t name: " << element->Attribute("name") << "\n";
                auto hw = element->FirstChildElement("hardware");
                auto limit = hw->FirstChildElement("limit");
                std::cout << "   limits: \n"
                          << "    - lower: " << atof(limit->Attribute("lower")) << std::endl
                          << "    - upper: " << atof(limit->Attribute("upper")) << std::endl
                          << "    - vel: " << atof(limit->Attribute("vel")) << std::endl
                          << "    - acc: " << atof(limit->Attribute("acc")) << std::endl
                          << "    - jerk: " << atof(limit->Attribute("jerk")) << std::endl;
                auto trans = hw->FirstChildElement("transform");
                std::cout << "  transform: \n"
                          << "    - ratio: " << trans->FloatAttribute("ratio", 2.0) << std::endl
                          << "    - offset_pos_cnt: " << trans->FloatAttribute("offset_pos_cnt", 1.0) << std::endl
                          << "    - cnt_per_unit: " << trans->FloatAttribute("cnt_per_unit", 22.0) << std::endl
                          << "    - torque_per_unit: " << trans->FloatAttribute("torque_per_unit", 33.0) << std::endl;
                if (trans->Attribute("user_unit_name"))
                    std::cout << "    - user_unit_name: " << trans->Attribute("user_unit_name") << std::endl;
                else
                    std::cout << "    - user_unit_name: " << "rad" << std::endl;
            }
        }
    }

}

TEST_CASE("chain_param") {
    urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDFFile("robot.urdf");
    if (!robot_model) {
        std::cerr << "Could not generate robot model" << std::endl;
        return;
    }

    KDL::Tree my_tree;
//    bool bOk = kdl_parser::treeFromUrdfModel(*robot_model, my_tree);
    if (!kdl_parser::treeFromUrdfModel(*robot_model, my_tree)) {
        std::cerr << "Could not extract kdl tree" << std::endl;
        return;
    }

    KDL::Chain chain;
    my_tree.getChain("base_link", "link_6", chain);

    double roll,pitch,yaw;

    // 创建映射表
    std::map<int, std::string> enumMap;
    enumMap[urdf::Joint::FIXED] = "fixed";
    enumMap[urdf::Joint::REVOLUTE] = "revolute";
    enumMap[urdf::Joint::PRISMATIC] = "prismatic";
    enumMap[urdf::Joint::CONTINUOUS] = "continuous";

    std::cout << "robot: " << std::endl;
    std::cout << "    name: " << robot_model->getName() << std::endl;

//    auto root = robot_model->getRoot();
//    std::cout << "  - name: " << root->name << std::endl;
//    std::cout << "    order: " << 0 << std::endl;
//    std::cout << "    mesh: " << std::dynamic_pointer_cast<urdf::Mesh>(root->visual->geometry)->filename << std::endl;

    std::vector<urdf::LinkSharedPtr> links;
    robot_model->getLinks(links);

    for(int i = 0; i < links.size(); i++) {
        std::cout << "  - name: " << links[i]->name << std::endl;
        std::cout << "    order: " << i << std::endl;
        if(links[i]->parent_joint) {
            std::cout << "    type: " << enumMap[links[i]->parent_joint->type] << std::endl;
            std::cout << "    translate: " << links[i]->parent_joint->parent_to_joint_origin_transform.position.x << ", "
                  << links[i]->parent_joint->parent_to_joint_origin_transform.position.y << ", "
                  << links[i]->parent_joint->parent_to_joint_origin_transform.position.z << std::endl;

            std::cout << "    rotate: " << links[i]->parent_joint->parent_to_joint_origin_transform.rotation.x << ", "
                  << links[i]->parent_joint->parent_to_joint_origin_transform.rotation.y << ", "
                  << links[i]->parent_joint->parent_to_joint_origin_transform.rotation.z << ", "
                  << links[i]->parent_joint->parent_to_joint_origin_transform.rotation.w << std::endl;

            std::cout << "    axis: " << links[i]->parent_joint->axis.x << ", "
                  << links[i]->parent_joint->axis.y << ", "
                  << links[i]->parent_joint->axis.z << std::endl;
        }

        links[i]->visual->origin.rotation.getRPY(roll, pitch, yaw);
        std::cout << "    translateLink: " << links[i]->visual->origin.position.x << ", " << links[i]->visual->origin.position.y << ", "
                  << links[i]->visual->origin.position.z << std::endl;

        std::cout << "    rotateLink: " << roll << ", " << pitch << ", " << yaw << std::endl;
        std::cout << "    mesh: " << std::dynamic_pointer_cast<urdf::Mesh>(links[i]->visual->geometry)->filename << std::endl;
    }

//    for (int i = 0; i < chain.getNrOfSegments(); ++i) {
//        auto link = robot_model->getLink(chain.getSegment(i).getName());
//        auto joint = robot_model->getJoint(chain.getSegment(i).getJoint().getName());
//        std::cout << "  - name: " << link->name << std::endl;
//        std::cout << "    order: " << i + 1 << std::endl;
//        std::cout << "    translate: " << joint->parent_to_joint_origin_transform.position.x << ", "
//                  << joint->parent_to_joint_origin_transform.position.y << ", "
//                  << joint->parent_to_joint_origin_transform.position.z << std::endl;
//        joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
//        std::cout << "    rotate: " << roll << ", " << pitch << ", " << yaw << std::endl;
//        std::cout << "    axis: " << joint->axis.x << ", " << joint->axis.y << ", " << joint->axis.z  << std::endl;
//
//        link->visual->origin.rotation.getRPY(roll, pitch, yaw);
//        std::cout << "    translateLink: " << link->visual->origin.position.x << ", " << link->visual->origin.position.y << ", "
//                  << link->visual->origin.position.z << std::endl;
//        std::cout << "    rotateLink: " << roll << ", " << pitch << ", " << yaw << std::endl;
//        std::cout << "    mesh: " << std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry)->filename << std::endl;
//
//    }

}