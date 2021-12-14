//
// Created by think on 2021/11/19.
//

//#define DOCTEST_CONFIG_IMPLEMENT
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>

#include <ethercat/hardware.h>
#include <ethercat/hardware_sim.h>

#include <drive.h>

#include <iostream>

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
            hw->waitForSignal();
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
            hw->waitForSignal();
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

TEST_CASE("Sync motion") {
    using namespace rocos;
    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<Hardware>();
    std::vector<Drive*> drives;
    for(int i = 0; i < 4; i++) {
        drives.push_back(new Drive(hw, i));
    }

    usleep(1000000);
    for(auto& drive : drives) {
        drive->setEnabled();
        std::cout << "Drive " << drive->getId() << " After Enabled: \n" << drive->getDriveState() << std::endl;
    }

    drives[0]->moveToPositionInCnt(0, 100000, 100000);
    drives[1]->moveToPositionInCnt(0, 100000, 100000);
    drives[2]->moveToPositionInCnt(0, 100000, 100000);
    drives[3]->moveToPositionInCnt(0, 5000000, 5000000);

//    drives[0]->moveToPositionInCnt(500000, 100000, 100000);
//    drives[1]->moveToPositionInCnt(500000, 100000, 100000);
//    drives[2]->moveToPositionInCnt(500000, 100000, 100000);
//    drives[3]->moveToPositionInCnt(25000000, 5000000, 5000000);


}