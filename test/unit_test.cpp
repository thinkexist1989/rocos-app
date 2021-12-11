//
// Created by think on 2021/11/19.
//

//#define DOCTEST_CONFIG_IMPLEMENT
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>

#include <ethercat/hardware.h>

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