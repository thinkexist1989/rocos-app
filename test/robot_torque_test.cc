#include <rocos_app/ethercat/hardware_sim.h>
#include <rocos_app/robot.h>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

int main(int argc, char **argv) {
    if (argc != 2) return 1;
    auto *hardware = new rocos::HardwareSim(20);
    auto *robot = new rocos::Robot(hardware, argv[1], "base_link", "link_7");
    try {
        Eigen::VectorXd command(7);
        for (int i = 0; i < 7; ++i) {
            robot->setJointMode(i, rocos::ModeOfOperation::CyclicSynchronousTorqueMode);
            robot->setJointTorquePerUnit(i, 100);
            command[i] = (i + 1) * 0.25;
        }
        if (robot->setTorque(command) != 0) throw std::runtime_error("Valid torque rejected");
        auto unchanged = [&] {
            for (int i = 0; i < 7; ++i)
                if (robot->getJointTorqueRaw(i) != static_cast<int16_t>(command[i] * 100))
                    throw std::runtime_error("Invalid command partially changed joint torques");
        };
        unchanged();
        auto reject = [&](const Eigen::VectorXd &bad) {
            if (robot->setTorque(bad) == 0) throw std::runtime_error("Invalid torque accepted");
            unchanged();
        };
        reject(Eigen::VectorXd::Zero(6));
        Eigen::VectorXd bad = Eigen::VectorXd::Zero(7);
        bad[6] = std::numeric_limits<double>::quiet_NaN();
        reject(bad);
        bad[6] = 1000; // Would overflow the drive's int16 after scaling.
        reject(bad);
        robot->setJointMode(6, rocos::ModeOfOperation::CyclicSynchronousPositionMode);
        reject(Eigen::VectorXd::Zero(7));
        robot->setDisabled();
        std::cout << "PASS: Robot::setTorque writes valid commands and rejects invalid vectors before any write" << std::endl;
        std::exit(0); // Robot workers have process lifetime.
    } catch (const std::exception &error) {
        robot->setDisabled();
        std::cerr << error.what() << std::endl;
        std::exit(1);
    }
}
