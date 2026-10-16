// Offline regression: inject PDO feedback through HardwareSim and exercise URDF -> Robot -> Drive.
#include <rocos_app/ethercat/hardware_sim.h>
#include <rocos_app/robot.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}
}

int main(int argc, char **argv) {
    if (argc != 3) return 1;
    // Robot and DriveGuard workers have process lifetime, as in robot_torque_test.
    auto *hardware = new rocos::HardwareSim(20);
    auto *robot = new rocos::Robot(hardware, argv[1], "base_link", "link_7");
    try {
        const double feedbackScale = std::stod(argv[2]);
        for (int j = 0; j < robot->getJointNum(); ++j) {
            const int id = robot->getJointHardwareId(j);
            const double countsPerUnit = robot->getJointCntPerUnit(j);
            hardware->setTargetPositionRaw(id, 12345);
            const double position = robot->getJointPosition(j);
            for (int32_t raw : {int32_t(0), int32_t(1), int32_t(-1), int32_t(234790), int32_t(-234790),
                                std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::min()}) {
                // HardwareSim echoes raw velocity. This injects 606C, without any real PDO writes.
                hardware->setTargetVelocityRaw(id, raw);
                check(robot->getJointVelocityRaw(j) == raw, "Raw feedback must remain unchanged");
                const double expected = static_cast<double>(raw) * feedbackScale / countsPerUnit;
                check(std::abs(robot->getJointVelocity(j) - expected) < 1e-10,
                      "URDF velocity feedback conversion failed");
                check(robot->getJointPosition(j) == position, "Velocity scale changed position conversion");
            }
            robot->setJointVelocity(j, 0.25);
            check(hardware->getActualVelocityRaw(id) == static_cast<int32_t>(0.25 * countsPerUnit),
                  "606C feedback scale must not alter 60FF target velocity conversion");
        }
        if (feedbackScale == 0.1) {
            hardware->setTargetVelocityRaw(robot->getJointHardwareId(5), 234790);
            check(std::abs(robot->getJointVelocity(5) - 1.1255104500769022) < 1e-9,
                  "Logged J6 feedback must convert to 1.12551 rad/s");
        }
        std::cout << "PASS: signed raw velocity, 606C unit conversion, int32 extremes, position and legacy defaults" << std::endl;
        std::exit(0);
    } catch (const std::exception &error) {
        std::cerr << error.what() << std::endl;
        std::exit(1);
    }
}
