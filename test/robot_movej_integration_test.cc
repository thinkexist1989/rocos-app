// Robot MoveJ integration example.
//
// Run from the project root after the hardware/EtherCAT runtime is ready:
//   ./build/bin/robot_movej_integration_test

#include <chrono>
#include <iostream>
#include <thread>

#include "src/robot.hpp"

int main() {
    rocos::Robot robot;

    std::cout << "Initial state: " << robot.GetStateString() << std::endl;
    if (robot.GetStateString() == "IDLE") {
        auto rc = robot.SetEnabled();
        std::cout << "SetEnabled result: " << static_cast<int>(rc)
                  << ", state: " << robot.GetStateString() << std::endl;
    }

    const int joint_num = robot.getJointNum();
    rocos::JntArray q_goal(static_cast<unsigned int>(joint_num));
    for (int i = 0; i < joint_num; ++i) {
        q_goal(i) = robot.getJointPosition(i);
    }

    if (joint_num > 0) {
        q_goal(1) += 1;
    }

    auto rc = robot.MoveJ(q_goal, 0.05, 0.5, 2.0);
    std::cout << "MoveJ result: " << static_cast<int>(rc)
              << ", state: " << robot.GetStateString() << std::endl;

    const auto start_time = std::chrono::steady_clock::now();
    bool pause_sent = false;
    bool resume_sent = false;

    while (robot.IsControlActive()) {
        const auto elapsed = std::chrono::steady_clock::now() - start_time;

        if (!pause_sent && elapsed >= std::chrono::seconds(4)) {
            rc = robot.PauseMotion();
            std::cout << "PauseMotion result: " << static_cast<int>(rc)
                      << ", state: " << robot.GetStateString() << std::endl;
            pause_sent = true;
        }

        if (pause_sent && !resume_sent &&
            elapsed >= std::chrono::milliseconds(7500)) {
            rc = robot.ResumeMotion();
            std::cout << "ResumeMotion result: " << static_cast<int>(rc)
                      << ", state: " << robot.GetStateString() << std::endl;
            resume_sent = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "Final state: " << robot.GetStateString() << std::endl;
    return 0;
}
