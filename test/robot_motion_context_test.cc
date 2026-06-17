#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>

#include <rocos_app/motion/robot_motion_context.h>

#include <vector>

namespace {

class FakeRobotRuntime {
public:
    int getJointNum() const { return 2; }

    bool IsEnabled() { return enabled; }

    double getJointPosition(int id) { return position[id]; }
    double getJointVelocity(int id) { return velocity[id]; }
    double getJointTorque(int id) { return torque[id]; }

    void setJointPosition(int id, double value) {
        ++position_writes;
        commanded_position[id] = value;
    }

    void setJointVelocity(int id, double value) {
        ++velocity_writes;
        commanded_velocity[id] = value;
    }

    void setJointTorque(int id, double value) {
        ++torque_writes;
        commanded_torque[id] = value;
    }

    void waitControlCycle() { ++wait_count; }

    bool enabled{true};
    std::vector<double> position{0.0, 0.1};
    std::vector<double> velocity{0.0, 0.0};
    std::vector<double> torque{0.2, -0.2};
    std::vector<double> commanded_position{0.0, 0.0};
    std::vector<double> commanded_velocity{0.0, 0.0};
    std::vector<double> commanded_torque{0.0, 0.0};
    int position_writes{0};
    int velocity_writes{0};
    int torque_writes{0};
    int wait_count{0};
};

rocos::motion::MotionSafetyLimits makeLimits() {
    rocos::motion::MotionSafetyLimits limits;
    limits.min_position = {-1.0, -1.0};
    limits.max_position = {1.0, 1.0};
    limits.max_command_velocity = {10.0, 10.0};
    limits.max_following_error = {1.0, 1.0};
    return limits;
}

rocos::motion::LowLevelCommand command(double q0, double q1) {
    rocos::motion::LowLevelCommand out;
    out.target_position = {q0, q1};
    out.target_velocity = {0.01, -0.01};
    return out;
}

}  // namespace

TEST_CASE("RobotMotionContext reads robot state snapshot") {
    FakeRobotRuntime robot;
    rocos::motion::MotionSafetyGuard guard(makeLimits());
    rocos::motion::RobotMotionContext<FakeRobotRuntime> context(robot, guard, 0.002);

    const auto snapshot = context.readStateSnapshot();

    CHECK(snapshot.enabled);
    CHECK(snapshot.q_actual == std::vector<double>{0.0, 0.1});
    CHECK(snapshot.q_dot_actual == std::vector<double>{0.0, 0.0});
    CHECK(snapshot.tau_actual == std::vector<double>{0.2, -0.2});
    CHECK(context.controlPeriod() == doctest::Approx(0.002));
}

TEST_CASE("RobotMotionContext writes checked low level command to robot") {
    FakeRobotRuntime robot;
    rocos::motion::MotionSafetyGuard guard(makeLimits());
    rocos::motion::RobotMotionContext<FakeRobotRuntime> context(robot, guard, 0.001);

    const auto result = context.writeLowLevelCommand(command(0.01, 0.09));

    REQUIRE(result.success);
    CHECK(robot.commanded_position == std::vector<double>{0.01, 0.09});
    CHECK(robot.commanded_velocity == std::vector<double>{0.01, -0.01});
    CHECK(robot.position_writes == 2);
    CHECK(robot.velocity_writes == 2);
    CHECK(robot.torque_writes == 0);
    CHECK(robot.wait_count == 1);
}

TEST_CASE("RobotMotionContext does not write command rejected by safety guard") {
    FakeRobotRuntime robot;
    rocos::motion::MotionSafetyGuard guard(makeLimits());
    rocos::motion::RobotMotionContext<FakeRobotRuntime> context(robot, guard, 0.001);

    const auto result = context.writeLowLevelCommand(command(2.0, 0.0));

    CHECK_FALSE(result.success);
    CHECK(result.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::PosLimit));
    CHECK(robot.position_writes == 0);
    CHECK(robot.velocity_writes == 0);
    CHECK(robot.wait_count == 0);
}
