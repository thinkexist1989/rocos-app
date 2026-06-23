#ifndef ROCOS_APP_MOTION_ROBOT_MOTION_CONTEXT_H
#define ROCOS_APP_MOTION_ROBOT_MOTION_CONTEXT_H

#include <rocos_app/motion/motion_context.h>

#include <string>

namespace rocos::motion {

template <typename RobotRuntime>
class RobotMotionContext final : public GuardedMotionContext {
public:
    RobotMotionContext(RobotRuntime& robot,
                       MotionSafetyGuard& safety_guard,
                       double control_period)
        : GuardedMotionContext(safety_guard),
          robot_(robot),
          control_period_(control_period) {}

    RobotStateSnapshot readStateSnapshot() const override {
        RobotStateSnapshot snapshot;
        const int joint_count = robot_.getJointNum();
        snapshot.q_actual.reserve(joint_count);
        snapshot.q_dot_actual.reserve(joint_count);
        snapshot.tau_actual.reserve(joint_count);

        for (int i = 0; i < joint_count; ++i) {
            snapshot.q_actual.push_back(robot_.getJointPosition(i));
            snapshot.q_dot_actual.push_back(robot_.getJointVelocity(i));
            snapshot.tau_actual.push_back(robot_.getJointTorque(i));
        }

        snapshot.enabled = robot_.IsEnabled();
        return snapshot;
    }

    double controlPeriod() const override {
        return control_period_;
    }

protected:
    MotionResult writeCheckedLowLevelCommand(
        const LowLevelCommand& command) override {
        const auto joint_count = static_cast<std::size_t>(robot_.getJointNum());

        if (!command.target_position.empty() &&
            command.target_position.size() != joint_count) {
            return dimensionError("position command dimension does not match robot");
        }
        if (!command.target_velocity.empty() &&
            command.target_velocity.size() != joint_count) {
            return dimensionError("velocity command dimension does not match robot");
        }
        if (!command.target_torque.empty() &&
            command.target_torque.size() != joint_count) {
            return dimensionError("torque command dimension does not match robot");
        }

        for (std::size_t i = 0; i < command.target_position.size(); ++i) {
            robot_.setJointPosition(static_cast<int>(i),
                                    command.target_position[i]);
        }
        for (std::size_t i = 0; i < command.target_velocity.size(); ++i) {
            robot_.setJointVelocity(static_cast<int>(i),
                                    command.target_velocity[i]);
        }
        for (std::size_t i = 0; i < command.target_torque.size(); ++i) {
            robot_.setJointTorque(static_cast<int>(i),
                                  command.target_torque[i]);
        }

        robot_.waitControlCycle();
        return MotionResult::ok();
    }

private:
    static MotionResult dimensionError(std::string message) {
        return MotionResult::failWithApiCode(
            MotionResultCode::InvalidCommand,
            static_cast<int>(ErrorCode::UnmatchedJointsNumber),
            std::move(message));
    }

    RobotRuntime& robot_;
    double control_period_{0.001};
};

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_ROBOT_MOTION_CONTEXT_H
