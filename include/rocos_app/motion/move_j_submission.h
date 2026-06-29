#ifndef ROCOS_APP_MOTION_MOVE_J_SUBMISSION_H
#define ROCOS_APP_MOTION_MOVE_J_SUBMISSION_H

#include <rocos_app/motion/move_j_command.h>

#include <cmath>
#include <memory>
#include <vector>

namespace rocos::motion {

template <typename RobotRuntime, typename Executor>
MotionResult submitMoveJ(RobotRuntime& robot,
                         Executor& executor,
                         const std::vector<double>& target_position,
                         double max_velocity,
                         double max_acceleration,
                         double control_period) {
    const auto joint_count = static_cast<std::size_t>(robot.getJointNum());
    if (joint_count == 0 || target_position.size() != joint_count) {
        return MotionResult::failWithApiCode(
            MotionResultCode::InvalidCommand,
            static_cast<int>(Result::UnmatchedJointsNumber),
            "MoveJ target dimension does not match robot joints");
    }

    if (!std::isfinite(max_velocity) ||
        !std::isfinite(max_acceleration) ||
        !std::isfinite(control_period)) {
        return MotionResult::failWithApiCode(
            MotionResultCode::InvalidNumber,
            static_cast<int>(Result::ParameterNanOrInf),
            "MoveJ scalar parameter contains NaN or Inf");
    }

    if (max_velocity <= 0.0 ||
        max_acceleration <= 0.0 ||
        control_period <= 0.0) {
        return MotionResult::fail(
            MotionResultCode::InvalidCommand,
            "MoveJ velocity acceleration and period must be positive");
    }

    MoveJCommand::Parameters params;
    params.q_goal = target_position;
    params.q_start.reserve(joint_count);
    params.max_velocity.assign(joint_count, max_velocity);
    params.max_acceleration.assign(joint_count, max_acceleration);
    params.max_jerk.reserve(joint_count);
    params.dt = control_period;

    for (std::size_t i = 0; i < joint_count; ++i) {
        const double q_start = robot.getJointPosition(static_cast<int>(i));
        const double q_goal = target_position[i];
        const double max_jerk = robot.getJntJerkLimit(static_cast<int>(i));

        if (!std::isfinite(q_start) ||
            !std::isfinite(q_goal) ||
            !std::isfinite(max_jerk)) {
            return MotionResult::failWithApiCode(
                MotionResultCode::InvalidNumber,
                static_cast<int>(Result::ParameterNanOrInf),
                "MoveJ joint parameter contains NaN or Inf");
        }

        if (max_jerk <= 0.0) {
            return MotionResult::fail(
                MotionResultCode::InvalidCommand,
                "MoveJ jerk limits must be positive");
        }

        params.q_start.push_back(q_start);
        params.max_jerk.push_back(max_jerk);
    }

    return executor.submit(std::make_unique<MoveJCommand>(std::move(params)));
}

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_MOVE_J_SUBMISSION_H
