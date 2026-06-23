#ifndef ROCOS_APP_MOTION_MOVE_L_SUBMISSION_H
#define ROCOS_APP_MOTION_MOVE_L_SUBMISSION_H

#include <rocos_app/motion/move_l_command.h>

#include <cmath>
#include <functional>
#include <memory>
#include <vector>

namespace rocos::motion {

template <typename RobotRuntime, typename Executor>
MotionResult submitMoveL(RobotRuntime& robot,
                         Executor& executor,
                         const KDL::Frame& target_pose,
                         double max_cartesian_velocity,
                         double max_cartesian_acceleration,
                         double control_period) {
    const auto joint_count = static_cast<std::size_t>(robot.getJointNum());
    if (joint_count == 0) {
        return MotionResult::fail(MotionResultCode::InvalidState,
                                  "robot has no joints configured");
    }

    if (!std::isfinite(max_cartesian_velocity) ||
        !std::isfinite(max_cartesian_acceleration) ||
        !std::isfinite(control_period)) {
        return MotionResult::failWithApiCode(
            MotionResultCode::InvalidNumber,
            static_cast<int>(ErrorCode::ParameterNanOrInf),
            "MoveL scalar parameter contains NaN or Inf");
    }
    if (max_cartesian_velocity <= 0.0 ||
        max_cartesian_acceleration <= 0.0 ||
        control_period <= 0.0) {
        return MotionResult::fail(MotionResultCode::InvalidCommand,
                                  "MoveL parameters must be positive");
    }

    MoveLCommand::Parameters params;
    params.pose_goal = target_pose;
    params.max_cartesian_velocity = max_cartesian_velocity;
    params.max_cartesian_acceleration = max_cartesian_acceleration;
    params.dt = control_period;

    params.q_current.reserve(joint_count);
    for (std::size_t i = 0; i < joint_count; ++i) {
        const double q = robot.getJointPosition(static_cast<int>(i));
        if (!std::isfinite(q)) {
            return MotionResult::failWithApiCode(
                MotionResultCode::InvalidNumber,
                static_cast<int>(ErrorCode::ParameterNanOrInf),
                "MoveL joint position contains NaN or Inf");
        }
        params.q_current.push_back(q);
    }

    // FK on current joint positions → pose_start
    KDL::JntArray q_in(joint_count);
    for (std::size_t i = 0; i < joint_count; ++i) {
        q_in(i) = params.q_current[i];
    }
    KDL::Frame frame_start;
    if (robot.JntToCart(q_in, frame_start) < 0) {
        return MotionResult::fail(MotionResultCode::PlanningFailed,
                                  "FK failed on current joint position");
    }
    params.pose_start = frame_start;

    // IK callback wraps Robot::CartToJnt
    MoveLCommand::IkCallback ik = [&robot](
                                      const std::vector<double>& q_seed,
                                      const std::vector<double>& pose_vec,
                                      std::vector<double>& q_out) -> bool {
        const auto n = q_seed.size();
        KDL::JntArray q_init(n);
        for (std::size_t i = 0; i < n; ++i) { q_init(i) = q_seed[i]; }
        KDL::Frame target;
        target.p = KDL::Vector(pose_vec[0], pose_vec[1], pose_vec[2]);
        target.M = KDL::Rotation::Quaternion(
            pose_vec[3], pose_vec[4], pose_vec[5], pose_vec[6]);
        KDL::JntArray q_result(n);
        if (robot.CartToJnt(q_init, target, q_result) < 0) {
            return false;
        }
        q_out.resize(n);
        for (std::size_t i = 0; i < n; ++i) { q_out[i] = q_result(i); }
        return true;
    };

    auto command = std::make_unique<MoveLCommand>(std::move(params), std::move(ik));
    return executor.submit(std::move(command));
}

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_MOVE_L_SUBMISSION_H
