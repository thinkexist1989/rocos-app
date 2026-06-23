#ifndef ROCOS_APP_MOTION_MOVE_C_THREE_POINT_COMMAND_H
#define ROCOS_APP_MOTION_MOVE_C_THREE_POINT_COMMAND_H

#include <rocos_app/motion/cartesian_geometry.h>
#include <rocos_app/motion/finite_motion_command.h>
#include <rocos_app/motion/model_provider.h>

#include <kdl/frames.hpp>

#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace rocos::motion {

class MoveCThreePointCommand final : public FiniteMotionCommand {
public:
    using IkCallback = std::function<bool(const std::vector<double>&,
                                          const std::vector<double>&,
                                          std::vector<double>&)>;

    struct Parameters {
        KDL::Frame pose_start;
        KDL::Frame pose_via;
        KDL::Frame pose_goal;
        std::vector<double> q_current;
        double max_cartesian_velocity{0.25};
        double max_cartesian_acceleration{1.0};
        bool orientation_fixed{false};
        double dt{0.001};
    };

    explicit MoveCThreePointCommand(Parameters params, IkCallback ik)
        : FiniteMotionCommand(params.dt),
          params_(std::move(params)),
          ik_callback_(std::move(ik)) {}

    std::string name() const override { return "MoveCThreePointCommand"; }

    ReferenceSpace producedReferenceSpace() const override {
        return ReferenceSpace::Joint;
    }

    MotionResult prepare(MotionContext& /*ctx*/, ModelProvider& /*model*/) override {
        if (!isFinite(params_.max_cartesian_velocity) ||
            !isFinite(params_.max_cartesian_acceleration) ||
            !isFinite(params_.dt) || params_.dt <= 0.0) {
            return MotionResult::fail(MotionResultCode::InvalidCommand,
                                      "invalid MoveC scalar parameters");
        }
        if (params_.max_cartesian_velocity <= 0.0 ||
            params_.max_cartesian_acceleration <= 0.0) {
            return MotionResult::fail(MotionResultCode::InvalidCommand,
                                      "MoveC Cartesian limits must be positive");
        }
        if (params_.q_current.empty()) {
            return MotionResult::fail(MotionResultCode::InvalidCommand,
                                      "MoveC q_current is empty");
        }
        if (!ik_callback_) {
            return MotionResult::fail(MotionResultCode::InvalidState,
                                      "MoveC requires an IK callback");
        }

        for (const auto q : params_.q_current) {
            if (!isFinite(q)) {
                return MotionResult::failWithApiCode(
                    MotionResultCode::InvalidNumber,
                    static_cast<int>(ErrorCode::ParameterNanOrInf),
                    "MoveC q_current contains NaN or Inf");
            }
        }

        if (!cartesian::computeCircleArcParams(
                center_frame_, theta_, params_.pose_start,
                params_.pose_via, params_.pose_goal)) {
            return MotionResult::fail(MotionResultCode::PlanningFailed,
                                      "failed to compute circle arc from three points");
        }

        const double radius = (params_.pose_start.p - center_frame_.p).Norm();
        path_length_ = radius * std::abs(theta_);
        dof_ = static_cast<int>(params_.q_current.size());
        q_seed_ = params_.q_current;

        if (path_length_ < cartesian::kEpsilon) {
            limits_ = cartesian::computeNormalizedLimits(0.0, 1.0, 1.0);
        } else {
            limits_ = cartesian::computeNormalizedLimits(
                path_length_,
                params_.max_cartesian_velocity,
                params_.max_cartesian_acceleration);
        }
        return MotionResult::ok();
    }

protected:
    MotionProfileLimits profileLimits() const override { return limits_; }

    MotionReference sample(double s, double s_dot, double s_ddot) const override {
        auto target_pose = cartesian::interpolateCircular(
            params_.pose_start, params_.pose_goal, center_frame_, theta_, s);

        if (params_.orientation_fixed) {
            target_pose.M = params_.pose_start.M;
        }

        std::vector<double> pose_vec(7);
        pose_vec[0] = target_pose.p.x();
        pose_vec[1] = target_pose.p.y();
        pose_vec[2] = target_pose.p.z();
        target_pose.M.GetQuaternion(pose_vec[3], pose_vec[4],
                                     pose_vec[5], pose_vec[6]);

        std::vector<double> q_out(dof_);
        if (!ik_callback_(q_seed_, pose_vec, q_out)) {
            return MotionReference{ReferenceSpace::None, JointReference{}};
        }
        q_seed_ = q_out;

        MotionReference ref;
        ref.space = ReferenceSpace::Joint;
        ref.joint.position = q_out;
        ref.joint.velocity.assign(dof_, s_dot * path_length_);
        ref.joint.acceleration.assign(dof_, s_ddot * path_length_);
        return ref;
    }

private:
    static bool isFinite(double v) { return std::isfinite(v); }

    Parameters params_;
    MotionProfileLimits limits_{1.0, 1.0, 1.0};
    double path_length_{0.0};
    double theta_{0.0};
    int dof_{0};
    KDL::Frame center_frame_;
    mutable std::vector<double> q_seed_;
    IkCallback ik_callback_;
};

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_MOVE_C_THREE_POINT_COMMAND_H
