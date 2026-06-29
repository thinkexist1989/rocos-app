#ifndef ROCOS_APP_MOTION_MOVE_C_CENTER_ANGLE_COMMAND_H
#define ROCOS_APP_MOTION_MOVE_C_CENTER_ANGLE_COMMAND_H

#include <rocos_app/motion/cartesian_geometry.h>
#include <rocos_app/motion/finite_motion_command.h>
#include <rocos_app/motion/model_provider.h>

#include <kdl/frames.hpp>

#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace rocos::motion {

class MoveCCenterAngleCommand final : public FiniteMotionCommand {
public:
    using IkCallback = std::function<bool(const std::vector<double>&,
                                          const std::vector<double>&,
                                          std::vector<double>&)>;

    struct Parameters {
        KDL::Frame pose_start;
        KDL::Frame center;
        double theta{0.0};
        int axis{2};
        std::vector<double> q_current;
        double max_cartesian_velocity{0.25};
        double max_cartesian_acceleration{1.0};
        bool orientation_fixed{false};
        double dt{0.001};
    };

    explicit MoveCCenterAngleCommand(Parameters params, IkCallback ik)
        : FiniteMotionCommand(params.dt),
          params_(std::move(params)),
          ik_callback_(std::move(ik)) {}

    std::string name() const override { return "MoveCCenterAngleCommand"; }

    ReferenceSpace producedReferenceSpace() const override {
        return ReferenceSpace::Joint;
    }

    MotionResult prepare(MotionContext& /*ctx*/, ModelProvider& /*model*/) override {
        if (!isFinite(params_.theta) ||
            !isFinite(params_.max_cartesian_velocity) ||
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
        if (std::abs(params_.theta) < cartesian::kEpsilon) {
            return MotionResult::fail(MotionResultCode::InvalidCommand,
                                      "MoveC theta is too close to zero");
        }
        if (params_.axis < 0 || params_.axis > 2) {
            return MotionResult::fail(MotionResultCode::InvalidCommand,
                                      "MoveC axis must be 0, 1, or 2");
        }
        if (!ik_callback_) {
            return MotionResult::fail(MotionResultCode::InvalidState,
                                      "MoveC requires an IK callback");
        }

        for (const auto q : params_.q_current) {
            if (!isFinite(q)) {
                return MotionResult::failWithApiCode(
                    MotionResultCode::InvalidNumber,
                    static_cast<int>(Result::ParameterNanOrInf),
                    "MoveC q_current contains NaN or Inf");
            }
        }

        buildCenterFrame();

        const double radius = (params_.pose_start.p - params_.center.p).Norm();
        path_length_ = radius * std::abs(params_.theta);
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
            params_.pose_start, pose_goal_for_arc_, center_frame_,
            params_.theta, s);

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

    void buildCenterFrame() {
        KDL::Vector axis_vec;
        switch (params_.axis) {
            case 0: axis_vec = KDL::Vector(1, 0, 0); break;
            case 1: axis_vec = KDL::Vector(0, 1, 0); break;
            default: axis_vec = KDL::Vector(0, 0, 1); break;
        }
        const KDL::Rotation rot = KDL::Rotation::Rot2(axis_vec, params_.theta);
        pose_goal_for_arc_.M = params_.pose_start.M;
        const KDL::Vector offset = params_.pose_start.p - params_.center.p;
        pose_goal_for_arc_.p = params_.center.p + rot * offset;
    }

    Parameters params_;
    MotionProfileLimits limits_{1.0, 1.0, 1.0};
    double path_length_{0.0};
    int dof_{0};
    KDL::Frame center_frame_;
    KDL::Frame pose_goal_for_arc_;
    mutable std::vector<double> q_seed_;
    IkCallback ik_callback_;
};

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_MOVE_C_CENTER_ANGLE_COMMAND_H
