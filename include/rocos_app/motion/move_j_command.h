#ifndef ROCOS_APP_MOTION_MOVE_J_COMMAND_H
#define ROCOS_APP_MOTION_MOVE_J_COMMAND_H

#include <rocos_app/motion/finite_motion_command.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace rocos::motion {

class MoveJCommand final : public FiniteMotionCommand {
public:
    struct Parameters {
        std::vector<double> q_start;
        std::vector<double> q_goal;
        std::vector<double> max_velocity;
        std::vector<double> max_acceleration;
        std::vector<double> max_jerk;
        double dt{0.001};
    };

    explicit MoveJCommand(Parameters params)
        : FiniteMotionCommand(params.dt), params_(std::move(params)) {}

    std::string name() const override { return "MoveJCommand"; }

    ReferenceSpace producedReferenceSpace() const override {
        return ReferenceSpace::Joint;
    }

    MotionResult prepare(MotionContext& /*ctx*/, ModelProvider& /*model*/) override {
        const auto n = params_.q_start.size();
        if (n == 0 ||
            params_.q_goal.size() != n ||
            params_.max_velocity.size() != n ||
            params_.max_acceleration.size() != n ||
            params_.max_jerk.size() != n ||
            params_.dt <= 0.0 ||
            !std::isfinite(params_.dt)) {
            return MotionResult::fail(MotionResultCode::InvalidCommand,
                                      "invalid MoveJ parameter dimensions");
        }

        limits_ = MotionProfileLimits{
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity()};
        has_motion_ = false;

        for (std::size_t i = 0; i < n; ++i) {
            if (!isFinite(params_.q_start[i]) ||
                !isFinite(params_.q_goal[i]) ||
                !isFinite(params_.max_velocity[i]) ||
                !isFinite(params_.max_acceleration[i]) ||
                !isFinite(params_.max_jerk[i])) {
                return MotionResult::failWithApiCode(
                    MotionResultCode::InvalidNumber,
                    static_cast<int>(Result::ParameterNanOrInf),
                    "MoveJ parameter contains NaN or Inf");
            }
            if (params_.max_velocity[i] <= 0.0 ||
                params_.max_acceleration[i] <= 0.0 ||
                params_.max_jerk[i] <= 0.0) {
                return MotionResult::fail(MotionResultCode::InvalidCommand,
                                          "MoveJ limits must be positive");
            }

            const double delta = std::abs(params_.q_goal[i] - params_.q_start[i]);
            if (delta < 1e-12) {
                continue;
            }
            has_motion_ = true;
            limits_.max_velocity =
                std::min(limits_.max_velocity, params_.max_velocity[i] / delta);
            limits_.max_acceleration =
                std::min(limits_.max_acceleration,
                         params_.max_acceleration[i] / delta);
            limits_.max_jerk =
                std::min(limits_.max_jerk, params_.max_jerk[i] / delta);
        }

        if (!has_motion_) {
            limits_ = MotionProfileLimits{1.0, 1.0, 1.0};
        }

        return MotionResult::ok();
    }

protected:
    MotionProfileLimits profileLimits() const override { return limits_; }

    MotionReference sample(double s,
                           double s_dot,
                           double s_ddot) const override { 
        MotionReference reference;
        reference.space = ReferenceSpace::Joint;
        const auto n = params_.q_start.size();
        reference.joint.position.resize(n);
        reference.joint.velocity.resize(n);
        reference.joint.acceleration.resize(n);

        for (std::size_t i = 0; i < n; ++i) {
            const double delta = params_.q_goal[i] - params_.q_start[i];
            reference.joint.position[i] = params_.q_start[i] + s * delta;
            reference.joint.velocity[i] = s_dot * delta;
            reference.joint.acceleration[i] = s_ddot * delta;
        }

        return reference;
    }

private:
    static bool isFinite(double value) { return std::isfinite(value); }

    Parameters params_;
    MotionProfileLimits limits_{1.0, 1.0, 1.0};
    bool has_motion_{false};
};

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_MOVE_J_COMMAND_H
