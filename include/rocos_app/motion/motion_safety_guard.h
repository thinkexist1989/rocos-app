#ifndef ROCOS_APP_MOTION_MOTION_SAFETY_GUARD_H
#define ROCOS_APP_MOTION_MOTION_SAFETY_GUARD_H

#include <rocos_app/motion/motion_types.h>

#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace rocos::motion {

struct RobotStateSnapshot {
    std::vector<double> q_actual;
    std::vector<double> q_dot_actual;
    std::vector<double> tau_actual;
    bool enabled{false};
    double stamp{0.0};
};

struct LowLevelCommand {
    std::vector<double> target_position;
    std::vector<double> target_velocity;
    std::vector<double> target_torque;

    bool hasTargetPosition() const { return !target_position.empty(); }
    bool hasTargetVelocity() const { return !target_velocity.empty(); }
    bool hasTargetTorque() const { return !target_torque.empty(); }
};

struct MotionSafetyLimits {
    std::vector<double> min_position;
    std::vector<double> max_position;
    std::vector<double> max_command_velocity;
    std::vector<double> max_following_error;
};

enum class SafetyViolationCode {
    None,
    NotEnabled,
    InvalidMode,
    InvalidNumber,
    PositionLimitExceeded,
    CommandVelocityLimitExceeded,
    CommandAccelerationLimitExceeded,
    FollowingErrorExceeded,
    TorqueLimitExceeded
};

struct SafetyCheckResult {
    bool ok{false};
    SafetyViolationCode code{SafetyViolationCode::None};
    int api_error_code{0};
    std::string message;

    static SafetyCheckResult success() {
        return SafetyCheckResult{true, SafetyViolationCode::None, 0, "ok"};
    }

    static SafetyCheckResult fail(SafetyViolationCode code,
                                  ErrorCode api_code,
                                  std::string message) {
        return SafetyCheckResult{false,
                                 code,
                                 static_cast<int>(api_code),
                                 std::move(message)};
    }
};

class MotionSafetyGuard {
public:
    explicit MotionSafetyGuard(MotionSafetyLimits limits)
        : limits_(std::move(limits)) {}

    SafetyCheckResult check(const LowLevelCommand& new_cmd,
                            const RobotStateSnapshot& actual,
                            double dt) const {
        if (!actual.enabled) {
            return SafetyCheckResult::fail(
                SafetyViolationCode::NotEnabled,
                ErrorCode::NotAllAtOpState,
                "robot is not enabled");
        }

        if (dt <= 0.0 || !std::isfinite(dt)) {
            return SafetyCheckResult::fail(
                SafetyViolationCode::InvalidNumber,
                ErrorCode::ParameterNanOrInf,
                "control period is invalid");
        }

        if (new_cmd.hasTargetPosition()) {
            const auto dimension_result =
                checkPositionDimensions(new_cmd, actual);
            if (!dimension_result.ok) {
                return dimension_result;
            }

            const auto finite_result = checkFinite(new_cmd.target_position);
            if (!finite_result.ok) {
                return finite_result;
            }

            const auto position_limit_result =
                checkPositionLimits(new_cmd.target_position);
            if (!position_limit_result.ok) {
                return position_limit_result;
            }

            const auto velocity_result =
                checkCommandVelocity(new_cmd.target_position, dt);
            if (!velocity_result.ok) {
                return velocity_result;
            }

            const auto following_result =
                checkFollowingError(new_cmd.target_position, actual.q_actual);
            if (!following_result.ok) {
                return following_result;
            }
        }

        return SafetyCheckResult::success();
    }

    void accept(const LowLevelCommand& accepted_cmd) {
        last_accepted_cmd_ = accepted_cmd;
    }

    void reset() {
        last_accepted_cmd_.reset();
    }

private:
    SafetyCheckResult checkPositionDimensions(
        const LowLevelCommand& new_cmd,
        const RobotStateSnapshot& actual) const {
        const auto n = new_cmd.target_position.size();
        if (actual.q_actual.size() != n ||
            limits_.min_position.size() != n ||
            limits_.max_position.size() != n ||
            limits_.max_command_velocity.size() != n ||
            limits_.max_following_error.size() != n) {
            return SafetyCheckResult::fail(
                SafetyViolationCode::InvalidNumber,
                ErrorCode::IllegalParameter,
                "position command dimensions do not match safety limits");
        }
        return SafetyCheckResult::success();
    }

    SafetyCheckResult checkFinite(const std::vector<double>& values) const {
        for (const double value : values) {
            if (!std::isfinite(value)) {
                return SafetyCheckResult::fail(
                    SafetyViolationCode::InvalidNumber,
                    ErrorCode::ParameterNanOrInf,
                    "position command contains NaN or Inf");
            }
        }
        return SafetyCheckResult::success();
    }

    SafetyCheckResult checkPositionLimits(
        const std::vector<double>& target_position) const {
        for (std::size_t i = 0; i < target_position.size(); ++i) {
            if (target_position[i] < limits_.min_position[i] ||
                target_position[i] > limits_.max_position[i]) {
                return SafetyCheckResult::fail(
                    SafetyViolationCode::PositionLimitExceeded,
                    ErrorCode::PosLimit,
                    "position command exceeds joint limits");
            }
        }
        return SafetyCheckResult::success();
    }

    SafetyCheckResult checkCommandVelocity(
        const std::vector<double>& target_position,
        double dt) const {
        if (!last_accepted_cmd_ ||
            !last_accepted_cmd_->hasTargetPosition()) {
            return SafetyCheckResult::success();
        }

        if (last_accepted_cmd_->target_position.size() !=
            target_position.size()) {
            return SafetyCheckResult::fail(
                SafetyViolationCode::InvalidNumber,
                ErrorCode::IllegalParameter,
                "accepted command dimensions do not match current command");
        }

        for (std::size_t i = 0; i < target_position.size(); ++i) {
            const double velocity =
                std::abs(target_position[i] -
                         last_accepted_cmd_->target_position[i]) /
                dt;
            if (velocity > limits_.max_command_velocity[i]) {
                return SafetyCheckResult::fail(
                    SafetyViolationCode::CommandVelocityLimitExceeded,
                    ErrorCode::SpeedLimit,
                    "position command velocity exceeds limit");
            }
        }
        return SafetyCheckResult::success();
    }

    SafetyCheckResult checkFollowingError(
        const std::vector<double>& target_position,
        const std::vector<double>& actual_position) const {
        for (std::size_t i = 0; i < target_position.size(); ++i) {
            if (std::abs(target_position[i] - actual_position[i]) >
                limits_.max_following_error[i]) {
                return SafetyCheckResult::fail(
                    SafetyViolationCode::FollowingErrorExceeded,
                    ErrorCode::NotFollowPositionCmd,
                    "position following error exceeds limit");
            }
        }
        return SafetyCheckResult::success();
    }

    MotionSafetyLimits limits_;
    std::optional<LowLevelCommand> last_accepted_cmd_;
};

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_MOTION_SAFETY_GUARD_H
