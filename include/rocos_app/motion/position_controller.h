#ifndef ROCOS_APP_MOTION_POSITION_CONTROLLER_H
#define ROCOS_APP_MOTION_POSITION_CONTROLLER_H

#include <rocos_app/motion/motion_controller.h>

#include <cmath>
#include <string>
#include <vector>

namespace rocos::motion {

class PositionController final : public MotionController {
public:
    ControllerType type() const override { return ControllerType::Position; }

    ReferenceSpace acceptedReferenceSpace() const override {
        return ReferenceSpace::Joint;
    }

    ComplianceSpace complianceSpace() const override {
        return ComplianceSpace::None;
    }

    MotionResult update(const MotionReference& reference,
                        LowLevelCommand& output) override {
        if (!hasReferenceSpace(reference.space, ReferenceSpace::Joint)) {
            return MotionResult::failWithApiCode(
                MotionResultCode::Unsupported,
                static_cast<int>(ErrorCode::MoveUnknown),
                "position controller requires a joint reference");
        }

        const auto& joint = reference.joint;
        if (joint.position.empty()) {
            return MotionResult::fail(MotionResultCode::InvalidCommand,
                                      "joint position reference is empty");
        }

        if (!allFinite(joint.position) ||
            !allFinite(joint.velocity) ||
            !allFinite(joint.acceleration)) {
            return MotionResult::failWithApiCode(
                MotionResultCode::InvalidNumber,
                static_cast<int>(ErrorCode::ParameterNanOrInf),
                "joint reference contains NaN or Inf");
        }

        if ((!joint.velocity.empty() &&
             joint.velocity.size() != joint.position.size()) ||
            (!joint.acceleration.empty() &&
             joint.acceleration.size() != joint.position.size())) {
            return MotionResult::fail(MotionResultCode::InvalidCommand,
                                      "joint reference dimensions do not match");
        }

        output.target_position = joint.position;
        output.target_velocity = joint.velocity;
        output.target_torque.clear();
        return MotionResult::ok();
    }

private:
    static bool allFinite(const std::vector<double>& values) {
        for (const double value : values) {
            if (!std::isfinite(value)) {
                return false;
            }
        }
        return true;
    }
};

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_POSITION_CONTROLLER_H
