#ifndef ROCOS_APP_MOTION_MOTION_CONTROLLER_H
#define ROCOS_APP_MOTION_MOTION_CONTROLLER_H

#include <rocos_app/motion/motion_safety_guard.h>

namespace rocos::motion {

enum class ControllerType {
    Position,
    JointImpedance,
    JointAdmittance,
    CartesianImpedance,
    CartesianAdmittance
};

enum class ComplianceSpace {
    None,
    Joint,
    Cartesian
};

class MotionController {
public:
    virtual ~MotionController() = default;

    virtual ControllerType type() const = 0;
    virtual ReferenceSpace acceptedReferenceSpace() const = 0;
    virtual ComplianceSpace complianceSpace() const = 0;

    virtual MotionResult activate() { return MotionResult::ok(); }
    virtual MotionResult deactivate() { return MotionResult::ok(); }

    virtual MotionResult update(const MotionReference& reference,
                                LowLevelCommand& output) = 0;

    virtual MotionResult safeStop() {
        return MotionResult::fail(MotionResultCode::Unsupported,
                                  "controller does not support safe stop");
    }
};

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_MOTION_CONTROLLER_H
