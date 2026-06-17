#ifndef ROCOS_APP_MOTION_MOTION_CONTEXT_H
#define ROCOS_APP_MOTION_MOTION_CONTEXT_H

#include <rocos_app/motion/motion_safety_guard.h>

namespace rocos::motion {

class MotionContext {
public:
    virtual ~MotionContext() = default;

    virtual RobotStateSnapshot readStateSnapshot() const = 0;
    virtual double controlPeriod() const = 0;
    virtual MotionResult writeLowLevelCommand(
        const LowLevelCommand& command) = 0;
};

class GuardedMotionContext : public MotionContext {
public:
    explicit GuardedMotionContext(MotionSafetyGuard& safety_guard)
        : safety_guard_(safety_guard) {}

    MotionResult writeLowLevelCommand(
        const LowLevelCommand& command) final {
        const auto actual = readStateSnapshot();
        const auto check_result =
            safety_guard_.check(command, actual, controlPeriod());

        if (!check_result.ok) {
            return MotionResult::failWithApiCode(
                MotionResultCode::SafetyViolation,
                check_result.api_error_code,
                check_result.message);
        }

        auto write_result = writeCheckedLowLevelCommand(command);
        if (!write_result.success) {
            return write_result;
        }

        safety_guard_.accept(command);
        return write_result;
    }

protected:
    virtual MotionResult writeCheckedLowLevelCommand(
        const LowLevelCommand& command) = 0;

private:
    MotionSafetyGuard& safety_guard_;
};

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_MOTION_CONTEXT_H
