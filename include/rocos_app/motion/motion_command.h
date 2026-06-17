#ifndef ROCOS_APP_MOTION_MOTION_COMMAND_H
#define ROCOS_APP_MOTION_MOTION_COMMAND_H

#include <rocos_app/motion/motion_types.h>

#include <string>

namespace rocos::motion {

class MotionCommand {
public:
    virtual ~MotionCommand() = default;

    virtual std::string name() const = 0;
    virtual ReferenceSpace producedReferenceSpace() const {
        return ReferenceSpace::None;
    }

    virtual bool supportsPause() const { return false; }
    virtual bool supportsResume() const { return false; }
    virtual bool supportsStop() const { return true; }

    virtual MotionResult prepare() = 0;
    virtual MotionResult start() = 0;
    virtual MotionStepResult update() = 0;

    virtual MotionResult pause() {
        return MotionResult::fail(MotionResultCode::Unsupported,
                                  "motion command does not support pause");
    }

    virtual MotionResult resume() {
        return MotionResult::fail(MotionResultCode::Unsupported,
                                  "motion command does not support resume");
    }

    virtual MotionResult stop() = 0;
};

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_MOTION_COMMAND_H
