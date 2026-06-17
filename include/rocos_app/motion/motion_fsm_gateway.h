#ifndef ROCOS_APP_MOTION_MOTION_FSM_GATEWAY_H
#define ROCOS_APP_MOTION_MOTION_FSM_GATEWAY_H

#include <rocos_app/motion/motion_types.h>

namespace rocos::motion {

class MotionFsmGateway {
public:
    virtual ~MotionFsmGateway() = default;

    virtual MotionResult requestStart() = 0;
    virtual MotionResult requestPause() = 0;
    virtual MotionResult notifyPaused() = 0;
    virtual MotionResult requestResume() = 0;
    virtual MotionResult notifyRunning() = 0;
    virtual MotionResult requestStop() = 0;
    virtual MotionResult notifyStopped() = 0;
    virtual MotionResult notifyError(const MotionResult& error) = 0;
};

class AcceptAllMotionFsmGateway final : public MotionFsmGateway {
public:
    MotionResult requestStart() override { return MotionResult::ok(); }
    MotionResult requestPause() override { return MotionResult::ok(); }
    MotionResult notifyPaused() override { return MotionResult::ok(); }
    MotionResult requestResume() override { return MotionResult::ok(); }
    MotionResult notifyRunning() override { return MotionResult::ok(); }
    MotionResult requestStop() override { return MotionResult::ok(); }
    MotionResult notifyStopped() override { return MotionResult::ok(); }
    MotionResult notifyError(const MotionResult&) override {
        return MotionResult::ok();
    }
};

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_MOTION_FSM_GATEWAY_H
