#ifndef ROCOS_APP_MOTION_ROBOT_FSM_GATEWAY_H
#define ROCOS_APP_MOTION_ROBOT_FSM_GATEWAY_H

#include <rocos_app/motion/motion_fsm_gateway.h>

#include <string>

namespace rocos::motion {

template <typename RobotFsmClient>
class BasicRobotFsmGateway final : public MotionFsmGateway {
public:
    explicit BasicRobotFsmGateway(RobotFsmClient& robot)
        : robot_(robot) {}

    MotionResult requestStart() override {
        return mapRequest(robot_.requestMotionStart(),
                          "robot FSM rejected motion start");
    }

    MotionResult requestPause() override {
        return mapRequest(robot_.requestMotionPause(),
                          "robot FSM rejected motion pause");
    }

    MotionResult requestResume() override {
        return mapRequest(robot_.requestMotionContinue(),
                          "robot FSM rejected motion continue");
    }

    MotionResult requestStop() override {
        return mapRequest(robot_.requestMotionStop(),
                          "robot FSM rejected motion stop");
    }

    MotionResult notifyPaused() override {
        return MotionResult::ok();
    }

    MotionResult notifyRunning() override {
        return MotionResult::ok();
    }

    MotionResult notifyStopped() override {
        return mapRequest(robot_.requestMotionStop(),
                          "robot FSM rejected motion stopped notification");
    }

    MotionResult notifyError(const MotionResult&) override {
        return mapRequest(robot_.notifyMotionError(),
                          "robot FSM rejected motion error notification");
    }

private:
    static MotionResult mapRequest(bool accepted, std::string message) {
        if (accepted) {
            return MotionResult::ok();
        }
        return MotionResult::failWithApiCode(
            MotionResultCode::InvalidState,
            static_cast<int>(DianaErrorCode::NotAllAtOpState),
            std::move(message));
    }

    RobotFsmClient& robot_;
};

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_ROBOT_FSM_GATEWAY_H
