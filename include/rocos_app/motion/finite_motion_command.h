#ifndef ROCOS_APP_MOTION_FINITE_MOTION_COMMAND_H
#define ROCOS_APP_MOTION_FINITE_MOTION_COMMAND_H

#include <rocos_app/UnitIntervalMotionProfile.h>
#include <rocos_app/motion/motion_command.h>

namespace rocos::motion {

struct MotionProfileLimits {
    double max_velocity{0.0};
    double max_acceleration{0.0};
    double max_jerk{0.0};
};

class FiniteMotionCommand : public MotionCommand {
public:
    explicit FiniteMotionCommand(double dt) : profile_(dt) {}
    virtual ~FiniteMotionCommand() = default;

    bool supportsPause() const override { return true; }
    bool supportsResume() const override { return true; }
    bool supportsStop() const override { return true; }

    MotionResult start(MotionContext& /*ctx*/) override {
        const auto limits = profileLimits();
        profile_.Reset();
        profile_.Start(limits.max_velocity,
                       limits.max_acceleration,
                       limits.max_jerk);
        if (profile_.HasError()) {
            status_ = MotionStepStatus::Failed;
            return MotionResult::fail(MotionResultCode::PlanningFailed,
                                      "failed to start finite motion profile");
        }
        status_ = MotionStepStatus::Running;
        return MotionResult::ok();
    }

    MotionResult pause() override {
        if (status_ != MotionStepStatus::Running) {
            return MotionResult::fail(MotionResultCode::InvalidState,
                                      "motion is not running");
        }
        profile_.Pause();
        if (profile_.HasError()) {
            status_ = MotionStepStatus::Failed;
            return MotionResult::fail(MotionResultCode::ExecutionFailed,
                                      "failed to pause finite motion profile");
        }
        status_ = MotionStepStatus::Running;
        pause_requested_ = true;
        return MotionResult::ok();
    }

    MotionResult resume() override {
        if (status_ != MotionStepStatus::Paused) {
            return MotionResult::fail(MotionResultCode::InvalidState,
                                      "motion is not paused");
        }
        profile_.Resume();
        if (profile_.HasError()) {
            status_ = MotionStepStatus::Failed;
            return MotionResult::fail(MotionResultCode::ExecutionFailed,
                                      "failed to resume finite motion profile");
        }
        status_ = MotionStepStatus::Running;
        pause_requested_ = false;
        return MotionResult::ok();
    }

    MotionResult stop() override {
        if (status_ == MotionStepStatus::Finished ||
            status_ == MotionStepStatus::Stopped) {
            return MotionResult::ok();
        }
        profile_.Stop();
        if (profile_.HasError()) {
            status_ = MotionStepStatus::Failed;
            return MotionResult::fail(MotionResultCode::ExecutionFailed,
                                      "failed to stop finite motion profile");
        }
        stopping_ = true;
        return MotionResult::ok();
    }

    MotionStepResult update(MotionContext& /*ctx*/,
                            ModelProvider& /*model*/,
                            bool /*required*/ = true) override {
        if (status_ == MotionStepStatus::Finished ||
            status_ == MotionStepStatus::Stopped ||
            status_ == MotionStepStatus::Paused ||
            status_ == MotionStepStatus::Failed) {
            return MotionStepResult{status_, MotionResult::ok(), std::nullopt};
        }

        profile_.Update();
        if (profile_.HasError()) {
            status_ = MotionStepStatus::Failed;
            return MotionStepResult{
                status_,
                MotionResult::fail(MotionResultCode::ExecutionFailed,
                                   "finite motion profile update failed"),
                std::nullopt};
        }

        auto reference = sample(profile_.position(),
                                profile_.velocity(),
                                profile_.acceleration());

        if (reference.space == ReferenceSpace::None) {
            status_ = MotionStepStatus::Failed;
            return MotionStepResult{
                status_,
                MotionResult::fail(MotionResultCode::ExecutionFailed,
                                   "IK solver failed during finite motion sample"),
                std::nullopt};
        }

        if (stopping_ && profile_.IsStopCompleted()) {
            status_ = MotionStepStatus::Stopped;
        } else if (pause_requested_ && !profile_.IsActive() &&
                   profile_.IsStopped()) {
            status_ = MotionStepStatus::Paused;
        } else if (profile_.HasReachedTarget()) {
            status_ = MotionStepStatus::Finished;
        } else {
            status_ = MotionStepStatus::Running;
        }

        return MotionStepResult{status_, MotionResult::ok(), std::move(reference)};
    }

    MotionStepStatus status() const { return status_; }

protected:
    virtual MotionProfileLimits profileLimits() const = 0;
    virtual MotionReference sample(double s,
                                   double s_dot,
                                   double s_ddot) const = 0;

private:
    UnitIntervalMotionProfile profile_;
    MotionStepStatus status_{MotionStepStatus::Stopped};
    bool pause_requested_{false};
    bool stopping_{false};
};

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_FINITE_MOTION_COMMAND_H
