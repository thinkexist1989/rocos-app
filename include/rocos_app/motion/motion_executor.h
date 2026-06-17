#ifndef ROCOS_APP_MOTION_MOTION_EXECUTOR_H
#define ROCOS_APP_MOTION_MOTION_EXECUTOR_H

#include <rocos_app/motion/motion_context.h>
#include <rocos_app/motion/motion_controller.h>
#include <rocos_app/motion/motion_command.h>
#include <rocos_app/motion/motion_fsm_gateway.h>
#include <rocos_app/motion/model_provider.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

namespace rocos::motion {

class NullMotionContext final : public MotionContext {
public:
    RobotStateSnapshot readStateSnapshot() const override {
        return RobotStateSnapshot{};
    }
    double controlPeriod() const override { return 0.001; }
    MotionResult writeLowLevelCommand(const LowLevelCommand& /*command*/) override {
        return MotionResult::ok();
    }
};

class MotionExecutor {
public:
    MotionExecutor()
        : fsm_(&default_fsm_),
          context_(&default_ctx_),
          model_(&default_model_) {}

    MotionExecutor(MotionFsmGateway& fsm)
        : fsm_(&fsm),
          context_(&default_ctx_),
          model_(&default_model_) {}

    MotionExecutor(MotionFsmGateway& fsm,
                   MotionController& controller,
                   MotionContext& context)
        : fsm_(&fsm),
          controller_(&controller),
          context_(&context),
          model_(&default_model_) {}

    MotionExecutor(MotionFsmGateway& fsm,
                   MotionController& controller,
                   MotionContext& context,
                   ModelProvider& model)
        : fsm_(&fsm),
          controller_(&controller),
          context_(&context),
          model_(&model) {}

    ~MotionExecutor() {
        stop();
        joinWorker();
    }

    MotionExecutor(const MotionExecutor&) = delete;
    MotionExecutor& operator=(const MotionExecutor&) = delete;

    MotionResult submit(std::unique_ptr<MotionCommand> command) {
        if (!command) {
            auto result = MotionResult::failWithApiCode(
                MotionResultCode::InvalidCommand,
                static_cast<int>(DianaErrorCode::ParameterPointerEqualsNullptr),
                "motion command is null");
            std::lock_guard<std::mutex> lock(mutex_);
            task_status_ = MotionTaskStatus::Failed;
            last_error_ = result;
            return result;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (current_) {
                auto result = MotionResult::fail(
                    MotionResultCode::Busy,
                    "motion executor already has an active command");
                last_error_ = result;
                return result;
            }
        }

        joinWorker();

        auto start_gate_result = fsm_->requestStart();
        if (!start_gate_result.success) {
            std::lock_guard<std::mutex> lock(mutex_);
            task_status_ = MotionTaskStatus::Failed;
            last_error_ = start_gate_result;
            return start_gate_result;
        }

        auto prepare_result = command->prepare(*context_, *model_);
        if (!prepare_result.success) {
            fsm_->notifyStopped();
            std::lock_guard<std::mutex> lock(mutex_);
            task_status_ = MotionTaskStatus::Failed;
            last_error_ = prepare_result;
            return prepare_result;
        }

        const auto match_result = checkControllerCompatibility(*command);
        if (!match_result.success) {
            fsm_->notifyStopped();
            std::lock_guard<std::mutex> lock(mutex_);
            task_status_ = MotionTaskStatus::Failed;
            last_error_ = match_result;
            return match_result;
        }

        auto start_result = command->start(*context_);
        if (!start_result.success) {
            fsm_->notifyStopped();
            std::lock_guard<std::mutex> lock(mutex_);
            task_status_ = MotionTaskStatus::Failed;
            last_error_ = start_result;
            return start_result;
        }

        if (controller_) {
            auto activate_result = controller_->activate();
            if (!activate_result.success) {
                fsm_->notifyStopped();
                std::lock_guard<std::mutex> lock(mutex_);
                task_status_ = MotionTaskStatus::Failed;
                last_error_ = activate_result;
                return activate_result;
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_ = std::move(command);
            task_status_ = MotionTaskStatus::Running;
            last_error_ = MotionResult::ok();
            stop_worker_ = false;
        }

        worker_ = std::thread(&MotionExecutor::workerLoop, this);
        return MotionResult::ok();
    }

    MotionResult pause() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!current_) {
            return MotionResult::fail(MotionResultCode::InvalidState,
                                      "no active motion command");
        }
        if (task_status_ == MotionTaskStatus::Paused) {
            return MotionResult::fail(MotionResultCode::InvalidState,
                                      "motion is already paused");
        }
        if (task_status_ != MotionTaskStatus::Running) {
            return MotionResult::fail(MotionResultCode::InvalidState,
                                      "motion is not running");
        }
        if (!current_->supportsPause()) {
            return MotionResult::fail(MotionResultCode::Unsupported,
                                      "current motion command does not support pause");
        }
        auto gate_result = fsm_->requestPause();
        if (!gate_result.success) {
            last_error_ = gate_result;
            return gate_result;
        }
        auto result = current_->pause();
        if (!result.success) {
            last_error_ = result;
        }
        return result;
    }

    MotionResult resume() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!current_) {
            return MotionResult::fail(MotionResultCode::InvalidState,
                                      "no active motion command");
        }
        if (!current_->supportsResume()) {
            return MotionResult::fail(MotionResultCode::Unsupported,
                                      "current motion command does not support resume");
        }
        auto gate_result = fsm_->requestResume();
        if (!gate_result.success) {
            last_error_ = gate_result;
            return gate_result;
        }
        auto result = current_->resume();
        if (result.success) {
            fsm_->notifyRunning();
            task_status_ = MotionTaskStatus::Running;
        } else {
            last_error_ = result;
        }
        return result;
    }

    MotionResult stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!current_) {
            return MotionResult::ok("no active motion command");
        }
        if (!current_->supportsStop()) {
            auto result = MotionResult::fail(MotionResultCode::Unsupported,
                                             "current motion command does not support stop");
            last_error_ = result;
            return result;
        }
        auto gate_result = fsm_->requestStop();
        if (!gate_result.success) {
            last_error_ = gate_result;
            return gate_result;
        }
        auto result = current_->stop();
        if (result.success) {
            task_status_ = MotionTaskStatus::Stopping;
        } else {
            last_error_ = result;
        }
        return result;
    }

    MotionTaskStatus currentTaskStatus() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return task_status_;
    }

    bool hasActiveCommand() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return current_ != nullptr;
    }

    MotionResult lastError() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_error_;
    }

private:
    void joinWorker() {
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    void workerLoop() {
        for (;;) {
            MotionStepResult step;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!current_ || stop_worker_) {
                    return;
                }
                step = current_->update(*context_, *model_, true);
                if (step.status == MotionStepStatus::Running) {
                    if (step.reference) {
                        auto dispatch_result =
                            dispatchReference(*step.reference);
                        if (!dispatch_result.success) {
                            task_status_ = MotionTaskStatus::Failed;
                            last_error_ = dispatch_result;
                            fsm_->notifyError(last_error_);
                            deactivateController();
                            current_.reset();
                            return;
                        }
                    }
                } else if (step.status == MotionStepStatus::Finished) {
                    fsm_->notifyStopped();
                    task_status_ = MotionTaskStatus::Finished;
                    last_error_ = MotionResult::ok();
                    deactivateController();
                    current_.reset();
                    return;
                } else if (step.status == MotionStepStatus::Stopped) {
                    fsm_->notifyStopped();
                    task_status_ = MotionTaskStatus::Cancelled;
                    last_error_ = MotionResult::ok();
                    deactivateController();
                    current_.reset();
                    return;
                } else if (step.status == MotionStepStatus::Paused) {
                    if (task_status_ != MotionTaskStatus::Paused) {
                        fsm_->notifyPaused();
                    }
                    task_status_ = MotionTaskStatus::Paused;
                } else if (step.status == MotionStepStatus::Failed ||
                           !step.result.success) {
                    task_status_ = MotionTaskStatus::Failed;
                    last_error_ = step.result.success
                                      ? MotionResult::fail(MotionResultCode::ExecutionFailed,
                                                           "motion command failed")
                                      : step.result;
                    fsm_->notifyError(last_error_);
                    deactivateController();
                    current_.reset();
                    return;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    MotionResult dispatchReference(const MotionReference& reference) {
        // 无 controller 且无 context：纯测试场景，直接忽略
        if (!controller_ && !context_) {
            return MotionResult::ok();
        }

        // 无 controller 时，JointReference 直接写入 context（有限运动命令直写）
        if (!controller_) {
            if (reference.space == ReferenceSpace::Joint) {
                LowLevelCommand command;
                command.target_position = reference.joint.position;
                command.target_velocity = reference.joint.velocity;
                return context_->writeLowLevelCommand(command);
            }
            return MotionResult::fail(MotionResultCode::InvalidState,
                                      "no controller for non-joint reference");
        }

        // 有 controller 就必须有 context
        if (!context_) {
            return MotionResult::fail(MotionResultCode::InvalidState,
                                      "motion controller requires context");
        }

        LowLevelCommand command;
        auto controller_result = controller_->update(reference, command);
        if (!controller_result.success) {
            return controller_result;
        }

        return context_->writeLowLevelCommand(command);
    }

    MotionResult checkControllerCompatibility(
        const MotionCommand& command) const {
        // 没有 controller 时，直接接受（JointReference 走直写路径）
        if (!controller_) {
            return MotionResult::ok();
        }

        const auto produced = command.producedReferenceSpace();
        if (produced == ReferenceSpace::None) {
            return MotionResult::ok();
        }

        if (hasReferenceSpace(controller_->acceptedReferenceSpace(), produced)) {
            return MotionResult::ok();
        }

        return MotionResult::failWithApiCode(
            MotionResultCode::Unsupported,
            static_cast<int>(DianaErrorCode::MoveUnknown),
            "motion command reference space does not match active controller");
    }

    void deactivateController() {
        if (controller_) {
            controller_->deactivate();
        }
    }

    mutable std::mutex mutex_;
    AcceptAllMotionFsmGateway default_fsm_;
    NullMotionContext default_ctx_;
    ModelProvider default_model_;
    MotionFsmGateway* fsm_{nullptr};
    MotionController* controller_{nullptr};
    MotionContext* context_{nullptr};
    ModelProvider* model_{nullptr};
    std::unique_ptr<MotionCommand> current_;
    std::thread worker_;
    bool stop_worker_{false};
    MotionTaskStatus task_status_{MotionTaskStatus::None};
    MotionResult last_error_{MotionResult::ok()};
};

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_MOTION_EXECUTOR_H
