#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>

#include <rocos_app/motion/motion_executor.h>
#include <rocos_app/motion/motion_context.h>
#include <rocos_app/motion/move_j_command.h>
#include <rocos_app/motion/position_controller.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

namespace {

class CountingCommand final : public rocos::motion::MotionCommand {
public:
    explicit CountingCommand(int finish_after_updates)
        : finish_after_updates_(finish_after_updates) {}

    std::string name() const override { return "CountingCommand"; }

    rocos::motion::MotionResult prepare() override {
        prepared_ = true;
        return rocos::motion::MotionResult::ok();
    }

    rocos::motion::MotionResult start() override {
        if (!prepared_) {
            return rocos::motion::MotionResult::fail(
                rocos::motion::MotionResultCode::InvalidState,
                "not prepared");
        }
        return rocos::motion::MotionResult::ok();
    }

    rocos::motion::MotionStepResult update() override {
        ++updates_;
        if (stop_requested_) {
            return rocos::motion::MotionStepResult{
                rocos::motion::MotionStepStatus::Stopped,
                rocos::motion::MotionResult::ok(),
                std::nullopt};
        }
        if (updates_ >= finish_after_updates_) {
            return rocos::motion::MotionStepResult{
                rocos::motion::MotionStepStatus::Finished,
                rocos::motion::MotionResult::ok(),
                std::nullopt};
        }
        return rocos::motion::MotionStepResult{
            rocos::motion::MotionStepStatus::Running,
            rocos::motion::MotionResult::ok(),
            std::nullopt};
    }

    rocos::motion::MotionResult stop() override {
        stop_requested_ = true;
        return rocos::motion::MotionResult::ok();
    }

private:
    int finish_after_updates_{0};
    int updates_{0};
    bool prepared_{false};
    bool stop_requested_{false};
};

struct TrackingStats {
    int prepare_count{0};
    int start_count{0};
};

class TrackingCommand final : public rocos::motion::MotionCommand {
public:
    explicit TrackingCommand(std::shared_ptr<TrackingStats> stats =
                                 std::make_shared<TrackingStats>())
        : stats_(std::move(stats)) {}

    std::string name() const override { return "TrackingCommand"; }
    bool supportsPause() const override { return true; }
    bool supportsResume() const override { return true; }

    rocos::motion::MotionResult prepare() override {
        ++stats_->prepare_count;
        return rocos::motion::MotionResult::ok();
    }

    rocos::motion::MotionResult start() override {
        ++stats_->start_count;
        return rocos::motion::MotionResult::ok();
    }

    rocos::motion::MotionStepResult update() override {
        if (stop_requested) {
            return rocos::motion::MotionStepResult{
                rocos::motion::MotionStepStatus::Stopped,
                rocos::motion::MotionResult::ok(),
                std::nullopt};
        }
        if (pause_requested) {
            return rocos::motion::MotionStepResult{
                rocos::motion::MotionStepStatus::Paused,
                rocos::motion::MotionResult::ok(),
                std::nullopt};
        }
        return rocos::motion::MotionStepResult{
            rocos::motion::MotionStepStatus::Running,
            rocos::motion::MotionResult::ok(),
            std::nullopt};
    }

    rocos::motion::MotionResult pause() override {
        pause_requested = true;
        return rocos::motion::MotionResult::ok();
    }

    rocos::motion::MotionResult resume() override {
        pause_requested = false;
        return rocos::motion::MotionResult::ok();
    }

    rocos::motion::MotionResult stop() override {
        stop_requested = true;
        return rocos::motion::MotionResult::ok();
    }

    bool pause_requested{false};
    bool stop_requested{false};

private:
    std::shared_ptr<TrackingStats> stats_;
};

class RecordingFsmGateway final : public rocos::motion::MotionFsmGateway {
public:
    rocos::motion::MotionResult requestStart() override {
        ++start_requests;
        return start_result;
    }

    rocos::motion::MotionResult requestPause() override {
        ++pause_requests;
        return pause_result;
    }

    rocos::motion::MotionResult notifyPaused() override {
        ++paused_notifications;
        return rocos::motion::MotionResult::ok();
    }

    rocos::motion::MotionResult requestResume() override {
        ++resume_requests;
        return resume_result;
    }

    rocos::motion::MotionResult notifyRunning() override {
        ++running_notifications;
        return rocos::motion::MotionResult::ok();
    }

    rocos::motion::MotionResult requestStop() override {
        ++stop_requests;
        return stop_result;
    }

    rocos::motion::MotionResult notifyStopped() override {
        ++stopped_notifications;
        return rocos::motion::MotionResult::ok();
    }

    rocos::motion::MotionResult notifyError(
        const rocos::motion::MotionResult& error) override {
        ++error_notifications;
        last_error = error;
        return rocos::motion::MotionResult::ok();
    }

    rocos::motion::MotionResult start_result{rocos::motion::MotionResult::ok()};
    rocos::motion::MotionResult pause_result{rocos::motion::MotionResult::ok()};
    rocos::motion::MotionResult resume_result{rocos::motion::MotionResult::ok()};
    rocos::motion::MotionResult stop_result{rocos::motion::MotionResult::ok()};
    int start_requests{0};
    int pause_requests{0};
    int paused_notifications{0};
    int resume_requests{0};
    int running_notifications{0};
    int stop_requests{0};
    int stopped_notifications{0};
    int error_notifications{0};
    rocos::motion::MotionResult last_error{rocos::motion::MotionResult::ok()};
};

class FailingPrepareCommand final : public rocos::motion::MotionCommand {
public:
    std::string name() const override { return "FailingPrepareCommand"; }

    rocos::motion::MotionResult prepare() override {
        return rocos::motion::MotionResult::failWithApiCode(
            rocos::motion::MotionResultCode::PlanningFailed,
            static_cast<int>(rocos::motion::DianaErrorCode::PlanMoveJ),
            "MoveJ planning failed");
    }

    rocos::motion::MotionResult start() override {
        return rocos::motion::MotionResult::ok();
    }

    rocos::motion::MotionStepResult update() override {
        return rocos::motion::MotionStepResult{
            rocos::motion::MotionStepStatus::Finished,
            rocos::motion::MotionResult::ok(),
            std::nullopt};
    }

    rocos::motion::MotionResult stop() override {
        return rocos::motion::MotionResult::ok();
    }
};

class ReferenceCommand final : public rocos::motion::MotionCommand {
public:
    explicit ReferenceCommand(rocos::motion::ReferenceSpace space)
        : space_(space) {}

    std::string name() const override { return "ReferenceCommand"; }

    rocos::motion::ReferenceSpace producedReferenceSpace() const override {
        return space_;
    }

    rocos::motion::MotionResult prepare() override {
        return rocos::motion::MotionResult::ok();
    }

    rocos::motion::MotionResult start() override {
        return rocos::motion::MotionResult::ok();
    }

    rocos::motion::MotionStepResult update() override {
        ++updates_;
        if (updates_ == 1) {
            rocos::motion::MotionReference reference;
            reference.space = space_;
            reference.joint.position = {0.1, -0.2};
            reference.joint.velocity = {0.01, -0.02};
            return rocos::motion::MotionStepResult{
                rocos::motion::MotionStepStatus::Running,
                rocos::motion::MotionResult::ok(),
                reference};
        }

        return rocos::motion::MotionStepResult{
            rocos::motion::MotionStepStatus::Finished,
            rocos::motion::MotionResult::ok(),
            std::nullopt};
    }

    rocos::motion::MotionResult stop() override {
        return rocos::motion::MotionResult::ok();
    }

private:
    rocos::motion::ReferenceSpace space_{rocos::motion::ReferenceSpace::None};
    int updates_{0};
};

class RecordingMotionContext final : public rocos::motion::MotionContext {
public:
    rocos::motion::RobotStateSnapshot readStateSnapshot() const override {
        return rocos::motion::RobotStateSnapshot{};
    }

    double controlPeriod() const override {
        return 0.001;
    }

    rocos::motion::MotionResult writeLowLevelCommand(
        const rocos::motion::LowLevelCommand& command) override {
        ++write_count;
        last_command = command;
        return write_result;
    }

    int write_count{0};
    rocos::motion::LowLevelCommand last_command;
    rocos::motion::MotionResult write_result{rocos::motion::MotionResult::ok()};
};

class RecordingController final : public rocos::motion::MotionController {
public:
    rocos::motion::ControllerType type() const override {
        return rocos::motion::ControllerType::Position;
    }

    rocos::motion::ReferenceSpace acceptedReferenceSpace() const override {
        return rocos::motion::ReferenceSpace::Joint;
    }

    rocos::motion::ComplianceSpace complianceSpace() const override {
        return rocos::motion::ComplianceSpace::None;
    }

    rocos::motion::MotionResult activate() override {
        ++activate_count;
        return activate_result;
    }

    rocos::motion::MotionResult deactivate() override {
        ++deactivate_count;
        return rocos::motion::MotionResult::ok();
    }

    rocos::motion::MotionResult update(
        const rocos::motion::MotionReference& reference,
        rocos::motion::LowLevelCommand& output) override {
        ++update_count;
        output.target_position = reference.joint.position;
        output.target_velocity = reference.joint.velocity;
        return rocos::motion::MotionResult::ok();
    }

    int activate_count{0};
    int deactivate_count{0};
    int update_count{0};
    rocos::motion::MotionResult activate_result{rocos::motion::MotionResult::ok()};
};

bool waitForStatus(rocos::motion::MotionExecutor& executor,
                   rocos::motion::MotionTaskStatus status,
                   int max_wait_ms = 1000) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(max_wait_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (executor.currentTaskStatus() == status) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

rocos::motion::MoveJCommand::Parameters makeExecutorMoveJParams() {
    rocos::motion::MoveJCommand::Parameters params;
    params.q_start = {0.0, 0.0};
    params.q_goal = {1.0, -1.0};
    params.max_velocity = {0.5, 0.5};
    params.max_acceleration = {1.0, 1.0};
    params.max_jerk = {4.0, 4.0};
    params.dt = 0.001;
    return params;
}

}  // namespace

TEST_CASE("MotionExecutor rejects null and busy submissions with Diana codes") {
    rocos::motion::MotionExecutor executor;

    const auto null_result = executor.submit(nullptr);
    CHECK_FALSE(null_result.success);
    CHECK(null_result.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::ParameterPointerEqualsNullptr));

    const auto first = executor.submit(std::make_unique<CountingCommand>(50));
    REQUIRE(first.success);
    CHECK(executor.currentTaskStatus() == rocos::motion::MotionTaskStatus::Running);

    const auto busy = executor.submit(std::make_unique<CountingCommand>(1));
    CHECK_FALSE(busy.success);
    CHECK(busy.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::ConflictTaskRunning));

    REQUIRE(executor.stop().success);
    CHECK(waitForStatus(executor, rocos::motion::MotionTaskStatus::Cancelled));
}

TEST_CASE("MotionExecutor records prepare failure and releases current command") {
    rocos::motion::MotionExecutor executor;

    const auto result = executor.submit(std::make_unique<FailingPrepareCommand>());
    CHECK_FALSE(result.success);
    CHECK(result.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::PlanMoveJ));
    CHECK(executor.currentTaskStatus() == rocos::motion::MotionTaskStatus::Failed);
    CHECK_FALSE(executor.hasActiveCommand());
    CHECK(executor.lastError().api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::PlanMoveJ));
}

TEST_CASE("MotionExecutor runs command to finished task status") {
    rocos::motion::MotionExecutor executor;

    const auto result = executor.submit(std::make_unique<CountingCommand>(3));
    REQUIRE(result.success);

    CHECK(waitForStatus(executor, rocos::motion::MotionTaskStatus::Finished));
    CHECK_FALSE(executor.hasActiveCommand());
    CHECK(executor.lastError().api_error_code == 0);
}

TEST_CASE("MotionExecutor keeps finite command active across pause and resume") {
    rocos::motion::MotionExecutor executor;

    auto command =
        std::make_unique<rocos::motion::MoveJCommand>(makeExecutorMoveJParams());
    const auto result = executor.submit(std::move(command));
    REQUIRE(result.success);

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    REQUIRE(executor.pause().success);
    CHECK(waitForStatus(executor, rocos::motion::MotionTaskStatus::Paused));
    CHECK(executor.hasActiveCommand());

    REQUIRE(executor.resume().success);
    CHECK(waitForStatus(executor, rocos::motion::MotionTaskStatus::Finished, 5000));
    CHECK_FALSE(executor.hasActiveCommand());
}

TEST_CASE("MotionExecutor asks FSM before preparing command") {
    RecordingFsmGateway fsm;
    fsm.start_result = rocos::motion::MotionResult::failWithApiCode(
        rocos::motion::MotionResultCode::InvalidState,
        static_cast<int>(rocos::motion::DianaErrorCode::NotAllAtOpState),
        "fsm rejected start");
    rocos::motion::MotionExecutor executor(fsm);

    auto stats = std::make_shared<TrackingStats>();
    auto command = std::make_unique<TrackingCommand>(stats);

    const auto result = executor.submit(std::move(command));

    CHECK_FALSE(result.success);
    CHECK(result.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::NotAllAtOpState));
    CHECK(fsm.start_requests == 1);
    CHECK(stats->prepare_count == 0);
    CHECK(stats->start_count == 0);
    CHECK_FALSE(executor.hasActiveCommand());
}

TEST_CASE("MotionExecutor sends FSM pause resume and stop events") {
    RecordingFsmGateway fsm;
    rocos::motion::MotionExecutor executor(fsm);

    const auto submit = executor.submit(std::make_unique<TrackingCommand>());
    REQUIRE(submit.success);
    CHECK(fsm.start_requests == 1);

    REQUIRE(executor.pause().success);
    CHECK(fsm.pause_requests == 1);
    CHECK(waitForStatus(executor, rocos::motion::MotionTaskStatus::Paused));
    CHECK(fsm.paused_notifications == 1);

    REQUIRE(executor.resume().success);
    CHECK(fsm.resume_requests == 1);
    CHECK(fsm.running_notifications == 1);

    REQUIRE(executor.stop().success);
    CHECK(fsm.stop_requests == 1);
    CHECK(waitForStatus(executor, rocos::motion::MotionTaskStatus::Cancelled));
    CHECK(fsm.stopped_notifications == 1);
}

TEST_CASE("MotionExecutor dispatches running reference through controller and context") {
    RecordingFsmGateway fsm;
    rocos::motion::PositionController controller;
    RecordingMotionContext context;
    rocos::motion::MotionExecutor executor(fsm, controller, context);

    const auto submit = executor.submit(
        std::make_unique<ReferenceCommand>(rocos::motion::ReferenceSpace::Joint));
    REQUIRE(submit.success);

    CHECK(waitForStatus(executor, rocos::motion::MotionTaskStatus::Finished));
    CHECK(context.write_count == 1);
    CHECK(context.last_command.target_position == std::vector<double>{0.1, -0.2});
    CHECK(context.last_command.target_velocity == std::vector<double>{0.01, -0.02});
}

TEST_CASE("MotionExecutor activates and deactivates active controller") {
    RecordingFsmGateway fsm;
    RecordingController controller;
    RecordingMotionContext context;
    rocos::motion::MotionExecutor executor(fsm, controller, context);

    const auto submit = executor.submit(
        std::make_unique<ReferenceCommand>(rocos::motion::ReferenceSpace::Joint));
    REQUIRE(submit.success);

    CHECK(waitForStatus(executor, rocos::motion::MotionTaskStatus::Finished));
    CHECK(controller.activate_count == 1);
    CHECK(controller.update_count == 1);
    CHECK(controller.deactivate_count == 1);
}

TEST_CASE("MotionExecutor rejects task when controller activation fails") {
    RecordingFsmGateway fsm;
    RecordingController controller;
    RecordingMotionContext context;
    controller.activate_result = rocos::motion::MotionResult::failWithApiCode(
        rocos::motion::MotionResultCode::HardwareFault,
        static_cast<int>(rocos::motion::DianaErrorCode::CommunicateError),
        "controller activation failed");
    rocos::motion::MotionExecutor executor(fsm, controller, context);

    const auto submit = executor.submit(
        std::make_unique<ReferenceCommand>(rocos::motion::ReferenceSpace::Joint));

    CHECK_FALSE(submit.success);
    CHECK(submit.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::CommunicateError));
    CHECK(executor.currentTaskStatus() == rocos::motion::MotionTaskStatus::Failed);
    CHECK(controller.activate_count == 1);
    CHECK(controller.deactivate_count == 0);
    CHECK(context.write_count == 0);
    CHECK(fsm.stopped_notifications == 1);
}

TEST_CASE("MotionExecutor rejects incompatible command and controller before start") {
    RecordingFsmGateway fsm;
    rocos::motion::PositionController controller;
    RecordingMotionContext context;
    rocos::motion::MotionExecutor executor(fsm, controller, context);

    const auto submit = executor.submit(
        std::make_unique<ReferenceCommand>(rocos::motion::ReferenceSpace::Cartesian));

    CHECK_FALSE(submit.success);
    CHECK(submit.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::MoveUnknown));
    CHECK(executor.currentTaskStatus() == rocos::motion::MotionTaskStatus::Failed);
    CHECK(context.write_count == 0);
    CHECK(executor.lastError().api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::MoveUnknown));
    CHECK(fsm.stopped_notifications == 1);
    CHECK(fsm.error_notifications == 0);
}

TEST_CASE("MotionExecutor fails task when context rejects low level command") {
    RecordingFsmGateway fsm;
    rocos::motion::PositionController controller;
    RecordingMotionContext context;
    context.write_result = rocos::motion::MotionResult::failWithApiCode(
        rocos::motion::MotionResultCode::SafetyViolation,
        static_cast<int>(rocos::motion::DianaErrorCode::SpeedLimit),
        "command velocity exceeds limit");
    rocos::motion::MotionExecutor executor(fsm, controller, context);

    const auto submit = executor.submit(
        std::make_unique<ReferenceCommand>(rocos::motion::ReferenceSpace::Joint));
    REQUIRE(submit.success);

    CHECK(waitForStatus(executor, rocos::motion::MotionTaskStatus::Failed));
    CHECK(context.write_count == 1);
    CHECK(executor.lastError().result ==
          rocos::motion::MotionResultCode::SafetyViolation);
    CHECK(executor.lastError().api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::SpeedLimit));
    CHECK(fsm.error_notifications == 1);
}
