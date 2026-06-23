#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>

#include <rocos_app/motion/move_j_submission.h>
#include <rocos_app/motion/motion_context.h>
#include <rocos_app/motion/model_provider.h>

#include <memory>
#include <vector>

namespace {

class StubContext final : public rocos::motion::MotionContext {
public:
    rocos::motion::RobotStateSnapshot readStateSnapshot() const override {
        return rocos::motion::RobotStateSnapshot{};
    }
    double controlPeriod() const override { return 0.001; }
    rocos::motion::MotionResult writeLowLevelCommand(
        const rocos::motion::LowLevelCommand&) override {
        return rocos::motion::MotionResult::ok();
    }
};

class FakeRobotRuntime {
public:
    int getJointNum() const { return 2; }

    double getJointPosition(int id) const { return position[id]; }
    double getJntJerkLimit(int id) const { return jerk[id]; }

    std::vector<double> position{0.1, -0.2};
    std::vector<double> jerk{20.0, 30.0};
};

class RecordingExecutor {
public:
    rocos::motion::MotionResult submit(
        std::unique_ptr<rocos::motion::MotionCommand> command) {
        ++submit_count;
        last_command_name = command ? command->name() : "";
        last_reference_space =
            command ? command->producedReferenceSpace()
                    : rocos::motion::ReferenceSpace::None;
        if (!submit_result.success) {
            return submit_result;
        }
        if (command) {
            StubContext stub_ctx;
            rocos::motion::ModelProvider stub_model;
            command->prepare(stub_ctx, stub_model);
        }
        stored_command = std::move(command);
        return rocos::motion::MotionResult::ok();
    }

    int submit_count{0};
    std::string last_command_name;
    rocos::motion::ReferenceSpace last_reference_space{
        rocos::motion::ReferenceSpace::None};
    std::unique_ptr<rocos::motion::MotionCommand> stored_command;
    rocos::motion::MotionResult submit_result{rocos::motion::MotionResult::ok()};
};

}  // namespace

TEST_CASE("submitMoveJ builds MoveJCommand and submits it to executor") {
    FakeRobotRuntime robot;
    RecordingExecutor executor;

    const auto result = rocos::motion::submitMoveJ(
        robot, executor, {0.4, 0.2}, 0.5, 1.0, 0.001);

    CHECK(result.success);
    CHECK(executor.submit_count == 1);
    CHECK(executor.last_command_name == "MoveJCommand");
    CHECK(executor.last_reference_space == rocos::motion::ReferenceSpace::Joint);
}

TEST_CASE("submitMoveJ rejects target dimension mismatch with Diana API code") {
    FakeRobotRuntime robot;
    RecordingExecutor executor;

    const auto result = rocos::motion::submitMoveJ(
        robot, executor, {0.4}, 0.5, 1.0, 0.001);

    CHECK_FALSE(result.success);
    CHECK(result.result == rocos::motion::MotionResultCode::InvalidCommand);
    CHECK(result.api_error_code ==
          static_cast<int>(rocos::motion::ErrorCode::UnmatchedJointsNumber));
    CHECK(executor.submit_count == 0);
}

TEST_CASE("submitMoveJ rejects invalid scalar parameters with Diana API code") {
    FakeRobotRuntime robot;
    RecordingExecutor executor;

    const auto result = rocos::motion::submitMoveJ(
        robot, executor, {0.4, 0.2}, -0.5, 1.0, 0.001);

    CHECK_FALSE(result.success);
    CHECK(result.result == rocos::motion::MotionResultCode::InvalidCommand);
    CHECK(result.api_error_code ==
          static_cast<int>(rocos::motion::ErrorCode::IllegalParameter));
    CHECK(executor.submit_count == 0);
}

TEST_CASE("submitMoveJ propagates executor Diana API result") {
    FakeRobotRuntime robot;
    RecordingExecutor executor;
    executor.submit_result = rocos::motion::MotionResult::fail(
        rocos::motion::MotionResultCode::Busy,
        "busy");

    const auto result = rocos::motion::submitMoveJ(
        robot, executor, {0.4, 0.2}, 0.5, 1.0, 0.001);

    CHECK_FALSE(result.success);
    CHECK(result.api_error_code ==
          static_cast<int>(rocos::motion::ErrorCode::ConflictTaskRunning));
    CHECK(executor.submit_count == 1);
}
