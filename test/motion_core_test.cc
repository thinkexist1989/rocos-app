#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>

#include <rocos_app/motion/diana_error_codes.h>
#include <rocos_app/motion/move_j_command.h>

#include <cmath>
#include <vector>

namespace {

constexpr double kDt = 0.001;

rocos::motion::MoveJCommand::Parameters makeMoveJParams() {
    rocos::motion::MoveJCommand::Parameters params;
    params.q_start = {0.0, 1.0, -0.5};
    params.q_goal = {1.0, -1.0, -0.5};
    params.max_velocity = {2.0, 4.0, 1.0};
    params.max_acceleration = {4.0, 8.0, 2.0};
    params.max_jerk = {20.0, 40.0, 10.0};
    params.dt = kDt;
    return params;
}

}  // namespace

TEST_CASE("Diana error mapper exposes external API codes") {
    using rocos::motion::DianaErrorCode;
    using rocos::motion::MotionResultCode;

    CHECK(rocos::motion::toDianaErrorCode(MotionResultCode::Ok) == 0);
    CHECK(rocos::motion::toDianaErrorCode(MotionResultCode::Busy) ==
          static_cast<int>(DianaErrorCode::ConflictTaskRunning));
    CHECK(rocos::motion::toDianaErrorCode(MotionResultCode::InvalidCommand) ==
          static_cast<int>(DianaErrorCode::IllegalParameter));
    CHECK(rocos::motion::toDianaErrorCode(MotionResultCode::PlanningFailed) ==
          static_cast<int>(DianaErrorCode::PlanError));
    CHECK(rocos::motion::toDianaErrorCode(MotionResultCode::SafetyViolation) ==
          static_cast<int>(DianaErrorCode::Fatal));
    CHECK(rocos::motion::toDianaErrorCode(MotionResultCode::HardwareFault) ==
          static_cast<int>(DianaErrorCode::JointRegistError));
}

TEST_CASE("MoveJCommand samples finite path references within configured limits") {
    rocos::motion::MoveJCommand command(makeMoveJParams());

    const auto prepare = command.prepare();
    REQUIRE(prepare.success);

    const auto start = command.start();
    REQUIRE(start.success);

    std::vector<rocos::motion::MotionReference> references;
    for (int i = 0; i < 10000; ++i) {
        const auto step = command.update();
        REQUIRE(step.result.success);
        if (step.reference) {
            references.push_back(*step.reference);
            const auto& joint = step.reference->joint;
            REQUIRE(joint.position.size() == 3);
            REQUIRE(joint.velocity.size() == 3);
            REQUIRE(joint.acceleration.size() == 3);
            CHECK(std::abs(joint.velocity[0]) <= 2.0 + 1e-9);
            CHECK(std::abs(joint.velocity[1]) <= 4.0 + 1e-9);
            CHECK(std::abs(joint.acceleration[0]) <= 4.0 + 1e-9);
            CHECK(std::abs(joint.acceleration[1]) <= 8.0 + 1e-9);
        }
        if (step.status == rocos::motion::MotionStepStatus::Finished) {
            break;
        }
    }

    REQUIRE_FALSE(references.empty());
    CHECK(command.status() == rocos::motion::MotionStepStatus::Finished);

    const auto& final_joint = references.back().joint;
    CHECK(final_joint.position[0] == doctest::Approx(1.0));
    CHECK(final_joint.position[1] == doctest::Approx(-1.0));
    CHECK(final_joint.position[2] == doctest::Approx(-0.5));
    CHECK(final_joint.velocity[0] == doctest::Approx(0.0));
    CHECK(final_joint.velocity[1] == doctest::Approx(0.0));
}

TEST_CASE("MoveJCommand pause resume and stop are handled by finite profile") {
    rocos::motion::MoveJCommand command(makeMoveJParams());
    REQUIRE(command.prepare().success);
    REQUIRE(command.start().success);

    for (int i = 0; i < 100; ++i) {
        REQUIRE(command.update().result.success);
    }

    REQUIRE(command.pause().success);
    for (int i = 0; i < 10000; ++i) {
        const auto step = command.update();
        REQUIRE(step.result.success);
        if (step.status == rocos::motion::MotionStepStatus::Paused) {
            break;
        }
    }
    REQUIRE(command.status() == rocos::motion::MotionStepStatus::Paused);

    REQUIRE(command.resume().success);
    for (int i = 0; i < 10000; ++i) {
        const auto step = command.update();
        REQUIRE(step.result.success);
        if (step.status == rocos::motion::MotionStepStatus::Finished) {
            break;
        }
    }
    REQUIRE(command.status() == rocos::motion::MotionStepStatus::Finished);

    rocos::motion::MoveJCommand stop_command(makeMoveJParams());
    REQUIRE(stop_command.prepare().success);
    REQUIRE(stop_command.start().success);
    for (int i = 0; i < 100; ++i) {
        REQUIRE(stop_command.update().result.success);
    }

    REQUIRE(stop_command.stop().success);
    for (int i = 0; i < 10000; ++i) {
        const auto step = stop_command.update();
        REQUIRE(step.result.success);
        if (step.status == rocos::motion::MotionStepStatus::Stopped) {
            break;
        }
    }
    CHECK(stop_command.status() == rocos::motion::MotionStepStatus::Stopped);
}

TEST_CASE("MoveJCommand rejects invalid parameters with Diana API error codes") {
    auto params = makeMoveJParams();
    params.q_goal[0] = std::numeric_limits<double>::quiet_NaN();

    rocos::motion::MoveJCommand command(params);
    const auto result = command.prepare();

    CHECK_FALSE(result.success);
    CHECK(result.result == rocos::motion::MotionResultCode::InvalidNumber);
    CHECK(result.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::ParameterNanOrInf));
}
