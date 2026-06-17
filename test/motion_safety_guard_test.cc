#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>

#include <rocos_app/motion/motion_safety_guard.h>

namespace {

rocos::motion::MotionSafetyLimits makeLimits() {
    rocos::motion::MotionSafetyLimits limits;
    limits.min_position = {-2.0, -2.0};
    limits.max_position = {2.0, 2.0};
    limits.max_command_velocity = {1.0, 1.0};
    limits.max_following_error = {0.5, 0.5};
    return limits;
}

rocos::motion::RobotStateSnapshot makeActual() {
    rocos::motion::RobotStateSnapshot actual;
    actual.q_actual = {0.0, 0.0};
    actual.q_dot_actual = {0.0, 0.0};
    actual.enabled = true;
    actual.stamp = 1.0;
    return actual;
}

rocos::motion::LowLevelCommand positionCommand(double q0, double q1) {
    rocos::motion::LowLevelCommand command;
    command.target_position = {q0, q1};
    return command;
}

}  // namespace

TEST_CASE("MotionSafetyGuard accepts first valid position command") {
    rocos::motion::MotionSafetyGuard guard(makeLimits());
    const auto command = positionCommand(0.1, -0.1);

    const auto result = guard.check(command, makeActual(), 0.001);

    CHECK(result.ok);
    CHECK(result.code == rocos::motion::SafetyViolationCode::None);
    CHECK(result.api_error_code == 0);
}

TEST_CASE("MotionSafetyGuard rejects disabled robot before writing") {
    rocos::motion::MotionSafetyGuard guard(makeLimits());
    auto actual = makeActual();
    actual.enabled = false;

    const auto result = guard.check(positionCommand(0.0, 0.0), actual, 0.001);

    CHECK_FALSE(result.ok);
    CHECK(result.code == rocos::motion::SafetyViolationCode::NotEnabled);
    CHECK(result.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::NotAllAtOpState));
}

TEST_CASE("MotionSafetyGuard rejects invalid number and position limit") {
    rocos::motion::MotionSafetyGuard guard(makeLimits());

    auto invalid = positionCommand(std::numeric_limits<double>::quiet_NaN(), 0.0);
    const auto invalid_result = guard.check(invalid, makeActual(), 0.001);
    CHECK_FALSE(invalid_result.ok);
    CHECK(invalid_result.code == rocos::motion::SafetyViolationCode::InvalidNumber);
    CHECK(invalid_result.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::ParameterNanOrInf));

    const auto limit_result = guard.check(positionCommand(3.0, 0.0),
                                          makeActual(),
                                          0.001);
    CHECK_FALSE(limit_result.ok);
    CHECK(limit_result.code ==
          rocos::motion::SafetyViolationCode::PositionLimitExceeded);
    CHECK(limit_result.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::PosLimit));
}

TEST_CASE("MotionSafetyGuard uses accepted command history for velocity checks") {
    rocos::motion::MotionSafetyGuard guard(makeLimits());
    const auto actual = makeActual();

    const auto first = positionCommand(0.0, 0.0);
    REQUIRE(guard.check(first, actual, 0.001).ok);
    guard.accept(first);

    const auto too_fast = guard.check(positionCommand(0.01, 0.0),
                                      actual,
                                      0.001);
    CHECK_FALSE(too_fast.ok);
    CHECK(too_fast.code ==
          rocos::motion::SafetyViolationCode::CommandVelocityLimitExceeded);
    CHECK(too_fast.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::SpeedLimit));

    const auto still_compared_to_first = guard.check(positionCommand(0.001, 0.0),
                                                     actual,
                                                     0.001);
    CHECK(still_compared_to_first.ok);
}

TEST_CASE("MotionSafetyGuard rejects following error against actual state") {
    rocos::motion::MotionSafetyGuard guard(makeLimits());
    auto actual = makeActual();
    actual.q_actual = {0.0, 0.0};

    const auto result = guard.check(positionCommand(0.6, 0.0),
                                    actual,
                                    0.001);

    CHECK_FALSE(result.ok);
    CHECK(result.code ==
          rocos::motion::SafetyViolationCode::FollowingErrorExceeded);
    CHECK(result.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::NotFollowPositionCmd));
}

TEST_CASE("MotionSafetyGuard reset clears accepted command history") {
    rocos::motion::MotionSafetyGuard guard(makeLimits());
    const auto actual = makeActual();

    const auto first = positionCommand(0.0, 0.0);
    REQUIRE(guard.check(first, actual, 0.001).ok);
    guard.accept(first);
    guard.reset();

    const auto result = guard.check(positionCommand(0.01, 0.0),
                                    actual,
                                    0.001);

    CHECK(result.ok);
}
