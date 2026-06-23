#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>

#include <rocos_app/motion/motion_context.h>

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

class FakeMotionContext : public rocos::motion::GuardedMotionContext {
public:
    FakeMotionContext(rocos::motion::MotionSafetyGuard& guard,
                      rocos::motion::RobotStateSnapshot actual,
                      double period)
        : GuardedMotionContext(guard),
          actual_(std::move(actual)),
          period_(period) {}

    rocos::motion::RobotStateSnapshot readStateSnapshot() const override {
        return actual_;
    }

    double controlPeriod() const override {
        return period_;
    }

    int write_count() const {
        return write_count_;
    }

    const rocos::motion::LowLevelCommand& last_written() const {
        return last_written_;
    }

protected:
    rocos::motion::MotionResult writeCheckedLowLevelCommand(
        const rocos::motion::LowLevelCommand& command) override {
        ++write_count_;
        last_written_ = command;
        return rocos::motion::MotionResult::ok();
    }

private:
    rocos::motion::RobotStateSnapshot actual_;
    double period_{0.001};
    int write_count_{0};
    rocos::motion::LowLevelCommand last_written_;
};

}  // namespace

TEST_CASE("GuardedMotionContext writes command only after safety check passes") {
    rocos::motion::MotionSafetyGuard guard(makeLimits());
    FakeMotionContext context(guard, makeActual(), 0.001);

    const auto first = context.writeLowLevelCommand(positionCommand(0.0, 0.0));
    REQUIRE(first.success);
    CHECK(context.write_count() == 1);
    CHECK(context.last_written().target_position[0] == doctest::Approx(0.0));

    const auto second = context.writeLowLevelCommand(positionCommand(0.001, 0.0));
    CHECK(second.success);
    CHECK(context.write_count() == 2);
}

TEST_CASE("GuardedMotionContext does not write rejected command") {
    rocos::motion::MotionSafetyGuard guard(makeLimits());
    FakeMotionContext context(guard, makeActual(), 0.001);

    REQUIRE(context.writeLowLevelCommand(positionCommand(0.0, 0.0)).success);

    const auto too_fast = context.writeLowLevelCommand(positionCommand(0.01, 0.0));

    CHECK_FALSE(too_fast.success);
    CHECK(too_fast.result == rocos::motion::MotionResultCode::SafetyViolation);
    CHECK(too_fast.api_error_code ==
          static_cast<int>(rocos::motion::ErrorCode::SpeedLimit));
    CHECK(context.write_count() == 1);
}

TEST_CASE("GuardedMotionContext does not accept command when hardware write fails") {
    class FailingWriteContext final : public FakeMotionContext {
    public:
        FailingWriteContext(rocos::motion::MotionSafetyGuard& guard,
                            rocos::motion::RobotStateSnapshot actual,
                            double period)
            : FakeMotionContext(guard, std::move(actual), period) {}

    protected:
        rocos::motion::MotionResult writeCheckedLowLevelCommand(
            const rocos::motion::LowLevelCommand&) override {
            return rocos::motion::MotionResult::failWithApiCode(
                rocos::motion::MotionResultCode::HardwareFault,
                static_cast<int>(rocos::motion::ErrorCode::CommunicateError),
                "hardware write failed");
        }
    };

    rocos::motion::MotionSafetyGuard guard(makeLimits());
    FailingWriteContext failing_context(guard, makeActual(), 0.001);
    FakeMotionContext recovery_context(guard, makeActual(), 0.001);

    const auto failed =
        failing_context.writeLowLevelCommand(positionCommand(0.0, 0.0));
    CHECK_FALSE(failed.success);
    CHECK(failed.api_error_code ==
          static_cast<int>(rocos::motion::ErrorCode::CommunicateError));

    const auto not_compared_to_failed_command =
        recovery_context.writeLowLevelCommand(positionCommand(0.01, 0.0));
    CHECK(not_compared_to_failed_command.success);
}

TEST_CASE("GuardedMotionContext maps safety failure to MotionResult") {
    rocos::motion::MotionSafetyGuard guard(makeLimits());
    auto actual = makeActual();
    actual.enabled = false;
    FakeMotionContext context(guard, actual, 0.001);

    const auto result = context.writeLowLevelCommand(positionCommand(0.0, 0.0));

    CHECK_FALSE(result.success);
    CHECK(result.result == rocos::motion::MotionResultCode::SafetyViolation);
    CHECK(result.api_error_code ==
          static_cast<int>(rocos::motion::ErrorCode::NotAllAtOpState));
    CHECK(context.write_count() == 0);
}
