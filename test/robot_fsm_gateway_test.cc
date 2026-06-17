#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>

#include <rocos_app/motion/robot_fsm_gateway.h>

namespace {

class FakeRobotFsmClient {
public:
    bool requestMotionStart() {
        ++start_requests;
        return start_result;
    }

    bool requestMotionPause() {
        ++pause_requests;
        return pause_result;
    }

    bool requestMotionContinue() {
        ++continue_requests;
        return continue_result;
    }

    bool requestMotionStop() {
        ++stop_requests;
        return stop_result;
    }

    bool notifyMotionError() {
        ++error_notifications;
        return error_result;
    }

    int start_requests{0};
    int pause_requests{0};
    int continue_requests{0};
    int stop_requests{0};
    int error_notifications{0};
    bool start_result{true};
    bool pause_result{true};
    bool continue_result{true};
    bool stop_result{true};
    bool error_result{true};
};

}  // namespace

TEST_CASE("RobotFsmGateway forwards motion lifecycle requests") {
    FakeRobotFsmClient robot;
    rocos::motion::BasicRobotFsmGateway<FakeRobotFsmClient> gateway(robot);

    CHECK(gateway.requestStart().success);
    CHECK(gateway.requestPause().success);
    CHECK(gateway.requestResume().success);
    CHECK(gateway.requestStop().success);
    CHECK(gateway.notifyStopped().success);

    CHECK(robot.start_requests == 1);
    CHECK(robot.pause_requests == 1);
    CHECK(robot.continue_requests == 1);
    CHECK(robot.stop_requests == 2);
}

TEST_CASE("RobotFsmGateway maps rejected FSM requests to Diana API code") {
    FakeRobotFsmClient robot;
    robot.start_result = false;
    rocos::motion::BasicRobotFsmGateway<FakeRobotFsmClient> gateway(robot);

    const auto result = gateway.requestStart();

    CHECK_FALSE(result.success);
    CHECK(result.result == rocos::motion::MotionResultCode::InvalidState);
    CHECK(result.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::NotAllAtOpState));
}

TEST_CASE("RobotFsmGateway reports error event to robot FSM") {
    FakeRobotFsmClient robot;
    rocos::motion::BasicRobotFsmGateway<FakeRobotFsmClient> gateway(robot);

    const auto result = gateway.notifyError(
        rocos::motion::MotionResult::failWithApiCode(
            rocos::motion::MotionResultCode::SafetyViolation,
            static_cast<int>(rocos::motion::DianaErrorCode::SpeedLimit),
            "speed limit"));

    CHECK(result.success);
    CHECK(robot.error_notifications == 1);
}
