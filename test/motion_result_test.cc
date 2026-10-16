// Native error paths and REST propagation use HardwareSim only.
#include <rocos_app/robot.h>
#include <rocos_app/robot_http_server.hpp>
#include <rocos_app/ethercat/hardware_sim.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <thread>

class TestSim : public rocos::HardwareSim {
public:
    std::atomic<bool> lose_enablement{false};
    rocos::DriveState getDriverState(int id) override {
        return lose_enablement && id == 0 ? rocos::DriveState::SwitchOnDisabled
                                         : HardwareSim::getDriverState(id);
    }
};

int main(int argc, char** argv) {
    using rocos::Result;
    auto check = [](bool value, const char* message) {
        if (!value) throw std::runtime_error(message);
    };
    try {
        check(argc == 2, "Expected URDF path");
        check(rocos::isSuccess(Result::NoError) && rocos::isSuccess(Result::PlanFinished), "Success classification");
        check(!rocos::isSuccess(Result::IkCalcFail) && !rocos::isSuccess(Result::FunctionNotSupported), "Error classification");
        check(rocos::resultCode(Result::PlanFinished) == 0, "PlanFinished must normalize to REST success");
        check(rocos::to_string(Result::JacobianCalcFail) == "JacobianCalcFail", "Missing error name");

        // The legacy controller owns process-lifetime threads; exit without
        // destroying objects while DriveGuard is still polling them.
        auto* hw = new TestSim;
        auto* robot = new rocos::Robot(hw, argv[1], "base_link", "link_7");
        KDL::JntArray q(robot->getJointNum());
        for (int i = 0; i < robot->getJointNum(); ++i) q(i) = robot->getJointPosition(i);
        auto expect = [&](int result, Result expected) { check(result == static_cast<int>(expected), "Incorrect native error code"); };
        expect(robot->MoveJ(KDL::JntArray(1), .2, .6, 0, 0), Result::UnmatchedJointsNumber);
        q(0) = std::numeric_limits<double>::quiet_NaN();
        expect(robot->MoveJ(q, .2, .6, 0, 0), Result::ParameterNanOrInf);
        q(0) = robot->getJointMaxPosition(0) + 1;
        expect(robot->MoveJ(q, .2, .6, 0, 0, true), Result::PosLimit);
        check(robot->getMotionReport().joint == 0, "Position error lost joint index");
        q(0) = 0;
        expect(robot->MoveJ(q, robot->getJointMaxVelocity(0) + 1, .6, 0, 0), Result::SpeedLimit);
        expect(robot->MoveJ(q, .2, robot->getJointMaxAcceleration(0) + 1, 0, 0), Result::AccLimit);
        expect(robot->MoveJ(q, .2, .6, 1, 0, true), Result::PlanInput);

        const int fd = socket(AF_INET, SOCK_STREAM, 0);
        check(fd >= 0, "socket failed");
        sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        check(bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0, "bind failed");
        socklen_t length = sizeof(addr);
        check(getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &length) == 0, "getsockname failed");
        const int port = ntohs(addr.sin_port); close(fd);
        rocos::RobotHttpServer server(robot);
        server.runAsync("127.0.0.1", port);
        httplib::Client client("127.0.0.1", port);
        client.set_read_timeout(3);
        const httplib::Headers headers{{"X-Rocos-Control-Token", "debug-token"}, {"X-Rocos-Client-Id", "debug-client"}};
        auto json = [&](const httplib::Result& response) {
            check(response && response->status == 200, "HTTP request failed");
            return nlohmann::json::parse(response->body);
        };
        auto state = [&]() { return json(client.Get("/api/robot/move_status"))["data"]; };
        auto terminal = [&]() {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            while (robot->isMotionBusy() && std::chrono::steady_clock::now() < deadline)
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            check(!robot->isMotionBusy(), "Motion failed to terminate");
            return state();
        };

        // A real unreachable target exercises TRAC-IK and captures its raw code.
        auto ik = json(client.Post("/api/robot/movej_ik", headers,
            R"({"pose":{"position":{"x":100,"y":100,"z":100},"orientation":{"w":1}},"velocity":0.2,"acceleration":0.6,"asynchronous":true})", "application/json"));
        check(!ik["success"] && ik["code"] == -2349 && ik["error_name"] == "IkCalcFail", "REST IK error mapping");
        check(ik["data"]["error"]["command"] == "movej_ik" && ik["data"]["error"].contains("native_code"), "IK native details missing");

        auto pose = json(client.Get("/api/robot/state"))["data"]["flange"];
        auto linear = json(client.Post("/api/robot/movel", headers,
            nlohmann::json{{"pose", pose}, {"velocity", .02}, {"acceleration", .1}, {"asynchronous", true}}.dump(), "application/json"));
        check(!linear["success"] && linear["code"] == -2318, "Degenerate linear planner failure mapping");
        pose["position"]["z"] = pose["position"]["z"].get<double>() + .005;
        linear = json(client.Post("/api/robot/movel", headers,
            nlohmann::json{{"pose", pose}, {"velocity", .02}, {"acceleration", .1}, {"asynchronous", true}}.dump(), "application/json"));
        check(!linear["success"] && linear["code"] == -2314, "Trajectory waypoint IK failure mapping");
        check(linear["data"]["error"].contains("waypoint") && linear["data"]["error"].contains("native_code"), "Trajectory IK metadata missing");

        // Fault injection uses native limit setters to reach a genuine async
        // execution path that HTTP validation would normally prevent.
        robot->setJntVelLimit(0, 0);
        q(0) = .2;
        expect(robot->MoveJ(q, .2, .6, 0, 0, true), Result::NoError);
        auto failure = terminal();
        check(failure["result"] == -2310 && failure["motion_outcome"] == "failed", "Async speed error lost");
        check(failure["error"]["joint"] == 0, "Async error joint missing");
        auto waited = json(client.Post("/api/robot/wait_move", headers, R"({"timeout":1})", "application/json"));
        check(!waited["success"] && waited["code"] == -2310 && waited["error_name"] == "SpeedLimit", "WaitMove lost async result");
        robot->setJntVelLimit(0, robot->getJointMaxVelocity(0));

        // Zero jerk reaches Ruckig::ErrorInvalidInput in the servo loop.
        robot->moveSingleAxis(0, .2, 0, .2, .6, 0, 0);
        failure = terminal();
        check(failure["result"] == -2316 && failure["error"]["native_code"] == -100, "Ruckig raw code was not mapped");
        check(failure["error"]["command"] == "single_axis", "Axis command missing");
        auto jerks = robot->getJntJerkLimits(); jerks[0] = robot->getJointMaxJerk(0); robot->setJntJerkLimits(jerks);

        // An impossible interpolation jerk reaches RunMoveJ's planner failure.
        jerks[0] = std::numeric_limits<double>::denorm_min(); robot->setJntJerkLimits(jerks);
        expect(robot->MoveJ(q, .2, .6, 0, 0, true), Result::NoError);
        failure = terminal();
        check(failure["result"] == -2317 && failure["error"]["stage"] == "planning", "Async MoveJ planning error mapping");
        jerks[0] = robot->getJointMaxJerk(0); robot->setJntJerkLimits(jerks);

        // Lost enablement during execution must not appear as completed.
        expect(robot->MoveJ(q, .1, .6, 0, 0, true), Result::NoError);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        hw->lose_enablement = true;
        failure = terminal();
        check(failure["result"] == -2212 && failure["motion_outcome"] == "failed", "Lost enablement reported as success");
        hw->lose_enablement = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        // New successful work clears old errors and does not reset a worker's
        // terminal error by publishing RUNNING after the worker starts.
        robot->moveSingleAxis(0, .04, 0, .2, .6, 5, 0);
        auto completed = terminal();
        check(completed["result"] == 0 && completed["error"].is_null() && completed["motion_outcome"] == "completed", "Error was not cleared for new motion");
        robot->moveSingleAxis(0, .5, 0, .1, .6, 5, 0);
        robot->requestMotionStop();
        auto stopped = terminal();
        check(stopped["result"] == 0 && stopped["motion_outcome"] == "stopped", "Stop incorrectly marked completed/failed");
        server.stop();
        std::cout << "PASS: native validation, IK, async planning, Ruckig mapping, enablement loss and REST terminal reports\n" << std::flush;
        std::_Exit(0);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n' << std::flush;
        std::_Exit(1);
    }
}
