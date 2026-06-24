#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>

#include <rocos_app/ethercat/hardware_sim.h>
#include <rocos_app/robot.h>
#include <rocos_app/robot_http_server.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <httplib.h>
#include <json.hpp>
#include <kdl/jntarray.hpp>

#include <chrono>
#include <thread>

namespace {

KDL::JntArray makeTarget(int joint_count, double value) {
    KDL::JntArray target(joint_count);
    for (int i = 0; i < joint_count; ++i) {
        target(i) = value;
    }
    return target;
}

void waitForHttpServer(httplib::Client& client) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
        auto response = client.Get("/api/robot/state");
        if (response && response->status == 200) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    FAIL("HTTP server did not start");
}

}  // namespace

TEST_CASE("Robot MoveJ runs through executor path on HardwareSim") {
    auto* hardware = new rocos::HardwareSim(20);
    auto* robot = new rocos::Robot(hardware, "config/robot.urdf", "base_link", "link_7");

    REQUIRE(robot->getJointNum() == 7);
    REQUIRE(robot->GetRobotState() == "STOPPED");

    const auto target = makeTarget(robot->getJointNum(), 0.02);
    const int result = robot->MoveJ(target, 0.5, 1.0, 0.0, 0.0, false);

    CHECK(result == 0);
    CHECK(robot->GetRobotState() == "STOPPED");
    for (int i = 0; i < robot->getJointNum(); ++i) {
        CHECK(robot->getJointPosition(i) == doctest::Approx(0.02).epsilon(1e-4));
    }
}

TEST_CASE("Robot MoveJ returns Diana API error code on HardwareSim invalid target") {
    auto* hardware = new rocos::HardwareSim(20);
    auto* robot = new rocos::Robot(hardware, "config/robot.urdf", "base_link", "link_7");

    const auto target = makeTarget(robot->getJointNum(), 4.0);
    const int result = robot->MoveJ(target, 0.5, 1.0, 0.0, 0.0, false);

    CHECK(result == static_cast<int>(rocos::motion::ErrorCode::PosLimit));
    CHECK(robot->GetRobotState() == "STOPPED");
}

TEST_CASE("Robot MoveJ can pause resume and stop through motion executor on HardwareSim") {
    auto* hardware = new rocos::HardwareSim(20);
    auto* robot = new rocos::Robot(hardware, "config/robot.urdf", "base_link", "link_7");

    const auto target = makeTarget(robot->getJointNum(), 0.3);
    REQUIRE(robot->MoveJ(target, 0.05, 0.1, 0.0, 0.0, true) == 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    CHECK(robot->Pause() == 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    CHECK(robot->GetRobotState() == "PAUSED");

    CHECK(robot->Continue() == 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    CHECK(robot->GetRobotState() == "RUNNING");

    CHECK(robot->Stop() == 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    CHECK(robot->GetRobotState() == "STOPPED");
}

TEST_CASE("HTTP API can pause and resume MoveJ on HardwareSim") {
    auto* hardware = new rocos::HardwareSim(20);
    auto* robot = new rocos::Robot(hardware, "config/robot.urdf", "base_link", "link_7");
    rocos::RobotHttpServer server(robot);
    server.runAsync("127.0.0.1", 18080);

    httplib::Client client("127.0.0.1", 18080);
    waitForHttpServer(client);

    nlohmann::json move_body;
    move_body["joints"] = std::vector<double>(robot->getJointNum(), 0.3);
    move_body["speed"] = 0.05;
    move_body["acceleration"] = 0.1;
    move_body["asynchronous"] = true;

    auto move_response =
        client.Post("/api/move/joint", move_body.dump(), "application/json");
    REQUIRE(move_response);
    CHECK(move_response->status == 200);
    CHECK(nlohmann::json::parse(move_response->body)["success"].get<bool>());

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto pause_response = client.Post("/api/move/pause", "", "application/json");
    REQUIRE(pause_response);
    CHECK(pause_response->status == 200);
    auto pause_json = nlohmann::json::parse(pause_response->body);
    CHECK(pause_json["success"].get<bool>());
    CHECK(pause_json["code"].get<int>() == 0);

    auto resume_response = client.Post("/api/move/resume", "", "application/json");
    REQUIRE(resume_response);
    CHECK(resume_response->status == 200);
    auto resume_json = nlohmann::json::parse(resume_response->body);
    CHECK(resume_json["success"].get<bool>());
    CHECK(resume_json["code"].get<int>() == 0);

    auto stop_response = client.Post("/api/move/stop", "", "application/json");
    REQUIRE(stop_response);
    CHECK(stop_response->status == 200);
    auto stop_json = nlohmann::json::parse(stop_response->body);
    CHECK(stop_json["success"].get<bool>());
    CHECK(stop_json["code"].get<int>() == 0);
}

TEST_CASE("Robot does not take ownership of externally managed HardwareSim") {
    boost::shared_ptr<rocos::HardwareInterface> hardware =
        boost::make_shared<rocos::HardwareSim>(20);

    {
        rocos::Robot robot(
            hardware.get(), "config/robot.urdf", "base_link", "link_7");
        CHECK(robot.GetRobotState() == "STOPPED");
    }

    CHECK(hardware->getSlaveNumber() == 20);
}
