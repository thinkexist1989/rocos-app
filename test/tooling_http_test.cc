#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "test/doctest.h"
#include "src/robot_http_server.hpp"
#include <filesystem>

namespace rocos {
struct RobotHttpServerTestAccess {
    static int start(RobotHttpServer& server, FrameRegistry& registry,
                     std::function<Result(ToolCalibrationSample&)> capture,
                     std::function<bool()> busy) {
        server.frame_registry_ = &registry;
        server.tcp_pose_ = [&registry, capture](Frame& tcp, const std::string& name) {
            ToolCalibrationSample sample;
            const auto result = capture(sample);
            return result == Result::NoError ? FrameRegistry::resolveTcp(*registry.snapshot(), sample.flange, tcp, name) : result;
        };
        server.state_snapshot_ = [&registry, capture] {
            Robot::RobotStateSnapshot state;
            ToolCalibrationSample sample;
            const auto read = capture(sample);
            const auto frames = registry.snapshot();
            state.flange = sample.flange;
            state.active_tool_frame_name = frames->active_tool;
            if (frames->tools.count(frames->active_tool)) state.active_tool_frame = frames->tools.at(frames->active_tool);
            state.tcp_valid = read == Result::NoError && FrameRegistry::resolveTcp(*frames, state.flange, state.tcp) == Result::NoError;
            return state;
        };
        server.calibration_capture_ = std::move(capture);
        server.tool_motion_busy_ = std::move(busy);
        int port = server.server_->bind_to_any_port("127.0.0.1");
        REQUIRE(port > 0);
        server.thread_ = std::make_unique<std::thread>([&] { server.server_->listen_after_bind(); });
        server.server_->wait_until_ready();
        return port;
    }
};
}
namespace {
using Json = nlohmann::json;
struct Fixture {
    std::filesystem::path directory;
    std::unique_ptr<rocos::FrameRegistry> registry;
    rocos::RobotHttpServer server{nullptr};
    std::unique_ptr<httplib::Client> client;
    httplib::Headers credentials;
    std::atomic<bool> busy{false};
    std::mutex capture_mutex;
    rocos::ToolCalibrationSample next;
    Fixture() {
        char pattern[] = "/tmp/rocos-tooling-http-XXXXXX";
        auto path = mkdtemp(pattern); REQUIRE(path);
        directory = path;
        registry = std::make_unique<rocos::FrameRegistry>((directory / "frames.yaml").string());
        const int port = rocos::RobotHttpServerTestAccess::start(server, *registry,
            [this](auto& sample) { std::lock_guard<std::mutex> lock(capture_mutex); sample = next; return rocos::Result::NoError; },
            [this] { return busy.load(); });
        client = std::make_unique<httplib::Client>("127.0.0.1", port);
        client->set_read_timeout(3);
        const auto owner = post("/api/control/acquire", {{"client_id", "test-tab"}}, false)["data"];
        credentials = {{"X-Rocos-Control-Token", owner["token"].get<std::string>()}, {"X-Rocos-Client-Id", "test-tab"}};
    }
    ~Fixture() { server.stop(); std::filesystem::remove_all(directory); }
    Json post(const std::string& path, const Json& body = Json::object(), bool auth = true) {
        const auto response = client->Post(path, auth ? credentials : httplib::Headers{}, body.dump(), "application/json");
        REQUIRE(response); CHECK(response->status == 200);
        return Json::parse(response->body);
    }
    Json get(const std::string& path) {
        const auto response = client->Get(path);
        REQUIRE(response); CHECK(response->status == 200); return Json::parse(response->body);
    }
    Json capture(int index, const KDL::Frame& frame, const std::string& session = "") {
        {
            std::lock_guard<std::mutex> lock(capture_mutex);
            next = {frame, {0.1, -0.2, 0.3}, 1792740000.0 + index};
        }
        Json body{{"index", index}, {"name", "TCP_calibrated"}};
        if (!session.empty()) body["session_id"] = session;
        // 客户端提交的伪造位姿不能覆盖控制器采集结果。
        body["pose"] = {{"x", 999}};
        return post("/api/calibration/pose", body);
    }
};
Json pose(double z = 0.15) {
    return {{"position", {{"x", 0}, {"y", 0}, {"z", z}}},
            {"orientation", {{"x", 0}, {"y", 0}, {"z", 0}, {"w", 1}}}};
}
}

TEST_CASE_FIXTURE(Fixture, "TCP queries are read only and state preserves the flange and fixed tool offset") {
    {
        std::lock_guard<std::mutex> lock(capture_mutex);
        next.flange = KDL::Frame(KDL::Rotation::RotZ(1.5707963267948966), KDL::Vector(1, 2, 3));
    }
    auto tcp = get("/api/robot/tcp");
    REQUIRE(tcp["success"] == true);
    CHECK(tcp["data"]["reference_frame"] == "base");
    CHECK(tcp["data"]["pose"]["position"]["y"] == 2);
    REQUIRE(post("/api/robot/tool_frame", {{"name", "tool1"}, {"frame", pose(0.1)}})["success"] == true);
    REQUIRE(post("/api/robot/active_tool_frame", {{"name", "tool1"}})["success"] == true);
    REQUIRE(post("/api/robot/tool_frame", {{"name", "other"}, {"frame", pose(0.2)}})["success"] == true);
    busy = true;
    CHECK(get("/api/robot/tcp")["data"]["pose"]["position"]["z"].get<double>() == doctest::Approx(3.1));
    CHECK(get("/api/robot/tcp?name=other")["data"]["pose"]["position"]["z"].get<double>() == doctest::Approx(3.2));
    const auto state = get("/api/robot/state")["data"];
    CHECK(state["active_tool_frame_name"] == "tool1");
    CHECK(state["flange"]["position"]["z"] == 3);
    CHECK(state["active_tool_frame"]["position"]["z"].get<double>() == doctest::Approx(0.1));
    CHECK(state["tcp"]["position"]["z"].get<double>() == doctest::Approx(3.1));
    CHECK(get("/api/robot/tcp?name=missing")["code"] == -3001);
    CHECK(get("/api/robot/tcp?name=bad%2Fname")["code"] == 1001);
    {
        std::lock_guard<std::mutex> lock(capture_mutex);
        next.flange.M.data[0] = 2;
    }
    CHECK(get("/api/robot/tcp")["code"] == static_cast<int>(rocos::Result::FkCalcFail));
    CHECK(get("/api/robot/state")["data"]["tcp"].is_null());
    CHECK(get("/api/robot/tool_frames")["data"]["active"] == "tool1");
}

TEST_CASE_FIXTURE(Fixture, "Tool APIs enforce credentials and persist current tool management") {
    CHECK(get("/api/robot/tool_frames")["data"]["names"].empty());
    CHECK(post("/api/robot/tool_frame", {{"name", "TCP"}, {"frame", pose()}}, false)["code"] == 3006);
    auto missing_client = credentials;
    missing_client.erase("X-Rocos-Client-Id");
    auto rejected = client->Post("/api/robot/tool_frame", missing_client, Json({{"name", "TCP"}, {"frame", pose()}}).dump(), "application/json");
    REQUIRE(rejected); CHECK(Json::parse(rejected->body)["code"] == 3007);
    REQUIRE(post("/api/robot/tool_frame", {{"name", "TCP"}, {"frame", pose()}, {"persist", true}})["success"] == true);
    CHECK(get("/api/robot/tool_frame?name=TCP")["data"]["frame"] == pose());
    REQUIRE(post("/api/robot/active_tool_frame", {{"name", "TCP"}, {"persist", true}})["success"] == true);
    rocos::FrameRegistry restored(registry->path());
    REQUIRE(restored.load() == rocos::Result::NoError);
    CHECK(restored.snapshot()->active_tool == "TCP");
    busy = true;
    CHECK(post("/api/robot/tool_frame", {{"name", "TCP"}, {"frame", pose(0.9)}})["code"] == 4005);
    CHECK(post("/api/robot/active_tool_frame", {{"name", ""}})["code"] == 4005);
    busy = false;
    CHECK(post("/api/robot/tool_frame", {{"name", "TCP"}, {"frame", "bad"}})["success"] == false);
    CHECK(post("/api/robot/tool_frame", {{"name", "TCP"}, {"frame", {{"position", {{"x", "bad"}}}, {"orientation", Json::object()}}}})["code"] == 1001);
    auto removed = client->Delete("/api/robot/tool_frame?name=TCP&persist=true", credentials);
    REQUIRE(removed); CHECK(Json::parse(removed->body)["success"] == true);
    CHECK(get("/api/robot/tool_frames")["data"]["active"] == "");
    REQUIRE(restored.load() == rocos::Result::NoError);
    CHECK(restored.names(true).empty());
}

TEST_CASE_FIXTURE(Fixture, "Six point HTTP capture calculate query and recapture keep result separate from tools") {
    const KDL::Vector tcp(0.01, -0.02, 0.18), pivot(0.5, 0.1, 0.4);
    std::array<KDL::Frame, 6> samples;
    samples[0].M = KDL::Rotation::RPY(0.5, 0.1, 0.3);
    samples[1].M = KDL::Rotation::RPY(-0.3, 0.6, 0.2);
    samples[2].M = KDL::Rotation::RPY(0.2, -0.4, -0.5);
    samples[3].M = KDL::Rotation::RPY(0, 0, 0);
    for (int i = 0; i < 4; ++i) samples[i].p = pivot - samples[i].M * tcp;
    samples[4] = samples[3]; samples[4].p = samples[4].p + KDL::Vector(0.1, 0, 0);
    samples[5] = samples[3]; samples[5].p = samples[5].p + KDL::Vector(0, 0, 0.1);
    CHECK(capture(2, samples[1])["code"] == 4001);
    auto first = capture(1, samples[0]);
    REQUIRE(first["success"] == true);
    const auto id = first["data"]["session_id"].get<std::string>();
    CHECK(first["data"]["samples"][0]["flange"]["position"]["x"].get<double>() < 1);
    CHECK(first["data"]["samples"][0]["joints"].size() == 3);
    CHECK(post("/api/calibration/run", {{"session_id", id}})["code"] == 4001);
    for (int i = 1; i < 6; ++i) REQUIRE(capture(i + 1, samples[i], id)["success"] == true);
    auto calculated = post("/api/calibration/run", {{"session_id", id}, {"max_error", 0.001}});
    REQUIRE(calculated["success"] == true);
    auto result = calculated["data"]["result"];
    CHECK(result["pose"]["position"]["z"].get<double>() == doctest::Approx(0.18));
    CHECK(result["metrics"]["max_error"].get<double>() < 1e-9);
    CHECK(get("/api/calibration/result?session_id=" + id)["data"]["result"] == result);
    CHECK(get("/api/robot/tool_frames")["data"]["names"].empty());
    REQUIRE(capture(1, samples[0], id)["success"] == true);
    CHECK(get("/api/calibration/result?session_id=" + id)["data"]["result"].is_null());
    CHECK(post("/api/calibration/run", {{"session_id", id}, {"max_error", -1}})["code"] == 1001);
    busy = true;
    CHECK(capture(1, samples[0], id)["code"] == 4005);
    busy = false;
    const auto new_owner = post("/api/control/takeover", {{"client_id", "new-tab"}}, false)["data"];
    credentials = {{"X-Rocos-Control-Token", new_owner["token"].get<std::string>()}, {"X-Rocos-Client-Id", "new-tab"}};
    CHECK(capture(2, samples[1], id)["code"] == 4004);
    CHECK(post("/api/calibration/run", {{"session_id", id}})["code"] == 4004);
}

TEST_CASE_FIXTURE(Fixture, "Six point API rejects degenerate captures without publishing a tool") {
    auto first = capture(1, KDL::Frame());
    REQUIRE(first["success"] == true);
    const auto id = first["data"]["session_id"].get<std::string>();
    for (int i = 2; i <= 6; ++i) REQUIRE(capture(i, KDL::Frame(), id)["success"] == true);
    CHECK(post("/api/calibration/run", {{"session_id", id}})["code"] == 4002);
    CHECK(get("/api/calibration/result?session_id=" + id)["data"]["result"].is_null());
    CHECK(get("/api/robot/tool_frames")["data"]["names"].empty());
}
