
#include "robot_http_server.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <kdl_parser/kdl_parser.hpp>
#include <sstream>

#include "lua_script_engine.hpp"
#include "robot.hpp"

namespace rocos {

namespace {

int resultCode(Result result) {
    return static_cast<int>(result);
}

bool resultSucceeded(Result result) {
    return result == Result::NoError || result == Result::PlanFinished;
}

const char* scriptResultMessage(Result result) {
    switch (result) {
        case Result::LuaStateConflict:
            return "script state conflict";
        case Result::LuaExecutionError:
            return "Lua script compilation or execution failed";
        case Result::LuaInvalidBreakpoint:
            return "invalid script breakpoint";
        case Result::LuaFileError:
            return "script file unavailable";
        case Result::LuaStopped:
            return "script stopped";
        default:
            return "script operation failed";
    }
}

double jsonNumberOr(const nlohmann::json& body,
                    const char* primary_key,
                    const char* fallback_key,
                    double default_value) {
    if (body.contains(primary_key)) {
        return body[primary_key].get<double>();
    }
    if (fallback_key != nullptr && body.contains(fallback_key)) {
        return body[fallback_key].get<double>();
    }
    return default_value;
}

bool isFinite(double value) {
    return std::isfinite(value);
}

bool directionSign(const std::string& direction, double& sign) {
    if (direction == "POSITIVE") {
        sign = 1.0;
        return true;
    }
    if (direction == "NEGATIVE") {
        sign = -1.0;
        return true;
    }
    if (direction == "NONE") {
        sign = 0.0;
        return true;
    }
    return false;
}

bool parseJogFrame(const std::string& frame, Robot::JogFrame& jog_frame) {
    if (frame == "BASE") {
        jog_frame = Robot::JogFrame::BASE;
        return true;
    }
    if (frame == "FLANGE") {
        jog_frame = Robot::JogFrame::FLANGE;
        return true;
    }
    if (frame == "TOOL") {
        jog_frame = Robot::JogFrame::TOOL;
        return true;
    }
    if (frame == "OBJECT") {
        jog_frame = Robot::JogFrame::OBJECT;
        return true;
    }
    return false;
}


bool parseFrameJson(const nlohmann::json& j, KDL::Frame& frame) {
    if (!j.is_object()) return false;

    if (j.contains("position") && j.contains("orientation") &&
        j["position"].is_object() && j["orientation"].is_object()) {
        const auto& pos = j["position"];
        const auto& rot = j["orientation"];
        const double x = pos.value("x", 0.0);
        const double y = pos.value("y", 0.0);
        const double z = pos.value("z", 0.0);
        const double qx = rot.value("x", 0.0);
        const double qy = rot.value("y", 0.0);
        const double qz = rot.value("z", 0.0);
        const double qw = rot.value("w", 1.0);
        if (!isFinite(x) || !isFinite(y) || !isFinite(z) ||
            !isFinite(qx) || !isFinite(qy) || !isFinite(qz) || !isFinite(qw)) {
            return false;
        }
        frame = KDL::Frame(KDL::Rotation::Quaternion(qx, qy, qz, qw),
                           KDL::Vector(x, y, z));
        return true;
    }

    const double x = j.value("x", 0.0);
    const double y = j.value("y", 0.0);
    const double z = j.value("z", 0.0);
    const double qx = j.value("qx", 0.0);
    const double qy = j.value("qy", 0.0);
    const double qz = j.value("qz", 0.0);
    const double qw = j.value("qw", 1.0);
    if (!isFinite(x) || !isFinite(y) || !isFinite(z) ||
        !isFinite(qx) || !isFinite(qy) || !isFinite(qz) || !isFinite(qw)) {
        return false;
    }
    frame = KDL::Frame(KDL::Rotation::Quaternion(qx, qy, qz, qw),
                       KDL::Vector(x, y, z));
    return true;
}

bool parseNamedFrameBody(const nlohmann::json& body,
                         std::string& name,
                         KDL::Frame& frame) {
    if (body.is_discarded() || !body.contains("name") || !body["name"].is_string()) {
        return false;
    }
    const nlohmann::json* frame_json = nullptr;
    if (body.contains("frame")) {
        frame_json = &body["frame"];
    } else if (body.contains("pose")) {
        frame_json = &body["pose"];
    }
    if (frame_json == nullptr) return false;

    name = body["name"].get<std::string>();
    return parseFrameJson(*frame_json, frame);
}

}  // namespace

// ============================================================================
// Construction / Destruction
// ============================================================================

RobotHttpServer::RobotHttpServer(Robot* robot, LuaScriptEngine* script_engine)
    : robot_(robot), script_engine_(script_engine), server_(new httplib::Server()), taskCounter_(0),
      poolShutdown_(false), activeWorkers_(0) {

    log_ptr_ = Logger::getInstance("HttpServer");

    // Start worker thread pool
    for (int i = 0; i < MAX_WORKERS; ++i) {
        workerThreads_.emplace_back([this]() {
            while (true) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(poolMutex_);
                    poolCv_.wait(lock, [this]() {
                        return poolShutdown_ || !taskQueue_.empty();
                    });
                    if (poolShutdown_ && taskQueue_.empty()) return;
                    if (taskQueue_.empty()) continue;
                    task = std::move(taskQueue_.front());
                    taskQueue_.pop();
                }
                ++activeWorkers_;
                try {
                    task();
                } catch (...) {
                    log_ptr_->error("Async task threw exception");
                }
                --activeWorkers_;
            }
        });
    }
    registerRoutes();
}

RobotHttpServer::~RobotHttpServer() {
    {
        std::lock_guard<std::mutex> lock(poolMutex_);
        poolShutdown_ = true;
    }
    poolCv_.notify_all();
    for (auto& t : workerThreads_) {
        if (t.joinable()) t.join();
    }
    stop();
}

void RobotHttpServer::submitTask(std::function<void()> func) {
    {
        std::lock_guard<std::mutex> lock(poolMutex_);
        taskQueue_.push(std::move(func));
    }
    poolCv_.notify_one();
}

// ============================================================================
// Server Lifecycle
// ============================================================================

void RobotHttpServer::run(const std::string& host, int port) {
    log_ptr_->info("HTTP Server listening on {}:{}", host, port);
    server_->listen(host, port);
}

void RobotHttpServer::runAsync(const std::string& host, int port) {
    thread_.reset(new std::thread([this, host, port]() {
        this->run(host, port);
    }));
}

void RobotHttpServer::stop() {
    if (server_) server_->stop();
    if (thread_ && thread_->joinable()) thread_->join();
}

// ============================================================================
// Route Registration
// ============================================================================

void RobotHttpServer::registerRoutes() {
    if (!server_->set_mount_point("/", "web") &&
        !server_->set_mount_point("/", "../web") &&
        !server_->set_mount_point("/", "../../web")) {
        log_ptr_->error("Failed to mount web UI directory. Tried: web, ../web, ../../web");
    }

    // CORS preflight
    server_->Options(".*", [this](const httplib::Request&, httplib::Response& res) {
        setCorsHeaders(res);
    });

    // Robot state
    server_->Get("/api/robot/state", [this](auto& req, auto& res) { handleGetRobotState(req, res); });
    server_->Get("/api/robot/info", [this](auto& req, auto& res) { handleGetRobotInfo(req, res); });
    server_->Get("/api/robot/urdf", [this](auto& req, auto& res) { handleGetRobotModel(req, res); });
    server_->Get("/api/robot/urdf/mesh", [this](auto& req, auto& res) { handleGetLinkMesh(req, res); });
    server_->Get("/api/robot/enabled", [this](auto& req, auto& res) { handleIsEnabled(req, res); });
    server_->Post("/api/robot/enable", [this](auto& req, auto& res) { handleEnable(req, res); });
    server_->Post("/api/robot/disable", [this](auto& req, auto& res) { handleDisable(req, res); });
    server_->Post("/api/robot/workmode", [this](auto& req, auto& res) { handleSetWorkMode(req, res); });

    // Robot motion control
    server_->Post("/api/robot/movej", [this](auto& req, auto& res) { handleMoveJ(req, res); });
    server_->Post("/api/robot/movej_ik", [this](auto& req, auto& res) { handleMoveJ_IK(req, res); });
    server_->Post("/api/robot/movel", [this](auto& req, auto& res) { handleMoveL(req, res); });
    server_->Post("/api/robot/movel_fk", [this](auto& req, auto& res) { handleMoveL_FK(req, res); });
    server_->Post("/api/robot/movec", [this](auto& req, auto& res) { handleMoveC(req, res); });
    server_->Post("/api/robot/pause", [this](auto& req, auto& res) { handlePause(req, res); });
    server_->Post("/api/robot/resume", [this](auto& req, auto& res) { handleResume(req, res); });
    server_->Post("/api/robot/stop", [this](auto& req, auto& res) { handleStop(req, res); });
    server_->Post("/api/robot/wait_move", [this](auto& req, auto& res) { handleWaitMove(req, res); });
    server_->Get("/api/robot/move_status", [this](auto& req, auto& res) { handleMoveStatus(req, res); });

    // Robot jogging — 全部向量传递
    server_->Post("/api/robot/jog/joint", [this](auto& req, auto& res) { handleJogJoint(req, res); });
    server_->Post("/api/robot/jog/cartesian", [this](auto& req, auto& res) { handleJogCartesian(req, res); });
    server_->Post("/api/robot/jog/nullspace", [this](auto& req, auto& res) { handleJogNullspace(req, res); });
    server_->Post("/api/robot/jog/svd", [this](auto& req, auto& res) { handleJogSvd(req, res); });
    server_->Post("/api/robot/jog/stop", [this](auto& req, auto& res) { handleJogStop(req, res); });

    // Legacy jogging aliases — 上位机仍在使用 flag 格式
    server_->Post("/api/drag/start", [this](auto& req, auto& res) { handleJogCompat(req, res); });
    server_->Post("/api/drag/stop", [this](auto& req, auto& res) { handleJogStop(req, res); });
    server_->Post("/api/robot/jog", [this](auto& req, auto& res) { handleJogCompat(req, res); });

    // Legacy motion aliases kept for existing clients during API migration.
    server_->Post("/api/move/joint", [this](auto& req, auto& res) { handleMoveJ(req, res); });
    server_->Post("/api/move/joint_ik", [this](auto& req, auto& res) { handleMoveJ_IK(req, res); });
    server_->Post("/api/move/linear", [this](auto& req, auto& res) { handleMoveL(req, res); });
    server_->Post("/api/move/linear_fk", [this](auto& req, auto& res) { handleMoveL_FK(req, res); });
    server_->Post("/api/move/circle", [this](auto& req, auto& res) { handleMoveC(req, res); });
    server_->Post("/api/move/pause", [this](auto& req, auto& res) { handlePause(req, res); });
    server_->Post("/api/move/resume", [this](auto& req, auto& res) { handleResume(req, res); });
    server_->Post("/api/move/stop", [this](auto& req, auto& res) { handleStop(req, res); });
    server_->Get("/api/move/status", [this](auto& req, auto& res) { handleMoveStatus(req, res); });

    // Frame management
    server_->Get("/api/robot/tool_frames", [this](auto& req, auto& res) { handleGetToolFrameNames(req, res); });
    server_->Get("/api/robot/object_frames", [this](auto& req, auto& res) { handleGetObjectFrameNames(req, res); });
    server_->Get("/api/robot/tool_frame", [this](auto& req, auto& res) { handleGetToolFrame(req, res); });
    server_->Get("/api/robot/object_frame", [this](auto& req, auto& res) { handleGetObjectFrame(req, res); });
    server_->Post("/api/robot/tool_frame", [this](auto& req, auto& res) { handleSetToolFrame(req, res); });
    server_->Post("/api/robot/object_frame", [this](auto& req, auto& res) { handleSetObjectFrame(req, res); });
    server_->Delete("/api/robot/tool_frame", [this](auto& req, auto& res) { handleRemoveToolFrame(req, res); });
    server_->Delete("/api/robot/object_frame", [this](auto& req, auto& res) { handleRemoveObjectFrame(req, res); });
    server_->Post("/api/robot/active_tool_frame", [this](auto& req, auto& res) { handleSetActiveToolFrame(req, res); });
    server_->Post("/api/robot/active_object_frame", [this](auto& req, auto& res) { handleSetActiveObjectFrame(req, res); });
    server_->Post("/api/robot/frames/load", [this](auto& req, auto& res) { handleLoadFrames(req, res); });
    server_->Post("/api/robot/frames/save", [this](auto& req, auto& res) { handleSaveFrames(req, res); });

    // Calibration
    server_->Post("/api/calibration/pose", [this](auto& req, auto& res) { handleSetPoseFrame(req, res); });
    server_->Post("/api/calibration/tool", [this](auto& req, auto& res) { handleSetToolFrame(req, res); });
    server_->Post("/api/calibration/object", [this](auto& req, auto& res) { handleSetObjectFrame(req, res); });
    server_->Post("/api/calibration/run", [this](auto& req, auto& res) { handleCalibrationRun(req, res); });
    server_->Get("/api/calibration/result", [this](auto& req, auto& res) { handleCalibrationResult(req, res); });

    // Lua script
    server_->Post("/api/script/upload", [this](auto& req, auto& res) { handleScriptUpload(req, res); });
    server_->Post("/api/script/run", [this](auto& req, auto& res) { handleScriptRun(req, res); });
    server_->Post("/api/script/pause", [this](auto& req, auto& res) { handleScriptPause(req, res); });
    server_->Post("/api/script/resume", [this](auto& req, auto& res) { handleScriptResume(req, res); });
    server_->Post("/api/script/stop", [this](auto& req, auto& res) { handleScriptStop(req, res); });
    server_->Post("/api/script/step", [this](auto& req, auto& res) { handleScriptStep(req, res); });
    server_->Get("/api/script/status", [this](auto& req, auto& res) { handleScriptStatus(req, res); });
    server_->Post("/api/script/breakpoint/add", [this](auto& req, auto& res) { handleScriptBreakpointAdd(req, res); });
    server_->Post("/api/script/breakpoint/remove", [this](auto& req, auto& res) { handleScriptBreakpointRemove(req, res); });
    server_->Post("/api/script/breakpoint/clear", [this](auto& req, auto& res) { handleScriptBreakpointClear(req, res); });

    
	    // 修改后：去掉 ".*",
	    server_->set_post_routing_handler([](const httplib::Request&, httplib::Response& res) {
	    if (res.body.empty() && (res.status == 0 || res.status == 404)) {
	        res.status = 404;
        // 按照设计文档，业务错误码使用 4 位编码；这里统一返回通用参数错误码 1001
        res.set_content(R"({"success":false,"code":1001,"message":"endpoint not found","data":null})", "application/json");
    }
});
}

// ============================================================================
// Utility: CORS
// ============================================================================

void RobotHttpServer::setCorsHeaders(httplib::Response& res) {
    res.set_header("Access-Control-Allow-Origin", "*");
    res.set_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS");
    res.set_header("Access-Control-Allow-Headers", "Content-Type");
}

// ============================================================================
// Utility: JSON Response
// ============================================================================

void RobotHttpServer::sendJson(httplib::Response& res, bool success,
                               int businessCode, const std::string& message,
                               const nlohmann::json& data) {
    setCorsHeaders(res);
    res.set_header("Content-Type", "application/json");

    nlohmann::json body;
    body["success"] = success;
    body["code"] = businessCode;
    body["message"] = message;
    body["data"] = data;

    res.set_content(body.dump(), "application/json");
}

void RobotHttpServer::sendScriptResult(httplib::Response& res, Result result,
                                       const std::string& success_message) {
    const bool success = result == Result::NoError;
    sendJson(res, success, resultCode(result),
             success ? success_message : scriptResultMessage(result),
             scriptStatusToJson());
}

nlohmann::json RobotHttpServer::scriptStatusToJson() const {
    if (script_engine_ == nullptr) {
        return nlohmann::json();
    }

    const LuaScriptEngine::Status status = script_engine_->GetStatus();
    nlohmann::json breakpoints = nlohmann::json::array();
    for (const auto& breakpoint : status.breakpoints) {
        breakpoints.push_back({{"filename", breakpoint.filename},
                               {"line", breakpoint.line}});
    }

    return {{"state", LuaScriptEngine::ToString(status.state)},
            {"script_id", status.script_id},
            {"filename", status.location.filename},
            {"line", status.location.line},
            {"error", status.error},
            {"motion_active", status.motion_active},
            {"breakpoints", std::move(breakpoints)}};
}

// ============================================================================
// Utility: Frame <-> JSON Conversion
// ============================================================================

KDL::Frame RobotHttpServer::jsonToFrame(const nlohmann::json& j) {
    double px = j.value("x", 0.0);
    double py = j.value("y", 0.0);
    double pz = j.value("z", 0.0);
    double qx = j.value("qx", 0.0);
    double qy = j.value("qy", 0.0);
    double qz = j.value("qz", 0.0);
    double qw = j.value("qw", 1.0);

    return KDL::Frame(
        KDL::Rotation::Quaternion(qx, qy, qz, qw),
        KDL::Vector(px, py, pz));
}

KDL::Frame RobotHttpServer::jsonToFrame(const nlohmann::json& pos,
                                        const nlohmann::json& rot) {
    KDL::Vector p(pos.value("x", 0.0),
                  pos.value("y", 0.0),
                  pos.value("z", 0.0));

    KDL::Rotation r = KDL::Rotation::Quaternion(
        rot.value("x", 0.0),
        rot.value("y", 0.0),
        rot.value("z", 0.0),
        rot.value("w", 1.0));

    return KDL::Frame(r, p);
}

std::string RobotHttpServer::getUrdfPath() {
    return "robot.urdf";
}

nlohmann::json RobotHttpServer::frameToJson(const KDL::Frame& frame) {
    double x, y, z, w;
    frame.M.GetQuaternion(x, y, z, w);

    nlohmann::json position;
    position["x"] = frame.p.x();
    position["y"] = frame.p.y();
    position["z"] = frame.p.z();

    nlohmann::json orientation;
    orientation["x"] = x;
    orientation["y"] = y;
    orientation["z"] = z;
    orientation["w"] = w;

    nlohmann::json j;
    j["position"] = position;
    j["orientation"] = orientation;

    return j;
}

// ============================================================================
// Utility: Async Task Management
// ============================================================================

std::string RobotHttpServer::generateTaskId() {
    std::lock_guard<std::mutex> lock(taskMutex_);
    ++taskCounter_;
    return "task_" + std::to_string(taskCounter_);
}

void RobotHttpServer::registerTask(const std::string& taskId, const std::string& type) {
    std::lock_guard<std::mutex> lock(taskMutex_);
    nlohmann::json info;
    info["task_id"] = taskId;
    info["type"] = type;
    info["status"] = "RUNNING";
    info["result"] = 0;
    info["message"] = "";
    taskMap_[taskId] = info;
}

void RobotHttpServer::updateTaskStatus(const std::string& taskId, const std::string& status,
                                       int result, const std::string& message) {
    std::lock_guard<std::mutex> lock(taskMutex_);
    auto it = taskMap_.find(taskId);
    if (it != taskMap_.end()) {
        it->second["status"] = status;
        it->second["result"] = result;
        it->second["message"] = message;
    }
}

nlohmann::json RobotHttpServer::getTaskInfo(const std::string& taskId) {
    std::lock_guard<std::mutex> lock(taskMutex_);
    auto it = taskMap_.find(taskId);
    if (it != taskMap_.end()) {
        return it->second;
    }
    return nlohmann::json();
}

void RobotHttpServer::cleanExpiredTasks() {
    std::lock_guard<std::mutex> lock(taskMutex_);
    auto now = std::chrono::steady_clock::now();
    for (auto it = taskMap_.begin(); it != taskMap_.end(); ) {
        auto& info = it->second;
        std::string status = info.value("status", "");
        if (status == "COMPLETED" || status == "FAILED" || status == "STOPPED") {
            if (info.contains("_finish_time")) {
                auto finish = std::chrono::steady_clock::time_point(
                    std::chrono::milliseconds(info["_finish_time_ms"].get<long long>()));
                if (std::chrono::duration_cast<std::chrono::seconds>(now - finish).count() > TASK_TTL_SECONDS) {
                    it = taskMap_.erase(it);
                    continue;
                }
            }
        }
        ++it;
    }
}

// ============================================================================
// Robot State Handlers
// ============================================================================

void RobotHttpServer::handleGetRobotState(const httplib::Request& req, httplib::Response& res) {
    auto snap = robot_->GetRobotStateSnapshot();

    nlohmann::json data;
    data["robot_state"] = snap.state_string;
    data["is_enabled"] = snap.is_enabled;
    data["is_running"] = snap.is_running;
    data["control_active"] = snap.control_active;
    data["motion_busy"] = snap.motion_busy;
    data["timestamp"] = snap.timestamp;

    // 关节状态
    nlohmann::json joint_states = nlohmann::json::array();
    for (const auto& js : snap.joints) {
        nlohmann::json j;
        j["id"] = js.id;
        j["name"] = js.name;
        j["status"] = js.status;
        j["position"] = js.position;
        j["velocity"] = js.velocity;
        j["torque"] = js.torque;
        j["load_torque"] = js.load_torque;
        joint_states.push_back(j);
    }
    data["joint_states"] = joint_states;

    // 法兰末端位姿
    data["flange"] = frameToJson(snap.flange);

    // 激活的坐标系
    data["active_tool_frame_name"] = snap.active_tool_frame_name;
    data["active_tool_frame"] = frameToJson(snap.active_tool_frame);
    data["active_object_frame_name"] = snap.active_object_frame_name;
    data["active_object_frame"] = frameToJson(snap.active_object_frame);

    // 硬件摘要
    nlohmann::json hw;
    hw["joint_num"] = snap.joint_num;
    hw["state"] = snap.hardware_state;
    data["hw_state"] = hw;

    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleGetRobotInfo(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/robot/info");

    nlohmann::json joint_infos = nlohmann::json::array();
    for (const auto& info : robot_->GetJointInfo()) {
        nlohmann::json ji;
        ji["id"] = info.id;
        ji["name"] = info.name;
        ji["cnt_per_unit"] = info.cnt_per_unit;
        ji["torque_per_unit"] = info.torque_per_unit;
        ji["ratio"] = info.ratio;
        ji["unit_name"] = info.unit_name;
        ji["zero_offset"] = info.zero_offset;
        joint_infos.push_back(ji);
    }

    nlohmann::json data;
    data["joint_infos"] = joint_infos;

    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleGetRobotModel(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/robot/urdf");

    const std::string urdf_path = getUrdfPath();
    std::ifstream file(urdf_path);
    if (!file.is_open()) {
        log_ptr_->error("Failed to open URDF file: {}", urdf_path);
        sendJson(res, false, 1001, "URDF file not found: " + urdf_path);
        return;
    }

    std::string content((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());

    setCorsHeaders(res);
    res.set_content(content, "text/xml");
}

void RobotHttpServer::handleGetLinkMesh(const httplib::Request& req, httplib::Response& res) {
    namespace fs = std::filesystem;

    std::string path = req.get_param_value("path");
    log_ptr_->info("GET /api/robot/urdf/mesh path={}", path);

    if (path.empty()) {
        sendJson(res, false, 1001, "Missing 'path' query parameter");
        return;
    }

    // 解析 package:// ROS 路径格式：package://pkg_name/relative/path.stl
    // 剥离 scheme + package 名，保留相对路径部分（如 meshes/link1.stl）
    std::string rel_path = path;
    if (path.rfind("package://", 0) == 0) {
        const auto after_pkg = path.find('/', 10); // 跳过 "package://"
        if (after_pkg == std::string::npos) {
            sendJson(res, false, 1001, "Invalid package:// path: " + path);
            return;
        }
        rel_path = path.substr(after_pkg + 1); // e.g., "meshes/link1.stl"
    }

    // 禁止路径穿越攻击
    if (rel_path.find("..") != std::string::npos) {
        sendJson(res, false, 1001, "Path traversal not allowed");
        return;
    }

    // 提取文件名，用于回退搜索
    const auto sep = rel_path.find_last_of("/\\");
    const std::string filename = (sep != std::string::npos)
        ? rel_path.substr(sep + 1)
        : rel_path;

    // 候选搜索根目录：当前目录、上级目录、上上级目录
    const std::vector<fs::path> search_bases = {".", "..", "../.." };

    std::string found_path;

    // 1. 先尝试 base/rel_path 精确匹配
    for (const auto& base : search_bases) {
        fs::path candidate = base / rel_path;
        std::error_code ec;
        if (fs::is_regular_file(candidate, ec)) {
            found_path = candidate.string();
            break;
        }
    }

    // 2. 精确匹配失败时，按文件名在各目录树中递归搜索
    if (found_path.empty()) {
        for (const auto& base : search_bases) {
            std::error_code ec;
            if (!fs::is_directory(base, ec)) continue;
            for (const auto& entry :
                 fs::recursive_directory_iterator(base,
                     fs::directory_options::skip_permission_denied, ec)) {
                if (entry.is_regular_file() &&
                    entry.path().filename().string() == filename) {
                    found_path = entry.path().string();
                    break;
                }
            }
            if (!found_path.empty()) break;
        }
    }

    if (found_path.empty()) {
        log_ptr_->error("Mesh file not found: {}", path);
        sendJson(res, false, 1001, "Mesh file not found: " + filename);
        return;
    }

    std::ifstream file(found_path, std::ios::binary);
    if (!file.is_open()) {
        log_ptr_->error("Failed to open mesh file: {}", found_path);
        sendJson(res, false, 1001, "Failed to open mesh file");
        return;
    }

    std::ostringstream ss;
    ss << file.rdbuf();
    const std::string content = ss.str();

    log_ptr_->info("Serving mesh: {}", found_path);
    setCorsHeaders(res);
    res.set_header("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    res.set_content(content, "application/octet-stream");
}

// ============================================================================
// Robot Control Handlers
// ============================================================================

void RobotHttpServer::handleEnable(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/enable");
    const Result result = robot_->SetEnabled();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["enabled"] = robot_->IsEnabled();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Robot enabled" : "Robot enable failed",
             data);
}

void RobotHttpServer::handleDisable(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/disable");
    const Result result = robot_->SetDisabled();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["enabled"] = robot_->IsEnabled();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Robot disabled" : "Robot disable failed",
             data);
}

void RobotHttpServer::handleIsEnabled(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/robot/enabled");
    nlohmann::json data;
    data["enabled"] = robot_->IsEnabled();
    data["disabled"] = robot_->IsDisabled();
    data["robot_state"] = robot_->GetStateString();
    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleSetWorkMode(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/workmode");

    // nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    // if (body.is_discarded() || !body.contains("mode")) {
    //     sendJson(res, false, 1001, "Invalid JSON or missing 'mode' field");
    //     return;
    // }
    //
    // std::string mode_str = body["mode"].get<std::string>();
    // Robot::WorkMode mode;
    //
    // if (mode_str == "position") {
    //     mode = Robot::WorkMode::Position;
    // } else if (mode_str == "ee_admit_teach") {
    //     mode = Robot::WorkMode::EeAdmitTeach;
    // } else if (mode_str == "jnt_admit_teach") {
    //     mode = Robot::WorkMode::JntAdmitTeach;
    // } else if (mode_str == "jnt_imp") {
    //     mode = Robot::WorkMode::JntImp;
    // } else if (mode_str == "cart_imp") {
    //     mode = Robot::WorkMode::CartImp;
    // } else {
    //     sendJson(res, false, 1006, "Unknown work mode: " + mode_str);
    //     return;
    // }
    //
    // bool ok = robot_->setWorkMode(mode);
    // if (ok) {
    //     sendJson(res, true, 0, "Work mode set to " + mode_str);
    // } else {
    //     sendJson(res, false, 3002, "Failed to set work mode");
    // }
}

// ============================================================================
// Motion Control Handlers
// ============================================================================

void RobotHttpServer::handleMoveJ(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/movej");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("joints") ||
        !body["joints"].is_array()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'joints' array");
        return;
    }

    const int joint_count = robot_->getJointNum();
    const auto& joints = body["joints"];
    if (static_cast<int>(joints.size()) != joint_count) {
        sendJson(res, false, 1003,
                 "Joint count mismatch: expected " + std::to_string(joint_count));
        return;
    }

    JntArray q_goal(static_cast<unsigned int>(joint_count));
    for (int i = 0; i < joint_count; ++i) {
        if (!joints[i].is_number()) {
            sendJson(res, false, 1001, "Joint values must be numbers");
            return;
        }
        const double value = joints[i].get<double>();
        if (!isFinite(value)) {
            sendJson(res, false, 1001, "Joint values must be finite");
            return;
        }
        q_goal(static_cast<unsigned int>(i)) = value;
    }

    const double velocity = jsonNumberOr(body, "velocity", "speed", 1.0);
    const double acceleration = jsonNumberOr(body, "acceleration", nullptr, 2.0);
    const double jerk = jsonNumberOr(body, "jerk", nullptr, 10.0);
    if (!isFinite(velocity) || !isFinite(acceleration) || !isFinite(jerk) ||
        velocity <= 0.0 || acceleration <= 0.0 || jerk <= 0.0) {
        sendJson(res, false, 1001, "velocity, acceleration and jerk must be positive finite numbers");
        return;
    }

    const Result result = robot_->MoveJ(q_goal, velocity, acceleration, jerk);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "MoveJ accepted" : "MoveJ failed",
             data);
}

void RobotHttpServer::handleMoveJ_IK(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/movej_ik");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("pose") ||
        !body["pose"].is_object()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'pose' object");
        return;
    }
    const auto& pose_json = body["pose"];
    Frame pose;
    if (pose_json.contains("position") && pose_json.contains("orientation")) {
        pose = jsonToFrame(pose_json["position"], pose_json["orientation"]);
    } else {
        pose = jsonToFrame(pose_json);
    }

    const double velocity = jsonNumberOr(body, "velocity", "speed", 1.0);
    const double acceleration = jsonNumberOr(body, "acceleration", nullptr, 2.0);
    const double jerk = jsonNumberOr(body, "jerk", nullptr, 10.0);
    if (!isFinite(velocity) || !isFinite(acceleration) || !isFinite(jerk) ||
        velocity <= 0.0 || acceleration <= 0.0 || jerk <= 0.0) {
        sendJson(res, false, 1001, "velocity, acceleration and jerk must be positive finite numbers");
        return;
    }

    const Result result = robot_->MoveJ_IK(pose, velocity, acceleration, jerk);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "MoveJ_IK accepted" : "MoveJ_IK failed",
             data);
}

void RobotHttpServer::handleMoveL(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/movel");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("pose") ||
        !body["pose"].is_object()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'pose' object");
        return;
    }

    const auto& pose_json = body["pose"];
    Frame pose;
    if (pose_json.contains("position") && pose_json.contains("orientation")) {
        pose = jsonToFrame(pose_json["position"], pose_json["orientation"]);
    } else {
        pose = jsonToFrame(pose_json);
    }

    const std::string tool_name = body.value("tool_name", std::string{});
    const double velocity = jsonNumberOr(body, "velocity", "speed", 1.0);
    const double acceleration = jsonNumberOr(body, "acceleration", nullptr, 2.0);
    const double jerk = jsonNumberOr(body, "jerk", nullptr, 10.0);
    if (!isFinite(velocity) || !isFinite(acceleration) || !isFinite(jerk) ||
        velocity <= 0.0 || acceleration <= 0.0 || jerk <= 0.0) {
        sendJson(res, false, 1001, "velocity, acceleration and jerk must be positive finite numbers");
        return;
    }

    const Result result = robot_->MoveL(pose, tool_name, velocity, acceleration, jerk);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "MoveL accepted" : "MoveL failed",
             data);
}

void RobotHttpServer::handleMoveL_FK(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/movel_fk");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("joints") ||
        !body["joints"].is_array()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'joints' array");
        return;
    }
    const int joint_count = robot_->getJointNum();
    const auto& joints = body["joints"];
    if (static_cast<int>(joints.size()) != joint_count) {
        sendJson(res, false, 1003,
                 "Joint count mismatch: expected " + std::to_string(joint_count));
        return;
    }

    JntArray q_goal(static_cast<unsigned int>(joint_count));
    for (int i = 0; i < joint_count; ++i) {
        if (!joints[i].is_number()) {
            sendJson(res, false, 1001, "Joint values must be numbers");
            return;
        }
        const double value = joints[i].get<double>();
        if (!isFinite(value)) {
            sendJson(res, false, 1001, "Joint values must be finite");
            return;
        }
        q_goal(static_cast<unsigned int>(i)) = value;
    }

    const double velocity = jsonNumberOr(body, "velocity", "speed", 1.0);
    const double acceleration = jsonNumberOr(body, "acceleration", nullptr, 2.0);
    const double jerk = jsonNumberOr(body, "jerk", nullptr, 10.0);
    if (!isFinite(velocity) || !isFinite(acceleration) || !isFinite(jerk) ||
        velocity <= 0.0 || acceleration <= 0.0 || jerk <= 0.0) {
        sendJson(res, false, 1001, "velocity, acceleration and jerk must be positive finite numbers");
        return;
    }

    const std::string tool_name = body.value("tool_name", std::string{});
    const Result result = robot_->MoveL_FK(q_goal, tool_name, velocity, acceleration, jerk);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "MoveL_FK accepted" : "MoveL_FK failed",
             data);
}

void RobotHttpServer::handleMoveC(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/movec");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded()) {
        sendJson(res, false, 1001, "Invalid JSON body");
        return;
    }

    Frame pose_start = robot_->getFlange();
    if (body.contains("pose_start") && !parseFrameJson(body["pose_start"], pose_start)) {
        sendJson(res, false, 1001, "Invalid 'pose_start' frame");
        return;
    }

    const double velocity = jsonNumberOr(body, "velocity", "speed", 1.0);
    const double acceleration = jsonNumberOr(body, "acceleration", nullptr, 2.0);
    const double jerk = jsonNumberOr(body, "jerk", nullptr, 10.0);
    if (!isFinite(velocity) || !isFinite(acceleration) || !isFinite(jerk) ||
        velocity <= 0.0 || acceleration <= 0.0 || jerk <= 0.0) {
        sendJson(res, false, 1001, "velocity, acceleration and jerk must be positive finite numbers");
        return;
    }

    Result result = Result::IllegalParameter;
    if (body.contains("center_frame") && body.contains("theta")) {
        Frame center_frame;
        if (!parseFrameJson(body["center_frame"], center_frame) || !body["theta"].is_number()) {
            sendJson(res, false, 1001, "Invalid 'center_frame' or 'theta'");
            return;
        }
        const double theta = body["theta"].get<double>();
        if (!isFinite(theta)) {
            sendJson(res, false, 1001, "theta must be finite");
            return;
        }
        result = robot_->MoveC(pose_start, center_frame, theta, velocity, acceleration, jerk);
    } else if (body.contains("pose_via") && (body.contains("pose_goal") || body.contains("pose_to"))) {
        Frame pose_via;
        Frame pose_goal;
        const auto& goal_json = body.contains("pose_goal") ? body["pose_goal"] : body["pose_to"];
        if (!parseFrameJson(body["pose_via"], pose_via) || !parseFrameJson(goal_json, pose_goal)) {
            sendJson(res, false, 1001, "Invalid 'pose_via' or 'pose_goal' frame");
            return;
        }
        result = robot_->MoveC(pose_start, pose_via, pose_goal, velocity, acceleration, jerk);
    } else {
        sendJson(res, false, 1001,
                 "MoveC requires either 'center_frame' + 'theta' or 'pose_via' + 'pose_goal'");
        return;
    }

    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "MoveC accepted" : "MoveC failed",
             data);
}

void RobotHttpServer::handleStop(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/stop");

    const Result result = robot_->StopMotion();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Motion stop requested" : "Motion stop failed",
             data);
}

void RobotHttpServer::handlePause(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/pause");

    const Result result = robot_->PauseMotion();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Motion pause requested" : "Motion pause failed",
             data);
}

void RobotHttpServer::handleResume(const httplib::Request& req, httplib::Response& res) {
  log_ptr_->info("POST /api/robot/resume");

    const Result result = robot_->ResumeMotion();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Motion resume requested" : "Motion resume failed",
             data);
}

void RobotHttpServer::handleWaitMove(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/wait_move");

    const Result result = robot_->WaitMove();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    data["motion_busy"] = robot_->IsMotionBusy();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Motion finished" : "WaitMove failed",
             data);
}

// ============================================================================
// Async Task Query
// ============================================================================

void RobotHttpServer::handleMoveStatus(const httplib::Request& req, httplib::Response& res) {
    std::string task_id = req.get_param_value("task_id");
    log_ptr_->info("GET /api/robot/move_status task_id={}", task_id);

    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["is_running"] = robot_->IsRunning();
    data["control_active"] = robot_->IsControlActive();
    if (!task_id.empty()) {
        data["task_id"] = task_id;
        data["task"] = getTaskInfo(task_id);
    }
    sendJson(res, true, 0, "ok", data);
}

// ============================================================================
// Lua Script Handlers
// ============================================================================

void RobotHttpServer::handleScriptUpload(const httplib::Request& req,
                                         httplib::Response& res) {
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }

    const nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("filename") ||
        !body["filename"].is_string() || !body.contains("source") ||
        !body["source"].is_string()) {
        sendJson(res, false, 1001,
                 "Invalid JSON or missing string 'filename'/'source' fields");
        return;
    }

    const Result result = script_engine_->LoadSource(
        body["source"].get<std::string>(), body["filename"].get<std::string>());
    sendScriptResult(res, result, "script uploaded");
}

void RobotHttpServer::handleScriptRun(const httplib::Request&,
                                      httplib::Response& res) {
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }
    sendScriptResult(res, script_engine_->Run(), "script started");
}

void RobotHttpServer::handleScriptPause(const httplib::Request&,
                                        httplib::Response& res) {
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }
    sendScriptResult(res, script_engine_->Pause(), "script pause requested");
}

void RobotHttpServer::handleScriptResume(const httplib::Request&,
                                         httplib::Response& res) {
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }
    sendScriptResult(res, script_engine_->Resume(), "script resumed");
}

void RobotHttpServer::handleScriptStop(const httplib::Request&,
                                       httplib::Response& res) {
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }
    sendScriptResult(res, script_engine_->Stop(), "script stop requested");
}

void RobotHttpServer::handleScriptStep(const httplib::Request&,
                                       httplib::Response& res) {
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }
    sendScriptResult(res, script_engine_->Step(), "script step requested");
}

void RobotHttpServer::handleScriptStatus(const httplib::Request&,
                                         httplib::Response& res) {
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }
    sendJson(res, true, 0, "ok", scriptStatusToJson());
}

void RobotHttpServer::handleScriptBreakpointAdd(const httplib::Request& req,
                                                httplib::Response& res) {
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }

    const nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("filename") ||
        !body["filename"].is_string() || !body.contains("line") ||
        !body["line"].is_number_integer()) {
        sendJson(res, false, 1001,
                 "Invalid JSON or missing 'filename'/'line' fields");
        return;
    }

    const Result result = script_engine_->AddBreakpoint(
        body["filename"].get<std::string>(), body["line"].get<int>());
    sendScriptResult(res, result, "breakpoint added");
}

void RobotHttpServer::handleScriptBreakpointRemove(const httplib::Request& req,
                                                   httplib::Response& res) {
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }

    const nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("filename") ||
        !body["filename"].is_string() || !body.contains("line") ||
        !body["line"].is_number_integer()) {
        sendJson(res, false, 1001,
                 "Invalid JSON or missing 'filename'/'line' fields");
        return;
    }

    const Result result = script_engine_->RemoveBreakpoint(
        body["filename"].get<std::string>(), body["line"].get<int>());
    sendScriptResult(res, result, "breakpoint removed");
}

void RobotHttpServer::handleScriptBreakpointClear(const httplib::Request&,
                                                  httplib::Response& res) {
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }
    sendScriptResult(res, script_engine_->ClearBreakpoints(),
                     "breakpoints cleared");
}


// ============================================================================
// Jogging Compatibility Handler — 旧 flag 格式转换
// ============================================================================

void RobotHttpServer::handleJogCompat(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/jog (compat flag mode)");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("flag") || !body["flag"].is_string() ||
        !body.contains("direction") || !body["direction"].is_string()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'flag'/'direction' fields");
        return;
    }

    const std::string flag = body["flag"].get<std::string>();
    const std::string direction = body["direction"].get<std::string>();
    const double speed = jsonNumberOr(body, "max_speed", "speed", 1.0);
    const double timeout = jsonNumberOr(body, "timeout", nullptr, 0.1);
    const double dir_threshold = jsonNumberOr(body, "dir_threshold", nullptr, 0.99);

    double sign = 0.0;
    if (!directionSign(direction, sign)) {
        sendJson(res, false, 1001, "Invalid direction");
        return;
    }

    Result result = Result::IllegalParameter;
    const int joint_count = robot_->getJointNum();

    if (sign == 0.0 || flag == "NONE") {
        result = robot_->StopMotion();
    } else if (flag.size() == 2 && flag[0] == 'J' && flag[1] >= '0' && flag[1] <= '9') {
        // 关节点动 flag → 向量
        const int joint_index = flag[1] - '0';
        if (joint_index >= joint_count) {
            sendJson(res, false, 1003,
                     "Joint index out of range: expected 0.." + std::to_string(joint_count - 1));
            return;
        }
        JntArray joint_direction(static_cast<unsigned int>(joint_count));
        joint_direction(static_cast<unsigned int>(joint_index)) = sign;
        result = robot_->MoveJogging(joint_direction, speed, timeout, dir_threshold);
    } else if (flag == "NULLSPACE") {
        // 零空间点动 flag → 向量
        const auto vector_json = body.contains("joints") ? body["joints"]
            : body.value("direction_vector", nlohmann::json::array());
        if (!vector_json.is_array() || static_cast<int>(vector_json.size()) != joint_count) {
            sendJson(res, false, 1001,
                     "NULLSPACE requires 'joints' array with " + std::to_string(joint_count) + " elements");
            return;
        }
        JntArray intent_direction(static_cast<unsigned int>(joint_count));
        for (int i = 0; i < joint_count; ++i) {
            if (!vector_json[i].is_number() || !isFinite(vector_json[i].get<double>())) {
                sendJson(res, false, 1001, "Invalid NULLSPACE direction values");
                return;
            }
            intent_direction(static_cast<unsigned int>(i)) = sign * vector_json[i].get<double>();
        }
        result = robot_->MoveNullJogging(intent_direction, speed, timeout, dir_threshold);
    } else {
        // 笛卡尔点动 flag → Twist
        const auto separator = flag.rfind('_');
        if (separator == std::string::npos) {
            sendJson(res, false, 1001, "Invalid jogging flag: " + flag);
            return;
        }
        const std::string frame = flag.substr(0, separator);
        const std::string axis = flag.substr(separator + 1);
        if (frame != "TOOL" && frame != "FLANGE" && frame != "OBJECT" && frame != "BASE") {
            sendJson(res, false, 1001, "Invalid frame in flag: " + flag);
            return;
        }

        Twist twist;
        if (axis == "X") twist.vel.x(sign);
        else if (axis == "Y") twist.vel.y(sign);
        else if (axis == "Z") twist.vel.z(sign);
        else if (axis == "ROLL") twist.rot.x(sign);
        else if (axis == "PITCH") twist.rot.y(sign);
        else if (axis == "YAW") twist.rot.z(sign);
        else {
            sendJson(res, false, 1001, "Invalid axis in flag: " + flag);
            return;
        }

        result = robot_->MoveJogging(twist, speed, timeout, dir_threshold);
    }

    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Dragging started" : "Dragging start failed",
             data);
}

// ============================================================================
// Jogging Handlers
// ============================================================================

void RobotHttpServer::handleJogJoint(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/jog/joint");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("joints") || !body["joints"].is_array()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'joints' array");
        return;
    }

    const int joint_count = robot_->getJointNum();
    const auto& joints_json = body["joints"];
    if (static_cast<int>(joints_json.size()) != joint_count) {
        sendJson(res, false, 1003,
                 "Joint count mismatch: expected " + std::to_string(joint_count));
        return;
    }

    JntArray joint_direction(static_cast<unsigned int>(joint_count));
    for (int i = 0; i < joint_count; ++i) {
        if (!joints_json[i].is_number()) {
            sendJson(res, false, 1001, "Joint direction values must be numbers");
            return;
        }
        const double value = joints_json[i].get<double>();
        if (!isFinite(value)) {
            sendJson(res, false, 1001, "Joint direction values must be finite");
            return;
        }
        joint_direction(static_cast<unsigned int>(i)) = value;
    }

    double sign = 1.0;
    if (body.contains("direction") && body["direction"].is_string()) {
        if (!directionSign(body["direction"].get<std::string>(), sign) || sign == 0.0) {
            sendJson(res, false, 1001, "Invalid direction");
            return;
        }
    }
    if (sign != 1.0) {
        for (unsigned int i = 0; i < joint_direction.rows(); ++i) {
            joint_direction(i) *= sign;
        }
    }

    const double speed = jsonNumberOr(body, "speed", nullptr, 1.0);
    const double timeout = jsonNumberOr(body, "timeout", nullptr, 0.1);
    const double dir_threshold = jsonNumberOr(body, "dir_threshold", nullptr, 0.99);
    if (!isFinite(speed) || !isFinite(timeout) || !isFinite(dir_threshold) ||
        speed <= 0.0 || timeout <= 0.0 || dir_threshold < -1.0 || dir_threshold > 1.0) {
        sendJson(res, false, 1001, "Invalid speed, timeout or direction threshold");
        return;
    }

    const Result result = robot_->MoveJogging(joint_direction, speed, timeout, dir_threshold);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Joint jogging started" : "Joint jogging failed",
             data);
}

void RobotHttpServer::handleJogCartesian(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/jog/cartesian");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("twist") || !body["twist"].is_array()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'twist' array");
        return;
    }

    const auto& twist_json = body["twist"];
    if (twist_json.size() != 6) {
        sendJson(res, false, 1001, "Twist must have exactly 6 elements");
        return;
    }

    Twist twist;
    for (size_t i = 0; i < 6; ++i) {
        if (!twist_json[i].is_number()) {
            sendJson(res, false, 1001, "Twist values must be numbers");
            return;
        }
        const double value = twist_json[i].get<double>();
        if (!isFinite(value)) {
            sendJson(res, false, 1001, "Twist values must be finite");
            return;
        }
        if (i == 0) twist.vel.x(value);
        else if (i == 1) twist.vel.y(value);
        else if (i == 2) twist.vel.z(value);
        else if (i == 3) twist.rot.x(value);
        else if (i == 4) twist.rot.y(value);
        else twist.rot.z(value);
    }

    double sign = 1.0;
    if (body.contains("direction") && body["direction"].is_string()) {
        if (!directionSign(body["direction"].get<std::string>(), sign) || sign == 0.0) {
            sendJson(res, false, 1001, "Invalid direction");
            return;
        }
    }
	    if (sign != 1.0) {
	        twist.vel = twist.vel * sign;
	        twist.rot = twist.rot * sign;
	    }

        Robot::JogFrame jog_frame = Robot::JogFrame::BASE;
        if (body.contains("frame")) {
            if (!body["frame"].is_string() ||
                !parseJogFrame(body["frame"].get<std::string>(), jog_frame)) {
                sendJson(res, false, 1001, "Invalid jog frame: expected BASE, FLANGE, TOOL or OBJECT");
                return;
            }
        }

    const double speed = jsonNumberOr(body, "speed", nullptr, 1.0);
    const double timeout = jsonNumberOr(body, "timeout", nullptr, 0.1);
    const double dir_threshold = jsonNumberOr(body, "dir_threshold", nullptr, 0.99);
    if (!isFinite(speed) || !isFinite(timeout) || !isFinite(dir_threshold) ||
        speed <= 0.0 || timeout <= 0.0 || dir_threshold < -1.0 || dir_threshold > 1.0) {
	        sendJson(res, false, 1001, "Invalid speed, timeout or direction threshold");
	        return;
    }

    const Result result = robot_->MoveJogging(twist, jog_frame, speed, timeout, dir_threshold);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Cartesian jogging started" : "Cartesian jogging failed",
             data);
}

void RobotHttpServer::handleJogNullspace(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/jog/nullspace");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded()) {
        sendJson(res, false, 1001, "Invalid JSON body");
        return;
    }
    const auto vector_json = body.contains("joints") ? body["joints"] : body.value("direction_vector", nlohmann::json::array());
    if (!vector_json.is_array()) {
        sendJson(res, false, 1001, "NULLSPACE jogging requires 'joints' array");
        return;
    }

    double sign = 1.0;
    if (body.contains("direction")) {
        if (!body["direction"].is_string() ||
            !directionSign(body["direction"].get<std::string>(), sign) || sign == 0.0) {
            sendJson(res, false, 1001, "Invalid direction");
            return;
        }
    }

    const double speed = jsonNumberOr(body, "max_speed", "speed", 1.0);
    const double timeout = jsonNumberOr(body, "timeout", nullptr, 0.1);
    const double dir_threshold = jsonNumberOr(body, "dir_threshold", nullptr, 0.99);
    if (!isFinite(speed) || !isFinite(timeout) || !isFinite(dir_threshold) ||
        speed <= 0.0 || timeout <= 0.0 || dir_threshold < -1.0 || dir_threshold > 1.0) {
        sendJson(res, false, 1001, "Invalid speed, timeout or direction threshold");
        return;
    }

    const int joint_count = robot_->getJointNum();
    if (static_cast<int>(vector_json.size()) != joint_count) {
        sendJson(res, false, 1003,
                 "Joint count mismatch: expected " + std::to_string(joint_count));
        return;
    }

    JntArray intent_direction(static_cast<unsigned int>(joint_count));
    for (int i = 0; i < joint_count; ++i) {
        if (!vector_json[i].is_number()) {
            sendJson(res, false, 1001, "NULLSPACE direction values must be numbers");
            return;
        }
        const double value = vector_json[i].get<double>();
        if (!isFinite(value)) {
            sendJson(res, false, 1001, "NULLSPACE direction values must be finite");
            return;
        }
        intent_direction(static_cast<unsigned int>(i)) = sign * value;
    }

    const Result result = robot_->MoveNullJogging(intent_direction, speed, timeout, dir_threshold);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Nullspace jogging started" : "Nullspace jogging failed",
             data);
}

void RobotHttpServer::handleJogSvd(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/jog/svd");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded()) {
        sendJson(res, false, 1001, "Invalid JSON body");
        return;
    }
    const auto speeds_json = body.contains("dim_speeds") ? body["dim_speeds"] : body.value("speeds", nlohmann::json::array());
    if (!speeds_json.is_array() || speeds_json.empty()) {
        sendJson(res, false, 1001, "SVD jogging requires 'dim_speeds' array");
        return;
    }

    std::vector<double> dim_speeds;
    dim_speeds.reserve(speeds_json.size());
    for (const auto& value_json : speeds_json) {
        if (!value_json.is_number()) {
            sendJson(res, false, 1001, "SVD jogging speeds must be numbers");
            return;
        }
        const double value = value_json.get<double>();
        if (!isFinite(value)) {
            sendJson(res, false, 1001, "SVD jogging speeds must be finite");
            return;
        }
        dim_speeds.push_back(value);
    }

    const double timeout = jsonNumberOr(body, "timeout", nullptr, 0.1);
    const double dir_threshold = jsonNumberOr(body, "dir_threshold", nullptr, 0.99);
    if (!isFinite(timeout) || !isFinite(dir_threshold) ||
        timeout <= 0.0 || dir_threshold < -1.0 || dir_threshold > 1.0) {
        sendJson(res, false, 1001, "Invalid timeout or direction threshold");
        return;
    }

    const Result result = robot_->MoveSvdJogging(dim_speeds, timeout, dir_threshold);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "SVD jogging started" : "SVD jogging failed",
             data);
}

void RobotHttpServer::handleJogStop(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/jog/stop");

    if (!robot_->IsMotionBusy()) {
        nlohmann::json data;
        data["robot_state"] = robot_->GetStateString();
        data["control_active"] = robot_->IsControlActive();
        sendJson(res, true, 0, "Jogging already stopped", data);
        return;
    }

    const Result result = robot_->StopMotion();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Jogging stopped" : "Jogging stop failed",
             data);
}



// ============================================================================
// Frame Management Handlers
// ============================================================================

void RobotHttpServer::handleGetToolFrameNames(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/robot/tool_frames");

    nlohmann::json data;
    data["names"] = robot_->GetToolFrameNames();
    data["active"] = robot_->GetActiveToolFrameName();
    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleGetObjectFrameNames(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/robot/object_frames");

    nlohmann::json data;
    data["names"] = robot_->GetObjectFrameNames();
    data["active"] = robot_->GetActiveObjectFrameName();
    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleGetToolFrame(const httplib::Request& req, httplib::Response& res) {
    const std::string name = req.get_param_value("name");
    log_ptr_->info("GET /api/robot/tool_frame name={}", name);
    if (name.empty()) {
        sendJson(res, false, 1001, "Missing 'name' query parameter");
        return;
    }

    Frame frame;
    const Result result = robot_->GetToolFrame(name, frame);
    nlohmann::json data;
    data["name"] = name;
    if (resultSucceeded(result)) data["frame"] = frameToJson(frame);
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "ok" : "Tool frame not found",
             data);
}

void RobotHttpServer::handleGetObjectFrame(const httplib::Request& req, httplib::Response& res) {
    const std::string name = req.get_param_value("name");
    log_ptr_->info("GET /api/robot/object_frame name={}", name);
    if (name.empty()) {
        sendJson(res, false, 1001, "Missing 'name' query parameter");
        return;
    }

    Frame frame;
    const Result result = robot_->GetObjectFrame(name, frame);
    nlohmann::json data;
    data["name"] = name;
    if (resultSucceeded(result)) data["frame"] = frameToJson(frame);
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "ok" : "Object frame not found",
             data);
}

void RobotHttpServer::handleSetPoseFrame(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/calibration/pose");
    sendJson(res, false, 1004, "Pose frame calibration endpoint is not implemented");
}

void RobotHttpServer::handleSetToolFrame(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/tool_frame");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    std::string name;
    Frame frame;
    if (!parseNamedFrameBody(body, name, frame)) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'name' and 'frame' fields");
        return;
    }

    const Result result = robot_->SetToolFrame(name, frame);
    nlohmann::json data;
    data["name"] = name;
    data["frame"] = frameToJson(frame);
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Tool frame set" : "Tool frame set failed",
             data);
}

void RobotHttpServer::handleSetObjectFrame(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/object_frame");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    std::string name;
    Frame frame;
    if (!parseNamedFrameBody(body, name, frame)) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'name' and 'frame' fields");
        return;
    }

    const Result result = robot_->SetObjectFrame(name, frame);
    nlohmann::json data;
    data["name"] = name;
    data["frame"] = frameToJson(frame);
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Object frame set" : "Object frame set failed",
             data);
}

void RobotHttpServer::handleRemoveToolFrame(const httplib::Request& req, httplib::Response& res) {
    const std::string name = req.get_param_value("name");
    log_ptr_->info("DELETE /api/robot/tool_frame name={}", name);
    if (name.empty()) {
        sendJson(res, false, 1001, "Missing 'name' query parameter");
        return;
    }

    const Result result = robot_->RemoveToolFrame(name);
    nlohmann::json data;
    data["name"] = name;
    data["active"] = robot_->GetActiveToolFrameName();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Tool frame removed" : "Tool frame remove failed",
             data);
}

void RobotHttpServer::handleRemoveObjectFrame(const httplib::Request& req, httplib::Response& res) {
    const std::string name = req.get_param_value("name");
    log_ptr_->info("DELETE /api/robot/object_frame name={}", name);
    if (name.empty()) {
        sendJson(res, false, 1001, "Missing 'name' query parameter");
        return;
    }

    const Result result = robot_->RemoveObjectFrame(name);
    nlohmann::json data;
    data["name"] = name;
    data["active"] = robot_->GetActiveObjectFrameName();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Object frame removed" : "Object frame remove failed",
             data);
}

void RobotHttpServer::handleSetActiveToolFrame(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/active_tool_frame");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("name") || !body["name"].is_string()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'name' field");
        return;
    }

    const std::string name = body["name"].get<std::string>();
    const Result result = robot_->SetActiveToolFrame(name);
    nlohmann::json data;
    data["active"] = robot_->GetActiveToolFrameName();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Active tool frame set" : "Active tool frame set failed",
             data);
}

void RobotHttpServer::handleSetActiveObjectFrame(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/active_object_frame");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("name") || !body["name"].is_string()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'name' field");
        return;
    }

    const std::string name = body["name"].get<std::string>();
    const Result result = robot_->SetActiveObjectFrame(name);
    nlohmann::json data;
    data["active"] = robot_->GetActiveObjectFrameName();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Active object frame set" : "Active object frame set failed",
             data);
}

void RobotHttpServer::handleLoadFrames(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/frames/load");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("path") || !body["path"].is_string()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'path' field");
        return;
    }

    const std::string path = body["path"].get<std::string>();
    const Result result = robot_->LoadFrames(path);
    nlohmann::json data;
    data["path"] = path;
    data["tool_frames"] = robot_->GetToolFrameNames();
    data["object_frames"] = robot_->GetObjectFrameNames();
    data["active_tool_frame"] = robot_->GetActiveToolFrameName();
    data["active_object_frame"] = robot_->GetActiveObjectFrameName();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Frames loaded" : "Frames load failed",
             data);
}

void RobotHttpServer::handleSaveFrames(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/frames/save");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("path") || !body["path"].is_string()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'path' field");
        return;
    }

    const std::string path = body["path"].get<std::string>();
    const Result result = robot_->SaveFrames(path);
    nlohmann::json data;
    data["path"] = path;
    data["tool_frames"] = robot_->GetToolFrameNames();
    data["object_frames"] = robot_->GetObjectFrameNames();
    data["active_tool_frame"] = robot_->GetActiveToolFrameName();
    data["active_object_frame"] = robot_->GetActiveObjectFrameName();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Frames saved" : "Frames save failed",
             data);
}

// ============================================================================
// Calibration Handlers
// ============================================================================

void RobotHttpServer::handleCalibrationRun(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/calibration/run");
    sendJson(res, false, 1004, "Calibration run endpoint is not implemented");
}

void RobotHttpServer::handleCalibrationResult(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/calibration/result");
    sendJson(res, false, 1004, "Calibration result endpoint is not implemented");
}

}  // namespace rocos
