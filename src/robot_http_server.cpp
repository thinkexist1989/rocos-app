
#include "robot_http_server.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <kdl_parser/kdl_parser.hpp>
#include <sstream>

#include "robot.hpp"

namespace rocos {

namespace {

int resultCode(Result result) {
    return static_cast<int>(result);
}

bool resultSucceeded(Result result) {
    return result == Result::NoError || result == Result::PlanFinished;
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

bool parseCartesianJogFlag(const std::string& flag, double sign, Twist& twist) {
    const auto separator = flag.rfind('_');
    if (separator == std::string::npos) return false;

    const std::string frame = flag.substr(0, separator);
    const std::string axis = flag.substr(separator + 1);
    if (frame != "TOOL" && frame != "FLANGE" && frame != "OBJECT" && frame != "BASE") {
        return false;
    }

    if (axis == "X") {
        twist.vel.x(sign);
    } else if (axis == "Y") {
        twist.vel.y(sign);
    } else if (axis == "Z") {
        twist.vel.z(sign);
    } else if (axis == "ROLL") {
        twist.rot.x(sign);
    } else if (axis == "PITCH") {
        twist.rot.y(sign);
    } else if (axis == "YAW") {
        twist.rot.z(sign);
    } else {
        return false;
    }

    return true;
}

}  // namespace

// ============================================================================
// Construction / Destruction
// ============================================================================

RobotHttpServer::RobotHttpServer(Robot* robot)
    : robot_(robot), server_(new httplib::Server()), taskCounter_(0),
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

    // Motion control
    server_->Post("/api/move/joint", [this](auto& req, auto& res) { handleMoveJ(req, res); });
    server_->Post("/api/move/joint_ik", [this](auto& req, auto& res) { handleMoveJ_IK(req, res); });
    server_->Post("/api/move/linear", [this](auto& req, auto& res) { handleMoveL(req, res); });
    server_->Post("/api/move/linear_fk", [this](auto& req, auto& res) { handleMoveL_FK(req, res); });

    server_->Post("/api/move/pause", [this](auto& req, auto& res) { handlePause(req, res); });
    server_->Post("/api/move/resume", [this](auto& req, auto& res) { handleResume(req, res); });
    server_->Post("/api/move/stop", [this](auto& req, auto& res) { handleStop(req, res); });
    server_->Get("/api/move/status", [this](auto& req, auto& res) { handleMoveStatus(req, res); });

    // Dragging
    server_->Post("/api/drag/start", [this](auto& req, auto& res) { handleDragStart(req, res); });
    server_->Post("/api/drag/stop", [this](auto& req, auto& res) { handleDragStop(req, res); });

    // Calibration
    server_->Post("/api/calibration/pose", [this](auto& req, auto& res) { handleSetPoseFrame(req, res); });
    server_->Post("/api/calibration/tool", [this](auto& req, auto& res) { handleSetToolFrame(req, res); });
    server_->Post("/api/calibration/object", [this](auto& req, auto& res) { handleSetObjectFrame(req, res); });
    server_->Post("/api/calibration/run", [this](auto& req, auto& res) { handleCalibrationRun(req, res); });
    server_->Get("/api/calibration/result", [this](auto& req, auto& res) { handleCalibrationResult(req, res); });

    
    // 修改后：去掉 ".*",
    server_->set_post_routing_handler([](const httplib::Request&, httplib::Response& res) {
    if (res.body.empty() && res.status == 0) {
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
    res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
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
    // log_ptr_->info("GET /api/robot/state");

    nlohmann::json data;

    data["robot_state"] = robot_->GetStateString();
    data["is_enabled"] = robot_->IsEnabled();
    data["is_disabled"] = robot_->IsDisabled();
    data["is_running"] = robot_->IsRunning();
    data["control_active"] = robot_->IsControlActive();
    // log_ptr_->info("Robot state: {}", data["robot_state"].get<std::string>());

    // Joint states
    nlohmann::json joint_states = nlohmann::json::array();
    for (int i = 0; i < robot_->getJointNum(); ++i) {
        nlohmann::json js;
        js["name"] = robot_->getJointName(i);
        js["status"] = robot_->getJointStatus(i);
        js["position"] = robot_->getJointPosition(i);
        js["velocity"] = robot_->getJointVelocity(i);
        js["torque"] = robot_->getJointTorque(i);
        js["load_torque"] = robot_->getJointLoadTorque(i);
        joint_states.push_back(js);
    }
    data["joint_states"] = joint_states;

    // Flange pose
    KDL::Frame flange = robot_->getFlange();
    data["flange"] = frameToJson(flange);

    // Tool pose
    KDL::Frame tool = robot_->getTool();
    data["tool"] = frameToJson(tool);

    // Object pose
    KDL::Frame object = robot_->getObject();
    data["object"] = frameToJson(object);

    // Hardware state
    nlohmann::json hw;
    hw["joint_num"] = robot_->getJointNum();
    if (robot_->hardware) {
        hw["state"] = static_cast<int>(robot_->hardware->GetState());
    }
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
    log_ptr_->info("POST /api/move/joint");

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
    log_ptr_->info("POST /api/move/joint_ik");

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
    log_ptr_->info("POST /api/move/linear");

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
    log_ptr_->info("POST /api/move/linear_fk");

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

void RobotHttpServer::handleStop(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/move/stop");

    const Result result = robot_->StopMotion();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Motion stop requested" : "Motion stop failed",
             data);
}

void RobotHttpServer::handlePause(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/move/pause");

    const Result result = robot_->PauseMotion();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Motion pause requested" : "Motion pause failed",
             data);
}

void RobotHttpServer::handleResume(const httplib::Request& req, httplib::Response& res) {
  log_ptr_->info("POST /api/move/resume");

    const Result result = robot_->ResumeMotion();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Motion resume requested" : "Motion resume failed",
             data);
}

// ============================================================================
// Async Task Query
// ============================================================================

void RobotHttpServer::handleMoveStatus(const httplib::Request& req, httplib::Response& res) {
    std::string task_id = req.get_param_value("task_id");
    log_ptr_->info("GET /api/move/status task_id={}", task_id);

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
// Dragging Handlers
// ============================================================================

void RobotHttpServer::handleDragStart(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/drag/start");

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
    if (!isFinite(speed) || !isFinite(timeout) || !isFinite(dir_threshold) ||
        speed <= 0.0 || timeout <= 0.0 || dir_threshold < -1.0 || dir_threshold > 1.0) {
        sendJson(res, false, 1001, "Invalid jogging speed, timeout or direction threshold");
        return;
    }

    double sign = 0.0;
    if (!directionSign(direction, sign)) {
        sendJson(res, false, 1001, "Invalid jogging direction");
        return;
    }

    Result result = Result::IllegalParameter;
    if (sign == 0.0) {
        result = robot_->StopMotion();
    } else if (flag.size() == 2 && flag[0] == 'J' && flag[1] >= '0' && flag[1] <= '9') {
        const int joint_index = flag[1] - '0';
        const int joint_count = robot_->getJointNum();
        if (joint_index >= joint_count) {
            sendJson(res, false, 1003,
                     "Joint index out of range: expected 0.." + std::to_string(joint_count - 1));
            return;
        }
        JntArray joint_direction(static_cast<unsigned int>(joint_count));
        joint_direction(static_cast<unsigned int>(joint_index)) = sign;
        result = robot_->MoveJogging(joint_direction, speed, timeout, dir_threshold);
    } else if (flag == "NULLSPACE") {
        const auto vector_json = body.contains("joints") ? body["joints"] : body.value("direction_vector", nlohmann::json::array());
        if (!vector_json.is_array()) {
            sendJson(res, false, 1001, "NULLSPACE jogging requires 'joints' or 'direction_vector' array");
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
        result = robot_->MoveNullJogging(intent_direction, speed, timeout, dir_threshold);
    } else {
        Twist twist;
        if (!parseCartesianJogFlag(flag, sign, twist)) {
            sendJson(res, false, 1001, "Invalid jogging flag");
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

void RobotHttpServer::handleDragStop(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/drag/stop");

    const Result result = robot_->StopMotion();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Dragging stopped" : "Dragging stop failed",
             data);
}

// ============================================================================
// Calibration Handlers
// ============================================================================

void RobotHttpServer::handleSetPoseFrame(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/calibration/pose");

    // nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    // if (body.is_discarded() || !body.contains("id") || !body.contains("pose")) {
    //     sendJson(res, false, 1001, "Invalid JSON or missing 'id'/'pose' fields");
    //     return;
    // }
    //
    // int id = body["id"].get<int>();
    // nlohmann::json pose_json = body["pose"];
    //
    // KDL::Frame pose;
    // if (pose_json.contains("position") && pose_json.contains("orientation")) {
    //     pose = jsonToFrame(pose_json["position"], pose_json["orientation"]);
    // } else {
    //     pose = jsonToFrame(pose_json);
    // }
    //
    // robot_->set_pose_frame(id, pose);
    // sendJson(res, true, 0, "Pose frame " + std::to_string(id) + " set");
}

void RobotHttpServer::handleSetToolFrame(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/calibration/tool");

    // nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    // if (body.is_discarded() || !body.contains("pose")) {
    //     sendJson(res, false, 1001, "Invalid JSON or missing 'pose' field");
    //     return;
    // }
    //
    // nlohmann::json pose_json = body["pose"];
    // KDL::Frame pose;
    // if (pose_json.contains("position") && pose_json.contains("orientation")) {
    //     pose = jsonToFrame(pose_json["position"], pose_json["orientation"]);
    // } else {
    //     pose = jsonToFrame(pose_json);
    // }
    //
    // robot_->set_tool_frame(pose);
    // sendJson(res, true, 0, "Tool frame set");
}

void RobotHttpServer::handleSetObjectFrame(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/calibration/object");

    // nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    // if (body.is_discarded() || !body.contains("pose")) {
    //     sendJson(res, false, 1001, "Invalid JSON or missing 'pose' field");
    //     return;
    // }
    //
    // nlohmann::json pose_json = body["pose"];
    // KDL::Frame pose;
    // if (pose_json.contains("position") && pose_json.contains("orientation")) {
    //     pose = jsonToFrame(pose_json["position"], pose_json["orientation"]);
    // } else {
    //     pose = jsonToFrame(pose_json);
    // }
    //
    // robot_->set_object_frame(pose);
    // sendJson(res, true, 0, "Object frame set");
}

void RobotHttpServer::handleCalibrationRun(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/calibration/run");

    // nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    // if (body.is_discarded() || !body.contains("frame")) {
    //     sendJson(res, false, 1001, "Invalid JSON or missing 'frame' field");
    //     return;
    // }
    //
    // std::string frame = body["frame"].get<std::string>();
    // if (frame != "tool" && frame != "object") {
    //     sendJson(res, false, 4004, "Invalid frame type: must be 'tool' or 'object'");
    //     return;
    // }
    //
    // robot_->tool_calibration(frame);
    //
    // bool error_state = robot_->getErrorStateOfCal();
    // if (error_state) {
    //     sendJson(res, false, 4002, "Calibration failed");
    // } else {
    //     KDL::Frame result = robot_->getPose_out();
    //     nlohmann::json data;
    //     data["result"] = frameToJson(result);
    //     sendJson(res, true, 0, "Calibration completed", data);
    // }
}

void RobotHttpServer::handleCalibrationResult(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/calibration/result");

    // bool error_state = robot_->getErrorStateOfCal();
    // KDL::Frame result = robot_->getPose_out();
    //
    // nlohmann::json data;
    // data["error_state"] = error_state;
    // data["pose"] = frameToJson(result);
    //
    // sendJson(res, true, 0, "ok", data);
}

} // namespace rocos
