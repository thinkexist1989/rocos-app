
#include <rocos_app/robot_http_server.hpp>

#include <spdlog/spdlog.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <kdl_parser/kdl_parser.hpp>
#include <random>
#include <sstream>

#include <rocos_app/robot.h>
#include <stdexcept>
#include <cctype>
#include <algorithm>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <kdl/chainjnttojacsolver.hpp>

namespace rocos {

namespace {

bool robotEnabled(Robot* robot) {
    if (robot->getJointNum() == 0) return false;
    for (int i = 0; i < robot->getJointNum(); ++i) {
        if (robot->getJointStatus(i) != 2) return false;
    }
    return true;
}

bool robotDisabled(Robot* robot) {
    for (int i = 0; i < robot->getJointNum(); ++i) {
        if (robot->getJointStatus(i) != 1) return false;
    }
    return true;
}

const char* jointStatusName(int status) {
    // Legacy RPC uses 1=disabled, 2=enabled, 3=fault. Strings are understood
    // by viz without confusing legacy values with its newer numeric enum.
    switch (status) {
        case 1: return "DISABLED";
        case 2: return "ENABLED";
        case 3: return "FAULT";
        default: return "UNKNOWN";
    }
}

std::string robotState(Robot* robot) {
    for (int i = 0; i < robot->getJointNum(); ++i) {
        if (robot->getJointStatus(i) == 3) return "FAULT";
    }
    // HardwareSim starts enabled while the legacy run state starts Disabled.
    // Use actual drive status for enablement so the HTTP summary is consistent.
    if (!robotEnabled(robot)) return robotDisabled(robot) ? "DISABLED" : "UNKNOWN";
    return robot->getRunState() == Robot::RunState::Running ? "RUNNING" : "STOPPED";
}

bool motionBusy(Robot* robot) {
    return robot->isMotionBusy();
}

std::string workModeName(Robot* robot) {
    switch (robot->getWorkMode()) {
        case Robot::WorkMode::Position: return "position";
        case Robot::WorkMode::EeAdmitTeach: return "ee_admit_teach";
        case Robot::WorkMode::JntAdmitTeach: return "jnt_admit_teach";
        case Robot::WorkMode::JntImp: return "jnt_imp";
        case Robot::WorkMode::CartImp: return "cart_imp";
    }
    return "unknown";
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

const char* modeName(ModeOfOperation mode) {
    switch (mode) {
        case ModeOfOperation::CyclicSynchronousPositionMode: return "CSP";
        case ModeOfOperation::CyclicSynchronousVelocityMode: return "CSV";
        case ModeOfOperation::CyclicSynchronousTorqueMode: return "CST";
        default: return "UNKNOWN";
    }
}

bool parseSync(const nlohmann::json& value, Robot::Synchronization& sync) {
    if (value.is_number_integer()) {
        const int n = value.get<int>();
        if (n < 0 || n > 2) return false;
        sync = static_cast<Robot::Synchronization>(n); return true;
    }
    if (!value.is_string()) return false;
    const auto text = value.get<std::string>();
    if (text == "none" || text == "SYNC_NONE") sync = Robot::SYNC_NONE;
    else if (text == "time" || text == "SYNC_TIME") sync = Robot::SYNC_TIME;
    else if (text == "phase" || text == "SYNC_PHASE") sync = Robot::SYNC_PHASE;
    else return false;
    return true;
}

bool validVector(const nlohmann::json& value, int size, std::vector<double>& result) {
    if (!value.is_array() || static_cast<int>(value.size()) != size) return false;
    result.clear();
    for (const auto& v : value) {
        if (!v.is_number()) return false;
        const double n = v.get<double>();
        if (!std::isfinite(n)) return false;
        result.push_back(n);
    }
    return true;
}

bool positionMode(Robot* robot, int id) {
    auto mode = robot->getJointMode(id);
    return mode == ModeOfOperation::CyclicSynchronousPositionMode || mode == ModeOfOperation::CyclicSynchronousVelocityMode;
}

// Convert a projected joint intent to the arm-angle sign used by the existing
// one-dimensional NULLSPACE controller. Its sign is determined with the same
// shoulder/wrist convention as SmartServo_Nullspace::is_same_on_direction.
bool nullspaceDirection(Robot* robot, const std::vector<double>& intent,
                        double threshold, int& direction, double& speed_scale) {
    const auto& chain = robot->kinematics_.getChain();
    KDL::JntArray q(7), target(7);
    for (int i = 0; i < 7; ++i) q(i) = robot->getJointPosition(i);
    KDL::Jacobian jac(7);
    KDL::ChainJntToJacSolver solver(chain);
    if (solver.JntToJac(q, jac) < 0) return false;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac.data, Eigen::ComputeFullV);
    // A singular configuration has more than one null direction; the legacy
    // single scalar cannot encode an arbitrary projection in that space.
    int rank = 0;
    for (double value : svd.singularValues()) if (value > 1e-5) ++rank;
    if (rank != 6) return false;
    const Eigen::VectorXd kernel = svd.matrixV().col(6);
    Eigen::Index selected;
    kernel.cwiseAbs().maxCoeff(&selected);
    const Eigen::VectorXd basis = kernel * kernel(selected);
    const Eigen::Map<const Eigen::VectorXd> desired(intent.data(), 7);
    const double coefficient = basis.dot(desired) / basis.squaredNorm();
    if (std::abs(coefficient) <= threshold) { direction = 0; speed_scale = 0; return true; }
    for (int i = 0; i < 7; ++i) target(i) = q(i) + basis(i) * DELTA_T;
    KDL::ChainFkSolverPos_recursive fk(chain);
    KDL::Frame elbow, next_elbow, shoulder, wrist;
    if (fk.JntToCart(q, elbow, 4) < 0 || fk.JntToCart(target, next_elbow, 4) < 0 ||
        fk.JntToCart(q, shoulder, 2) < 0 || fk.JntToCart(q, wrist, 6) < 0) return false;
    KDL::Vector axis;
    double angle = (elbow.M.Inverse() * next_elbow.M).GetRotAngle(axis);
    axis = elbow.M * axis;
    const auto sw = wrist.p - shoulder.p;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(sw(i)) > DELTA_T && KDL::sign(sw(i)) != KDL::sign(axis(i))) { angle = -angle; break; }
    }
    if (std::abs(angle) < 1e-12) return false;
    direction = (angle > 0 ? 1 : -1) * (coefficient > 0 ? 1 : -1);
    speed_scale = std::abs(coefficient) / desired.norm();
    return true;
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


}  // namespace

// ============================================================================
// Construction / Destruction
// ============================================================================

RobotHttpServer::RobotHttpServer(Robot* robot, LuaScriptEngine* script_engine)
    : robot_(robot), script_engine_(script_engine), server_(new httplib::Server()), taskCounter_(0),
      poolShutdown_(false), activeWorkers_(0) {

    log_ptr_ = spdlog::default_logger();
    // Allow restarting after TIME_WAIT, but keep a single controller per port.
    // cpp-httplib otherwise enables SO_REUSEPORT on Linux.
    server_->set_socket_options([](socket_t socket) {
        httplib::set_socket_opt(socket, SOL_SOCKET, SO_REUSEADDR, 1);
    });
    if (script_engine != nullptr) {
        throw std::invalid_argument("Lua scripts are not supported by this Robot version");
    }

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
    if (robot_) watchdog_thread_ = std::thread(&RobotHttpServer::jogWatchdog, this);
}

RobotHttpServer::~RobotHttpServer() {
    stop();
    watchdog_shutdown_ = true;
    if (watchdog_thread_.joinable()) watchdog_thread_.join();
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
    if (!server_->bind_to_port(host, port)) {
        throw std::runtime_error("Cannot bind HTTP server to " + host + ":" + std::to_string(port));
    }
    log_ptr_->info("HTTP Server listening on {}:{}", host, port);
    if (!server_->listen_after_bind()) {
        throw std::runtime_error("HTTP server listen failed");
    }
}

void RobotHttpServer::runAsync(const std::string& host, int port) {
    if (thread_ && thread_->joinable()) {
        throw std::logic_error("HTTP server is already started");
    }
    if (!server_->bind_to_port(host, port)) {
        throw std::runtime_error("Cannot bind HTTP server to " + host + ":" + std::to_string(port));
    }
    thread_.reset(new std::thread([this]() {
        if (!server_->listen_after_bind()) {
            log_ptr_->error("HTTP server listen failed");
        }
    }));
    server_->wait_until_ready();
    if (!server_->is_running()) {
        thread_->join();
        throw std::runtime_error("HTTP server failed to start");
    }
    log_ptr_->info("HTTP Server listening on {}:{}", host, port);
}

void RobotHttpServer::stop() {
    if (server_) server_->stop();
    if (thread_ && thread_->joinable()) thread_->join();
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (robot_ && jog_active_) robot_->requestMotionStop();
    jog_active_ = false;
}

// ============================================================================
// Route Registration
// ============================================================================

void RobotHttpServer::registerRoutes() {
    server_->set_exception_handler([this](const httplib::Request&, httplib::Response& res,
                                         std::exception_ptr error) {
        try { std::rethrow_exception(error); }
        catch (const nlohmann::json::exception&) {
            sendJson(res, false, 1001, "Invalid JSON field type or value");
        }
        catch (const std::exception& e) {
            log_ptr_->error("HTTP request failed: {}", e.what());
            sendJson(res, false, 1004, "Request failed");
        }
        catch (...) { sendJson(res, false, 1004, "Request failed"); }
    });
    if (!server_->set_mount_point("/", "web") &&
        !server_->set_mount_point("/", "../web") &&
        !server_->set_mount_point("/", "../../web")) {
        log_ptr_->info("No bundled web UI; serve viz separately. HTTP API is available.");
    }

    // CORS preflight
    server_->Options(".*", [this](const httplib::Request&, httplib::Response& res) {
        setCorsHeaders(res);
    });

    // Robot state
    server_->Get("/api/robot/state", [this](auto& req, auto& res) { handleGetRobotState(req, res); });
    server_->Get("/api/robot/info", [this](auto& req, auto& res) { handleGetRobotInfo(req, res); });
    server_->Get("/api/robot/impedance", [this](auto& req, auto& res) { handleGetImpedance(req, res); });
    server_->Get("/api/robot/urdf", [this](auto& req, auto& res) { handleGetRobotModel(req, res); });
    server_->Get("/api/robot/urdf/mesh", [this](auto& req, auto& res) { handleGetLinkMesh(req, res); });
    server_->Get("/api/robot/enabled", [this](auto& req, auto& res) { handleIsEnabled(req, res); });
    server_->Post("/api/robot/enable", [this](auto& req, auto& res) { handleEnable(req, res); });
    server_->Post("/api/robot/disable", [this](auto& req, auto& res) { handleDisable(req, res); });
    server_->Post("/api/robot/reset", [this](auto& req, auto& res) { handleReset(req, res); });
    server_->Post("/api/robot/workmode", [this](auto& req, auto& res) { handleSetWorkMode(req, res); });
    server_->Post("/api/robot/load_params", [this](auto& req, auto& res) { handleSetLoadParameters(req, res); });
    server_->Get("/api/robot/load_params", [this](auto& req, auto& res) { handleGetLoadParameters(req, res); });
    server_->Post("/api/robot/load_params/save", [this](auto& req, auto& res) { handleSaveLoadParameters(req, res); });

    for (const auto* action : {"enable", "disable", "move", "stop"}) {
        server_->Post(std::string("/api/robot/single_axis/") + action,
            [this](auto& req, auto& res) { handleSingleAxisCommand(req, res); });
        server_->Post(std::string("/api/robot/multi_axis/") + action,
            [this](auto& req, auto& res) { handleMultiAxisCommand(req, res); });
    }
    server_->Post("/api/robot/multi_axis/sync", [this](auto& req, auto& res) { handleMultiAxisCommand(req, res); });
    server_->Post("/api/robot/dragging", [this](auto& req, auto& res) { handleJogCompat(req, res); });
    server_->Post("/api/robot/jog", [this](auto& req, auto& res) { handleJogCompat(req, res); });

    // Robot servo control
    server_->Post("/api/robot/servo/start", [this](auto& req, auto& res) { handleServoStart(req, res); });
    server_->Post("/api/robot/servo/stop", [this](auto& req, auto& res) { handleServoStop(req, res); });
    server_->Post("/api/robot/servo/mode", [this](auto& req, auto& res) { handleServoMode(req, res); });

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
    server_->Get("/api/script/list", [this](auto& req, auto& res) { handleScriptList(req, res); });
    server_->Get("/api/script/download", [this](auto& req, auto& res) { handleScriptDownload(req, res); });
    server_->Post("/api/script/breakpoint/add", [this](auto& req, auto& res) { handleScriptBreakpointAdd(req, res); });
    server_->Post("/api/script/breakpoint/remove", [this](auto& req, auto& res) { handleScriptBreakpointRemove(req, res); });
    server_->Post("/api/script/breakpoint/clear", [this](auto& req, auto& res) { handleScriptBreakpointClear(req, res); });

    // Control rights (single-holder lock)
    server_->Post("/api/control/acquire", [this](auto& req, auto& res) { handleControlAcquire(req, res); });
    server_->Post("/api/control/release", [this](auto& req, auto& res) { handleControlRelease(req, res); });
    server_->Post("/api/control/takeover", [this](auto& req, auto& res) { handleControlTakeover(req, res); });
    server_->Get("/api/control/status", [this](auto& req, auto& res) { handleControlStatus(req, res); });

    
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
    res.set_header("Access-Control-Allow-Headers",
                   "Content-Type, X-Rocos-Control-Token, X-Rocos-Client-Id");
    res.set_header("Access-Control-Expose-Headers",
                   "X-Rocos-Control-Token, X-Rocos-Client-Id");
}

// ============================================================================
// Utility: JSON Response
// ============================================================================

void RobotHttpServer::sendJson(httplib::Response& res, bool success,
                               int businessCode, const std::string& message,
                               const nlohmann::json& data, const std::string& error_name) {
    setCorsHeaders(res);
    res.set_header("Content-Type", "application/json");

    nlohmann::json body;
    body["success"] = success;
    body["code"] = businessCode;
    body["message"] = message;
    body["data"] = data;
    if (!success) {
        std::string name = error_name;
        if (name.empty()) {
            switch (businessCode) {
                case 1001: name = "InvalidParameter"; break;
                case 1002: name = "InvalidJointId"; break;
                case 1003: name = "InvalidVectorLength"; break;
                case 1004: name = "RequestFailed"; break;
                case 1006: name = "InvalidWorkMode"; break;
                case 2001: name = "NullspaceMappingFailed"; break;
                case 3001: name = "NotEnabled"; break;
                case 3003: name = "MotionBusy"; break;
                case 3005: name = "InvalidMotionMode"; break;
                case 3006: name = "ControlTokenMissing"; break;
                case 3007: name = "ControlTokenInvalidOrExpired"; break;
                case 3008: name = "ControlAlreadyHeld"; break;
                case 4001: name = "CalibrationPosesIncomplete"; break;
                case 4002: name = "CalibrationFailed"; break;
                case 4004: name = "InvalidCalibrationFrame"; break;
                case 5002: name = "WaitMoveTimeout"; break;
                default: name = rocos::to_string(static_cast<Result>(businessCode)); break;
            }
        }
        body["error_name"] = name;
    }

    res.set_content(body.dump(), "application/json");
}

// ============================================================================
// Utility: Control Rights (single-holder lock)
// ============================================================================

void RobotHttpServer::sendUnsupported(httplib::Response& res, const std::string& feature) {
    sendJson(res, false, 3007, feature + " is not supported by this Robot version",
             nullptr, "FunctionNotSupported");
}

bool RobotHttpServer::requireIdle(httplib::Response& res) {
    if (robot_->isMotionBusy()) {
        sendJson(res, false, 3003, "Robot is executing a motion; stop it before changing this setting");
        return false;
    }
    return true;
}

nlohmann::json RobotHttpServer::motionData() const {
    const auto type = robot_->getMotionType();
    const bool busy = robot_->isMotionBusy();
    const auto report = robot_->getMotionReport();
    const char* name = type == Robot::MotionType::Axis ? "axis"
        : type == Robot::MotionType::Trajectory ? "trajectory"
        : type == Robot::MotionType::Dragging ? "dragging"
        : type == Robot::MotionType::Teaching ? "teaching" : "none";
    nlohmann::json error = nullptr;
    if (report.result != 0) {
        error = {{"code", report.result}, {"error_name", rocos::to_string(static_cast<Result>(report.result))},
                 {"message", resultMessage(static_cast<Result>(report.result))},
                 {"command", report.command}, {"stage", report.stage}};
        if (report.joint >= 0) error["joint"] = report.joint;
        if (report.waypoint >= 0) error["waypoint"] = report.waypoint;
        if (report.has_native_code) error["native_code"] = report.native_code;
    }
    return {{"robot_state", robotState(robot_)}, {"control_active", busy},
            {"motion_busy", busy}, {"motion_type", name},
            {"stop_requested", robot_->motionStopRequested()},
            {"result", report.result}, {"result_name", rocos::to_string(static_cast<Result>(report.result))},
            {"motion_outcome", report.outcome}, {"error", error}};
}

void RobotHttpServer::sendMotionResult(httplib::Response& res, int result,
                                      const std::string& message, bool asynchronous) {
    result = rocos::resultCode(static_cast<Result>(result));
    if (result == 0 && !asynchronous) {
        while (robot_->isMotionBusy()) std::this_thread::sleep_for(std::chrono::milliseconds(2));
        result = robot_->getLastMotionResult();
    }
    // Older jogging/teaching paths can still return unclassified native codes.
    if (result != 0 && rocos::to_string(static_cast<Result>(result)).find("Unknown(") == 0) {
        result = robot_->recordMotionError(Result::MoveUnknown, "execution", -1, -1, result, true);
    }
    sendJson(res, result == 0, result,
             result == 0 ? message : resultMessage(static_cast<Result>(result)), motionData(),
             result == 0 ? "" : rocos::to_string(static_cast<Result>(result)));
}

void RobotHttpServer::jogWatchdog() {
    while (!watchdog_shutdown_) {
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (jog_active_ && (!robot_->isMotionBusy() ||
                               std::chrono::steady_clock::now() >= jog_deadline_)) {
                if (robot_->getMotionType() == Robot::MotionType::Dragging) robot_->requestMotionStop();
                jog_active_ = false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void RobotHttpServer::dispatchJog(httplib::Response& res, int flag, int direction,
                                 double speed, double acceleration, double timeout) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (direction == 0) {
        if (robot_->getMotionType() == Robot::MotionType::Dragging) robot_->requestMotionStop();
        jog_active_ = false;
        sendMotionResult(res, 0, "Jog stop requested");
        return;
    }
    if (robot_->getWorkMode() != Robot::WorkMode::Position) {
        sendJson(res, false, 3005, "Jogging requires position work mode"); return;
    }
    for (int i = 0; i < robot_->getJointNum(); ++i) {
        if (!positionMode(robot_, i)) {
            sendJson(res, false, 3005, "Jogging requires CSP/CSV joint modes"); return;
        }
    }
    if (robot_->isMotionBusy() && (robot_->getMotionType() != Robot::MotionType::Dragging ||
        robot_->motionStopRequested() || (jog_active_ && flag != jog_flag_))) {
        sendJson(res, false, 3003, "Stop the current motion before starting another jog"); return;
    }
    if (!robotEnabled(robot_)) { sendJson(res, false, 3001, "Robot is not enabled"); return; }
    for (int i = 0; i < robot_->getJointNum(); ++i) {
        if (speed > robot_->getJointMaxVelocity(i) || acceleration > robot_->getJointMaxAcceleration(i)) {
            sendJson(res, false, 1001, "Jogging limits exceed the robot limits"); return;
        }
    }
    const int result = robot_->Dragging(static_cast<Robot::DRAGGING_FLAG>(flag),
        static_cast<Robot::DRAGGING_DIRRECTION>(direction), speed, acceleration);
    if (result == 0) {
        jog_active_ = true;
        jog_flag_ = flag;
        jog_deadline_ = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout));
    }
    sendMotionResult(res, result, "Jog accepted");
}

std::string RobotHttpServer::makeConnectionKey(const std::string& remote_addr, int remote_port) {
    if (remote_addr.empty()) {
        return "unknown:" + std::to_string(remote_port);
    }
    return remote_addr + ":" + std::to_string(remote_port);
}

std::string RobotHttpServer::makeConnectionKey(const httplib::Request& req) {
    return makeConnectionKey(req.remote_addr, req.remote_port);
}

std::string RobotHttpServer::extractClientId(const httplib::Request& req) {
    if (req.has_header(CONTROL_CLIENT_ID_HEADER)) {
        return req.get_header_value(CONTROL_CLIENT_ID_HEADER);
    }
    const nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (!body.is_discarded() && body.contains("client_id") && body["client_id"].is_string()) {
        return body["client_id"].get<std::string>();
    }
    return std::string{};
}

std::string RobotHttpServer::generateControlToken() {
    // 生成 UUID v4 字符串。使用 random_device + mt19937_64 提供足够的熵。
    static thread_local std::mt19937_64 rng(
        std::random_device{}() ^
        static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<uint64_t> dist;
    const uint64_t a = dist(rng);
    const uint64_t b = dist(rng);

    unsigned char bytes[16];
    for (int i = 0; i < 8; ++i) bytes[i]     = static_cast<unsigned char>((a >> (8 * i)) & 0xFF);
    for (int i = 0; i < 8; ++i) bytes[i + 8] = static_cast<unsigned char>((b >> (8 * i)) & 0xFF);
    bytes[6] = static_cast<unsigned char>((bytes[6] & 0x0F) | 0x40);  // version 4
    bytes[8] = static_cast<unsigned char>((bytes[8] & 0x3F) | 0x80);  // variant RFC4122

    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (int i = 0; i < 16; ++i) {
        oss << std::setw(2) << static_cast<int>(bytes[i]);
        if (i == 3 || i == 5 || i == 7 || i == 9) oss << '-';
    }
    return oss.str();
}

RobotHttpServer::AuthResult
RobotHttpServer::checkAndRenewToken(const httplib::Request& req) {
    if (!req.has_header(CONTROL_TOKEN_HEADER)) return AuthResult::MissingToken;
    const std::string token = req.get_header_value(CONTROL_TOKEN_HEADER);

    // 调试旁路：使用预设 DEBUG_TOKEN + DEBUG_CLIENT_ID 可直接控制机器人，无需 acquire。
    // 仅用于 curl/脚本调试，生产环境可注释掉此段以禁用。
    if (token == DEBUG_TOKEN) {
        const std::string req_client_id = extractClientId(req);
        if (req_client_id == DEBUG_CLIENT_ID) {
            log_ptr_->info("Debug bypass: token accepted (client_id={})", req_client_id);
            return AuthResult::OK;
        }
    }

    std::lock_guard<std::mutex> lock(control_mutex_);
    if (current_token_.empty()) return AuthResult::InvalidToken;

    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_seen_at_).count()
            > CONTROL_TTL_SECONDS) {
        clearControl_locked("ttl expired");
        return AuthResult::Expired;
    }
    if (token != current_token_) return AuthResult::InvalidToken;

    // 验证 client_id：若 acquire 时记录了 client_id，后续请求必须携带相同的 client_id。
    // 这防止同一浏览器不同标签页通过共享 localStorage 窃取 token 绕过控制权检查。
    if (!current_owner_client_id_.empty()) {
        const std::string req_client_id = extractClientId(req);
        if (req_client_id.empty()) {
            log_ptr_->warn(
                "Control token valid but request missing X-Rocos-Client-Id "
                "(owner client_id={})", current_owner_client_id_);
            return AuthResult::InvalidToken;
        }
        if (req_client_id != current_owner_client_id_) {
            log_ptr_->warn(
                "Control token valid but client_id mismatch "
                "(expected={}, got={})",
                current_owner_client_id_, req_client_id);
            return AuthResult::InvalidToken;
        }
    }

    last_seen_at_ = now;
    return AuthResult::OK;
}

bool RobotHttpServer::ensureControl(const httplib::Request& req,
                                    httplib::Response& res) {
    switch (checkAndRenewToken(req)) {
        case AuthResult::OK:
            return true;
        case AuthResult::MissingToken:
            sendJson(res, false, 3006,
                     "no control right, call POST /api/control/acquire first");
            return false;
        case AuthResult::InvalidToken:
            sendJson(res, false, 3007, "control token invalid");
            return false;
        case AuthResult::Expired:
            sendJson(res, false, 3007, "control token expired");
            return false;
    }
    return false;
}

void RobotHttpServer::clearControl_locked(const char* reason) {
    if (!current_token_.empty()) {
        log_ptr_->info("Control released (owner={}@{}:{}, reason={})",
                       current_owner_name_, current_owner_ip_, current_owner_port_, reason);
    }
    current_token_.clear();
    current_owner_ip_.clear();
    current_owner_port_ = -1;
    current_owner_connection_.clear();
    current_owner_client_id_.clear();
    current_owner_name_.clear();
    current_owner_agent_.clear();
    acquired_at_ = {};
    last_seen_at_ = {};
}

nlohmann::json RobotHttpServer::controlOwnerJson_locked() const {
    nlohmann::json data;
    const bool has_owner = !current_token_.empty();
    data["has_owner"] = has_owner;
    data["owner_ip"] = current_owner_ip_;
    data["owner_port"] = current_owner_port_;
    data["owner_connection"] = current_owner_connection_;
    data["owner_client_id"] = current_owner_client_id_;
    data["owner_name"] = current_owner_name_;
    data["owner_agent"] = current_owner_agent_;

    if (has_owner) {
        const auto now = std::chrono::steady_clock::now();
        const auto age = std::chrono::duration_cast<std::chrono::seconds>(now - acquired_at_).count();
        const auto idle = std::chrono::duration_cast<std::chrono::seconds>(now - last_seen_at_).count();
        const long long expires_in = CONTROL_TTL_SECONDS - idle;
        data["held_for_seconds"] = age;
        data["idle_seconds"] = idle;
        data["expires_in_seconds"] = expires_in > 0 ? expires_in : 0;
    } else {
        data["held_for_seconds"] = 0;
        data["idle_seconds"] = 0;
        data["expires_in_seconds"] = 0;
    }
    data["ttl_seconds"] = CONTROL_TTL_SECONDS;
    return data;
}

void RobotHttpServer::handleControlAcquire(const httplib::Request& req,
                                           httplib::Response& res) {
    log_ptr_->info("POST /api/control/acquire from {}", req.remote_addr);

    const nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    std::string client_name;
    if (!body.is_discarded() && body.contains("client_name") &&
        body["client_name"].is_string()) {
        client_name = body["client_name"].get<std::string>();
    }
    std::string client_id = extractClientId(req);
    if (client_id.empty() && !body.is_discarded() && body.contains("client_id") &&
        body["client_id"].is_string()) {
        client_id = body["client_id"].get<std::string>();
    }
    // 若客户端未提供 client_id，服务端自动生成一个，
    // 确保每个 token 都有绑定的 client_id，防止同源标签页 token 窃取。
    const bool client_id_auto_generated = client_id.empty();
    if (client_id_auto_generated) {
        client_id = generateControlToken();  // UUID v4 作为 client_id
    }
    const std::string user_agent = req.has_header("User-Agent")
        ? req.get_header_value("User-Agent") : std::string{};

    std::lock_guard<std::mutex> lock(control_mutex_);

    // 若已有持有者但已过期则先清空
    if (!current_token_.empty()) {
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_seen_at_).count()
                > CONTROL_TTL_SECONDS) {
            clearControl_locked("ttl expired on acquire");
        }
    }

    if (!current_token_.empty()) {
        nlohmann::json data = controlOwnerJson_locked();
        sendJson(res, false, 3008, "control already held by another client", data);
        return;
    }

    current_token_ = generateControlToken();
    current_owner_ip_ = req.remote_addr;
    current_owner_port_ = req.remote_port;
    current_owner_connection_ = makeConnectionKey(req);
    current_owner_client_id_ = client_id;
    current_owner_name_ = client_name;
    current_owner_agent_ = user_agent;
    const auto now = std::chrono::steady_clock::now();
    acquired_at_ = now;
    last_seen_at_ = now;

    log_ptr_->info("Control acquired by {}@{}:{} ({}) client_id={} auto={}",
                   current_owner_name_, current_owner_ip_, current_owner_port_,
                   current_owner_agent_, current_owner_client_id_, client_id_auto_generated);

    nlohmann::json data = controlOwnerJson_locked();
    data["token"] = current_token_;
    // 返回 client_id，前端必须存储并在后续请求中通过 X-Rocos-Client-Id 头回传
    data["client_id"] = current_owner_client_id_;
    if (client_id_auto_generated) {
        data["client_id_auto"] = true;  // 提示前端这是服务端生成的
    }
    sendJson(res, true, 0, "control acquired", data);
}

void RobotHttpServer::handleControlRelease(const httplib::Request& req,
                                           httplib::Response& res) {
    log_ptr_->info("POST /api/control/release from {}", req.remote_addr);

    if (!req.has_header(CONTROL_TOKEN_HEADER)) {
        sendJson(res, false, 3006,
                 "no control right, cannot release without token");
        return;
    }
    const std::string token = req.get_header_value(CONTROL_TOKEN_HEADER);

    std::lock_guard<std::mutex> lock(control_mutex_);
    if (current_token_.empty() || token != current_token_) {
        sendJson(res, false, 3007, "control token invalid");
        return;
    }
    // 释放时也校验 client_id，防止窃取 token 的标签页恶意释放控制权
    if (!current_owner_client_id_.empty()) {
        const std::string req_client_id = extractClientId(req);
        if (req_client_id != current_owner_client_id_) {
            sendJson(res, false, 3007, "control token invalid");
            return;
        }
    }
    clearControl_locked("released by owner");
    sendJson(res, true, 0, "control released", controlOwnerJson_locked());
}

void RobotHttpServer::handleControlTakeover(const httplib::Request& req,
                                            httplib::Response& res) {
    log_ptr_->info("POST /api/control/takeover from {}", req.remote_addr);

    const nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    std::string client_name;
    if (!body.is_discarded() && body.contains("client_name") &&
        body["client_name"].is_string()) {
        client_name = body["client_name"].get<std::string>();
    }
    std::string client_id = extractClientId(req);
    if (client_id.empty() && !body.is_discarded() && body.contains("client_id") &&
        body["client_id"].is_string()) {
        client_id = body["client_id"].get<std::string>();
    }
    const bool client_id_auto_generated = client_id.empty();
    if (client_id_auto_generated) {
        client_id = generateControlToken();
    }
    const std::string user_agent = req.has_header("User-Agent")
        ? req.get_header_value("User-Agent") : std::string{};

    std::lock_guard<std::mutex> lock(control_mutex_);
    if (!current_token_.empty()) {
        log_ptr_->warn("Control taken over: previous owner {}@{}:{} forcibly evicted by {}@{}:{}",
                       current_owner_name_, current_owner_ip_, current_owner_port_,
                       client_name, req.remote_addr, req.remote_port);
        clearControl_locked("takeover");
    }

    current_token_ = generateControlToken();
    current_owner_ip_ = req.remote_addr;
    current_owner_port_ = req.remote_port;
    current_owner_connection_ = makeConnectionKey(req);
    current_owner_client_id_ = client_id;
    current_owner_name_ = client_name;
    current_owner_agent_ = user_agent;
    const auto now = std::chrono::steady_clock::now();
    acquired_at_ = now;
    last_seen_at_ = now;

    nlohmann::json data = controlOwnerJson_locked();
    data["token"] = current_token_;
    data["client_id"] = current_owner_client_id_;
    if (client_id_auto_generated) {
        data["client_id_auto"] = true;
    }
    sendJson(res, true, 0, "control taken over", data);
}

void RobotHttpServer::handleControlStatus(const httplib::Request&,
                                          httplib::Response& res) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    // 顺手做一次懒清理：读接口发现已过期则视为无持有者
    if (!current_token_.empty()) {
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_seen_at_).count()
                > CONTROL_TTL_SECONDS) {
            clearControl_locked("ttl expired on status query");
        }
    }
    sendJson(res, true, 0, "ok", controlOwnerJson_locked());
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
    return robot_->getUrdfFilePath();
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
    std::lock_guard<std::mutex> lock(command_mutex_);
    const auto raw_text = req.get_param_value("raw_data");
    if (!raw_text.empty() && raw_text != "true" && raw_text != "false" && raw_text != "1" && raw_text != "0") {
        sendJson(res, false, 1001, "raw_data must be true or false"); return;
    }
    const bool raw = raw_text == "true" || raw_text == "1";
    auto data = motionData();
    data["is_enabled"] = robotEnabled(robot_);
    data["is_running"] = robot_->isMotionBusy();
    data["work_mode"] = workModeName(robot_);
    data["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    data["raw_data"] = raw;
    data["api_profile"] = "legacy_robot";
    data["joint_states"] = nlohmann::json::array();
    for (int i = 0; i < robot_->getJointNum(); ++i) {
        data["joint_states"].push_back({{"id", i}, {"name", robot_->getJointName(i)},
            {"status", jointStatusName(robot_->getJointStatus(i))}, {"mode", modeName(robot_->getJointMode(i))},
            {"position", raw ? robot_->getJointPositionRaw(i) : robot_->getJointPosition(i)},
            {"velocity", raw ? robot_->getJointVelocityRaw(i) : robot_->getJointVelocity(i)},
            {"torque", raw ? robot_->getJointTorqueRaw(i) : robot_->getJointTorque(i)},
            {"load_torque", raw ? robot_->getJointLoadTorqueRaw(i) : robot_->getJointLoadTorque(i)}, {"raw_data", raw}});
    }
    data["flange"] = frameToJson(robot_->getFlange());
    data["tool"] = frameToJson(robot_->getTool());
    data["object"] = frameToJson(robot_->getObject());
    data["active_tool_frame_name"] = "default";
    data["active_tool_frame"] = frameToJson(robot_->getT_tool_());
    data["active_object_frame_name"] = "default";
    data["active_object_frame"] = frameToJson(robot_->getT_object_());
    data["t_tool"] = data["active_tool_frame"];
    data["t_object"] = data["active_object_frame"];
    data["pose_out"] = frameToJson(robot_->getPose_out());
    data["calibration_feedback"] = {{"error_state", robot_->getErrorStateOfCal()}};
    auto hw = robot_->getHardwareInterface();
    data["hw_state"] = {{"joint_num", robot_->getJointNum()}, {"state", static_cast<int>(hw->getHardwareState())},
        {"hw_type", static_cast<int>(hw->getHardwareType())}, {"timestamp", hw->getTimestamp()},
        {"min_cycle_time", hw->getMinCycleTime()}, {"max_cycle_time", hw->getMaxCycleTime()},
        {"current_cycle_time", hw->getCurrCycleTime()}, {"slave_num", hw->getSlaveNumber()}};
    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleGetRobotInfo(const httplib::Request& req, httplib::Response& res) {
    nlohmann::json infos = nlohmann::json::array();
    for (int i = 0; i < robot_->getJointNum(); ++i) {
        infos.push_back({{"id", i}, {"name", robot_->getJointName(i)},
            {"cnt_per_unit", robot_->getJointCntPerUnit(i)},
            {"torque_per_unit", robot_->getJointTorquePerUnit(i)},
            {"ratio", robot_->getJointRatio(i)},
            {"unit_name", robot_->getJointUserUnitName(i)},
            {"zero_offset", robot_->getJointPosZeroOffset(i)}});
    }
    sendJson(res, true, 0, "ok", {{"joint_infos", infos}, {"api_profile", "legacy_robot"},
        {"capabilities", {{"joint_jerk", true}, {"cartesian_jerk", false},
                          {"jog_single_direction", true}, {"servo_mode_switch", false}}}});
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

void RobotHttpServer::handleGetImpedance(const httplib::Request& /*req*/,
                                          httplib::Response& res) {
    sendUnsupported(res, "Impedance parameter query");
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
    if (rel_path.find("..") != std::string::npos || fs::path(rel_path).is_absolute()) {
        sendJson(res, false, 1001, "Path traversal not allowed");
        return;
    }

    // 提取文件名，用于回退搜索
    const auto sep = rel_path.find_last_of("/\\");
    const std::string filename = (sep != std::string::npos)
        ? rel_path.substr(sep + 1)
        : rel_path;

    // Resolve relative to the active URDF, independent of the process cwd.
    // Do not search neighboring robot models for identically named meshes.
    const fs::path model_dir = fs::absolute(getUrdfPath()).parent_path();
    const std::vector<fs::path> search_bases = {model_dir};

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

    if (found_path.empty() && fs::is_regular_file(model_dir / filename)) {
        found_path = (model_dir / filename).string();
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
    if (!ensureControl(req, res)) return;
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!requireIdle(res)) return;
    robot_->setEnabled();
    const bool enabled = robotEnabled(robot_);
    sendJson(res, enabled, enabled ? 0 : 1004,
             enabled ? "Robot enabled" : "Robot enable failed",
             {{"robot_state", robotState(robot_)}, {"enabled", enabled}});
}

void RobotHttpServer::handleDisable(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!requireIdle(res)) return;
    robot_->setDisabled();
    const bool disabled = robotDisabled(robot_);
    sendJson(res, disabled, disabled ? 0 : 1004,
             disabled ? "Robot disabled" : "Robot disable failed",
             {{"robot_state", robotState(robot_)}, {"enabled", robotEnabled(robot_)}});
}

void RobotHttpServer::handleReset(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Fault reset");
}

void RobotHttpServer::handleIsEnabled(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/robot/enabled");
    nlohmann::json data;
    data["enabled"] = robotEnabled(robot_);
    data["disabled"] = robotDisabled(robot_);
    data["robot_state"] = robotState(robot_);
    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleSetWorkMode(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    const auto body = nlohmann::json::parse(req.body, nullptr, false);
    if (!body.is_object() || !body.contains("mode")) {
        sendJson(res, false, 1001, "Missing mode"); return;
    }
    Robot::WorkMode value;
    if (body["mode"].is_number_integer()) {
        const int n = body["mode"].get<int>();
        if (n < 0 || n > 4) { sendJson(res, false, 1006, "Invalid work mode"); return; }
        value = static_cast<Robot::WorkMode>(n);
    } else if (body["mode"].is_string()) {
        const auto mode = body["mode"].get<std::string>();
        if (mode == "position" || mode == "Position") value = Robot::WorkMode::Position;
        else if (mode == "ee_admit_teach" || mode == "EeAdmitTeach") value = Robot::WorkMode::EeAdmitTeach;
        else if (mode == "jnt_admit_teach" || mode == "JntAdmitTeach") value = Robot::WorkMode::JntAdmitTeach;
        else if (mode == "jnt_imp" || mode == "JntImp") value = Robot::WorkMode::JntImp;
        else if (mode == "cart_imp" || mode == "CartImp") value = Robot::WorkMode::CartImp;
        else { sendJson(res, false, 1006, "Invalid work mode"); return; }
    } else { sendJson(res, false, 1001, "mode must be a string or integer"); return; }
    std::lock_guard<std::mutex> lock(command_mutex_);
    const bool motion_blocked = robot_->isMotionBusy() && robot_->getMotionType() != Robot::MotionType::Teaching;
    const bool ok = robot_->setWorkMode(value);
    sendJson(res, ok, ok ? 0 : motion_blocked ? 3003 : 1004,
        ok ? "Work mode switched" : motion_blocked ? "Cannot switch work mode during this motion" : "Work mode activation failed; check controller configuration",
        {{"mode", workModeName(robot_)}, {"robot_state", robotState(robot_)},
         {"controller_implemented", value != Robot::WorkMode::JntImp && value != Robot::WorkMode::CartImp}});
}

void RobotHttpServer::handleSingleAxisCommand(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    const auto body = nlohmann::json::parse(req.body, nullptr, false);
    if (!body.is_object() || !body.contains("id") || !body["id"].is_number_integer()) {
        sendJson(res, false, 1001, "Expected integer joint id"); return;
    }
    const int id = body["id"].get<int>();
    if (id < 0 || id >= robot_->getJointNum()) { sendJson(res, false, 1002, "Joint id out of range"); return; }
    const auto action = req.path.substr(req.path.find_last_of('/') + 1);
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (action == "stop") {
        if (robot_->isMotionBusy() && robot_->getMotionType() != Robot::MotionType::Axis) {
            sendJson(res, false, 3003, "Use unified stop for a coordinated trajectory or jogging"); return;
        }
        robot_->stopSingleAxis(id); sendMotionResult(res, 0, "Axis stop requested"); return;
    }
    if (action != "move" && !requireIdle(res)) return;
    if (action == "enable" || action == "disable") {
        if (action == "enable") robot_->setJointEnabled(id); else robot_->setJointDisabled(id);
        sendJson(res, true, 0, "Axis state command completed", {{"id", id}, {"status", jointStatusName(robot_->getJointStatus(id))}}); return;
    }
    if (robot_->isMotionBusy() && robot_->getMotionType() != Robot::MotionType::Axis) {
        sendJson(res, false, 3003, "Another motion is running"); return;
    }
    if (robot_->getWorkMode() != Robot::WorkMode::Position || !positionMode(robot_, id)) {
        sendJson(res, false, 3005, "Axis movement requires position work mode and CSP/CSV"); return;
    }
    if (robot_->getJointStatus(id) != 2) { sendJson(res, false, 3001, "Joint is disabled"); return; }
    if (!body.contains("pos") || !body["pos"].is_number()) { sendJson(res, false, 1001, "Missing numeric pos"); return; }
    const double scale = body.value("raw_data", false) ? robot_->getJointCntPerUnit(id) : 1.0;
    if (!std::isfinite(scale) || scale <= 0) { sendJson(res, false, 1001, "Invalid encoder scale"); return; }
    const double pos = body["pos"].get<double>() / scale;
    const double velocity = body.contains("max_vel") ? body["max_vel"].get<double>() / scale : robot_->getJntVelLimit(id);
    const double acceleration = body.contains("max_acc") ? body["max_acc"].get<double>() / scale : robot_->getJntAccLimit(id);
    const double jerk = body.contains("max_jerk") ? body["max_jerk"].get<double>() / scale : robot_->getJntJerkLimit(id);
    const double time = body.value("least_time", 0.0);
    if (!isFinite(pos) || !isFinite(velocity) || !isFinite(acceleration) || !isFinite(jerk) || !isFinite(time) ||
        velocity <= 0 || acceleration <= 0 || jerk <= 0 || time < 0 ||
        velocity > robot_->getJointMaxVelocity(id) || acceleration > robot_->getJointMaxAcceleration(id) ||
        jerk > robot_->getJointMaxJerk(id) ||
        pos < robot_->getJointMinPosition(id) || pos > robot_->getJointMaxPosition(id)) {
        sendJson(res, false, 1001, "Invalid axis position or motion limits"); return;
    }
    robot_->moveSingleAxis(id, pos, 0, velocity, acceleration, jerk, time);
    sendMotionResult(res, 0, "Axis move accepted");
}

void RobotHttpServer::handleMultiAxisCommand(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    const auto body = req.body.empty() ? nlohmann::json::object() : nlohmann::json::parse(req.body, nullptr, false);
    if (!body.is_object()) { sendJson(res, false, 1001, "Invalid JSON"); return; }
    const auto action = req.path.substr(req.path.find_last_of('/') + 1);
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (action == "stop") {
        if (robot_->isMotionBusy() && robot_->getMotionType() != Robot::MotionType::Axis) {
            sendJson(res, false, 3003, "Use unified stop for this motion"); return;
        }
        robot_->stopMultiAxis(); sendMotionResult(res, 0, "Multi-axis stop requested"); return;
    }
    if (!requireIdle(res)) return;
    const int n = robot_->getJointNum();
    if (action == "enable") { robot_->setEnabled(); sendMotionResult(res, 0, "Enable command completed"); return; }
    if (action == "disable") { robot_->setDisabled(); sendMotionResult(res, 0, "Disable command completed"); return; }
    Robot::Synchronization sync = robot_->getSynchronization();
    if (body.contains("sync") && !parseSync(body["sync"], sync)) { sendJson(res, false, 1001, "Invalid sync mode"); return; }
    if (action == "sync") {
        const auto value = body.contains("sync") ? body["sync"] : body.value("value", nlohmann::json());
        if (!parseSync(value, sync)) { sendJson(res, false, 1001, "Invalid sync mode"); return; }
        robot_->setSynchronization(sync); sendJson(res, true, 0, "Synchronization switched", {{"sync", static_cast<int>(sync)}}); return;
    }
    if (robot_->getWorkMode() != Robot::WorkMode::Position || !robotEnabled(robot_)) {
        sendJson(res, false, 3005, "Movement requires enabled position work mode"); return;
    }
    std::vector<double> pos, vel = robot_->getJntVelLimits(), acc = robot_->getJntAccLimits(), jerk = robot_->getJntJerkLimits();
    const auto target = body.contains("target_pos") ? body["target_pos"] : body.value("joints", nlohmann::json());
    if (!validVector(target, n, pos)) { sendJson(res, false, 1003, "Expected target_pos for every joint"); return; }
    for (const auto* key : {"max_vel", "max_acc", "max_jerk"}) {
        auto& values = std::string(key) == "max_vel" ? vel : std::string(key) == "max_acc" ? acc : jerk;
        if (body.contains(key) && !validVector(body[key], n, values)) { sendJson(res, false, 1003, "Invalid motion-limit array"); return; }
    }
    const bool raw = body.value("raw_data", false);
    const double time = body.value("least_time", 0.0);
    if (!isFinite(time) || time < 0) { sendJson(res, false, 1001, "Invalid least_time"); return; }
    for (int i = 0; i < n; ++i) {
        if (!positionMode(robot_, i)) { sendJson(res, false, 3005, "Expected CSP/CSV joint modes"); return; }
        if (raw) {
            const double scale = robot_->getJointCntPerUnit(i);
            if (!isFinite(scale) || scale <= 0) { sendJson(res, false, 1001, "Invalid encoder scale"); return; }
            pos[i] /= scale;
            if (body.contains("max_vel")) vel[i] /= scale;
            if (body.contains("max_acc")) acc[i] /= scale;
            if (body.contains("max_jerk")) jerk[i] /= scale;
        }
        if (pos[i] < robot_->getJointMinPosition(i) || pos[i] > robot_->getJointMaxPosition(i) ||
            vel[i] <= 0 || acc[i] <= 0 || jerk[i] <= 0 ||
            vel[i] > robot_->getJointMaxVelocity(i) || acc[i] > robot_->getJointMaxAcceleration(i) ||
            jerk[i] > robot_->getJointMaxJerk(i)) { sendJson(res, false, 1001, "Invalid axis position or motion limits"); return; }
    }
    robot_->setSynchronization(sync);
    robot_->moveMultiAxis(pos, std::vector<double>(n, 0), vel, acc, jerk, time);
    sendMotionResult(res, 0, "Multi-axis move accepted");
}

void RobotHttpServer::handleSetLoadParameters(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Load parameters");
}

void RobotHttpServer::handleGetLoadParameters(const httplib::Request& req, httplib::Response& res) {
    sendUnsupported(res, "Load parameters");
}

void RobotHttpServer::handleSaveLoadParameters(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Load parameter persistence");
}

// ============================================================================
// Servo Control Handlers
// ============================================================================

void RobotHttpServer::handleServoStart(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "UDP servo control");
}

void RobotHttpServer::handleServoStop(const httplib::Request& req, httplib::Response& res) {
    handleStop(req, res);
}

void RobotHttpServer::handleServoMode(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Servo mode selection");
}

// ============================================================================
// Motion Control Handlers
// ============================================================================

void RobotHttpServer::handleMoveJ(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/movej");
    if (!ensureControl(req, res)) return;

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
    const double jerk = body.value("jerk", -1.0);
    if (body.contains("jerk") && (!isFinite(jerk) || jerk <= 0)) {
        sendJson(res, false, 1001, "jerk must be positive and finite"); return;
    }
    const double time = jsonNumberOr(body, "time", nullptr, 0.0);
    const double radius = jsonNumberOr(body, "radius", nullptr, 0.0);
    const bool asynchronous = body.value("asynchronous", false);
    if (!isFinite(time) || !isFinite(radius) || time < 0 || radius < 0) {
        sendJson(res, false, 1001, "time and radius must be nonnegative finite numbers");
        return;
    }
    if (!isFinite(velocity) || !isFinite(acceleration) ||
        velocity <= 0.0 || acceleration <= 0.0) {
        sendJson(res, false, 1001, "velocity and acceleration must be positive finite numbers");
        return;
    }

    std::unique_lock<std::mutex> command_lock(command_mutex_);
    if (!requireIdle(res)) return;
    if (robot_->getWorkMode() != Robot::WorkMode::Position) {
        sendJson(res, false, 3005, "Motion requires position work mode"); return;
    }
    if (body.contains("jerk")) {
        for (int i = 0; i < robot_->getJointNum(); ++i) {
            if (jerk > robot_->getJointMaxJerk(i)) {
                sendJson(res, false, 1001, "jerk exceeds a joint limit"); return;
            }
        }
        std::vector<double> limits(robot_->getJointNum(), jerk);
        robot_->setJntJerkLimits(limits);
    }
    const int result = robot_->MoveJ(q_goal, velocity, acceleration, time, radius, true);
    command_lock.unlock();
    sendMotionResult(res, result, "Motion accepted", asynchronous);
}

void RobotHttpServer::handleMoveJ_IK(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/movej_ik");
    if (!ensureControl(req, res)) return;

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
    const double jerk = body.value("jerk", -1.0);
    if (body.contains("jerk") && (!isFinite(jerk) || jerk <= 0)) {
        sendJson(res, false, 1001, "jerk must be positive and finite"); return;
    }
    const double time = jsonNumberOr(body, "time", nullptr, 0.0);
    const double radius = jsonNumberOr(body, "radius", nullptr, 0.0);
    const bool asynchronous = body.value("asynchronous", false);
    if (!isFinite(time) || !isFinite(radius) || time < 0 || radius < 0) {
        sendJson(res, false, 1001, "time and radius must be nonnegative finite numbers");
        return;
    }
    if (!isFinite(velocity) || !isFinite(acceleration) ||
        velocity <= 0.0 || acceleration <= 0.0) {
        sendJson(res, false, 1001, "velocity and acceleration must be positive finite numbers");
        return;
    }

    std::unique_lock<std::mutex> command_lock(command_mutex_);
    if (!requireIdle(res)) return;
    if (robot_->getWorkMode() != Robot::WorkMode::Position) {
        sendJson(res, false, 3005, "Motion requires position work mode"); return;
    }
    if (body.contains("jerk")) {
        for (int i = 0; i < robot_->getJointNum(); ++i) {
            if (jerk > robot_->getJointMaxJerk(i)) {
                sendJson(res, false, 1001, "jerk exceeds a joint limit"); return;
            }
        }
        std::vector<double> limits(robot_->getJointNum(), jerk);
        robot_->setJntJerkLimits(limits);
    }
    const int result = robot_->MoveJ_IK(pose, velocity, acceleration, time, radius, true);
    command_lock.unlock();
    sendMotionResult(res, result, "Motion accepted", asynchronous);
}

void RobotHttpServer::handleMoveL(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/movel");
    if (!ensureControl(req, res)) return;

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

    if (body.contains("tool_name")) {
        sendUnsupported(res, "Named tool selection for motion");
        return;
    }
    const double velocity = jsonNumberOr(body, "velocity", "speed", 1.0);
    const double acceleration = jsonNumberOr(body, "acceleration", nullptr, 2.0);
    if (body.contains("jerk")) {
        sendUnsupported(res, "Per-command jerk for Cartesian/MoveJ motion");
        return;
    }
    const double time = jsonNumberOr(body, "time", nullptr, 0.0);
    const double radius = jsonNumberOr(body, "radius", nullptr, 0.0);
    const bool asynchronous = body.value("asynchronous", false);
    if (!isFinite(time) || !isFinite(radius) || time < 0 || radius < 0) {
        sendJson(res, false, 1001, "time and radius must be nonnegative finite numbers");
        return;
    }
    if (!isFinite(velocity) || !isFinite(acceleration) ||
        velocity <= 0.0 || acceleration <= 0.0) {
        sendJson(res, false, 1001, "velocity and acceleration must be positive finite numbers");
        return;
    }

    std::unique_lock<std::mutex> command_lock(command_mutex_);
    if (!requireIdle(res)) return;
    if (robot_->getWorkMode() != Robot::WorkMode::Position) {
        sendJson(res, false, 3005, "Motion requires position work mode"); return;
    }
    const int result = robot_->MoveL(pose, velocity, acceleration, time, radius, true);
    command_lock.unlock();
    sendMotionResult(res, result, "Motion accepted", asynchronous);
}

void RobotHttpServer::handleMoveL_FK(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/movel_fk");
    if (!ensureControl(req, res)) return;

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
    if (body.contains("jerk")) {
        sendUnsupported(res, "Per-command jerk for Cartesian/MoveJ motion");
        return;
    }
    const double time = jsonNumberOr(body, "time", nullptr, 0.0);
    const double radius = jsonNumberOr(body, "radius", nullptr, 0.0);
    const bool asynchronous = body.value("asynchronous", false);
    if (!isFinite(time) || !isFinite(radius) || time < 0 || radius < 0) {
        sendJson(res, false, 1001, "time and radius must be nonnegative finite numbers");
        return;
    }
    if (!isFinite(velocity) || !isFinite(acceleration) ||
        velocity <= 0.0 || acceleration <= 0.0) {
        sendJson(res, false, 1001, "velocity and acceleration must be positive finite numbers");
        return;
    }

    if (body.contains("tool_name")) {
        sendUnsupported(res, "Named tool selection for motion");
        return;
    }
    std::unique_lock<std::mutex> command_lock(command_mutex_);
    if (!requireIdle(res)) return;
    if (robot_->getWorkMode() != Robot::WorkMode::Position) {
        sendJson(res, false, 3005, "Motion requires position work mode"); return;
    }
    const int result = robot_->MoveL_FK(q_goal, velocity, acceleration, time, radius, true);
    command_lock.unlock();
    sendMotionResult(res, result, "Motion accepted", asynchronous);
}

void RobotHttpServer::handleMoveC(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/movec");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded()) {
        sendJson(res, false, 1001, "Invalid JSON body");
        return;
    }

    if (body.contains("pose_start")) {
        sendUnsupported(res, "Explicit MoveC start pose");
        return;
    }

    const double velocity = jsonNumberOr(body, "velocity", "speed", 1.0);
    const double acceleration = jsonNumberOr(body, "acceleration", nullptr, 2.0);
    if (body.contains("jerk")) {
        sendUnsupported(res, "Per-command jerk for Cartesian/MoveJ motion");
        return;
    }
    const double time = jsonNumberOr(body, "time", nullptr, 0.0);
    const double radius = jsonNumberOr(body, "radius", nullptr, 0.0);
    const bool asynchronous = body.value("asynchronous", false);
    if (!isFinite(time) || !isFinite(radius) || time < 0 || radius < 0) {
        sendJson(res, false, 1001, "time and radius must be nonnegative finite numbers");
        return;
    }
    if (!isFinite(velocity) || !isFinite(acceleration) ||
        velocity <= 0.0 || acceleration <= 0.0) {
        sendJson(res, false, 1001, "velocity and acceleration must be positive finite numbers");
        return;
    }

    std::unique_lock<std::mutex> command_lock(command_mutex_);
    if (!requireIdle(res)) return;
    if (robot_->getWorkMode() != Robot::WorkMode::Position) {
        sendJson(res, false, 3005, "Motion requires position work mode"); return;
    }
    int result = -1;
    if (body.contains("center_frame")) {
        sendUnsupported(res, "MoveC center_frame variant");
        return;
    } else if (body.contains("pose_via") && (body.contains("pose_goal") || body.contains("pose_to"))) {
        Frame pose_via;
        Frame pose_goal;
        const auto& goal_json = body.contains("pose_goal") ? body["pose_goal"] : body["pose_to"];
        if (!parseFrameJson(body["pose_via"], pose_via) || !parseFrameJson(goal_json, pose_goal)) {
            sendJson(res, false, 1001, "Invalid 'pose_via' or 'pose_goal' frame");
            return;
        }
        result = robot_->MoveC(pose_via, pose_goal, velocity, acceleration, time, radius, Robot::UNCONSTRAINED, true);
    } else {
        sendJson(res, false, 1001,
                 "MoveC requires 'pose_via' and 'pose_goal' (or 'pose_to')");
        return;
    }

    command_lock.unlock();
    sendMotionResult(res, result, "Motion accepted", asynchronous);
}

void RobotHttpServer::handleStop(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    std::lock_guard<std::mutex> lock(command_mutex_);
    auto before = motionData();
    robot_->requestMotionStop();
    jog_active_ = false;
    auto data = motionData(); data["stopped_motion_type"] = before["motion_type"];
    sendJson(res, true, 0, "Stop requested", data);
}

void RobotHttpServer::handlePause(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Motion pause");
}

void RobotHttpServer::handleResume(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Motion resume");
}

void RobotHttpServer::handleWaitMove(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    auto body = req.body.empty() ? nlohmann::json::object() : nlohmann::json::parse(req.body, nullptr, false);
    if (!body.is_object()) { sendJson(res, false, 1001, "Invalid JSON"); return; }
    const double timeout = body.value("timeout", 30.0);
    if (!std::isfinite(timeout) || timeout <= 0) { sendJson(res, false, 1001, "timeout must be positive"); return; }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    while (robot_->isMotionBusy() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (robot_->isMotionBusy()) { sendJson(res, false, 5002, "WaitMove timed out", motionData()); return; }
    sendMotionResult(res, robot_->getLastMotionResult(), "Motion finished");
}

// ============================================================================
// Async Task Query
// ============================================================================

void RobotHttpServer::handleMoveStatus(const httplib::Request& req, httplib::Response& res) {
    auto data = motionData();
    const auto id = req.get_param_value("task_id");
    if (!id.empty()) { data["task_id"] = id; data["task"] = getTaskInfo(id); }
    sendJson(res, true, 0, "ok", data);
}

// ============================================================================
// Lua Script Handlers
// ============================================================================

void RobotHttpServer::handleScriptUpload(const httplib::Request& req,
                                         httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Lua scripts");
}

void RobotHttpServer::handleScriptRun(const httplib::Request& req,
                                      httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Lua scripts");
}

void RobotHttpServer::handleScriptPause(const httplib::Request& req,
                                        httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Lua scripts");
}

void RobotHttpServer::handleScriptResume(const httplib::Request& req,
                                         httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Lua scripts");
}

void RobotHttpServer::handleScriptStop(const httplib::Request& req,
                                       httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Lua scripts");
}

void RobotHttpServer::handleScriptStep(const httplib::Request& req,
                                       httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Lua scripts");
}

void RobotHttpServer::handleScriptStatus(const httplib::Request&,
                                         httplib::Response& res) {
    sendUnsupported(res, "Lua scripts");
}

void RobotHttpServer::handleScriptList(const httplib::Request&,
                                       httplib::Response& res) {
    sendUnsupported(res, "Lua scripts");
}

void RobotHttpServer::handleScriptDownload(const httplib::Request& req,
                                           httplib::Response& res) {
    sendUnsupported(res, "Lua scripts");
}

void RobotHttpServer::handleScriptBreakpointAdd(const httplib::Request& req,
                                                httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Lua scripts");
}

void RobotHttpServer::handleScriptBreakpointRemove(const httplib::Request& req,
                                                   httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Lua scripts");
}

void RobotHttpServer::handleScriptBreakpointClear(const httplib::Request& req,
                                                  httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Lua scripts");
}


// ============================================================================
// Jogging Compatibility Handler — 旧 flag 格式转换
// ============================================================================

void RobotHttpServer::handleVectorJog(const httplib::Request& req, httplib::Response& res,
                                      const std::string& kind) {
    if (!ensureControl(req, res)) return;
    const auto body = nlohmann::json::parse(req.body, nullptr, false);
    if (!body.is_object()) { sendJson(res, false, 1001, "Invalid JSON"); return; }
    const int n = robot_->getJointNum();
    const char* key = kind == "cartesian" ? "twist" : kind == "svd" ? "dim_speeds" : "joints";
    const int size = kind == "cartesian" ? 6 : kind == "svd" ? n - 6 : n;
    if ((kind == "svd" || kind == "nullspace") && n != 7) {
        sendUnsupported(res, "Legacy nullspace jogging requires a seven-joint robot"); return;
    }
    std::vector<double> values;
    if (!body.contains(key) || !validVector(body[key], size, values)) { sendJson(res, false, 1003, "Invalid direction-vector length or values"); return; }
    const double threshold = body.value("dir_threshold", 1e-6);
    const double timeout = body.value("timeout", 0.3);
    double speed = body.value("speed", 0.25);
    double acceleration = jsonNumberOr(body, "acceleration", "max_acceleration", 1.0);
    if (!isFinite(threshold) || threshold < 0 || !isFinite(timeout) || timeout <= 0 || timeout > 5 ||
        !isFinite(speed) || speed <= 0 || !isFinite(acceleration) || acceleration <= 0) {
        sendJson(res, false, 1001, "Invalid jogging speed, acceleration, threshold or timeout"); return;
    }
    int axis = -1;
    for (int i = 0; i < size; ++i) {
        if (std::abs(values[i]) <= threshold) continue;
        if (axis != -1 && kind != "nullspace") { sendUnsupported(res, "Simultaneous vector directions are not representable by legacy Dragging"); return; }
        axis = i;
    }
    int flag = axis;
    if (kind == "cartesian") {
        const auto frame = body.value("frame", std::string("BASE"));
        int base;
        if (frame == "BASE") base = 400;
        else if (frame == "FLANGE") base = 200;
        else if (frame == "TOOL") base = 100;
        else if (frame == "OBJECT") base = 300;
        else { sendJson(res, false, 1001, "Invalid jogging frame"); return; }
        flag = base + std::max(axis, 0);
    } else if (kind == "svd" || kind == "nullspace") {
        flag = 500;
        if (kind == "svd" && axis >= 0) speed = std::abs(values[axis]);
    }
    int direction = axis < 0 ? 0 : values[axis] > 0 ? 1 : -1;
    if (kind == "nullspace" && axis >= 0) {
        double scale;
        if (!nullspaceDirection(robot_, values, threshold, direction, scale)) {
            sendJson(res, false, 2001, "Cannot map joint intent to legacy nullspace at this configuration"); return;
        }
        if (direction != 0) speed *= scale;
    }
    dispatchJog(res, flag, direction, speed, acceleration, timeout);
}

void RobotHttpServer::handleJogCompat(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    const auto body = nlohmann::json::parse(req.body, nullptr, false);
    if (!body.is_object() || !body.contains("flag") || !body["flag"].is_number_integer()) {
        sendJson(res, false, 1001, "Expected numeric Dragging flag"); return;
    }
    const int flag = body["flag"].get<int>();
    const bool valid = (flag >= 0 && flag < robot_->getJointNum()) || flag == 500 ||
        ((flag / 100 >= 1 && flag / 100 <= 4) && flag % 100 <= 5);
    if (!valid || (flag == 500 && robot_->getJointNum() != 7)) { sendJson(res, false, 1002, "Invalid Dragging flag"); return; }
    const auto dir = body.contains("dir") ? body["dir"] : body.value("direction", nlohmann::json("NONE"));
    double sign = 0;
    if (dir.is_string()) {
        if (!directionSign(dir.get<std::string>(), sign)) { sendJson(res, false, 1001, "Invalid dragging direction"); return; }
    } else if (dir.is_number_integer() && std::abs(dir.get<int>()) <= 1) sign = dir.get<int>();
    else { sendJson(res, false, 1001, "Invalid dragging direction"); return; }
    const double speed = jsonNumberOr(body, "max_speed", "speed", 0.25);
    const double acc = jsonNumberOr(body, "max_acceleration", "acceleration", 1.0);
    const double timeout = body.value("timeout", 0.3);
    if (!isFinite(speed) || speed <= 0 || !isFinite(acc) || acc <= 0 || !isFinite(timeout) || timeout <= 0 || timeout > 5) {
        sendJson(res, false, 1001, "Invalid Dragging parameters"); return;
    }
    dispatchJog(res, flag, static_cast<int>(sign), speed, acc, timeout);
}

// ============================================================================
// Jogging Handlers
// ============================================================================

void RobotHttpServer::handleJogJoint(const httplib::Request& req, httplib::Response& res) {
    handleVectorJog(req, res, "joint");
}

void RobotHttpServer::handleJogCartesian(const httplib::Request& req, httplib::Response& res) {
    handleVectorJog(req, res, "cartesian");
}

void RobotHttpServer::handleJogNullspace(const httplib::Request& req, httplib::Response& res) {
    handleVectorJog(req, res, "nullspace");
}

void RobotHttpServer::handleJogSvd(const httplib::Request& req, httplib::Response& res) {
    handleVectorJog(req, res, "svd");
}

void RobotHttpServer::handleJogStop(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (robot_->getMotionType() == Robot::MotionType::Dragging) robot_->requestMotionStop();
    jog_active_ = false;
    sendMotionResult(res, 0, "Jog stop requested");
}

// ============================================================================
// Frame Management Handlers
// ============================================================================

void RobotHttpServer::handleGetToolFrameNames(const httplib::Request& req, httplib::Response& res) {
    sendJson(res, true, 0, "ok", {{"names", {"default"}}, {"active", "default"}});
}

void RobotHttpServer::handleGetObjectFrameNames(const httplib::Request& req, httplib::Response& res) {
    sendJson(res, true, 0, "ok", {{"names", {"default"}}, {"active", "default"}});
}

void RobotHttpServer::handleGetToolFrame(const httplib::Request& req, httplib::Response& res) {
    if (req.has_param("name") && req.get_param_value("name") != "default") {
        sendUnsupported(res, "Named frames");
        return;
    }
    sendJson(res, true, 0, "ok", {{"name", "default"}, {"frame", frameToJson(robot_->getT_tool_())}});
}

void RobotHttpServer::handleGetObjectFrame(const httplib::Request& req, httplib::Response& res) {
    if (req.has_param("name") && req.get_param_value("name") != "default") {
        sendUnsupported(res, "Named frames");
        return;
    }
    sendJson(res, true, 0, "ok", {{"name", "default"}, {"frame", frameToJson(robot_->getT_object_())}});
}

void RobotHttpServer::handleSetPoseFrame(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    const auto body = nlohmann::json::parse(req.body, nullptr, false);
    KDL::Frame frame;
    if (!body.is_object() || !body.contains("id") || !body["id"].is_number_integer() ||
        !body.contains("pose") || !parseFrameJson(body["pose"], frame)) {
        sendJson(res, false, 1001, "Expected integer id and pose");
        return;
    }
    const int id = body["id"].get<int>();
    if (id < 1 || id > 9) {
        sendJson(res, false, 1001, "Calibration pose id must be between 1 and 9");
        return;
    }
    robot_->set_pose_frame(id, frame);
    sendJson(res, true, 0, "Calibration pose updated");
}

void RobotHttpServer::handleSetToolFrame(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!requireIdle(res)) return;
    const auto body = nlohmann::json::parse(req.body, nullptr, false);
    if (!body.is_object()) {
        sendJson(res, false, 1001, "Invalid JSON body");
        return;
    }
    if (body.contains("name") && body["name"] != "default") {
        sendUnsupported(res, "Named frames");
        return;
    }
    KDL::Frame frame;
    const auto& value = body.contains("frame") ? body["frame"] : body.value("pose", nlohmann::json());
    if (!parseFrameJson(value, frame)) {
        sendJson(res, false, 1001, "Invalid frame or pose");
        return;
    }
    robot_->set_tool_frame(frame);
    sendJson(res, true, 0, "Default frame updated", {{"frame", frameToJson(frame)}});
}

void RobotHttpServer::handleSetObjectFrame(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!requireIdle(res)) return;
    const auto body = nlohmann::json::parse(req.body, nullptr, false);
    if (!body.is_object()) {
        sendJson(res, false, 1001, "Invalid JSON body");
        return;
    }
    if (body.contains("name") && body["name"] != "default") {
        sendUnsupported(res, "Named frames");
        return;
    }
    KDL::Frame frame;
    const auto& value = body.contains("frame") ? body["frame"] : body.value("pose", nlohmann::json());
    if (!parseFrameJson(value, frame)) {
        sendJson(res, false, 1001, "Invalid frame or pose");
        return;
    }
    robot_->set_object_frame(frame);
    sendJson(res, true, 0, "Default frame updated", {{"frame", frameToJson(frame)}});
}

void RobotHttpServer::handleRemoveToolFrame(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Named tool frames");
}

void RobotHttpServer::handleRemoveObjectFrame(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Named object frames");
}

void RobotHttpServer::handleSetActiveToolFrame(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Named tool frames");
}

void RobotHttpServer::handleSetActiveObjectFrame(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Named object frames");
}

void RobotHttpServer::handleLoadFrames(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Named frame loading");
}

void RobotHttpServer::handleSaveFrames(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    sendUnsupported(res, "Named frame persistence");
}

// ============================================================================
// Calibration Handlers
// ============================================================================

void RobotHttpServer::handleCalibrationRun(const httplib::Request& req, httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    const auto body = nlohmann::json::parse(req.body, nullptr, false);
    if (!body.is_object() || !body.contains("frame") || !body["frame"].is_string()) {
        sendJson(res, false, 1001, "Expected frame: tool or object"); return;
    }
    const auto frame = body["frame"].get<std::string>();
    if (frame != "tool" && frame != "object") { sendJson(res, false, 4004, "Invalid calibration frame"); return; }
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!requireIdle(res)) return;
    const int start = frame == "tool" ? 1 : 7, end = frame == "tool" ? 6 : 9;
    for (int id = start; id <= end; ++id) if (!robot_->hasCalibrationPose(id)) {
        sendJson(res, false, 4001, "Calibration poses are incomplete"); return;
    }
    const auto origin = robot_->get_pose_frame(frame == "tool" ? 4 : 7);
    const auto a = robot_->get_pose_frame(frame == "tool" ? 5 : 8).p - origin.p;
    const auto b = robot_->get_pose_frame(frame == "tool" ? 6 : 9).p - origin.p;
    if (a.Norm() < 1e-9 || b.Norm() < 1e-9 || (a * b).Norm() < 1e-9) {
        sendJson(res, false, 4002, "Calibration points are coincident or collinear"); return;
    }
    if (frame == "tool") {
        Eigen::Matrix<double, 9, 3> rotations;
        for (int row = 0; row < 3; ++row) {
            const auto first = robot_->get_pose_frame(row + 1);
            const auto second = robot_->get_pose_frame(row + 2);
            for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
                rotations(row * 3 + i, j) = first.M(i, j) - second.M(i, j);
        }
        if (rotations.fullPivLu().rank() != 3) {
            sendJson(res, false, 4002, "Tool calibration rotations do not constrain all axes"); return;
        }
    }
    robot_->tool_calibration(frame);
    const auto pose = robot_->getPose_out();
    bool finite = true;
    for (int i = 0; i < 3; ++i) finite = finite && isFinite(pose.p(i));
    for (double value : pose.M.data) finite = finite && isFinite(value);
    const bool ok = !robot_->getErrorStateOfCal() && finite;
    sendJson(res, ok, ok ? 0 : 4002, ok ? "Calibration completed" : "Calibration failed",
             {{"frame", frame}, {"error_state", !ok}, {"pose", frameToJson(pose)}});
}

void RobotHttpServer::handleCalibrationResult(const httplib::Request& req, httplib::Response& res) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    sendJson(res, true, 0, "ok", {{"error_state", robot_->getErrorStateOfCal()},
                                  {"pose", frameToJson(robot_->getPose_out())}});
}

}  // namespace rocos
