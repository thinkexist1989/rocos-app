// Copyright 2021, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// HTTP REST API Server for ROCOS-App Robot Controller
// Replaces gRPC with HTTP/JSON using cpp-httplib

#include <rocos_app/robot_http_server.h>
#include <rocos_app/robot.h>

#include <spdlog/spdlog.h>

#include <kdl_parser/kdl_parser.hpp>

#include <chrono>
#include <ctime>
#include <fstream>
#include <sstream>

namespace rocos {

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
    // CORS preflight
    server_->Options(".*", [this](const httplib::Request&, httplib::Response& res) {
        setCorsHeaders(res);
    });

    // Robot state
    server_->Get("/api/robot/state", [this](auto& req, auto& res) { handleGetRobotState(req, res); });
    server_->Get("/api/robot/info", [this](auto& req, auto& res) { handleGetRobotInfo(req, res); });
    server_->Get("/api/robot/model", [this](auto& req, auto& res) { handleGetRobotModel(req, res); });
    server_->Get("/api/robot/model/mesh", [this](auto& req, auto& res) { handleGetLinkMesh(req, res); });
    server_->Get("/api/robot/enabled", [this](auto& req, auto& res) { handleIsEnabled(req, res); });
    server_->Post("/api/robot/enable", [this](auto& req, auto& res) { handleEnable(req, res); });
    server_->Post("/api/robot/disable", [this](auto& req, auto& res) { handleDisable(req, res); });
    server_->Post("/api/robot/workmode", [this](auto& req, auto& res) { handleSetWorkMode(req, res); });

    // Motion control
    server_->Post("/api/move/joint", [this](auto& req, auto& res) { handleMoveJ(req, res); });
    server_->Post("/api/move/joint_ik", [this](auto& req, auto& res) { handleMoveJ_IK(req, res); });
    server_->Post("/api/move/linear", [this](auto& req, auto& res) { handleMoveL(req, res); });
    server_->Post("/api/move/linear_fk", [this](auto& req, auto& res) { handleMoveL_FK(req, res); });
    server_->Post("/api/move/circle", [this](auto& req, auto& res) { handleMoveC(req, res); });
    server_->Post("/api/move/path", [this](auto& req, auto& res) { handleMoveP(req, res); });
    server_->Post("/api/move/pathway", [this](auto& req, auto& res) { handleMovePath(req, res); });
    server_->Post("/api/move/stop", [this](auto& req, auto& res) { handleStop(req, res); });
    server_->Get("/api/move/status", [this](auto& req, auto& res) { handleMoveStatus(req, res); });

    // Single axis
    server_->Post("/api/axis/single/enable", [this](auto& req, auto& res) { handleSingleAxisEnable(req, res); });
    server_->Post("/api/axis/single/disable", [this](auto& req, auto& res) { handleSingleAxisDisable(req, res); });
    server_->Post("/api/axis/single/move", [this](auto& req, auto& res) { handleSingleAxisMove(req, res); });
    server_->Post("/api/axis/single/stop", [this](auto& req, auto& res) { handleSingleAxisStop(req, res); });

    // Multi axis
    server_->Post("/api/axis/multi/enable", [this](auto& req, auto& res) { handleMultiAxisEnable(req, res); });
    server_->Post("/api/axis/multi/disable", [this](auto& req, auto& res) { handleMultiAxisDisable(req, res); });
    server_->Post("/api/axis/multi/move", [this](auto& req, auto& res) { handleMultiAxisMove(req, res); });
    server_->Post("/api/axis/multi/stop", [this](auto& req, auto& res) { handleMultiAxisStop(req, res); });
    server_->Post("/api/axis/multi/sync", [this](auto& req, auto& res) { handleMultiAxisSync(req, res); });

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
    return robot_->urdf_file_path_;
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

    data["robot_state"] = robot_->GetRobotState();
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
    hw["hw_type"] = static_cast<int>(robot_->hw_interface_->getHardwareType());
    hw["min_cycle_time"] = robot_->hw_interface_->getMinCycleTime();
    hw["max_cycle_time"] = robot_->hw_interface_->getMaxCycleTime();
    hw["current_cycle_time"] = robot_->hw_interface_->getCurrCycleTime();
    hw["slave_num"] = robot_->hw_interface_->getSlaveNumber();
    data["hw_state"] = hw;

    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleGetRobotInfo(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/robot/info");

    nlohmann::json joint_infos = nlohmann::json::array();
    for (int i = 0; i < robot_->getJointNum(); ++i) {
        nlohmann::json ji;
        ji["name"] = robot_->getJointName(i);
        ji["cnt_per_unit"] = robot_->getJointCntPerUnit(i);
        ji["torque_per_unit"] = robot_->getJointTorquePerUnit(i);
        ji["ratio"] = robot_->getJointRatio(i);
        ji["unit_name"] = robot_->getJointUserUnitName(i);
        ji["zero_offset"] = robot_->getJointPosZeroOffset(i);
        joint_infos.push_back(ji);
        log_ptr_->info("Joint {}: cnt_per_unit={}, torque_per_unit={}, ratio={}, unit_name={}, zero_offset={}",
                     ji["name"].get<std::string>(),
                     ji["cnt_per_unit"].get<double>(),
                     ji["torque_per_unit"].get<double>(),
                     ji["ratio"].get<double>(),
                     ji["unit_name"].get<std::string>(),
                     ji["zero_offset"].get<int32_t>());
    }

    nlohmann::json data;
    data["joint_infos"] = joint_infos;

    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleGetRobotModel(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/robot/model");

    urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDFFile(getUrdfPath());
    if (!robot_model) {
        sendJson(res, false, 1001, "Could not parse robot model");
        return;
    }

    nlohmann::json data;
    data["name"] = robot_model->getName();

    std::vector<urdf::LinkSharedPtr> links;
    robot_model->getLinks(links);

    nlohmann::json links_arr = nlohmann::json::array();
    for (size_t i = 0; i < links.size(); ++i) {
        nlohmann::json link;
        link["name"] = links[i]->name;
        link["order"] = static_cast<int>(i);

        if (links[i]->parent_joint) {
            // Joint type mapping
            std::string type_str;
            switch (links[i]->parent_joint->type) {
                case urdf::Joint::FIXED:      type_str = "fixed";      break;
                case urdf::Joint::REVOLUTE:   type_str = "revolute";   break;
                case urdf::Joint::PRISMATIC:  type_str = "prismatic";  break;
                case urdf::Joint::CONTINUOUS: type_str = "continuous"; break;
                default:                      type_str = "unknown";    break;
            }
            link["type"] = type_str;

            // Joint origin translation
            link["translate"] = {
                {"x", links[i]->parent_joint->parent_to_joint_origin_transform.position.x},
                {"y", links[i]->parent_joint->parent_to_joint_origin_transform.position.y},
                {"z", links[i]->parent_joint->parent_to_joint_origin_transform.position.z}
            };

            // Joint origin rotation (RPY)
            double roll, pitch, yaw;
            links[i]->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
            link["rotate"] = {
                {"x", roll},
                {"y", pitch},
                {"z", yaw}
            };

            // Joint axis
            link["axis"] = {
                {"x", links[i]->parent_joint->axis.x},
                {"y", links[i]->parent_joint->axis.y},
                {"z", links[i]->parent_joint->axis.z}
            };
        }

        if (links[i]->visual) {
            link["translateLink"] = {
                {"x", links[i]->visual->origin.position.x},
                {"y", links[i]->visual->origin.position.y},
                {"z", links[i]->visual->origin.position.z}
            };

            double roll_l, pitch_l, yaw_l;
            links[i]->visual->origin.rotation.getRPY(roll_l, pitch_l, yaw_l);
            link["rotateLink"] = {
                {"x", roll_l},
                {"y", pitch_l},
                {"z", yaw_l}
            };

            auto mesh_geo = std::dynamic_pointer_cast<urdf::Mesh>(links[i]->visual->geometry);
            if (mesh_geo) {
                std::string mesh_file = mesh_geo->filename;
                // Extract filename from path (e.g., "/foo/bar/link1.stl" -> "link1.stl")
                std::string::size_type pos = mesh_file.find_last_of("/\\");
                if (pos != std::string::npos) {
                    mesh_file = mesh_file.substr(pos + 1);
                }
                link["mesh"] = mesh_file;
            }
        }

        links_arr.push_back(link);
    }

    data["links"] = links_arr;

    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleGetLinkMesh(const httplib::Request& req, httplib::Response& res) {
    // Query parameter: ?path=relative/path/to/mesh.stl
    std::string path = req.get_param_value("path");
    log_ptr_->info("GET /api/robot/model/mesh path={}", path);

    if (path.empty()) {
        sendJson(res, false, 1001, "Missing 'path' query parameter");
        return;
    }

    std::string urdf_path = getUrdfPath();
    // Extract parent directory from urdf path
    std::string::size_type pos = urdf_path.find_last_of("/\\");
    std::string urdf_dir = (pos != std::string::npos) ? urdf_path.substr(0, pos) : ".";
    std::string file_path = urdf_dir + "/" + path;

    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
        log_ptr_->error("Failed to open mesh file: {}", file_path);
        sendJson(res, false, 1001, "Mesh file not found");
        return;
    }

    file.seekg(0, std::ios::end);
    std::streampos file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> buffer(file_size);
    file.read(buffer.data(), file_size);

    // Extract filename from path
    std::string filename = path;
    pos = filename.find_last_of("/\\");
    if (pos != std::string::npos) {
        filename = filename.substr(pos + 1);
    }

    setCorsHeaders(res);
    res.set_header("Content-Type", "application/octet-stream");
    res.set_header("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    res.set_content(std::string(buffer.data(), buffer.size()), "application/octet-stream");
}

// ============================================================================
// Robot Control Handlers
// ============================================================================

void RobotHttpServer::handleEnable(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/enable");
    robot_->SetEnabled();
    sendJson(res, true, 0, "Robot enabled");
}

void RobotHttpServer::handleDisable(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/disable");
    robot_->SetDisabled();
    sendJson(res, true, 0, "Robot disabled");
}

void RobotHttpServer::handleIsEnabled(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/robot/enabled");
    bool enabled = robot_->IsEnabled();
    nlohmann::json data;
    data["enabled"] = enabled;
    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleSetWorkMode(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/workmode");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("mode")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'mode' field");
        return;
    }

    std::string mode_str = body["mode"].get<std::string>();
    Robot::WorkMode mode;

    if (mode_str == "position") {
        mode = Robot::WorkMode::Position;
    } else if (mode_str == "ee_admit_teach") {
        mode = Robot::WorkMode::EeAdmitTeach;
    } else if (mode_str == "jnt_admit_teach") {
        mode = Robot::WorkMode::JntAdmitTeach;
    } else if (mode_str == "jnt_imp") {
        mode = Robot::WorkMode::JntImp;
    } else if (mode_str == "cart_imp") {
        mode = Robot::WorkMode::CartImp;
    } else {
        sendJson(res, false, 1006, "Unknown work mode: " + mode_str);
        return;
    }

    bool ok = robot_->setWorkMode(mode);
    if (ok) {
        sendJson(res, true, 0, "Work mode set to " + mode_str);
    } else {
        sendJson(res, false, 3002, "Failed to set work mode");
    }
}

// ============================================================================
// Motion Control Handlers
// ============================================================================

void RobotHttpServer::handleMoveJ(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/move/joint");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("joints")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'joints' array");
        return;
    }

    int jnt_num = robot_->getJointNum();
    auto joints_arr = body["joints"];
    if (static_cast<int>(joints_arr.size()) != jnt_num) {
        sendJson(res, false, 1003, "Joint count mismatch: expected " + std::to_string(jnt_num));
        return;
    }

    KDL::JntArray q(jnt_num);
    for (int i = 0; i < jnt_num; ++i) {
        q(i) = joints_arr[i].get<double>();
    }

    double speed = body.value("speed", 1.05);
    double acceleration = body.value("acceleration", 1.4);
    double time = body.value("time", 0.0);
    double radius = body.value("radius", 0.0);
    bool asynchronous = body.value("asynchronous", false);

    if (asynchronous) {
        std::string taskId = generateTaskId();
        registerTask(taskId, "MoveJ");

        submitTask([this, q, speed, acceleration, time, radius, taskId]() {
            int result = robot_->MoveJ(q, speed, acceleration, time, radius, false);
            if (result == 0) {
                updateTaskStatus(taskId, "COMPLETED", result, "ok");
            } else {
                updateTaskStatus(taskId, "FAILED", result, "MoveJ failed");
            }
        });

        nlohmann::json data;
        data["task_id"] = taskId;
        sendJson(res, true, 0, "MoveJ started", data);
    } else {
        std::atomic<bool> done{false};
        std::atomic<int> result{-1};
        submitTask([&]() {
            result = robot_->MoveJ(q, speed, acceleration, time, radius, false);
            done = true;
        });
        // Wait for completion with timeout (30s)
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
        while (!done && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!done) {
            sendJson(res, false, 5002, "Motion timed out after 30s");
        } else if (result == 0) {
            sendJson(res, true, 0, "MoveJ completed");
        } else {
            sendJson(res, false, 2004, "MoveJ failed with code " + std::to_string(result));
        }
    }
}

void RobotHttpServer::handleMoveJ_IK(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/move/joint_ik");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("pose")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'pose' object");
        return;
    }

    nlohmann::json pose_json = body["pose"];
    KDL::Frame pose;
    if (pose_json.contains("position") && pose_json.contains("orientation")) {
        pose = jsonToFrame(pose_json["position"], pose_json["orientation"]);
    } else {
        pose = jsonToFrame(pose_json);
    }

    double speed = body.value("speed", 1.05);
    double acceleration = body.value("acceleration", 1.4);
    double time = body.value("time", 0.0);
    double radius = body.value("radius", 0.0);
    bool asynchronous = body.value("asynchronous", false);

    // Get current joint positions as initial guess for IK
    int jnt_num = robot_->getJointNum();
    KDL::JntArray q_init(jnt_num);
    for (int i = 0; i < jnt_num; ++i) {
        q_init(i) = robot_->getJointPosition(i);
    }

    // Perform IK
    KDL::JntArray q_target(jnt_num);
    int ik_result = robot_->CartToJnt(q_init, pose, q_target);

    if (ik_result < 0) {
        sendJson(res, false, 2001, "Inverse kinematics: no solution found for target pose");
        return;
    }

    if (asynchronous) {
        std::string taskId = generateTaskId();
        registerTask(taskId, "MoveJ_IK");

        submitTask([this, q_target, speed, acceleration, time, radius, taskId]() {
            int result = robot_->MoveJ(q_target, speed, acceleration, time, radius, false);
            if (result == 0) {
                updateTaskStatus(taskId, "COMPLETED", result, "ok");
            } else {
                updateTaskStatus(taskId, "FAILED", result, "MoveJ_IK failed");
            }
        });

        nlohmann::json data;
        data["task_id"] = taskId;
        sendJson(res, true, 0, "MoveJ_IK started", data);
    } else {
        int result = robot_->MoveJ(q_target, speed, acceleration, time, radius, false);
        if (result == 0) {
            sendJson(res, true, 0, "MoveJ_IK completed");
        } else {
            sendJson(res, false, 2004, "MoveJ_IK failed with code " + std::to_string(result));
        }
    }
}

void RobotHttpServer::handleMoveL(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/move/linear");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("pose")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'pose' object");
        return;
    }

    nlohmann::json pose_json = body["pose"];
    KDL::Frame pose;
    if (pose_json.contains("position") && pose_json.contains("orientation")) {
        pose = jsonToFrame(pose_json["position"], pose_json["orientation"]);
    } else {
        pose = jsonToFrame(pose_json);
    }

    double speed = body.value("speed", 1.05);
    double acceleration = body.value("acceleration", 1.4);
    double time = body.value("time", 0.0);
    double radius = body.value("radius", 0.0);
    bool asynchronous = body.value("asynchronous", false);

    if (asynchronous) {
        std::string taskId = generateTaskId();
        registerTask(taskId, "MoveL");

        submitTask([this, pose, speed, acceleration, time, radius, taskId]() {
            int result = robot_->MoveL(pose, speed, acceleration, time, radius, false);
            if (result == 0) {
                updateTaskStatus(taskId, "COMPLETED", result, "ok");
            } else {
                updateTaskStatus(taskId, "FAILED", result, "MoveL failed");
            }
        });

        nlohmann::json data;
        data["task_id"] = taskId;
        sendJson(res, true, 0, "MoveL started", data);
    } else {
        int result = robot_->MoveL(pose, speed, acceleration, time, radius, false);
        if (result == 0) {
            sendJson(res, true, 0, "MoveL completed");
        } else {
            sendJson(res, false, 2004, "MoveL failed with code " + std::to_string(result));
        }
    }
}

void RobotHttpServer::handleMoveL_FK(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/move/linear_fk");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("joints")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'joints' array");
        return;
    }

    int jnt_num = robot_->getJointNum();
    auto joints_arr = body["joints"];
    if (static_cast<int>(joints_arr.size()) != jnt_num) {
        sendJson(res, false, 1003, "Joint count mismatch: expected " + std::to_string(jnt_num));
        return;
    }

    KDL::JntArray q(jnt_num);
    for (int i = 0; i < jnt_num; ++i) {
        q(i) = joints_arr[i].get<double>();
    }

    double speed = body.value("speed", 1.05);
    double acceleration = body.value("acceleration", 1.4);
    double time = body.value("time", 0.0);
    double radius = body.value("radius", 0.0);
    bool asynchronous = body.value("asynchronous", false);

    if (asynchronous) {
        std::string taskId = generateTaskId();
        registerTask(taskId, "MoveL_FK");

        submitTask([this, q, speed, acceleration, time, radius, taskId]() {
            int result = robot_->MoveL_FK(q, speed, acceleration, time, radius, false);
            if (result == 0) {
                updateTaskStatus(taskId, "COMPLETED", result, "ok");
            } else {
                updateTaskStatus(taskId, "FAILED", result, "MoveL_FK failed");
            }
        });

        nlohmann::json data;
        data["task_id"] = taskId;
        sendJson(res, true, 0, "MoveL_FK started", data);
    } else {
        int result = robot_->MoveL_FK(q, speed, acceleration, time, radius, false);
        if (result == 0) {
            sendJson(res, true, 0, "MoveL_FK completed");
        } else {
            sendJson(res, false, 2004, "MoveL_FK failed with code " + std::to_string(result));
        }
    }
}

void RobotHttpServer::handleMoveC(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/move/circle");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("pose_via") || !body.contains("pose_to")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'pose_via'/'pose_to'");
        return;
    }

    KDL::Frame pose_via;
    {
        nlohmann::json via_json = body["pose_via"];
        if (via_json.contains("position") && via_json.contains("orientation")) {
            pose_via = jsonToFrame(via_json["position"], via_json["orientation"]);
        } else {
            pose_via = jsonToFrame(via_json);
        }
    }

    KDL::Frame pose_to;
    {
        nlohmann::json to_json = body["pose_to"];
        if (to_json.contains("position") && to_json.contains("orientation")) {
            pose_to = jsonToFrame(to_json["position"], to_json["orientation"]);
        } else {
            pose_to = jsonToFrame(to_json);
        }
    }

    double speed = body.value("speed", 0.25);
    double acceleration = body.value("acceleration", 1.2);
    double time = body.value("time", 0.0);
    double radius = body.value("radius", 0.0);
    Robot::OrientationMode mode = Robot::UNCONSTRAINED;
    if (body.contains("mode")) {
        if (body["mode"].is_string()) {
            std::string mode_str = body["mode"].get<std::string>();
            if (mode_str == "UNCONSTRAINED" || mode_str == "unconstrained") {
                mode = Robot::UNCONSTRAINED;
            } else if (mode_str == "FIXED" || mode_str == "fixed") {
                mode = Robot::FIXED;
            } else {
                sendJson(res, false, 1001, "Unknown mode: " + mode_str + " (expected UNCONSTRAINED/FIXED)");
                return;
            }
        } else if (body["mode"].is_number()) {
            mode = static_cast<Robot::OrientationMode>(body["mode"].get<int>());
        }
    }
    bool asynchronous = body.value("asynchronous", false);

    if (asynchronous) {
        std::string taskId = generateTaskId();
        registerTask(taskId, "MoveC");

        submitTask([this, pose_via, pose_to, speed, acceleration, time, radius, mode, taskId]() {
            int result = robot_->MoveC(pose_via, pose_to, speed, acceleration, time, radius,
                                       mode, false);
            if (result == 0) {
                updateTaskStatus(taskId, "COMPLETED", result, "ok");
            } else {
                updateTaskStatus(taskId, "FAILED", result, "MoveC failed");
            }
        });

        nlohmann::json data;
        data["task_id"] = taskId;
        sendJson(res, true, 0, "MoveC started", data);
    } else {
        int result = robot_->MoveC(pose_via, pose_to, speed, acceleration, time, radius,
                                   mode, false);
        if (result == 0) {
            sendJson(res, true, 0, "MoveC completed");
        } else {
            sendJson(res, false, 2004, "MoveC failed with code " + std::to_string(result));
        }
    }
}

void RobotHttpServer::handleMoveP(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/move/path");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("pose")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'pose' object");
        return;
    }

    nlohmann::json pose_json = body["pose"];
    KDL::Frame pose;
    if (pose_json.contains("position") && pose_json.contains("orientation")) {
        pose = jsonToFrame(pose_json["position"], pose_json["orientation"]);
    } else {
        pose = jsonToFrame(pose_json);
    }

    double speed = body.value("speed", 1.05);
    double acceleration = body.value("acceleration", 1.4);
    double time = body.value("time", 0.0);
    double radius = body.value("radius", 0.0);
    bool asynchronous = body.value("asynchronous", false);

    if (asynchronous) {
        std::string taskId = generateTaskId();
        registerTask(taskId, "MoveP");

        submitTask([this, pose, speed, acceleration, time, radius, taskId]() {
            int result = robot_->MoveP(pose, speed, acceleration, time, radius, false);
            if (result == 0) {
                updateTaskStatus(taskId, "COMPLETED", result, "ok");
            } else {
                updateTaskStatus(taskId, "FAILED", result, "MoveP failed");
            }
        });

        nlohmann::json data;
        data["task_id"] = taskId;
        sendJson(res, true, 0, "MoveP started", data);
    } else {
        int result = robot_->MoveP(pose, speed, acceleration, time, radius, false);
        if (result == 0) {
            sendJson(res, true, 0, "MoveP completed");
        } else {
            sendJson(res, false, 2004, "MoveP failed with code " + std::to_string(result));
        }
    }
}

void RobotHttpServer::handleMovePath(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/move/pathway");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("waypoints")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'waypoints' array");
        return;
    }

    auto waypoints = body["waypoints"];
    if (waypoints.empty()) {
        sendJson(res, false, 1001, "Waypoints array is empty");
        return;
    }

    // Robot::Path and Robot::PathEntry have private constructors,
    // so we cannot populate them from outside the Robot class.
    // Return 501 (Not Implemented) with clear message.
    log_ptr_->warn("MovePath: Robot::Path is not yet accessible from outside the class");
    // Mark as Not Implemented using business code 2004 per API design
    sendJson(res, false, 2004, "MovePath not implemented: Robot::Path has private constructors, needs Robot class API extension");
}

void RobotHttpServer::handleStop(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/move/stop");
    robot_->stopMultiAxis();
    sendJson(res, true, 0, "Motion stopped");
}

// ============================================================================
// Async Task Query
// ============================================================================

void RobotHttpServer::handleMoveStatus(const httplib::Request& req, httplib::Response& res) {
    std::string task_id = req.get_param_value("task_id");
    log_ptr_->info("GET /api/move/status task_id={}", task_id);

    if (task_id.empty()) {
        sendJson(res, false, 1001, "Missing 'task_id' query parameter");
        return;
    }

    nlohmann::json info = getTaskInfo(task_id);
    if (info.is_null()) {
        // Task not found
        nlohmann::json data;
        data["task_id"] = task_id;
        sendJson(res, false, 5001, "Task not found", data);
    } else {
        sendJson(res, true, 0, "ok", info);
    }
}

// ============================================================================
// Single Axis Control Handlers
// ============================================================================

void RobotHttpServer::handleSingleAxisEnable(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/axis/single/enable");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("id")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'id' field");
        return;
    }

    int id = body["id"].get<int>();
    if (id < 0 || id >= robot_->getJointNum()) {
        sendJson(res, false, 1002, "Invalid joint id: " + std::to_string(id));
        return;
    }

    robot_->setJointEnabled(id);
    sendJson(res, true, 0, "Joint " + std::to_string(id) + " enabled");
}

void RobotHttpServer::handleSingleAxisDisable(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/axis/single/disable");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("id")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'id' field");
        return;
    }

    int id = body["id"].get<int>();
    if (id < 0 || id >= robot_->getJointNum()) {
        sendJson(res, false, 1002, "Invalid joint id: " + std::to_string(id));
        return;
    }

    robot_->setJointDisabled(id);
    sendJson(res, true, 0, "Joint " + std::to_string(id) + " disabled");
}

void RobotHttpServer::handleSingleAxisMove(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/axis/single/move");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("id") || !body.contains("pos")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'id'/'pos' fields");
        return;
    }

    int id = body["id"].get<int>();
    if (id < 0 || id >= robot_->getJointNum()) {
        sendJson(res, false, 1002, "Invalid joint id: " + std::to_string(id));
        return;
    }

    double max_vel = body.value("max_vel", -1.0);
    double max_acc = body.value("max_acc", -1.0);
    double max_jerk = body.value("max_jerk", -1.0);
    double least_time = body.value("least_time", -1.0);
    bool raw_data = body.value("raw_data", false);

    double pos = body["pos"].get<double>();

    if (raw_data) {
        // Convert from raw encoder counts to user units
        double cnt_per_unit = robot_->getJointCntPerUnit(id);
        if (cnt_per_unit != 0.0) {
            pos = pos / cnt_per_unit;
        }
        if (max_vel > 0) {
            max_vel = max_vel / cnt_per_unit;
        }
        if (max_acc > 0) {
            max_acc = max_acc / cnt_per_unit;
        }
        if (max_jerk > 0) {
            max_jerk = max_jerk / cnt_per_unit;
        }
    }

    robot_->moveSingleAxis(id, pos, 0.0, max_vel, max_acc, max_jerk, least_time);
    sendJson(res, true, 0, "Joint " + std::to_string(id) + " move started");
}

void RobotHttpServer::handleSingleAxisStop(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/axis/single/stop");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("id")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'id' field");
        return;
    }

    int id = body["id"].get<int>();
    if (id < 0 || id >= robot_->getJointNum()) {
        sendJson(res, false, 1002, "Invalid joint id: " + std::to_string(id));
        return;
    }

    robot_->stopSingleAxis(id);
    sendJson(res, true, 0, "Joint " + std::to_string(id) + " stopped");
}

// ============================================================================
// Multi Axis Control Handlers
// ============================================================================

void RobotHttpServer::handleMultiAxisEnable(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/axis/multi/enable");
    robot_->setEnabled();
    sendJson(res, true, 0, "All joints enabled");
}

void RobotHttpServer::handleMultiAxisDisable(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/axis/multi/disable");
    robot_->setDisabled();
    sendJson(res, true, 0, "All joints disabled");
}

void RobotHttpServer::handleMultiAxisMove(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/axis/multi/move");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded()) {
        sendJson(res, false, 1001, "Invalid JSON body");
        return;
    }

    int jnt_num = robot_->getJointNum();

    // Required fields
    if (!body.contains("target_pos") || !body.contains("max_vel") ||
        !body.contains("max_acc") || !body.contains("max_jerk")) {
        sendJson(res, false, 1001, "Missing required fields: target_pos, max_vel, max_acc, max_jerk");
        return;
    }

    auto target_pos_arr = body["target_pos"];
    auto max_vel_arr = body["max_vel"];
    auto max_acc_arr = body["max_acc"];
    auto max_jerk_arr = body["max_jerk"];

    if (static_cast<int>(target_pos_arr.size()) != jnt_num ||
        static_cast<int>(max_vel_arr.size()) != jnt_num ||
        static_cast<int>(max_acc_arr.size()) != jnt_num ||
        static_cast<int>(max_jerk_arr.size()) != jnt_num) {
        sendJson(res, false, 1003, "Array length mismatch: expected " + std::to_string(jnt_num) + " elements");
        return;
    }

    std::vector<double> target_pos(jnt_num);
    std::vector<double> max_vel(jnt_num);
    std::vector<double> max_acc(jnt_num);
    std::vector<double> max_jerk(jnt_num);

    for (int i = 0; i < jnt_num; ++i) {
        target_pos[i] = target_pos_arr[i].get<double>();
        max_vel[i] = max_vel_arr[i].get<double>();
        max_acc[i] = max_acc_arr[i].get<double>();
        max_jerk[i] = max_jerk_arr[i].get<double>();
    }

    double least_time = body.value("least_time", -1.0);

    robot_->moveMultiAxis(target_pos, max_vel, max_acc, max_jerk, max_jerk, least_time);
    sendJson(res, true, 0, "Multi-axis move started");
}

void RobotHttpServer::handleMultiAxisStop(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/axis/multi/stop");
    robot_->stopMultiAxis();
    sendJson(res, true, 0, "All axes stopped");
}

void RobotHttpServer::handleMultiAxisSync(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/axis/multi/sync");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("sync")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'sync' field");
        return;
    }

    std::string sync_str = body["sync"].get<std::string>();
    Robot::Synchronization sync;

    if (sync_str == "none" || sync_str == "SYNC_NONE") {
        sync = Robot::SYNC_NONE;
    } else if (sync_str == "time" || sync_str == "SYNC_TIME") {
        sync = Robot::SYNC_TIME;
    } else if (sync_str == "phase" || sync_str == "SYNC_PHASE") {
        sync = Robot::SYNC_PHASE;
    } else {
        sendJson(res, false, 1001, "Unknown sync mode: " + sync_str);
        return;
    }

    robot_->setSynchronization(sync);
    sendJson(res, true, 0, "Synchronization set to " + sync_str);
}

// ============================================================================
// Dragging Handlers
// ============================================================================

void RobotHttpServer::handleDragStart(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/drag/start");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("flag") || !body.contains("direction")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'flag'/'direction' fields");
        return;
    }

    std::string flag_str = body["flag"].get<std::string>();
    std::string dir_str = body["direction"].get<std::string>();
    double max_speed = body.value("max_speed", 1.0);
    double max_acceleration = body.value("max_acceleration", 1.0);

    // Map flag string to enum
    Robot::DRAGGING_FLAG flag;
    if (flag_str == "J0")            flag = Robot::DRAGGING_FLAG::J0;
    else if (flag_str == "J1")       flag = Robot::DRAGGING_FLAG::J1;
    else if (flag_str == "J2")       flag = Robot::DRAGGING_FLAG::J2;
    else if (flag_str == "J3")       flag = Robot::DRAGGING_FLAG::J3;
    else if (flag_str == "J4")       flag = Robot::DRAGGING_FLAG::J4;
    else if (flag_str == "J5")       flag = Robot::DRAGGING_FLAG::J5;
    else if (flag_str == "J6")       flag = Robot::DRAGGING_FLAG::J6;
    else if (flag_str == "TOOL_X")   flag = Robot::DRAGGING_FLAG::TOOL_X;
    else if (flag_str == "TOOL_Y")   flag = Robot::DRAGGING_FLAG::TOOL_Y;
    else if (flag_str == "TOOL_Z")   flag = Robot::DRAGGING_FLAG::TOOL_Z;
    else if (flag_str == "TOOL_ROLL")    flag = Robot::DRAGGING_FLAG::TOOL_ROLL;
    else if (flag_str == "TOOL_PITCH")   flag = Robot::DRAGGING_FLAG::TOOL_PITCH;
    else if (flag_str == "TOOL_YAW")     flag = Robot::DRAGGING_FLAG::TOOL_YAW;
    else if (flag_str == "FLANGE_X")     flag = Robot::DRAGGING_FLAG::FLANGE_X;
    else if (flag_str == "FLANGE_Y")     flag = Robot::DRAGGING_FLAG::FLANGE_Y;
    else if (flag_str == "FLANGE_Z")     flag = Robot::DRAGGING_FLAG::FLANGE_Z;
    else if (flag_str == "FLANGE_ROLL")  flag = Robot::DRAGGING_FLAG::FLANGE_ROLL;
    else if (flag_str == "FLANGE_PITCH") flag = Robot::DRAGGING_FLAG::FLANGE_PITCH;
    else if (flag_str == "FLANGE_YAW")   flag = Robot::DRAGGING_FLAG::FLANGE_YAW;
    else if (flag_str == "OBJECT_X")     flag = Robot::DRAGGING_FLAG::OBJECT_X;
    else if (flag_str == "OBJECT_Y")     flag = Robot::DRAGGING_FLAG::OBJECT_Y;
    else if (flag_str == "OBJECT_Z")     flag = Robot::DRAGGING_FLAG::OBJECT_Z;
    else if (flag_str == "OBJECT_ROLL")  flag = Robot::DRAGGING_FLAG::OBJECT_ROLL;
    else if (flag_str == "OBJECT_PITCH") flag = Robot::DRAGGING_FLAG::OBJECT_PITCH;
    else if (flag_str == "OBJECT_YAW")   flag = Robot::DRAGGING_FLAG::OBJECT_YAW;
    else if (flag_str == "BASE_X")       flag = Robot::DRAGGING_FLAG::BASE_X;
    else if (flag_str == "BASE_Y")       flag = Robot::DRAGGING_FLAG::BASE_Y;
    else if (flag_str == "BASE_Z")       flag = Robot::DRAGGING_FLAG::BASE_Z;
    else if (flag_str == "BASE_ROLL")    flag = Robot::DRAGGING_FLAG::BASE_ROLL;
    else if (flag_str == "BASE_PITCH")   flag = Robot::DRAGGING_FLAG::BASE_PITCH;
    else if (flag_str == "BASE_YAW")     flag = Robot::DRAGGING_FLAG::BASE_YAW;
    else if (flag_str == "NULLSPACE")    flag = Robot::DRAGGING_FLAG::NULLSPACE;
    else {
        sendJson(res, false, 1001, "Unknown dragging flag: " + flag_str);
        return;
    }

    // Map direction string to enum
    Robot::DRAGGING_DIRRECTION dir;
    if (dir_str == "POSITIVE" || dir_str == "POSITION" || dir_str == "1") {
        dir = Robot::DRAGGING_DIRRECTION::POSITION;
    } else if (dir_str == "NEGATIVE" || dir_str == "-1") {
        dir = Robot::DRAGGING_DIRRECTION::NEGATIVE;
    } else if (dir_str == "NONE" || dir_str == "0") {
        dir = Robot::DRAGGING_DIRRECTION::NONE;
    } else {
        sendJson(res, false, 1001, "Unknown dragging direction: " + dir_str);
        return;
    }

    robot_->Dragging(flag, dir, max_speed, max_acceleration);
    sendJson(res, true, 0, "Dragging started");
}

void RobotHttpServer::handleDragStop(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/drag/stop");
    robot_->Dragging(Robot::DRAGGING_FLAG::J0,
                     Robot::DRAGGING_DIRRECTION::NONE,
                     0.0, 0.0);
    sendJson(res, true, 0, "Dragging stopped");
}

// ============================================================================
// Calibration Handlers
// ============================================================================

void RobotHttpServer::handleSetPoseFrame(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/calibration/pose");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("id") || !body.contains("pose")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'id'/'pose' fields");
        return;
    }

    int id = body["id"].get<int>();
    nlohmann::json pose_json = body["pose"];

    KDL::Frame pose;
    if (pose_json.contains("position") && pose_json.contains("orientation")) {
        pose = jsonToFrame(pose_json["position"], pose_json["orientation"]);
    } else {
        pose = jsonToFrame(pose_json);
    }

    robot_->set_pose_frame(id, pose);
    sendJson(res, true, 0, "Pose frame " + std::to_string(id) + " set");
}

void RobotHttpServer::handleSetToolFrame(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/calibration/tool");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("pose")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'pose' field");
        return;
    }

    nlohmann::json pose_json = body["pose"];
    KDL::Frame pose;
    if (pose_json.contains("position") && pose_json.contains("orientation")) {
        pose = jsonToFrame(pose_json["position"], pose_json["orientation"]);
    } else {
        pose = jsonToFrame(pose_json);
    }

    robot_->set_tool_frame(pose);
    sendJson(res, true, 0, "Tool frame set");
}

void RobotHttpServer::handleSetObjectFrame(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/calibration/object");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("pose")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'pose' field");
        return;
    }

    nlohmann::json pose_json = body["pose"];
    KDL::Frame pose;
    if (pose_json.contains("position") && pose_json.contains("orientation")) {
        pose = jsonToFrame(pose_json["position"], pose_json["orientation"]);
    } else {
        pose = jsonToFrame(pose_json);
    }

    robot_->set_object_frame(pose);
    sendJson(res, true, 0, "Object frame set");
}

void RobotHttpServer::handleCalibrationRun(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/calibration/run");

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("frame")) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'frame' field");
        return;
    }

    std::string frame = body["frame"].get<std::string>();
    if (frame != "tool" && frame != "object") {
        sendJson(res, false, 4004, "Invalid frame type: must be 'tool' or 'object'");
        return;
    }

    robot_->tool_calibration(frame);

    bool error_state = robot_->getErrorStateOfCal();
    if (error_state) {
        sendJson(res, false, 4002, "Calibration failed");
    } else {
        KDL::Frame result = robot_->getPose_out();
        nlohmann::json data;
        data["result"] = frameToJson(result);
        sendJson(res, true, 0, "Calibration completed", data);
    }
}

void RobotHttpServer::handleCalibrationResult(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/calibration/result");

    bool error_state = robot_->getErrorStateOfCal();
    KDL::Frame result = robot_->getPose_out();

    nlohmann::json data;
    data["error_state"] = error_state;
    data["pose"] = frameToJson(result);

    sendJson(res, true, 0, "ok", data);
}

} // namespace rocos
