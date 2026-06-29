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

#pragma once

#include <httplib.h>

#include <atomic>
#include <condition_variable>
#include <json.hpp>
#include <kdl/frames.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "logger.hpp"
#include "robot.hpp"

namespace rocos {

class RobotHttpServer {
public:
    explicit RobotHttpServer(Robot* robot);
    ~RobotHttpServer();

    /// Start server (blocking)
    void run(const std::string& host = "0.0.0.0", int port = 8080);

    /// Start server in a separate thread (non-blocking)
    void runAsync(const std::string& host = "0.0.0.0", int port = 8080);

    /// Stop the server
    void stop();

private:
    void registerRoutes();

    // ---- Robot State ----
    void handleGetRobotState(const httplib::Request& req, httplib::Response& res);
    void handleGetRobotInfo(const httplib::Request& req, httplib::Response& res);
    void handleGetRobotModel(const httplib::Request& req, httplib::Response& res);
    void handleGetLinkMesh(const httplib::Request& req, httplib::Response& res);

    // ---- Robot Control ----
    void handleEnable(const httplib::Request& req, httplib::Response& res);
    void handleDisable(const httplib::Request& req, httplib::Response& res);
    void handleIsEnabled(const httplib::Request& req, httplib::Response& res);
    void handleSetWorkMode(const httplib::Request& req, httplib::Response& res);

    // ---- Motion Control ----
    void handleMoveJ(const httplib::Request& req, httplib::Response& res);
    void handleMoveJ_IK(const httplib::Request& req, httplib::Response& res);
    void handleMoveL(const httplib::Request& req, httplib::Response& res);
    void handleMoveL_FK(const httplib::Request& req, httplib::Response& res);

    void handlePause(const httplib::Request& req, httplib::Response& res);
    void handleResume(const httplib::Request& req, httplib::Response& res);
    void handleStop(const httplib::Request& req, httplib::Response& res);

    // ---- Dragging ----
    void handleDragStart(const httplib::Request& req, httplib::Response& res);
    void handleDragStop(const httplib::Request& req, httplib::Response& res);

    // ---- Async Task Query ----
    void handleMoveStatus(const httplib::Request& req, httplib::Response& res);

    // ---- Calibration ----
    void handleSetPoseFrame(const httplib::Request& req, httplib::Response& res);
    void handleSetToolFrame(const httplib::Request& req, httplib::Response& res);
    void handleSetObjectFrame(const httplib::Request& req, httplib::Response& res);
    void handleCalibrationRun(const httplib::Request& req, httplib::Response& res);
    void handleCalibrationResult(const httplib::Request& req, httplib::Response& res);

    // ---- Utility ----
    void sendJson(httplib::Response& res, bool success,
                  int businessCode, const std::string& message,
                  const nlohmann::json& data = nlohmann::json());

    KDL::Frame jsonToFrame(const nlohmann::json& j);

    KDL::Frame jsonToFrame(const nlohmann::json& pos, const nlohmann::json& rot);
    nlohmann::json frameToJson(const KDL::Frame& frame);
    std::string getUrdfPath();

    // ---- Async Task Management ----
    std::string generateTaskId();
    void registerTask(const std::string& taskId, const std::string& type);
    void updateTaskStatus(const std::string& taskId, const std::string& status,
                          int result = 0, const std::string& message = "ok");
    nlohmann::json getTaskInfo(const std::string& taskId);
    void cleanExpiredTasks();

    // ---- CORS & Preflight ----
    void setCorsHeaders(httplib::Response& res);

    // ---- Thread Pool ----
    void submitTask(std::function<void()> func);

    Robot* robot_;
    std::unique_ptr<httplib::Server> server_;
    std::unique_ptr<std::thread> thread_;
    std::mutex taskMutex_;
    std::map<std::string, nlohmann::json> taskMap_;
    long long taskCounter_;

    // Thread pool for async motion tasks
    std::vector<std::thread> workerThreads_;
    std::queue<std::function<void()>> taskQueue_;
    std::mutex poolMutex_;
    std::condition_variable poolCv_;
    bool poolShutdown_;
    std::atomic<int> activeWorkers_;
    static const int MAX_WORKERS = 8;

    Logger::logger_ptr log_ptr_ = nullptr;

    // Task TTL
    static const int TASK_TTL_SECONDS = 3600; // 1 hour
};

} // namespace rocos
