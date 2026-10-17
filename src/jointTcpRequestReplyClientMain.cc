// TCP 一问一答客户端：目标 → Ruckig 1 ms 参考 → CSP/CST；每条有效指令回复一行当前状态 JSON。
// 网络线程不访问 Robot，控制线程不解析 JSON、不调用 socket 收发。
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <rocos_app/robot.h>
#include <rocos_app/joint_impedance.h>
#include <ruckig/ruckig.hpp>
#include <json.hpp>
#include <gflags/gflags.h>
#include <yaml-cpp/yaml.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

DEFINE_string(config, "config/identified_impedance.yaml", "辨识阻抗 YAML 配置");
DEFINE_string(host, "127.0.0.1", "上位机 TCP 服务端地址");
DEFINE_int32(port, 9000, "上位机 TCP 服务端端口");
DEFINE_string(base, "base_link", "运动链基座");
DEFINE_string(tip, "link_7", "运动链末端");
DEFINE_bool(sim, true, "使用 HardwareSim；不模拟力矩动力学");
DEFINE_int32(id, 0, "真机硬件实例编号");
DEFINE_bool(check_only, false, "只检查配置，不连接 TCP 或硬件");
DEFINE_double(command_timeout, 1.0, "连续无有效指令的告警时间，秒；超时继续执行最后目标，不退出");
DEFINE_double(movej_speed, 0.15, "stop 模式 MoveJ 的最大关节速度，rad/s");
DEFINE_double(movej_acceleration, 0.3, "stop 模式 MoveJ 的最大关节加速度，rad/s^2");
DEFINE_bool(tcp_log_data, true, "打印每帧 TCP 收发数据；关闭后仅打印连接、错误和统计信息");

namespace {
using Clock = std::chrono::steady_clock;
using Json = nlohmann::json;
using Joints = std::array<double, 7>;
constexpr size_t jointCount = 7;
constexpr double stationarySpeed = 0.2;
constexpr size_t maxLineSize = 16384;
constexpr size_t maxPendingReplies = 1024;
volatile std::sig_atomic_t interrupted = 0;
void signalHandler(int) { interrupted = 1; }
void require(bool ok, const std::string &message) {
    if (!ok) throw std::runtime_error(message);
}

// 分块输出避免本程序的多线程日志交错；不在控制线程或状态邮箱锁内打印。
void logTcp(const char *event, const std::string &title, const std::string &details = "", bool error = false) {
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
    localtime_r(&time, &local);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    std::ostringstream output;
    output << '\n' << '[' << std::put_time(&local, "%Y-%m-%d %H:%M:%S") << '.'
           << std::setfill('0') << std::setw(3) << milliseconds << ' ' << std::put_time(&local, "%z")
           << "] [TCP/" << event << "] " << title << '\n';
    if (!details.empty()) {
        output << "  +------------------------------------------------------------\n";
        std::istringstream lines(details);
        std::string line;
        while (std::getline(lines, line)) output << "  | " << line << '\n';
        output << "  +------------------------------------------------------------\n";
    }
    static std::mutex logMutex;
    std::lock_guard<std::mutex> lock(logMutex);
    (error ? std::cerr : std::cout) << output.str() << std::flush;
}

std::string socketEndpoint(int fd, bool peer) {
    sockaddr_storage address{};
    socklen_t size = sizeof(address);
    const int result = peer ? getpeername(fd, reinterpret_cast<sockaddr *>(&address), &size)
                            : getsockname(fd, reinterpret_cast<sockaddr *>(&address), &size);
    if (result != 0) return "(unavailable)";
    char host[NI_MAXHOST], service[NI_MAXSERV];
    if (getnameinfo(reinterpret_cast<sockaddr *>(&address), size, host, sizeof(host), service, sizeof(service),
                    NI_NUMERICHOST | NI_NUMERICSERV) != 0) return "(unavailable)";
    return address.ss_family == AF_INET6 ? "[" + std::string(host) + "]:" + service + " (IPv6)"
                                        : std::string(host) + ':' + service + " (IPv4)";
}

enum class Mode { Position, Impedance, ImpedanceFriction, Free, Stop };
bool torqueMode(Mode mode) {
    return mode == Mode::Impedance || mode == Mode::ImpedanceFriction || mode == Mode::Free;
}

rocos::JointImpedanceTuning freeModeTuning() {
    rocos::JointImpedanceTuning tuning;
    tuning.kp = Eigen::VectorXd::Zero(jointCount);
    tuning.kd.resize(jointCount);
    tuning.kd << 4, 4, 4, 3, 2, 1.5, 1;
    tuning.frictionScale = 0.6;
    return tuning;
}

struct Command {
    Mode mode = Mode::Position;
    Joints q{};
    uint64_t sequence = 0;
    Clock::time_point received;
};

struct Feedback {
    uint64_t timestamp = 0;
    Joints q{}, dq{}, actual{}, calculated{}, friction{};
    bool torqueValid = false;
};

Joints readArray(const Json &value, const char *name) {
    require(value.is_array() && value.size() == jointCount, std::string(name) + " must contain seven numbers");
    Joints result;
    for (size_t j = 0; j < jointCount; ++j) {
        require(value[j].is_number(), std::string(name) + " contains a non-number");
        result[j] = value[j].get<double>();
        require(std::isfinite(result[j]), std::string(name) + " contains a nonfinite number");
    }
    return result;
}

Command parseCommand(const std::string &line, const rocos::JointImpedanceConfig &config) {
    // 禁止重复字段，避免模式或目标被 JSON 中的后一个同名字段悄悄覆盖。
    std::vector<std::set<std::string>> keys;
    auto callback = [&](int depth, Json::parse_event_t event, Json &value) {
        require(depth <= 8, "JSON nesting is too deep");
        if (event == Json::parse_event_t::object_start) keys.emplace_back();
        if (event == Json::parse_event_t::key)
            require(keys.back().insert(value.get<std::string>()).second, "Duplicate JSON field");
        if (event == Json::parse_event_t::object_end) keys.pop_back();
        return true;
    };
    const auto data = Json::parse(line, callback);
    require(data.is_object() && data.at("frame_header") == "0xAA56", "Invalid command frame_header");
    const auto &stamp = data.at("timestamp_us");
    require(stamp.is_number_unsigned() || (stamp.is_number_integer() && stamp.get<int64_t>() >= 0),
            "timestamp_us must be a nonnegative integer");
    const auto mode = data.at("mode").get<std::string>();
    Command command;
    if (mode == "position") command.mode = Mode::Position;
    else if (mode == "impedance") command.mode = Mode::Impedance;
    else if (mode == "impedance_friction") command.mode = Mode::ImpedanceFriction;
    else if (mode == "free") command.mode = Mode::Free;
    else if (mode == "stop") command.mode = Mode::Stop;
    else throw std::runtime_error("Unknown command mode: " + mode);
    // free 无位置目标，允许省略 q；若携带 q，仍按相同格式和位置限位校验。
    if (command.mode != Mode::Free || data.contains("q")) {
        command.q = readArray(data.at("q"), "q");
        for (size_t j = 0; j < jointCount; ++j)
            require(command.q[j] >= config.lower[j] && command.q[j] <= config.upper[j],
                    "Target exceeds position limits: " + config.jointNames[j]);
    }
    // 兼容上位机携带 tau 的旧消息，但不读取、不存储，也不参与任何控制计算。
    command.received = Clock::now(); // 超时依据本地接收时间，不依赖两台机器的时钟同步。
    return command;
}

Json feedbackJson(const Feedback &sample) {
    return {{"frame_header", "0xAA55"}, {"timestamp_us", sample.timestamp},
            {"q", sample.q}, {"dq", sample.dq}, {"tau_actual", sample.actual},
            {"tau_calculated", sample.torqueValid ? Json(sample.calculated) : Json(Joints{})},
            {"tau_friction", sample.torqueValid ? Json(sample.friction) : Json(Joints{})}};
}

int connectToServer(const std::string &host, int port) {
    logTcp("CONNECTING", "正在连接服务端", "配置地址 : " + host + "\n服务端口 : " + std::to_string(port));
    addrinfo hints{}, *addresses = nullptr;
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    const int error = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &addresses);
    require(error == 0, std::string("TCP address: ") + gai_strerror(error));
    int connected = -1;
    for (auto *address = addresses; address && !interrupted; address = address->ai_next) {
        const int fd = socket(address->ai_family, SOCK_STREAM | SOCK_CLOEXEC, address->ai_protocol);
        if (fd < 0) continue;
        if (fcntl(fd, F_SETFL, O_NONBLOCK) == 0) {
            int result = connect(fd, address->ai_addr, address->ai_addrlen);
            if (result < 0 && errno == EINPROGRESS) {
                pollfd event{fd, POLLOUT, 0};
                int socketError = 0;
                socklen_t size = sizeof(socketError);
                result = poll(&event, 1, 3000) > 0 &&
                    getsockopt(fd, SOL_SOCKET, SO_ERROR, &socketError, &size) == 0 && socketError == 0 ? 0 : -1;
            }
            if (result == 0) { connected = fd; break; }
        }
        close(fd);
    }
    freeaddrinfo(addresses);
    require(connected >= 0, "Cannot connect to TCP server " + host + ":" + std::to_string(port));
    int yes = 1;
    setsockopt(connected, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    logTcp("CONNECTED", "TCP 连接成功",
           "服务端配置 : " + host + "\n服务端端口 : " + std::to_string(port) +
           "\n实际远端   : " + socketEndpoint(connected, true) +
           "\n本地端点   : " + socketEndpoint(connected, false) +
           "\n通信方式   : 每条有效 command 回复一帧状态 JSON（NDJSON）");
    return connected;
}

// 两个有界邮箱：控制目标和反馈只保留最新快照；网络线程按有效指令数逐帧回复。
class TcpClient {
public:
    TcpClient(const std::string &host, int port, const rocos::JointImpedanceConfig &config)
        : config_(config), fd_(connectToServer(host, port)) {
        try { worker_ = std::thread([this] { networkLoop(); }); }
        catch (...) { close(fd_); throw; }
    }
    ~TcpClient() { stop(); }
    void stop() {
        running_ = false;
        if (worker_.joinable()) {
            shutdown(fd_, SHUT_RDWR);
            worker_.join();
            close(fd_);
        }
    }
    void check() {
        if (failed_) {
            std::lock_guard<std::mutex> lock(mutex_);
            throw std::runtime_error(error_);
        }
    }
    bool latest(Command &command) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock() || command_.sequence == 0 || command_.sequence == command.sequence) return false;
        command = command_;
        return true;
    }
    void publish(const Feedback &feedback) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) return;
        feedback_ = feedback;
        haveFeedback_ = true;
    }

private:
    void receive(const char *data, size_t size, std::string &buffer, size_t &pendingReplies,
                 uint64_t &framesReceived) {
        buffer.append(data, size);
        size_t newline;
        while (pendingReplies < maxPendingReplies && (newline = buffer.find('\n')) != std::string::npos) {
            require(newline <= maxLineSize, "TCP command exceeds 16 KiB");
            const auto line = buffer.substr(0, newline);
            buffer.erase(0, newline + 1);
            if (line.empty() || line == "\r") continue;
            ++framesReceived;
            Command command;
            try {
                command = parseCommand(line, config_);
            } catch (const std::exception &error) {
                std::string details = "接收帧号 : " + std::to_string(framesReceived) +
                    "\n帧字节数 : " + std::to_string(line.size() + 1) + "（包含换行）\n拒绝原因 : " + error.what();
                if (FLAGS_tcp_log_data)
                    details += "\n原始数据（转义显示） : " + Json(line).dump(-1, ' ', false, Json::error_handler_t::replace);
                logTcp("REJECTED", "Rejected TCP command", details, true);
                continue;
            }
            // TCP 粘包中的每条有效指令都要回复，不能随最新目标邮箱一起合并。
            {
                std::lock_guard<std::mutex> lock(mutex_);
                command.sequence = command_.sequence + 1;
                command_ = command;
            }
            ++pendingReplies;
            lastCommandReceived_ = command.received;
            if (commandTimedOut_) {
                commandTimedOut_ = false;
                logTcp("RESUMED", "已收到新指令，继续处理", "指令编号 : " + std::to_string(command.sequence));
            }
            if (FLAGS_tcp_log_data) {
                logTcp("RX", "服务端 -> 客户端 | 收到有效 command",
                       "接收帧号 : " + std::to_string(framesReceived) +
                       "\n指令编号 : " + std::to_string(command.sequence) +
                       "\n帧字节数 : " + std::to_string(line.size() + 1) + "（包含换行）" +
                       "\n待回复数 : " + std::to_string(pendingReplies) +
                       "\nJSON 数据 :\n" + Json::parse(line).dump(2));
            }
        }
        // 回复积压时暂停读取后续指令；缓存内可能包含多行，只检查第一行的长度。
        require(std::min(buffer.find('\n'), buffer.size()) <= maxLineSize,
                "TCP command exceeds 16 KiB without a newline");
    }
    void networkLoop() {
        std::string incoming, outgoing, outgoingLog;
        size_t sent = 0, pendingReplies = 0;
        uint64_t framesReceived = 0, bytesReceived = 0, framesSent = 0, bytesSent = 0;
        try {
            auto sendStarted = Clock::now();
            bool sendTimedOut = false;
            while (running_) {
                const auto now = Clock::now();
                if (!incoming.empty() && pendingReplies < maxPendingReplies)
                    receive("", 0, incoming, pendingReplies, framesReceived);
                if (lastCommandReceived_ != Clock::time_point{} && !commandTimedOut_ &&
                    std::chrono::duration<double>(now - lastCommandReceived_).count() > FLAGS_command_timeout) {
                    commandTimedOut_ = true;
                    logTcp("TIMEOUT", "TCP 指令超时，继续执行并保持最后目标",
                           "告警阈值 : " + std::to_string(FLAGS_command_timeout) +
                           " s\n程序与连接保持运行；等待下一条有效 command");
                }
                if (outgoing.empty() && pendingReplies > 0) {
                    Feedback sample;
                    bool available;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        sample = feedback_;
                        available = haveFeedback_;
                    }
                    if (available) {
                        const auto reply = feedbackJson(sample);
                        outgoing = reply.dump() + '\n';
                        if (FLAGS_tcp_log_data) outgoingLog = reply.dump(2);
                        sent = 0;
                        sendStarted = now;
                        sendTimedOut = false;
                        --pendingReplies;
                    }
                    // 首次采样前收到的指令保留回复计数，等待真实状态，禁止发送默认零值。
                }
                if (!outgoing.empty() && !sendTimedOut && now - sendStarted >= std::chrono::seconds(1)) {
                    sendTimedOut = true;
                    logTcp("TIMEOUT", "TCP 状态发送超时，保留当前帧并继续重试",
                           "未发字节 : " + std::to_string(outgoing.size() - sent) + "/" +
                           std::to_string(outgoing.size()) +
                           "\n程序与连接保持运行；请服务端继续读取回复");
                }
                // 有界积压采用 TCP 背压，不因服务端暂时不读取状态而溢出退出。
                pollfd event{fd_, short((pendingReplies < maxPendingReplies ? POLLIN : 0) |
                                       (outgoing.empty() ? 0 : POLLOUT)), 0};
                const int result = poll(&event, 1, 1);
                if (result < 0 && errno == EINTR) continue;
                require(result >= 0, "TCP poll failed");
                require(!(event.revents & (POLLERR | POLLHUP | POLLNVAL)), "TCP connection closed");
                if (event.revents & POLLIN) {
                    char bytes[4096];
                    const auto count = recv(fd_, bytes, sizeof(bytes), 0);
                    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) continue;
                    require(count > 0, "TCP server disconnected");
                    bytesReceived += static_cast<size_t>(count);
                    receive(bytes, static_cast<size_t>(count), incoming, pendingReplies, framesReceived);
                }
                if ((event.revents & POLLOUT) && !outgoing.empty()) {
                    const auto count = send(fd_, outgoing.data() + sent, outgoing.size() - sent, MSG_NOSIGNAL);
                    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) continue;
                    require(count > 0, "TCP feedback send failed");
                    sent += static_cast<size_t>(count);
                    bytesSent += static_cast<size_t>(count);
                    if (sent == outgoing.size()) {
                        ++framesSent;
                        if (sendTimedOut)
                            logTcp("RESUMED", "TCP 状态发送已恢复", "回复编号 : " + std::to_string(framesSent));
                        if (FLAGS_tcp_log_data) {
                            logTcp("TX", "客户端 -> 服务端 | 状态帧已完整写入 socket",
                                   "回复编号 : " + std::to_string(framesSent) +
                                   "\n帧字节数 : " + std::to_string(outgoing.size()) + "（包含换行）" +
                                   "\n待回复数 : " + std::to_string(pendingReplies) +
                                   "\nJSON 数据 :\n" + outgoingLog);
                        }
                        outgoing.clear();
                        sent = 0;
                    }
                }
            }
        } catch (const std::exception &error) {
            // stop() 的 shutdown 会唤醒 poll/recv，正常退出不报告连接故障。
            if (running_) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    error_ = error.what();
                    failed_ = true;
                }
                logTcp("ERROR", "TCP 网络故障", error.what(), true);
            }
        }
        logTcp("CLOSED", "TCP 网络线程已停止",
               "接收帧数 : " + std::to_string(framesReceived) + "（含被拒绝的非空消息）" +
               "\n接收字节 : " + std::to_string(bytesReceived) +
               "\n发送帧数 : " + std::to_string(framesSent) +
               "\n发送字节 : " + std::to_string(bytesSent) +
               "\n未发完帧 : " + std::to_string(pendingReplies + (outgoing.empty() ? 0 : 1)) +
               "\n未发字节 : " + std::to_string(outgoing.size() - sent));
    }
    const rocos::JointImpedanceConfig &config_;
    int fd_;
    std::atomic<bool> running_{true}, failed_{false};
    std::mutex mutex_;
    Command command_;
    Feedback feedback_;
    bool haveFeedback_ = false;
    Clock::time_point lastCommandReceived_;
    bool commandTimedOut_ = false;
    std::string error_;
    std::thread worker_;
};

class JointClientControl {
public:
    JointClientControl(rocos::Robot &robot, rocos::JointImpedanceController &controller,
                       rocos::JointImpedanceController &freeController, TcpClient &client)
        : robot_(robot), controller_(controller), freeController_(freeController), config_(controller.config()), client_(client) {
        require(robot_.getJointNum() == jointCount, "Robot must have seven joints");
        require(!config_.needsTemperature, "TCP protocol has no measured temperature input");
        const auto rated = YAML::LoadFile(FLAGS_config)["rated_torque"].as<std::vector<double>>();
        require(rated.size() == jointCount, "rated_torque must contain seven values");
        for (size_t j = 0; j < jointCount; ++j) {
            indices_[j] = -1;
            for (size_t r = 0; r < jointCount; ++r)
                if (robot_.getJointHardwareId(r) == config_.hardwareIds[j]) indices_[j] = static_cast<int>(r);
            require(indices_[j] >= 0, "Missing Robot joint: " + config_.jointNames[j]);
            rated_[j] = rated[j];
            require(std::isfinite(rated_[j]) && rated_[j] > 0 &&
                    config_.torqueLimit[j] * 1000.0 / rated_[j] <= 32767, "Invalid rated torque or drive torque range");
            robot_.setJointTorquePerUnit(indices_[j], 1000.0 / rated_[j]);
            input_.max_velocity[j] = std::min(config_.velocityLimit[j], robot_.getJointMaxVelocity(indices_[j]));
            input_.max_acceleration[j] = robot_.getJointMaxAcceleration(indices_[j]);
            input_.max_jerk[j] = robot_.getJointMaxJerk(indices_[j]);
            require(std::isfinite(input_.max_acceleration[j]) && input_.max_acceleration[j] > 0 &&
                    std::isfinite(input_.max_jerk[j]) && input_.max_jerk[j] > 0, "Invalid Ruckig limits");
        }
        require(FLAGS_movej_speed <= *std::min_element(input_.max_velocity.begin(), input_.max_velocity.end()) &&
                FLAGS_movej_acceleration <= *std::min_element(input_.max_acceleration.begin(), input_.max_acceleration.end()),
                "MoveJ speed/acceleration exceeds joint limits");
        robot_.setDisabled();
        resetSampling();
        // 等待驱动器反馈刷新后检查实际速度，实时循环由总线同步。
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        sample();
        require(stationary(), "Robot must be stationary before TCP control");
        resetPlanner(q_);
    }

    void run() {
        waitForBus();
        while (!interrupted) {
            client_.check();
            sample();
            client_.latest(desired_);
            // 指令间隔过长仅由网络线程告警；控制线程继续执行并保持最后目标。
            if (!desired_.sequence) {
                publish();
                waitForBus(); // 首条有效指令到来前保持下使能，只更新状态快照，不主动发送。
                continue;
            }
            if (moving_) {
                if (robot_.isMotionBusy()) { publish(); waitForBus(); continue; }
                require(robot_.getLastMotionResult() == 0, "stop MoveJ execution failed");
                moving_ = false;
                // MoveJ 完成后的参考是其规划终点，不能跳回有跟随误差的实际位置。
                resetPlanner(Eigen::Map<const Eigen::VectorXd>(active_.q.data(), jointCount));
            }
            if (!enabled_) {
                activate();
            } else if (needsTransition()) {
                // 先由原控制方式执行 Ruckig 减速，静止后才切换驱动器或启动 MoveJ。
                if (!braking_) { braking_ = true; transitionStarted_ = Clock::now(); }
                require(Clock::now() - transitionStarted_ < std::chrono::seconds(10), "Mode transition did not settle");
                input_.control_interface = ruckig::ControlInterface::Velocity;
                const bool finished = interpolate();
                sendReference();
                publish();
                if (finished && stationary()) { activate(); waitForBus(); }
                continue;
            } else {
                braking_ = false;
                active_ = desired_;
            }
            if (moving_) { publish(); waitForBus(); continue; }
            if (active_.mode == Mode::Free) {
                // 无弹簧目标：参考随实际姿态更新，退出 free 时从当前姿态接续规划。
                resetPlanner(q_);
                reference_ = q_;
                sendReference();
                publish();
                continue;
            }
            input_.control_interface = ruckig::ControlInterface::Position;
            input_.target_position = active_.q;
            interpolate();
            sendReference();
            publish();
        }
    }

private:
    void waitForBus() { robot_.getHardwareInterface()->waitForSignal(0); }
    void readState() {
        for (size_t j = 0; j < jointCount; ++j) {
            q_[j] = robot_.getJointPosition(indices_[j]);
            dq_[j] = robot_.getJointVelocity(indices_[j]); // Drive 已按 URDF vel_per_unit 换算为 rad/s。
        }
    }
    void resetSampling() { lastSample_ = Clock::now(); }
    bool stationary() const { return dq_.cwiseAbs().maxCoeff() < stationarySpeed; }
    void sample() {
        readState();
        const auto now = Clock::now();
        const double dt = std::chrono::duration<double>(now - lastSample_).count();
        require(dt > 0 && std::isfinite(dt), "Invalid joint sampling interval");
        require(!enabled_ || FLAGS_sim || dt <= 0.020, "Joint control cycle exceeded 20 ms");
        lastSample_ = now;
        feedback_.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        for (size_t j = 0; j < jointCount; ++j) {
            feedback_.q[j] = q_[j];
            feedback_.dq[j] = dq_[j];
            feedback_.actual[j] = robot_.getJointTorque(indices_[j]) * rated_[j] / 1000.0;
            require(std::isfinite(q_[j]) && std::isfinite(dq_[j]) && std::isfinite(feedback_.actual[j]), "Nonfinite joint feedback");
            require(q_[j] >= config_.lower[j] && q_[j] <= config_.upper[j], "Position exceeds limits: " + config_.jointNames[j]);
            require(std::abs(dq_[j]) <= config_.velocityLimit[j], "Safety velocity limit exceeded: " + config_.jointNames[j]);
        }
        if (enabled_) require(robot_.isEnabled(), "Robot was disabled during TCP control");
    }
    void resetPlanner(const Eigen::VectorXd &position) {
        for (size_t j = 0; j < jointCount; ++j) {
            input_.current_position[j] = position[j];
            input_.current_velocity[j] = 0;
            input_.current_acceleration[j] = 0;
            input_.target_velocity[j] = 0;
            input_.target_acceleration[j] = 0;
        }
        planner_.reset();
    }
    bool interpolate() {
        const auto result = planner_.update(input_, output_);
        require(result >= 0, "Ruckig failed: " + std::to_string(static_cast<int>(result)));
        for (size_t j = 0; j < jointCount; ++j) {
            reference_[j] = output_.new_position[j];
            require(reference_[j] >= config_.lower[j] && reference_[j] <= config_.upper[j],
                    "Interpolated position exceeds limits: " + config_.jointNames[j]);
        }
        output_.pass_to_input(input_); // 新的 10 Hz 目标从当前插值位置、速度和加速度继续规划。
        return result == ruckig::Result::Finished;
    }
    bool needsTransition() const {
        return torqueMode(desired_.mode) != torqueMode(active_.mode) ||
            (desired_.mode == Mode::Stop && (active_.mode != Mode::Stop || desired_.q != active_.q));
    }
    void fillPosition(const Eigen::VectorXd &q) {
        for (size_t j = 0; j < jointCount; ++j) robotPosition_(indices_[j]) = q[j];
    }
    void calculateTorque(const Eigen::VectorXd &reference, Mode mode) {
        auto &controller = mode == Mode::Free ? freeController_ : controller_;
        requested_ = controller.computeTorque(q_, dq_, reference, noTemperature_,
                                             mode == Mode::ImpedanceFriction || mode == Mode::Free);
        require(requested_.allFinite(), "Nonfinite requested torque");
        for (size_t j = 0; j < jointCount; ++j) {
            feedback_.calculated[j] = requested_[j];
            feedback_.friction[j] = controller.frictionTorque()[j];
        }
    }
    void sendReference() {
        if (torqueMode(active_.mode)) {
            calculateTorque(reference_, active_.mode);
            for (size_t j = 0; j < jointCount; ++j) {
                const double bounded = std::clamp(requested_[j], -config_.torqueLimit[j], config_.torqueLimit[j]);
                const double step = config_.torqueRateLimit[j] * DELTA_T;
                sent_[j] = std::clamp(bounded, sent_[j] - step, sent_[j] + step);
                robotTorque_[indices_[j]] = sent_[j];
            }
            require(robot_.setTorque(robotTorque_) == 0, "Robot rejected CST torque");
        } else {
            fillPosition(reference_);
            require(robot_.servoJ(robotPosition_) == 0, "Robot rejected CSP reference");
        }
    }
    void activate() {
        require(!interrupted && stationary(), "Mode switch requires stationary joints");
        const bool switchDrive = !enabled_ || torqueMode(desired_.mode) != torqueMode(active_.mode);
        if (switchDrive) {
            readState();
            require(stationary(), "Mode switch requires stationary joints");
            if (torqueMode(desired_.mode)) {
                // qd=q，不叠加远端目标误差；阻尼及模型补偿使用驱动器实际速度。
                calculateTorque(q_, desired_.mode);
                require((requested_.cwiseAbs().array() <= config_.torqueLimit.array()).all(), "Initial compensation exceeds torque limits");
                sent_ = requested_;
                for (size_t j = 0; j < jointCount; ++j) {
                    robotTorque_[indices_[j]] = sent_[j];
                    // setTorque() 要求已经处于 CST；切模式前先用单轴接口预置同一份 Nm 目标。
                    robot_.setJointTorque(indices_[j], sent_[j]);
                }
            } else {
                // 先复制实际位置，避免进入 CSP 时使用上次留下的目标位置。
                for (size_t j = 0; j < jointCount; ++j)
                    robot_.setJointPosition(indices_[j], q_[j]);
            }
            // 七轴目标全部预置并经过一个总线周期后，才写模式；在线切换不下使能。
            waitForBus();
            const auto mode = torqueMode(desired_.mode)
                ? rocos::ModeOfOperation::CyclicSynchronousTorqueMode
                : rocos::ModeOfOperation::CyclicSynchronousPositionMode;
            for (int r : indices_) robot_.setJointMode(r, mode);
            if (torqueMode(desired_.mode))
                require(robot_.setTorque(robotTorque_) == 0, "Cannot preload CST compensation");
            else
                waitForBus();
            client_.check();
            require(!interrupted, "Interrupted during mode switch");
            if (!enabled_) {
                // 仅首次启动需要上使能，CSP/CST 在线交接保持使能状态。
                robot_.setEnabled();
                enabled_ = true;
            }
            require(robot_.isEnabled(), "Robot is not enabled after mode switch");
        }
        active_ = desired_;
        braking_ = false;
        resetPlanner(q_);
        resetSampling();
        if (active_.mode == Mode::Stop) {
            fillPosition(Eigen::Map<const Eigen::VectorXd>(active_.q.data(), jointCount));
            require(robot_.MoveJ(robotPosition_, FLAGS_movej_speed, FLAGS_movej_acceleration, 0, 0, true) == 0,
                    "Robot rejected stop MoveJ target");
            moving_ = true;
        }
    }
    void publish() {
        feedback_.torqueValid = enabled_ && torqueMode(active_.mode);
        client_.publish(feedback_);
    }

    rocos::Robot &robot_;
    rocos::JointImpedanceController &controller_;
    rocos::JointImpedanceController &freeController_;
    const rocos::JointImpedanceConfig &config_;
    TcpClient &client_;
    std::array<int, jointCount> indices_{};
    Joints rated_{};
    Eigen::VectorXd q_ = Eigen::VectorXd::Zero(jointCount), dq_ = q_, reference_ = q_;
    Eigen::VectorXd requested_ = q_, sent_ = q_, robotTorque_ = q_, noTemperature_;
    KDL::JntArray robotPosition_{jointCount};
    ruckig::Ruckig<jointCount> planner_{DELTA_T};
    ruckig::InputParameter<jointCount> input_;
    ruckig::OutputParameter<jointCount> output_;
    Command desired_, active_;
    Feedback feedback_;
    Clock::time_point lastSample_, transitionStarted_;
    bool enabled_ = false, braking_ = false, moving_ = false;
};
} // namespace

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    std::signal(SIGPIPE, SIG_IGN);
    try {
        require(FLAGS_port > 0 && FLAGS_port <= 65535, "Invalid TCP port");
        require(std::isfinite(FLAGS_command_timeout) && FLAGS_command_timeout > 0, "Invalid command_timeout");
        require(std::isfinite(FLAGS_movej_speed) && FLAGS_movej_speed > 0 &&
                std::isfinite(FLAGS_movej_acceleration) && FLAGS_movej_acceleration > 0, "Invalid MoveJ limits");
        rocos::JointImpedanceController controller(FLAGS_config);
        rocos::JointImpedanceController freeController(FLAGS_config, freeModeTuning());
        if (FLAGS_check_only) {
            std::cout << "TCP controller configuration and models validated" << std::endl;
            return 0;
        }
        logTcp("START", "启动 TCP 一问一答客户端",
               "配置文件 : " + FLAGS_config +
               "\n运行模式 : " + (FLAGS_sim ? std::string("HardwareSim 仿真") : std::string("真实硬件")) +
               "\n超时告警 : " + std::to_string(FLAGS_command_timeout) + " s（继续执行最后目标，不退出）" +
               "\nfree 参数 : kp=[0,0,0,0,0,0,0], kd=[4,4,4,3,2,1.5,1], friction_scale=0.6" +
               "\n逐帧日志 : " + (FLAGS_tcp_log_data ? std::string("开启（--notcp_log_data 可关闭）") : std::string("关闭")));
        TcpClient client(FLAGS_host, FLAGS_port, controller.config());
        const auto &ids = controller.config().hardwareIds;
        rocos::HardwareInterface *hardware = FLAGS_sim
            ? static_cast<rocos::HardwareInterface *>(new rocos::HardwareSim(std::max(20, *std::max_element(ids.begin(), ids.end()) + 1)))
            : new rocos::Hardware(controller.config().urdf, FLAGS_id);
        // 与现有示例相同，Robot 后台线程随进程结束；退出前显式下使能并停止网络线程。
        auto *robot = new rocos::Robot(hardware, controller.config().urdf, FLAGS_base, FLAGS_tip);
        int result = 0;
        try {
            JointClientControl control(*robot, controller, freeController, client);
            control.run();
        } catch (const std::exception &error) {
            robot->setDisabled();
            logTcp("ERROR", "TCP 控制故障", error.what(), true);
            result = 1;
        }
        robot->setDisabled();
        client.stop();
        logTcp("STOP", "TCP control stopped; robot disabled");
        std::exit(result);
    } catch (const std::exception &error) {
        logTcp("ERROR", "TCP 客户端启动失败", error.what(), true);
        return 1;
    }
}
