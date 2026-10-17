// 使用本机回环 TCP 验证一问一答，不创建 Robot、不连接硬件。
#define main jointTcpRequestReplyExampleMain
#include "../src/jointTcpRequestReplyClientMain.cc"
#undef main

namespace {
struct Socket {
    int fd = -1;
    ~Socket() { if (fd >= 0) close(fd); }
};

void sendAll(int fd, const std::string &data) {
    size_t sent = 0;
    while (sent < data.size()) {
        const auto count = send(fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
        require(count > 0, "Test server send failed");
        sent += static_cast<size_t>(count);
    }
}

void expectQuiet(int fd, const std::string &buffer) {
    require(buffer.empty(), "Unexpected extra response bytes");
    pollfd event{fd, POLLIN, 0};
    require(poll(&event, 1, 80) == 0, "Client sent unsolicited or duplicate feedback");
}

Json readReply(int fd, std::string &buffer) {
    const auto deadline = Clock::now() + std::chrono::seconds(2);
    while (buffer.find('\n') == std::string::npos) {
        require(Clock::now() < deadline, "Timed out waiting for reply");
        pollfd event{fd, POLLIN, 0};
        const int result = poll(&event, 1, 20);
        require(result >= 0, "Test server poll failed");
        if (result == 0) continue;
        char bytes[4096];
        const auto count = recv(fd, bytes, sizeof(bytes), 0);
        require(count > 0, "Client disconnected before reply");
        buffer.append(bytes, static_cast<size_t>(count));
    }
    const auto newline = buffer.find('\n');
    const auto reply = Json::parse(buffer.substr(0, newline));
    buffer.erase(0, newline + 1);
    return reply;
}

void verifyFreeTorque(const std::string &configPath) {
    rocos::JointImpedanceController regular(configPath);
    rocos::JointImpedanceController free(configPath, freeModeTuning());
    auto fullTuning = freeModeTuning();
    fullTuning.kd.setZero();
    fullTuning.frictionScale = 1;
    rocos::JointImpedanceController fullFriction(configPath, fullTuning);
    const auto originalKp = regular.config().kp;
    const auto originalKd = regular.config().kd;
    const double originalScale = regular.config().frictionScale;
    Eigen::VectorXd q = Eigen::VectorXd::Zero(jointCount);
    Eigen::VectorXd dq(jointCount);
    dq << .03, -.04, .05, -.06, .02, -.025, .015;
    fullFriction.computeTorque(q, dq, q);
    const Eigen::VectorXd expectedFriction = .6 * fullFriction.frictionTorque();
    Eigen::VectorXd expectedKd(jointCount);
    expectedKd << 4, 4, 4, 3, 2, 1.5, 1;
    const Eigen::VectorXd expected = fullFriction.rigidTorque() + expectedFriction - expectedKd.cwiseProduct(dq);
    const Eigen::VectorXd actual = free.computeTorque(q, dq, q);
    require((actual - expected).cwiseAbs().maxCoeff() < 1e-10, "free must use its fixed damping and 0.6 friction scale");
    require((free.frictionTorque() - expectedFriction).cwiseAbs().maxCoeff() < 1e-10, "free friction feedback is incorrect");
    const Eigen::VectorXd otherTarget = (q.array() + .2).matrix().cwiseMax(regular.config().lower).cwiseMin(regular.config().upper);
    require((free.computeTorque(q, dq, otherTarget) - actual).cwiseAbs().maxCoeff() < 1e-10,
            "free target must not produce position stiffness");
    require(regular.config().kp == originalKp && regular.config().kd == originalKd &&
            regular.config().frictionScale == originalScale, "free changed regular impedance tuning");
    auto invalid = freeModeTuning();
    invalid.frictionScale = 1.1;
    bool rejected = false;
    try { rocos::JointImpedanceController bad(configPath, invalid); }
    catch (const std::exception &) { rejected = true; }
    require(rejected, "Invalid override friction scale was accepted");
}
} // namespace

int main(int argc, char **argv) {
    try {
        require(argc == 2, "Expected configuration path");
        verifyFreeTorque(argv[1]);
        FLAGS_tcp_log_data = false;
        Socket listener{socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0)};
        require(listener.fd >= 0, "Cannot create test server socket");
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        require(bind(listener.fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0 &&
                listen(listener.fd, 1) == 0, "Cannot listen on loopback");
        socklen_t size = sizeof(address);
        require(getsockname(listener.fd, reinterpret_cast<sockaddr *>(&address), &size) == 0,
                "Cannot get test server port");

        rocos::JointImpedanceConfig config;
        config.lower = Eigen::VectorXd::Constant(7, -1);
        config.upper = Eigen::VectorXd::Constant(7, 1);
        config.jointNames = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
        TcpClient client("127.0.0.1", ntohs(address.sin_port), config);
        Socket server{accept(listener.fd, nullptr, nullptr)};
        require(server.fd >= 0, "Cannot accept test client");
        std::string buffer;
        expectQuiet(server.fd, buffer);

        Json command = {{"frame_header", "0xAA56"}, {"timestamp_us", 1},
                        {"mode", "position"}, {"q", Joints{}}};
        const std::string line = command.dump() + '\n';
        // 首次真实采样前的请求不能丢失，也不能返回默认零状态。
        sendAll(server.fd, line);
        expectQuiet(server.fd, buffer);
        // 服务端短时积压超过回复上限时应用背压，不能丢回复或导致退出。
        const size_t queuedCommands = maxPendingReplies + 32;
        std::string burst;
        for (size_t i = 0; i < queuedCommands; ++i) burst += line;
        sendAll(server.fd, burst);
        expectQuiet(server.fd, buffer);
        client.check();
        Feedback state;
        state.timestamp = 123;
        state.q[0] = .25;
        // 即使保存着上次阻抗模式的计算值，位置/stop 反馈也必须输出七个零。
        state.calculated.fill(1.5);
        state.friction.fill(.25);
        require(feedbackJson(state)["tau_calculated"] == Json(Joints{}) &&
                feedbackJson(state)["tau_friction"] == Json(Joints{}), "CSP torque fields must be zero arrays");
        state.torqueValid = true;
        require(feedbackJson(state)["tau_calculated"] == Json(state.calculated) &&
                feedbackJson(state)["tau_friction"] == Json(state.friction), "CST torque feedback was changed");
        state.torqueValid = false;
        client.publish(state);
        require(readReply(server.fd, buffer) == feedbackJson(state), "Initial reply is not current state");
        for (size_t i = 0; i < queuedCommands; ++i)
            require(readReply(server.fd, buffer) == feedbackJson(state), "Backpressure lost a queued reply");
        expectQuiet(server.fd, buffer);

        state.timestamp = 456;
        state.q[0] = .5;
        client.publish(state);
        expectQuiet(server.fd, buffer); // 更新状态本身不触发发送。
        sendAll(server.fd, line.substr(0, 9));
        expectQuiet(server.fd, buffer); // TCP 分包未收齐一行时不回复。
        sendAll(server.fd, line.substr(9));
        require(readReply(server.fd, buffer) == feedbackJson(state), "Reply did not use latest state");
        expectQuiet(server.fd, buffer);

        // 粘包中三条有效指令各回复一次，非法消息和空行不触发回复。
        std::string batch = "not json\n\n";
        for (double q : {.1, .2, .3}) {
            command["q"][0] = q;
            batch += command.dump() + '\n';
        }
        sendAll(server.fd, batch);
        for (int i = 0; i < 3; ++i)
            require(readReply(server.fd, buffer) == feedbackJson(state), "Coalesced command lost its reply");
        expectQuiet(server.fd, buffer);
        Command latest;
        require(client.latest(latest) && latest.sequence == 5 + queuedCommands && latest.q[0] == .3,
                "Control mailbox did not retain latest valid target");
        Json freeCommand = {{"frame_header", "0xAA56"}, {"timestamp_us", 2}, {"mode", "free"}};
        require(parseCommand(freeCommand.dump(), config).mode == Mode::Free && torqueMode(Mode::Free),
                "free command must select CST without requiring q");
        auto badFree = freeCommand;
        badFree["q"] = {2, 0, 0, 0, 0, 0, 0};
        bool rejected = false;
        try { parseCommand(badFree.dump(), config); }
        catch (const std::exception &) { rejected = true; }
        require(rejected, "Supplied free q must still be validated");
        sendAll(server.fd, freeCommand.dump() + '\n');
        require(readReply(server.fd, buffer) == feedbackJson(state), "free command did not receive a state reply");
        expectQuiet(server.fd, buffer);
        require(client.latest(latest) && latest.mode == Mode::Free, "free did not reach the command mailbox");
        client.check();
        client.stop();
        std::cout << "PASS: no unsolicited feedback, one reply per valid command, latest state, "
                     "startup sampling, split/coalesced messages, rejected commands, bounded reply backpressure and fixed free torque\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
