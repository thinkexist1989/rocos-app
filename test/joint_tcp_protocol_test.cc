// 仅检查消息解析和反馈语义，不创建 Robot、不连接硬件。
#define main jointTcpClientExampleMain
#include "../src/jointTcpClientMain.cc"
#undef main

int main() {
    try {
        rocos::JointImpedanceConfig config;
        config.lower = Eigen::VectorXd::Constant(7, -1);
        config.upper = Eigen::VectorXd::Constant(7, 1);
        config.jointNames = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
        Json command = {{"frame_header", "0xAA56"}, {"timestamp_us", 1},
                        {"mode", "position"}, {"q", Joints{}}, {"tau", Joints{}}};
        for (const auto *mode : {"position", "impedance", "impedance_friction", "stop"}) {
            command["mode"] = mode;
            const auto expected = parseCommand(command.dump(), config);
            for (const Json &ignored : {Json(nullptr), Json("unused"), Json{1000000, -1000000}, Json::object()}) {
                command["tau"] = ignored;
                const auto parsed = parseCommand(command.dump(), config);
                require(parsed.q == expected.q && parsed.mode == expected.mode, "Ignored tau changed command");
            }
        }
        auto rejects = [&](const std::string &text) {
            bool rejected = false;
            try { parseCommand(text, config); } catch (const std::exception &) { rejected = true; }
            require(rejected, "Invalid command was accepted");
        };
        Json bad = command;
        bad.erase("q"); rejects(bad.dump()); // stop 也必须携带 q。
        bad = command; bad["q"] = {0, 0}; rejects(bad.dump());
        bad = command; bad["q"][1] = 1.01; rejects(bad.dump());
        bad = command; bad["q"][1] = true; rejects(bad.dump());
        bad = command; bad["mode"] = "torque"; rejects(bad.dump());
        bad = command; bad["frame_header"] = "0xAA55"; rejects(bad.dump());
        bad = command; bad["timestamp_us"] = -1; rejects(bad.dump());
        bad = command; bad["timestamp_us"] = 1.5; rejects(bad.dump());
        rejects("{\"mode\":\"position\",\"mode\":\"stop\"}");
        rejects("{\"q\":[NaN]}");
        rejects("not json");
        command.erase("tau");
        require(parseCommand(command.dump(), config).q == Joints{}, "Missing tau changed target");
        Feedback state;
        auto feedback = feedbackJson(state);
        require(feedback["tau_calculated"].is_null() && feedback["tau_friction"].is_null(), "CSP feedback must be null");
        state.torqueValid = true;
        feedback = feedbackJson(state);
        require(feedback["tau_calculated"].size() == 7 && feedback["tau_friction"].size() == 7, "CST feedback must contain seven torques");
        std::cout << "PASS: four command modes, ignored tau, stop target, bounds/types and nullable torque feedback\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
