// 仅验证诊断历史和故障输出，不创建 Robot、不连接硬件。
#define main jointImpedanceVelocityExampleMain
#include "../src/jointImpedanceVelocityMain.cc"
#undef main

int main() {
    try {
        rocos::JointImpedanceConfig config;
        config.jointNames = {"joint_1", "joint_2"};
        config.kp = Eigen::VectorXd::Zero(2);
        config.kd = Eigen::VectorXd::Constant(2, 4);
        config.velocityLimit = Eigen::VectorXd::Constant(2, 2);
        FLAGS_debug_interval = 0;
        Diagnostics diagnostics(2);
        DebugSample sample(2);
        sample.commandComputed = sample.commandSent = true;
        sample.sent[1] = 0.4;
        sample.requested[1] = 0.6;
        for (int i = 0; i < 700; ++i) {
            sample.time = i * 0.001;
            sample.cycleMs = 1;
            sample.q[1] = 1.5 * sample.time;
            sample.dq[1] = 1.5;
            diagnostics.estimatePositionVelocity(sample);
            if (i < 20) require(!std::isfinite(sample.dqPosition[1]), "Insufficient position window was reported as valid");
            if (i >= 21) {
                require(std::abs(sample.dqPosition[1] - 1.5) < 1e-10, "Position velocity failed across ring wrap");
                require(sample.positionWindowMs >= 20, "Position window was shorter than 20 ms");
            }
            diagnostics.record(sample);
        }
        // 模拟保护在计算力矩前抛异常；必须保存当前异常输入及上一成功命令。
        sample.time = 0.7;
        sample.q[1] = 1.05;
        sample.dq[1] = 2.006143;
        sample.rawVelocity[1] = 1226064;
        sample.commandComputed = sample.commandSent = false;
        diagnostics.estimatePositionVelocity(sample);
        diagnostics.record(sample);
        std::ostringstream captured;
        auto *old = std::cerr.rdbuf(captured.rdbuf());
        diagnostics.printFault(config);
        std::cerr.rdbuf(old);
        const auto output = captured.str();
        for (const auto *text : {"故障现场", "故障前历史", "joint_2", "OVER", "2.0061",
                                 "1226064", "0.4000", "no/no", "上一成功周期", "缓存 512 周期"})
            require(output.find(text) != std::string::npos, "Fault data or stale-command label missing");
        require(output.find("[故障前历史] joint_1") == std::string::npos,
                "Fault history selected a joint that did not exceed its limit");
        sample.dq[0] = -2.1;
        diagnostics.updateLatest(sample);
        captured.str("");
        old = std::cerr.rdbuf(captured.rdbuf());
        diagnostics.printFault(config);
        std::cerr.rdbuf(old);
        require(captured.str().find("[故障前历史] joint_1") != std::string::npos,
                "Negative-direction overspeed was omitted");
        require(captured.str().find("[故障前历史] joint_2") != std::string::npos,
                "Simultaneous overspeed was omitted");
        require(displayValue(-0.000001) == 0 && sample.dq[1] == 2.006143,
                "Display formatting changed control data");
        std::cout << "PASS: position diagnostic window, ring wrap, fault input retention, signed overspeed, debug disabled\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
