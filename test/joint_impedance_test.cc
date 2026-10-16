#include <rocos_app/joint_impedance.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

Eigen::VectorXd readVector(const YAML::Node &node) {
    Eigen::VectorXd result(node.size());
    for (size_t i = 0; i < node.size(); ++i) result[i] = node[i].as<double>();
    return result;
}

int main(int argc, char **argv) {
    try {
        if (argc != 3) throw std::runtime_error("Expected configuration and Studio reference cases");
        rocos::JointImpedanceController controller(argv[1]);
        const auto cases = YAML::LoadFile(argv[2]);
        for (const auto &sample : cases) {
            const auto q = readVector(sample["q"]);
            const auto dq = readVector(sample["dq"]);
            const auto target = readVector(sample["target"]);
            const auto temperature = readVector(sample["temperature"]);
            const auto expected = readVector(sample["torque"]);
            const auto &actual = controller.computeTorque(q, dq, target, temperature);
            if ((actual - expected).cwiseAbs().maxCoeff() > 1e-9)
                throw std::runtime_error("Controller differs from Studio reference");
            const auto reconstructed = controller.rigidTorque() + controller.frictionTorque()
                + controller.config().kp.cwiseProduct(target - q) - controller.config().kd.cwiseProduct(dq);
            if ((actual - reconstructed).cwiseAbs().maxCoeff() > 1e-9)
                throw std::runtime_error("Debug torque components do not match command");
        }
        Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
        Eigen::VectorXd dq = q, target = q;
        auto mustReject = [&](const Eigen::VectorXd &badQ, const Eigen::VectorXd &badTarget) {
            try { controller.computeTorque(badQ, dq, badTarget); }
            catch (const std::exception &) { return; }
            throw std::runtime_error("Invalid input was accepted");
        };
        mustReject(Eigen::VectorXd::Zero(6), target);
        target[6] = std::numeric_limits<double>::quiet_NaN();
        mustReject(q, target);
        target[6] = controller.config().upper[6] + 1;
        mustReject(q, target);
        if (controller.config().needsTemperature) mustReject(q, q);
        // 每个关节在正负阈值处允许计算，刚刚超过自定义阈值即拒绝，独立于 URDF 极限。
        const auto temperature = Eigen::VectorXd::Constant(7, 25);
        for (int j = 0; j < 7; ++j) {
            for (double sign : {-1.0, 1.0}) {
                dq.setZero();
                const double limit = controller.config().velocityLimit[j];
                dq[j] = sign * limit;
                controller.computeTorque(q, dq, q, temperature);
                dq[j] = sign * std::nextafter(limit, std::numeric_limits<double>::infinity());
                bool rejected = false;
                try { controller.computeTorque(q, dq, q, temperature); }
                catch (const std::exception &error) {
                    const std::string message = error.what();
                    rejected = message.find("Safety velocity limit exceeded: " + controller.config().jointNames[j]) != std::string::npos;
                }
                if (!rejected) throw std::runtime_error("Custom velocity guard failed");
            }
        }
        std::cout << "PASS: Studio torque reference, debug components, invalid input and bidirectional speed guards" << std::endl;
    } catch (const std::exception &error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
