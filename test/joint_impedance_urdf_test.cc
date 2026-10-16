// 直接测试独立入口中的控制器；使用 KDL 作为独立动力学参考，不连接硬件。
#define main urdfImpedanceExampleMain
#include "../src/jointImpedanceUrdfMain.cc"
#undef main

#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <limits>
#include <random>

int main(int argc, char **argv) {
    try {
        require(argc == 3, "Expected YAML configuration and URDF leaf link");
        UrdfImpedanceController controller(argv[1]);
        const auto &config = controller.config();
        KDL::Tree tree;
        KDL::Chain chain;
        require(kdl_parser::treeFromFile(config.urdf, tree), "Cannot load KDL tree");
        require(tree.getChain(tree.getRootSegment()->first, argv[2], chain), "Cannot load KDL chain");
        require(chain.getNrOfJoints() == config.jointNames.size(), "KDL joint count mismatch");
        int index = 0;
        for (const auto &segment : chain.segments) {
            if (segment.getJoint().getType() != KDL::Joint::None)
                require(segment.getJoint().getName() == config.jointNames[index++], "KDL joint order mismatch");
        }

        const int n = config.jointNames.size();
        KDL::ChainDynParam dynamics(chain, KDL::Vector(0, 0, -9.81));
        KDL::JntArray kdlQ(n), gravity(n);
        Eigen::VectorXd q(n), dq(n), target(n);
        std::mt19937 random(42);
        std::uniform_real_distribution<double> unit(-1, 1);
        for (int sample = 0; sample < 100; ++sample) {
            for (int j = 0; j < n; ++j) {
                q[j] = .3 * unit(random);
                dq[j] = .5 * config.velocityLimit[j] * unit(random);
                target[j] = q[j] + .05 * unit(random);
            }
            kdlQ.data = q;
            require(dynamics.JntToGravity(kdlQ, gravity) == 0, "KDL gravity failed");
            const auto &actual = controller.computeTorque(q, dq, target);
            const Eigen::VectorXd expected = gravity.data
                + config.kp.cwiseProduct(target - q) - config.kd.cwiseProduct(dq);
            require((actual - expected).cwiseAbs().maxCoeff() < 1e-9, "Gravity + actual-velocity PD differs from KDL reference");
            require((controller.rigidTorque() - gravity.data).cwiseAbs().maxCoeff() < 1e-9,
                    "URDF debug compensation must contain gravity only");
        }

        q.setZero();
        for (int j = 0; j < n; ++j) {
            for (double sign : {-1.0, 1.0}) {
                dq.setZero();
                dq[j] = sign * config.velocityLimit[j];
                controller.computeTorque(q, dq, q); // 阈值处允许，刚刚超过则拒绝。
                dq[j] = sign * std::nextafter(config.velocityLimit[j], std::numeric_limits<double>::infinity());
                bool rejected = false;
                try { controller.computeTorque(q, dq, q); }
                catch (const std::exception &error) {
                    rejected = std::string(error.what()).find("Safety velocity limit exceeded: " + config.jointNames[j]) != std::string::npos;
                }
                require(rejected, "Custom velocity guard failed");
            }
        }
        std::cout << "PASS: 100 KDL gravity + actual-velocity PD references, gravity-only debug compensation and bidirectional speed guards" << std::endl;
    } catch (const std::exception &error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
