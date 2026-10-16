// Hold the starting position with identified dynamics and joint impedance.
// Replace qDesired in the control thread to connect an upstream trajectory.
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <rocos_app/robot.h>
#include <rocos_app/joint_impedance.h>
#include <gflags/gflags.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

DEFINE_string(config, "config/identified_impedance.yaml", "Impedance YAML configuration");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");
DEFINE_bool(sim, true, "Use HardwareSim (interface test, no physical torque dynamics)");
DEFINE_int32(id, 0, "Hardware instance ID");
DEFINE_bool(check_only, false, "Validate configuration and models without connecting hardware");
DEFINE_double(duration, 0, "Hold duration in seconds; zero holds until SIGINT/SIGTERM");

namespace {
volatile std::sig_atomic_t stopped = 0;
void signalHandler(int) { stopped = 1; }
void require(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}

std::vector<int> mapRobotJoints(rocos::Robot &robot, const rocos::JointImpedanceConfig &config) {
    const int n = config.jointNames.size();
    require(robot.getJointNum() == n, "Robot/model joint count mismatch");
    std::vector<int> indices(n, -1);
    for (int j = 0; j < n; ++j) {
        for (int r = 0; r < n; ++r) {
            if (robot.getJointHardwareId(r) == config.hardwareIds[j]) indices[j] = r;
        }
        require(indices[j] >= 0, "Model joint not found in Robot");
    }
    return indices;
}

void readJointState(rocos::Robot &robot, const std::vector<int> &indices,
                    Eigen::VectorXd &q, Eigen::VectorXd &dq) {
    for (size_t j = 0; j < indices.size(); ++j) {
        q[j] = robot.getJointPosition(indices[j]);
        dq[j] = robot.getJointVelocity(indices[j]);
    }
}

void limitTorque(const rocos::JointImpedanceConfig &config, const Eigen::VectorXd &requested,
                 Eigen::VectorXd &previous) {
    for (int j = 0; j < requested.size(); ++j) {
        const double limit = config.torqueLimit[j];
        const double step = config.torqueRateLimit[j] * DELTA_T;
        const double bounded = std::max(-limit, std::min(limit, requested[j]));
        previous[j] = std::max(previous[j] - step, std::min(previous[j] + step, bounded));
    }
}

void run(rocos::Robot &robot, rocos::JointImpedanceController &controller) {
    const auto &config = controller.config();
    const auto indices = mapRobotJoints(robot, config);
    const int n = indices.size();
    require(!config.needsTemperature, "Hold example has no temperature source; supply measured temperatures through the C++ interface");
    robot.setDisabled();
    Eigen::VectorXd q(n), dq(n), qDesired(n), previous(n), command(n);
    readJointState(robot, indices, q, dq);
    require(q.allFinite() && dq.allFinite() && dq.cwiseAbs().maxCoeff() < 0.02, "Robot must be stationary");
    qDesired = q; // Initial spring deflection is zero; no automatic MoveJ.
    previous = controller.computeTorque(q, dq, qDesired); // Also warms up the model before enabling.
    require((previous.cwiseAbs().array() <= config.torqueLimit.array()).all(), "Initial compensation exceeds torque limits");
    for (int j = 0; j < n; ++j) {
        const int r = indices[j];
        require(FLAGS_sim || robot.getJointStatus(r) != 2, "Drive did not disable before switching to CST");
        robot.setJointMode(r, rocos::ModeOfOperation::CyclicSynchronousTorqueMode);
        command[r] = previous[j];
    }
    require(!stopped && robot.setTorque(command) == 0, "Cannot preload CST torque");
    robot.setEnabled();
    require(robot.isEnabled(), "Robot did not enable");
    std::cout << "Holding current position; target order:";
    for (const auto &name : config.jointNames) std::cout << ' ' << name;
    std::cout << std::endl;

    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    auto last = start;
    while (!stopped) {
        const auto now = Clock::now();
        if (FLAGS_duration > 0 && std::chrono::duration<double>(now - start).count() >= FLAGS_duration) break;
        require(FLAGS_sim || std::chrono::duration<double>(now - last).count() <= 0.020, "Control cycle exceeded 20 ms");
        last = now;
        require(robot.isEnabled(), "Robot was disabled during impedance control");
        readJointState(robot, indices, q, dq);
        // C++ integration point: set qDesired here from the upstream trajectory.
        // Use continuous targets in config.jointNames order; no network I/O here.
        const auto &torque = controller.computeTorque(q, dq, qDesired);
        limitTorque(config, torque, previous);
        for (int j = 0; j < n; ++j) {
            command[indices[j]] = previous[j];
        }
        require(robot.setTorque(command) == 0, "Robot rejected CST torque"); // Waits one bus cycle.
    }
}
} // namespace

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    try {
        require(std::isfinite(FLAGS_duration) && FLAGS_duration >= 0, "Invalid duration");
        rocos::JointImpedanceController controller(FLAGS_config);
        if (FLAGS_check_only) {
            std::cout << "Impedance configuration and Studio models validated" << std::endl;
            return 0;
        }
        const auto &config = controller.config();
        rocos::HardwareInterface *hw;
        if (FLAGS_sim)
            hw = new rocos::HardwareSim(std::max(20, *std::max_element(config.hardwareIds.begin(), config.hardwareIds.end()) + 1));
        else
            hw = new rocos::Hardware(config.urdf, FLAGS_id);
        rocos::Robot robot(hw, config.urdf, FLAGS_base, FLAGS_tip);
        int result = 0;
        try {
            run(robot, controller);
        } catch (const std::exception &error) {
            std::cerr << error.what() << std::endl;
            result = 1;
        }
        robot.setDisabled();
        std::cout << "Impedance control stopped; robot disabled" << std::endl;
        // Robot/DriveGuard own process-lifetime worker threads (as in dynamicsIdMain).
        std::exit(result);
    } catch (const std::exception &error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
