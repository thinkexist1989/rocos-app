// 关节阻抗示例：读取配置和模型 → 切换 CST → 预置补偿力矩 → 上使能控制。
// 默认保持启动位置；外部轨迹只需在控制线程中更新 qDesired。
// hardware 头文件放在 robot.h 前，避免现有 RESET 宏与 EtherCAT 枚举冲突。
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
#include <vector>

DEFINE_string(config, "config/identified_impedance.yaml", "阻抗参数和辨识模型路径的 YAML 配置");
DEFINE_string(base, "base_link", "运动链基座名称");
DEFINE_string(tip, "link_7", "运动链末端名称");
DEFINE_bool(sim, true, "使用 HardwareSim 核对接口；不模拟真实力矩动力学");
DEFINE_int32(id, 0, "真机硬件实例编号");
DEFINE_bool(check_only, false, "仅检查配置和模型，不连接硬件");
DEFINE_double(duration, 0, "保持时间，单位秒；0 表示运行至 SIGINT/SIGTERM 中断");

namespace {
constexpr double stationarySpeed = 0.02; // 启动时允许的实测速度，rad/s。
constexpr double maxControlGap = 0.020; // 真机控制周期超时阈值，s。
volatile std::sig_atomic_t stopped = 0;
// 信号处理只置标志，下使能由主线程执行。
void signalHandler(int) { stopped = 1; }
void require(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}

// 模型列顺序不一定等于 Robot 顺序，用 URDF 硬件 ID 建立一一映射。
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
        // 当前直接使用驱动器实测速度，此处没有附加滤波，也不使用期望速度。
        dq[j] = robot.getJointVelocity(indices[j]);
    }
}

// 先限绝对力矩，再限制相邻周期的变化量；previous 保存实际下发前的限制结果。
void limitTorque(const rocos::JointImpedanceConfig &config, const Eigen::VectorXd &requested,
                 Eigen::VectorXd &previous) {
    for (int j = 0; j < requested.size(); ++j) {
        const double limit = config.torqueLimit[j]; // Nm。
        const double step = config.torqueRateLimit[j] * DELTA_T; // Nm/s × 控制周期。
        const double bounded = std::max(-limit, std::min(limit, requested[j]));
        previous[j] = std::max(previous[j] - step, std::min(previous[j] + step, bounded));
    }
}

void runController(rocos::Robot &robot, rocos::JointImpedanceController &controller) {
    const auto &config = controller.config();
    const auto indices = mapRobotJoints(robot, config);
    const int n = indices.size();
    require(!config.needsTemperature, "Hold example has no temperature source; supply measured temperatures through the C++ interface");

    // 先下使能，再读取起始姿态；模型首次计算也在上使能前完成。
    robot.setDisabled();
    Eigen::VectorXd q(n), dq(n), qDesired(n), previous(n), command(n);
    readJointState(robot, indices, q, dq);
    require(q.allFinite() && dq.allFinite() && dq.cwiseAbs().maxCoeff() < stationarySpeed, "Robot must be stationary");
    qDesired = q; // 弹簧初始误差为零，保持当前位置。
    previous = controller.computeTorque(q, dq, qDesired);
    require((previous.cwiseAbs().array() <= config.torqueLimit.array()).all(), "Initial compensation exceeds torque limits");

    // 所有关节确认退出使能后，再逐个切换 CST，避免检查到一半才发现失败。
    for (int r : indices) {
        require(FLAGS_sim || robot.getJointStatus(r) != 2, "Drive did not disable before switching to CST");
    }
    for (int j = 0; j < n; ++j) {
        const int r = indices[j];
        robot.setJointMode(r, rocos::ModeOfOperation::CyclicSynchronousTorqueMode);
        command[r] = previous[j]; // 模型顺序转换为 Robot 顺序。
    }
    // 终端输出可能阻塞，放在上使能前；控制循环内不打印。
    std::cout << "Starting position hold; target order:";
    for (const auto &name : config.jointNames) std::cout << ' ' << name;
    std::cout << std::endl;

    // 下使能时先写入补偿力矩，上使能后即使用该值，而非从零力矩起步。
    require(!stopped && robot.setTorque(command) == 0, "Cannot preload CST torque");
    require(!stopped, "Interrupted before enabling");
    robot.setEnabled();
    require(robot.isEnabled(), "Robot did not enable");

    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    auto last = start;
    while (!stopped) {
        const auto now = Clock::now();
        if (FLAGS_duration > 0 && std::chrono::duration<double>(now - start).count() >= FLAGS_duration) break;
        require(FLAGS_sim || std::chrono::duration<double>(now - last).count() <= maxControlGap, "Control cycle exceeded 20 ms");
        last = now;
        require(robot.isEnabled(), "Robot was disabled during impedance control");
        readJointState(robot, indices, q, dq);
        // 外部轨迹接入点：更新 qDesired，保持位置连续且按 config.jointNames 排列。
        // 网络收发等可能阻塞的工作放在上层线程，不放进此循环。
        const auto &torque = controller.computeTorque(q, dq, qDesired);
        limitTorque(config, torque, previous);
        for (int j = 0; j < n; ++j) {
            command[indices[j]] = previous[j];
        }
        require(robot.setTorque(command) == 0, "Robot rejected CST torque"); // 内部等待一个总线周期。
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
            runController(robot, controller);
            robot.setDisabled();
        } catch (const std::exception &error) {
            // 先停机再打印，避免日志输出阻塞而延迟下使能。
            robot.setDisabled();
            std::cerr << error.what() << std::endl;
            result = 1;
        }
        std::cout << "Impedance control stopped; robot disabled" << std::endl;
        // Robot 当前析构函数不会停止后台线程，显式下使能后保持对象至进程退出。
        std::exit(result);
    } catch (const std::exception &error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
