// 关节阻抗示例：读取配置和模型 → 切换 CST → 预置补偿力矩 → 上使能控制。
// 默认保持启动位置；外部轨迹只需在控制线程中更新 qDesired。
// hardware 头文件放在 robot.h 前，避免现有 RESET 宏与 EtherCAT 枚举冲突。
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <rocos_app/robot.h>
#include <rocos_app/joint_impedance.h>
#include <gflags/gflags.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

DEFINE_string(config, "config/identified_impedance.yaml", "阻抗参数和辨识模型路径的 YAML 配置");
DEFINE_string(base, "base_link", "运动链基座名称");
DEFINE_string(tip, "link_7", "运动链末端名称");
DEFINE_bool(sim, true, "使用 HardwareSim 核对接口；不模拟真实力矩动力学");
DEFINE_int32(id, 0, "真机硬件实例编号");
DEFINE_bool(check_only, false, "仅检查配置和模型，不连接硬件");
DEFINE_double(duration, 0, "保持时间，单位秒；0 表示运行至 SIGINT/SIGTERM 中断");
DEFINE_double(debug_interval, 0.5, "调试打印间隔，秒；0 关闭，开启时至少 0.1 秒");

namespace {
using Clock = std::chrono::steady_clock;
constexpr double stationarySpeed = 0.02; // 启动时允许的实测速度，rad/s。
constexpr double maxControlGap = 0.020; // 真机控制周期超时阈值，s。
volatile std::sig_atomic_t stopped = 0;
// 信号处理只置标志，下使能由主线程执行。
void signalHandler(int) { stopped = 1; }
void require(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}

// 调试快照只保存一个周期的数据；控制线程使用的向量在上使能前分配。
struct DebugSample {
    double time = 0, cycleMs = 0;
    // LoadTorque 当前接口直接返回底层反馈，不擅自当作 Nm 或再次换算。
    Eigen::VectorXd q, dq, target, loadTorque, rigid, friction, requested, sent;
    explicit DebugSample(int n)
        : q(n), dq(n), target(n), loadTorque(n), rigid(n), friction(n), requested(n), sent(n) {}
};

// 终端可能阻塞：控制线程用 try_lock 投递快照，忙时丢弃；后台线程负责格式化和打印。
// 主程序先下使能，再停止打印线程，避免等待终端输出时机械臂仍在运行。
class DebugPrinter {
public:
    explicit DebugPrinter(const rocos::JointImpedanceConfig &config)
        : config_(config), sample_(config.jointNames.size()) {
        if (FLAGS_debug_interval > 0) worker_ = std::thread([this] {
            try { printLoop(); }
            catch (const std::exception &) { /* 调试输出失败只结束打印线程，不终止控制进程。 */ }
        });
    }
    ~DebugPrinter() { stop(); }

    void publish(const DebugSample &sample) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) return;
        sample_ = sample;
        pending_ = true;
        lock.unlock();
        ready_.notify_one();
    }

    void stop() {
        if (!worker_.joinable()) return;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        ready_.notify_one();
        worker_.join();
    }

private:
    void printLoop() {
        DebugSample snapshot(config_.jointNames.size());
        for (;;) {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                ready_.wait(lock, [this] { return pending_ || stopping_; });
                if (stopping_) return;
                snapshot = sample_;
                pending_ = false;
            }
            // 此处已经释放互斥锁，打印速度不影响控制线程。
            std::ostringstream text;
            text << std::fixed << std::setprecision(4)
                 << "[阻抗调试] t=" << snapshot.time << " s, cycle=" << snapshot.cycleMs << " ms\n"
                 << "joint q(rad) qd(rad) error(rad) dq(rad/s) rigid(Nm) friction(Nm)"
                    " spring(Nm) damping(Nm) requested(Nm) sent(Nm) load(raw)\n";
            for (int j = 0; j < snapshot.q.size(); ++j) {
                const double error = snapshot.target[j] - snapshot.q[j];
                text << config_.jointNames[j] << ' ' << snapshot.q[j] << ' ' << snapshot.target[j]
                     << ' ' << error << ' ' << snapshot.dq[j] << ' ' << snapshot.rigid[j]
                     << ' ' << snapshot.friction[j] << ' ' << config_.kp[j] * error
                     << ' ' << -config_.kd[j] * snapshot.dq[j] << ' ' << snapshot.requested[j]
                     << ' ' << snapshot.sent[j] << ' ' << snapshot.loadTorque[j] << '\n';
            }
            std::cout << text.str() << std::flush;
        }
    }

    const rocos::JointImpedanceConfig &config_;
    DebugSample sample_;
    std::mutex mutex_;
    std::condition_variable ready_;
    bool pending_ = false, stopping_ = false;
    std::thread worker_;
};

// 额定力矩按模型关节顺序读取；限幅仍使用 Nm，额定值只用于驱动器千分比换算。
std::vector<double> loadRatedTorques(const std::string &path, const Eigen::VectorXd &torqueLimits) {
    const auto yaml = YAML::LoadFile(path);
    const auto values = yaml["rated_torque"];
    require(values.IsDefined() && values.IsSequence() && values.size() == size_t(torqueLimits.size()),
            "rated_torque must contain one value per joint");
    auto rated = values.as<std::vector<double>>();
    for (int j = 0; j < torqueLimits.size(); ++j) {
        require(std::isfinite(rated[j]) && rated[j] > 0, "rated_torque must be finite and positive");
        const double scale = 1000.0 / rated[j];
        require(std::isfinite(scale) && torqueLimits[j] * scale <= 32767,
                "Torque limit exceeds int16 drive range after rated torque conversion");
    }
    return rated;
}

// Robot::setTorque 内部还会乘 torque_per_unit；在此覆盖比例，确保只换算一次。
// 此后传入 Robot 的仍是 Nm，驱动器收到的值为 torque / ratedTorque * 1000。
void configureTorqueUnits(rocos::Robot &robot, const std::vector<int> &indices,
                          const std::vector<double> &ratedTorque) {
    for (size_t j = 0; j < indices.size(); ++j)
        robot.setJointTorquePerUnit(indices[j], 1000.0 / ratedTorque[j]);
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

void readJointPositions(rocos::Robot &robot, const std::vector<int> &indices, Eigen::VectorXd &q) {
    for (size_t j = 0; j < indices.size(); ++j)
        q[j] = robot.getJointPosition(indices[j]);
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

void runController(rocos::Robot &robot, rocos::JointImpedanceController &controller, DebugPrinter &debug,
                   const std::vector<double> &ratedTorque) {
    const auto &config = controller.config();
    const auto indices = mapRobotJoints(robot, config);
    const int n = indices.size();
    require(!config.needsTemperature, "Hold example has no temperature source; supply measured temperatures through the C++ interface");

    // 先下使能，再读取起始姿态；模型首次计算也在上使能前完成。
    robot.setDisabled();
    configureTorqueUnits(robot, indices, ratedTorque);
    Eigen::VectorXd q(n), dq(n), previousQ(n), qDesired(n), previous(n), command(n);
    DebugSample sample(n);
    // 启动静止检查使用 20 ms 窗口的位置差分，减小编码器量化的影响。
    readJointPositions(robot, indices, previousQ);
    const auto stationaryStart = Clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    readJointPositions(robot, indices, q);
    dq = (q - previousQ) / std::chrono::duration<double>(Clock::now() - stationaryStart).count();
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
    // 启动参数直接在上使能前打印；运行中的输出交给后台线程。
    std::cout << "Starting position hold; units: rad, rad/s, Nm, Nm/s\n"
              << "joint hardware_id q_start kp kd velocity_limit torque_limit torque_rate_limit rated_torque preload\n";
    for (int j = 0; j < n; ++j) {
        std::cout << config.jointNames[j] << ' ' << config.hardwareIds[j] << ' ' << qDesired[j]
                  << ' ' << config.kp[j] << ' ' << config.kd[j] << ' ' << config.velocityLimit[j]
                  << ' ' << config.torqueLimit[j] << ' ' << config.torqueRateLimit[j]
                  << ' ' << ratedTorque[j] << ' ' << previous[j] << '\n';
    }
    std::cout << "摩擦补偿系数：" << config.frictionScale << "（YAML 启动时读取）\n"
              << "速度来源：实际位置差分 / 实际采样间隔，不做在线滤波。\n"
              << "超速故障将下使能并退出，需人工确认后重新启动。" << std::endl;

    // command 保持 Nm，Robot 按 1000/ratedTorque 换算后下发；预置和周期命令使用同一比例。
    // 下使能时先写入补偿力矩，上使能后即使用该值，而非从零力矩起步。
    require(!stopped && robot.setTorque(command) == 0, "Cannot preload CST torque");
    require(!stopped, "Interrupted before enabling");
    robot.setEnabled();
    require(robot.isEnabled(), "Robot did not enable");

    readJointPositions(robot, indices, previousQ);
    const auto start = Clock::now();
    auto last = start;
    // 保持预置力矩并等待一个总线周期，为首周期位置差分提供有效时间间隔。
    require(!stopped && robot.setTorque(command) == 0, "Cannot hold preloaded CST torque");
    double nextDebug = 0;
    while (!stopped) {
        require(robot.isEnabled(), "Robot was disabled during impedance control");
        readJointPositions(robot, indices, q);
        const auto now = Clock::now();
        const double elapsed = std::chrono::duration<double>(now - start).count();
        const double cycle = std::chrono::duration<double>(now - last).count();
        if (FLAGS_duration > 0 && elapsed >= FLAGS_duration) break;
        require(std::isfinite(cycle) && cycle > 0, "Invalid sampling interval");
        require(FLAGS_sim || cycle <= maxControlGap, "Control cycle exceeded 20 ms");
        // 同一个差分速度用于动力学、摩擦、阻尼和超速检查；不依赖驱动器速度单位。
        dq = (q - previousQ) / cycle;
        previousQ = q;
        last = now;
        // 外部轨迹接入点：更新 qDesired，保持位置连续且按 config.jointNames 排列。
        // 网络收发等可能阻塞的工作放在上层线程，不放进此循环。
        // computeTorque 首先检查自定义 velocity_limit，超速则抛异常，不再下发新的力矩。
        const auto &torque = controller.computeTorque(q, dq, qDesired);
        limitTorque(config, torque, previous);
        for (int j = 0; j < n; ++j) {
            command[indices[j]] = previous[j];
        }
        require(robot.setTorque(command) == 0, "Robot rejected CST torque"); // 内部等待一个总线周期。
        if (FLAGS_debug_interval > 0 && elapsed >= nextDebug) {
            sample.time = elapsed;
            sample.cycleMs = cycle * 1000;
            sample.q = q;
            sample.dq = dq;
            sample.target = qDesired;
            sample.rigid = controller.rigidTorque();
            sample.friction = controller.frictionTorque();
            sample.requested = torque;
            sample.sent = previous;
            for (int j = 0; j < n; ++j) sample.loadTorque[j] = robot.getJointLoadTorque(indices[j]);
            debug.publish(sample);
            nextDebug = elapsed + FLAGS_debug_interval;
        }
    }
}
} // namespace

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    std::signal(SIGPIPE, SIG_IGN); // 关闭输出管道不应意外终止仍在运行的控制进程。
    try {
        require(std::isfinite(FLAGS_duration) && FLAGS_duration >= 0, "Invalid duration");
        require(std::isfinite(FLAGS_debug_interval) &&
                (FLAGS_debug_interval == 0 || FLAGS_debug_interval >= 0.1), "debug_interval must be 0 or at least 0.1 s");
        rocos::JointImpedanceController controller(FLAGS_config);
        const auto ratedTorque = loadRatedTorques(FLAGS_config, controller.config().torqueLimit);
        if (FLAGS_check_only) {
            std::cout << "Impedance configuration and Studio models validated" << std::endl;
            return 0;
        }
        const auto &config = controller.config();
        DebugPrinter debug(config); // 打印线程及缓存先于 Robot 创建，初始化失败时尚未连接硬件。
        rocos::HardwareInterface *hw;
        if (FLAGS_sim)
            hw = new rocos::HardwareSim(std::max(20, *std::max_element(config.hardwareIds.begin(), config.hardwareIds.end()) + 1));
        else
            hw = new rocos::Hardware(config.urdf, FLAGS_id);
        rocos::Robot robot(hw, config.urdf, FLAGS_base, FLAGS_tip);
        int result = 0;
        try {
            runController(robot, controller, debug, ratedTorque);
            robot.setDisabled();
        } catch (const std::exception &error) {
            // 先停机再打印，避免日志输出阻塞而延迟下使能。
            robot.setDisabled();
            std::cerr << error.what() << std::endl;
            result = 1;
        }
        debug.stop(); // 此时已下使能，可以等待打印线程退出。
        std::cout << "Impedance control stopped; robot disabled" << std::endl;
        // Robot 当前析构函数不会停止后台线程，显式下使能后保持对象至进程退出。
        std::exit(result);
    } catch (const std::exception &error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
