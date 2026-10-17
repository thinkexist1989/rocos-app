// 关节阻抗示例：读取配置和模型 → 切换 CST → 预置补偿力矩 → 上使能控制。
// 默认保持启动位置；外部轨迹只需在控制线程中更新 qDesired。
// 速度反馈验证版本：直接使用 getJointVelocity()，按 URDF vel_per_unit 换算。
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
#include <limits>
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

// 所有快照和故障历史均在上使能前分配，控制线程只复制数据，不格式化或写终端。
struct DebugSample {
    double time = 0, cycleMs = 0, positionWindowMs = 0;
    bool commandComputed = false, commandSent = false;
    // rigid/friction/requested 是最近成功计算值；故障周期未计算时必须明确标注。
    Eigen::VectorXd q, dq, dqPosition, target, loadTorque, actualTorque, rawVelocity,
                    rigid, friction, requested, sent;
    explicit DebugSample(int n)
        : q(Eigen::VectorXd::Zero(n)), dq(q), dqPosition(q), target(q), loadTorque(q),
          actualTorque(q), rawVelocity(q), rigid(q), friction(q), requested(q), sent(q) {
        dqPosition.setConstant(std::numeric_limits<double>::quiet_NaN());
    }
};

// 只对显示值消除 -0.0000；不修改控制数据。
double displayValue(double value) { return std::abs(value) < 0.00005 ? 0.0 : value; }
void cell(std::ostream &out, double value, int width = 11) {
    if (std::isfinite(value)) out << std::setw(width) << displayValue(value);
    else out << std::setw(width) << "--";
}
size_t jointWidth(const rocos::JointImpedanceConfig &config) {
    size_t width = 10;
    for (const auto &name : config.jointNames) width = std::max(width, name.size() + 1);
    return width;
}

std::string formatSample(const rocos::JointImpedanceConfig &config, const DebugSample &s) {
    const auto width = jointWidth(config);
    std::ostringstream out;
    out << std::fixed << std::setprecision(4)
        << "\n[阻抗调试] t=" << s.time << " s | cycle=" << s.cycleMs << " ms"
        << " | 位置差分窗口=" << s.positionWindowMs << " ms\n"
        << "状态：位置 rad，速度 rad/s；dq_pos 仅诊断，不参与控制；OVER 表示当前超速。\n"
        << std::left << std::setw(width) << "joint" << std::right;
    for (const auto *name : {"q", "q_target", "error", "dq_drive", "dq_pos", "vel_limit"})
        out << std::setw(11) << name;
    out << std::setw(15) << "vel_raw" << std::setw(8) << "status" << '\n'
        << std::string(width + 89, '-') << '\n';
    for (int j = 0; j < s.q.size(); ++j) {
        out << std::left << std::setw(width) << config.jointNames[j] << std::right;
        for (double value : {s.q[j], s.target[j], s.target[j] - s.q[j], s.dq[j],
                             s.dqPosition[j], config.velocityLimit[j]}) cell(out, value);
        out << std::setw(15) << std::setprecision(0) << s.rawVelocity[j] << std::setprecision(4)
            << std::setw(8) << (!std::isfinite(s.dq[j]) ? "INVALID" :
                               std::abs(s.dq[j]) > config.velocityLimit[j] ? "OVER" : "OK") << '\n';
    }
    out << "力矩：Nm；sent 为最近成功下发值，tau_raw/load_raw 为原始反馈。\n";
    if (!s.commandComputed)
        out << "本周期未完成力矩计算；rigid/friction/requested 显示上一成功周期，spring/damping 按当前状态诊断计算。\n";
    else if (!s.commandSent)
        out << "本周期已计算，但未确认下发成功；sent 仍为上一成功下发值。\n";
    out << std::left << std::setw(width) << "joint" << std::right;
    for (const auto *name : {"rigid", "friction", "spring", "damping", "requested", "sent", "tau_raw", "load_raw"})
        out << std::setw(11) << name;
    out << '\n' << std::string(width + 88, '-') << '\n';
    for (int j = 0; j < s.q.size(); ++j) {
        out << std::left << std::setw(width) << config.jointNames[j] << std::right;
        for (double value : {s.rigid[j], s.friction[j], config.kp[j] * (s.target[j] - s.q[j]),
                             -config.kd[j] * s.dq[j], s.requested[j], s.sent[j],
                             s.actualTorque[j], s.loadTorque[j]}) cell(out, value);
        out << '\n';
    }
    return out.str();
}

// 保存最近 512 个控制周期，默认 1 kHz 下约 0.5 秒；不受终端打印间隔影响。
class Diagnostics {
public:
    explicit Diagnostics(int n) {
        samples_.reserve(512);
        for (int i = 0; i < 512; ++i) samples_.emplace_back(n);
    }
    void estimatePositionVelocity(DebugSample &s) const {
        s.positionWindowMs = 0;
        s.dqPosition.setConstant(std::numeric_limits<double>::quiet_NaN());
        // 使用至少 20 ms 的实际位置差分，避免把单周期编码器量化当作速度跳变。
        for (size_t age = 0; age < count_; ++age) {
            const auto &old = samples_[(next_ + samples_.size() - 1 - age) % samples_.size()];
            const double dt = s.time - old.time;
            if (dt >= 0.020) {
                s.dqPosition = (s.q - old.q) / dt;
                s.positionWindowMs = dt * 1000;
                return;
            }
        }
    }
    void record(const DebugSample &s) {
        samples_[next_] = s;
        next_ = (next_ + 1) % samples_.size();
        count_ = std::min(count_ + 1, samples_.size());
    }
    void updateLatest(const DebugSample &s) {
        if (count_) samples_[(next_ + samples_.size() - 1) % samples_.size()] = s;
    }
    void printFault(const rocos::JointImpedanceConfig &config) const {
        if (!count_) return;
        const auto &last = chronological(count_ - 1);
        std::cerr << "\n[故障现场：已执行下使能，以下为停机前采集的数据]\n" << formatSample(config, last);
        // 展示全部超限关节；其他故障则展示速度/限值比例最大的关节。
        bool hasOverspeed = false;
        for (int j = 0; j < last.dq.size(); ++j) {
            if (std::abs(last.dq[j]) > config.velocityLimit[j]) {
                printJointHistory(config, j);
                hasOverspeed = true;
            }
        }
        if (!hasOverspeed) {
            Eigen::Index j;
            last.dq.cwiseAbs().cwiseQuotient(config.velocityLimit).maxCoeff(&j);
            printJointHistory(config, j);
        }
        std::cerr << "诊断：dq_drive 与 dq_pos 持续同向、量级接近，支持实际加速；"
                     "持续比例偏差需核对位置和速度单位；单点差异还需考虑反馈延迟和非原子 PDO 读取。\n"
                     "friction 与 dq 同向会注入力矩；需结合实际摩擦、重力、外力和 sent 判断，不能单凭补偿值断言失稳。\n";
    }
private:
    const DebugSample &chronological(size_t index) const {
        return samples_[(next_ + samples_.size() - count_ + index) % samples_.size()];
    }
    void printJointHistory(const rocos::JointImpedanceConfig &config, int j) const {
        std::ostringstream out;
        out << std::fixed << std::setprecision(4)
            << "\n[故障前历史] " << config.jointNames[j] << " | vel_limit=" << config.velocityLimit[j]
            << " rad/s | 缓存 " << count_ << " 周期；每约 10 ms 一行，最后 10 周期全部保留\n"
            << "  t(s)    dt(ms)       q(rad)    dq_drive      dq_pos        vel_raw"
               "    friction     damping   requested        sent    tau_raw  calc/sent\n";
        double lastPrinted = -1;
        for (size_t i = 0; i < count_; ++i) {
            const auto &s = chronological(i);
            if (i && i + 10 < count_ && s.time - lastPrinted < 0.010) continue;
            out << std::setw(7) << s.time << std::setw(10) << s.cycleMs;
            for (double value : {s.q[j], s.dq[j], s.dqPosition[j]}) cell(out, value);
            out << std::setw(15) << std::setprecision(0) << s.rawVelocity[j] << std::setprecision(4);
            for (double value : {s.friction[j], -config.kd[j] * s.dq[j], s.requested[j],
                                 s.sent[j], s.actualTorque[j]}) cell(out, value);
            out << "    " << (s.commandComputed ? "yes" : "no") << '/'
                << (s.commandSent ? "yes" : "no") << '\n';
            lastPrinted = s.time;
        }
        std::cerr << out.str();
    }
    std::vector<DebugSample> samples_;
    size_t next_ = 0, count_ = 0;
};

// 终端可能阻塞：控制线程 try_lock 投递，后台线程负责格式化；停机后再等待输出结束。
class DebugPrinter {
public:
    explicit DebugPrinter(const rocos::JointImpedanceConfig &config)
        : config_(config), sample_(config.jointNames.size()) {
        if (FLAGS_debug_interval > 0) worker_ = std::thread([this] {
            try { printLoop(); }
            catch (const std::exception &) { /* 打印失败不终止控制。 */ }
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
                if (stopping_ && !pending_) return;
                snapshot = sample_;
                pending_ = false;
            }
            std::cout << formatSample(config_, snapshot) << std::flush;
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

void readJointState(rocos::Robot &robot, const std::vector<int> &indices,
                    Eigen::VectorXd &q, Eigen::VectorXd &dq) {
    for (size_t j = 0; j < indices.size(); ++j) {
        q[j] = robot.getJointPosition(indices[j]);
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

void runController(rocos::Robot &robot, rocos::JointImpedanceController &controller, DebugPrinter &debug,
                   Diagnostics &diagnostics,
                   const std::vector<double> &ratedTorque) {
    const auto &config = controller.config();
    const auto indices = mapRobotJoints(robot, config);
    const int n = indices.size();
    require(!config.needsTemperature, "Hold example has no temperature source; supply measured temperatures through the C++ interface");

    // 先下使能，再读取起始姿态；模型首次计算也在上使能前完成。
    robot.setDisabled();
    configureTorqueUnits(robot, indices, ratedTorque);
    Eigen::VectorXd q(n), dq(n), qDesired(n), previous(n), command(n);
    DebugSample sample(n);
    // 启动静止检查与运行控制均使用驱动器实测速度。
    readJointState(robot, indices, q, dq);
    require(q.allFinite() && dq.allFinite() && dq.cwiseAbs().maxCoeff() < stationarySpeed, "Robot must be stationary");
    qDesired = q; // 弹簧初始误差为零，保持当前位置。
    previous = controller.computeTorque(q, dq, qDesired);
    require((previous.cwiseAbs().array() <= config.torqueLimit.array()).all(), "Initial compensation exceeds torque limits");
    sample.target = qDesired;
    sample.rigid = controller.rigidTorque();
    sample.friction = controller.frictionTorque();
    sample.requested = previous;
    sample.sent = previous;

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
    const auto width = jointWidth(config);
    std::cout << "\n[阻抗启动] config=" << FLAGS_config << "\nURDF=" << config.urdf
              << "\n单位：位置 rad，速度 rad/s，力矩 Nm，力矩变化率 Nm/s\n"
              << std::left << std::setw(width) << "joint" << std::right << std::setw(6) << "hw_id";
    for (const auto *name : {"q_start", "kp", "kd", "vel_limit", "tau_limit", "tau_rate", "rated", "preload"})
        std::cout << std::setw(11) << name;
    std::cout << '\n' << std::string(width + 94, '-') << '\n' << std::fixed << std::setprecision(4);
    for (int j = 0; j < n; ++j) {
        std::cout << std::left << std::setw(width) << config.jointNames[j] << std::right
                  << std::setw(6) << config.hardwareIds[j];
        for (double value : {qDesired[j], config.kp[j], config.kd[j], config.velocityLimit[j],
                             config.torqueLimit[j], config.torqueRateLimit[j], ratedTorque[j], previous[j]})
            cell(std::cout, value);
        std::cout << '\n';
    }
    if (config.kp.isZero())
        std::cout << "当前 kp 全为 0：没有位置恢复力，仅有模型/摩擦补偿和速度阻尼，偏离启动位置不会被弹簧拉回。\n";
    std::cout << "摩擦补偿系数：" << config.frictionScale << "（YAML 启动时读取）\n"
              << "速度来源：getJointVelocity()，原始速度 / URDF vel_per_unit，不做在线滤波。\n"
              << "保护：单周期超速即下使能退出；停机后打印故障现场和历史，不自动恢复。\n"
              << "诊断：每周期缓存反馈，位置差分仅作对照；--debug_interval=0 仍保留故障记录。" << std::endl;

    // command 保持 Nm，Robot 按 1000/ratedTorque 换算后下发；预置和周期命令使用同一比例。
    // 下使能时先写入补偿力矩，上使能后即使用该值，而非从零力矩起步。
    require(!stopped && robot.setTorque(command) == 0, "Cannot preload CST torque");
    require(!stopped, "Interrupted before enabling");
    robot.setEnabled();
    require(robot.isEnabled(), "Robot did not enable");

    const auto start = Clock::now();
    auto last = start;
    // 保持预置力矩并等待一个总线周期，再读取反馈进入控制循环。
    require(!stopped && robot.setTorque(command) == 0, "Cannot hold preloaded CST torque");
    double nextDebug = 0;
    while (!stopped) {
        require(robot.isEnabled(), "Robot was disabled during impedance control");
        readJointState(robot, indices, q, dq);
        const auto now = Clock::now();
        const double elapsed = std::chrono::duration<double>(now - start).count();
        const double cycle = std::chrono::duration<double>(now - last).count();
        if (FLAGS_duration > 0 && elapsed >= FLAGS_duration) break;
        sample.time = elapsed;
        sample.cycleMs = cycle * 1000;
        sample.q = q;
        sample.dq = dq;
        sample.commandComputed = false;
        sample.commandSent = false;
        for (int j = 0; j < n; ++j) {
            sample.rawVelocity[j] = robot.getJointVelocityRaw(indices[j]);
            sample.actualTorque[j] = robot.getJointTorqueRaw(indices[j]);
            sample.loadTorque[j] = robot.getJointLoadTorque(indices[j]);
        }
        diagnostics.estimatePositionVelocity(sample);
        // 保护/模型计算可能抛异常，先保存本周期输入和上一成功下发值。
        diagnostics.record(sample);
        require(std::isfinite(cycle) && cycle > 0, "Invalid sampling interval");
        require(FLAGS_sim || cycle <= maxControlGap, "Control cycle exceeded 20 ms");
        // 同一个 getJointVelocity() 反馈用于动力学、摩擦、阻尼和超速检查。
        last = now;
        // 外部轨迹接入点：更新 qDesired，保持位置连续且按 config.jointNames 排列。
        // 网络收发等可能阻塞的工作放在上层线程，不放进此循环。
        // computeTorque 首先检查自定义 velocity_limit，超速则抛异常，不再下发新的力矩。
        const auto &torque = controller.computeTorque(q, dq, qDesired);
        sample.rigid = controller.rigidTorque();
        sample.friction = controller.frictionTorque();
        sample.requested = torque;
        sample.commandComputed = true;
        diagnostics.updateLatest(sample);
        limitTorque(config, torque, previous);
        for (int j = 0; j < n; ++j) {
            command[indices[j]] = previous[j];
        }
        require(robot.setTorque(command) == 0, "Robot rejected CST torque"); // 内部等待一个总线周期。
        sample.sent = previous;
        sample.commandSent = true;
        diagnostics.updateLatest(sample);
        if (FLAGS_debug_interval > 0 && elapsed >= nextDebug) {
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
        Diagnostics diagnostics(config.jointNames.size());
        DebugPrinter debug(config); // 打印线程及缓存先于 Robot 创建，初始化失败时尚未连接硬件。
        rocos::HardwareInterface *hw;
        if (FLAGS_sim)
            hw = new rocos::HardwareSim(std::max(20, *std::max_element(config.hardwareIds.begin(), config.hardwareIds.end()) + 1));
        else
            hw = new rocos::Hardware(config.urdf, FLAGS_id);
        rocos::Robot robot(hw, config.urdf, FLAGS_base, FLAGS_tip);
        int result = 0;
        try {
            runController(robot, controller, debug, diagnostics, ratedTorque);
            robot.setDisabled();
        } catch (const std::exception &error) {
            // 先停机再打印，避免日志输出阻塞而延迟下使能。
            robot.setDisabled();
            debug.stop(); // 先结束后台输出，避免现场记录与正常快照交错。
            std::cerr << error.what() << std::endl;
            diagnostics.printFault(config);
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
