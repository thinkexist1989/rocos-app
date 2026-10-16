// URDF 关节阻抗：测试时将动力学计算的速度置零，只补偿重力 g(q)，不补偿摩擦。
// 默认保持启动位置；外部轨迹只需在控制线程中更新 qDesired。
// hardware 头文件放在 robot.h 前，避免现有 RESET 宏与 EtherCAT 枚举冲突。
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <rocos_app/robot.h>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <yaml-cpp/yaml.h>
#include <tinyxml2.h>
#include <gflags/gflags.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

DEFINE_string(config, "config/urdf_impedance.yaml", "URDF 路径和阻抗参数的 YAML 配置");
DEFINE_string(base, "base_link", "运动链基座名称");
DEFINE_string(tip, "link_7", "运动链末端名称");
DEFINE_bool(sim, true, "使用 HardwareSim 核对接口；不模拟真实力矩动力学");
DEFINE_int32(id, 0, "真机硬件实例编号");
DEFINE_bool(check_only, false, "仅检查配置和模型，不连接硬件");
DEFINE_double(duration, 0, "保持时间，单位秒；0 表示运行至 SIGINT/SIGTERM 中断");
DEFINE_double(debug_interval, 0.5, "调试打印间隔，秒；0 关闭，开启时至少 0.1 秒");

namespace {
constexpr double stationarySpeed = 0.02; // 启动时允许的实测速度，rad/s。
constexpr double maxControlGap = 0.020; // 真机控制周期超时阈值，s。
volatile std::sig_atomic_t stopped = 0;
// 信号处理只置标志，下使能由主线程执行。
void signalHandler(int) { stopped = 1; }
void require(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}

// 根据表头确定列宽：关节名左对齐，其余列右对齐。仅在非实时打印阶段使用。
class LogTable {
public:
    LogTable(std::ostream &output, std::initializer_list<const char *> columns) : output_(output) {
        int lineWidth = 0;
        for (const auto *column : columns) {
            widths_.push_back(std::max(12, static_cast<int>(std::char_traits<char>::length(column))));
            writeCell(widths_.size() - 1, column);
            lineWidth += widths_.back();
        }
        output_ << '\n' << std::string(lineWidth + 2 * (widths_.size() - 1), '-') << '\n';
    }

    template <typename... Values>
    void row(const Values &...values) {
        size_t column = 0;
        (writeCell(column++, values), ...);
        output_ << '\n';
    }

private:
    template <typename Value>
    void writeCell(size_t column, const Value &value) {
        if (column > 0) output_ << "  ";
        output_ << (column == 0 ? std::left : std::right) << std::setw(widths_[column]) << value;
    }

    std::ostream &output_;
    std::vector<int> widths_;
};

// 配置数组按 URDF 动力学模型的关节顺序排列，启动时打印该顺序。
struct ImpedanceConfig {
    std::string urdf;
    std::vector<std::string> jointNames;
    std::vector<int> hardwareIds;
    Eigen::VectorXd kp, kd, velocityLimit, torqueLimit, torqueRateLimit;
    Eigen::VectorXd lower, upper, velocity;
};

void require(bool ok, const std::string &message) {
    if (!ok) throw std::runtime_error(message);
}

Eigen::VectorXd readVector(const YAML::Node &node, int count, const std::string &name) {
    require(node.IsDefined() && node.IsSequence() && node.size() == size_t(count), "Wrong length: " + name);
    Eigen::VectorXd result(count);
    for (int i = 0; i < count; ++i) result[i] = node[i].as<double>();
    require(result.allFinite(), "Nonfinite values: " + name);
    return result;
}

const tinyxml2::XMLElement *findJoint(const tinyxml2::XMLElement *robot, const std::string &name) {
    for (auto *joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint")) {
        if (joint->Attribute("name", name.c_str())) return joint;
    }
    throw std::runtime_error("Joint missing from URDF: " + name);
}

double readAttribute(const tinyxml2::XMLElement *element, const char *name) {
    double value = 0;
    require(element && element->QueryDoubleAttribute(name, &value) == tinyxml2::XML_SUCCESS,
            "Missing URDF attribute: " + std::string(name));
    require(std::isfinite(value), "Nonfinite URDF attribute: " + std::string(name));
    return value;
}

class UrdfImpedanceController {
public:
    explicit UrdfImpedanceController(const std::string &path) {
        const auto yaml = YAML::LoadFile(path);
        const auto urdf = yaml["urdf"].as<std::string>();
        require(!urdf.empty(), "Empty URDF path");
        const auto slash = path.find_last_of('/');
        config_.urdf = urdf.front() == '/' ? urdf
            : (slash == std::string::npos ? "" : path.substr(0, slash + 1)) + urdf;

        // 一次性读取关节结构和连杆惯性，运行时直接使用 URDF 的质量、质心与惯量。
        pinocchio::urdf::buildModel(config_.urdf, model_);
        const int n = model_.nv;
        require(n == 7 && model_.nq == n && model_.njoints == n + 1,
                "Expected seven fixed-base single-DOF joints");
        for (int i = 1; i <= n; ++i) {
            require(model_.joints[i].nq() == 1 && model_.joints[i].nv() == 1, "Unsupported joint type");
            config_.jointNames.push_back(model_.names[i]);
        }
        data_.reset(new pinocchio::Data(model_));
        command_.resize(n);
        zeroVelocity_ = Eigen::VectorXd::Zero(n);
        loadGains(yaml);
        loadHardwareLimits();
    }

    const ImpedanceConfig &config() const { return config_; }
    const Eigen::VectorXd &rigidTorque() const { return data_->nle; }

    const Eigen::VectorXd &computeTorque(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                                        const Eigen::VectorXd &qDesired) {
        validateInput(q, dq, qDesired);
        // 仅将动力学输入速度置零：C(q,0)·0 + g(q) = g(q)，暂不补偿科氏/离心项。
        // 实测 dq 仍用于上面的安全检查和下面的阻尼项；零向量在初始化时分配。
        const auto &rigid = pinocchio::nonLinearEffects(model_, *data_, q, zeroVelocity_);
        command_ = rigid + config_.kp.cwiseProduct(qDesired - q) - config_.kd.cwiseProduct(dq);
        require(command_.allFinite(), "Nonfinite impedance torque");
        return command_;
    }

private:
    void loadGains(const YAML::Node &yaml) {
        const int n = config_.jointNames.size();
        config_.kp = readVector(yaml["kp"], n, "kp");
        config_.kd = readVector(yaml["kd"], n, "kd");
        config_.velocityLimit = readVector(yaml["velocity_limit"], n, "velocity_limit");
        config_.torqueLimit = readVector(yaml["torque_limit"], n, "torque_limit");
        config_.torqueRateLimit = readVector(yaml["torque_rate_limit"], n, "torque_rate_limit");
        require((config_.kp.array() >= 0).all() && (config_.kd.array() >= 0).all(), "kp/kd must be nonnegative");
        require((config_.velocityLimit.array() > 0).all(), "velocity_limit must be positive");
        require((config_.torqueLimit.array() > 0).all(), "Torque limits must be positive");
        require((config_.torqueRateLimit.array() > 0).all(), "Torque rate limits must be positive");
    }

    // 沿用原版的模型/硬件限位交集、硬件 ID 映射与驱动器单位检查。
    void loadHardwareLimits() {
        config_.lower = model_.lowerPositionLimit;
        config_.upper = model_.upperPositionLimit;
        config_.velocity = model_.velocityLimit;
        require(config_.lower.allFinite() && config_.upper.allFinite(), "Nonfinite position limits");
        require(config_.velocity.allFinite(), "Nonfinite velocity limits");
        tinyxml2::XMLDocument xml;
        require(xml.LoadFile(config_.urdf.c_str()) == tinyxml2::XML_SUCCESS, "Cannot read hardware URDF");
        const auto *robot = xml.FirstChildElement("robot");
        require(robot, "URDF has no robot element");
        std::set<int> ids;
        for (int i = 0; i < model_.nv; ++i) {
            const auto *joint = findJoint(robot, config_.jointNames[i]);
            require(joint->Attribute("type", "revolute"), "Only revolute joints are supported");
            const auto *hardware = joint->FirstChildElement("hardware");
            require(hardware && hardware->Attribute("type", "driver"), "Missing driver hardware");
            const auto *limit = hardware->FirstChildElement("limit");
            const auto *transform = hardware->FirstChildElement("transform");
            require(limit && transform, "Missing hardware limits/transform");
            require(transform->Attribute("user_unit_name", "rad"), "Joint position unit must be rad");
            int id = -1;
            require(hardware->QueryIntAttribute("id", &id) == tinyxml2::XML_SUCCESS, "Missing hardware ID");
            require(id >= 0 && id < 4096 && ids.insert(id).second, "Invalid/duplicate hardware ID");
            config_.hardwareIds.push_back(id);
            config_.lower[i] = std::max(config_.lower[i], readAttribute(limit, "lower"));
            config_.upper[i] = std::min(config_.upper[i], readAttribute(limit, "upper"));
            config_.velocity[i] = std::min(config_.velocity[i], readAttribute(limit, "vel"));
            require(config_.lower[i] < config_.upper[i], "Empty position limit interval");
            require(config_.velocity[i] > 0 && config_.velocityLimit[i] <= config_.velocity[i],
                    "velocity_limit exceeds URDF velocity limit");
            const double effort = std::min(model_.effortLimit[i], readAttribute(limit, "effort"));
            require(std::isfinite(effort) && config_.torqueLimit[i] <= effort, "Torque limit exceeds URDF effort");
            const double scale = readAttribute(transform, "torque_per_unit");
            require(scale != 0, "Torque scale must be nonzero");
            require(config_.torqueLimit[i] * std::abs(scale) <= 32767, "Torque limit exceeds int16 drive range");
            require(readAttribute(transform, "cnt_per_unit") != 0, "Position scale must be nonzero");
        }
    }

    void validateInput(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                       const Eigen::VectorXd &qDesired) const {
        const int n = config_.jointNames.size();
        require(q.size() == n && dq.size() == n && qDesired.size() == n, "Wrong state/target size");
        require(q.allFinite() && dq.allFinite() && qDesired.allFinite(), "Nonfinite state/target");
        for (int i = 0; i < n; ++i) {
            // 使用自定义实测速度阈值，任一周期超速就退出；不限制位置跟随误差。
            if (std::abs(dq[i]) > config_.velocityLimit[i]) {
                throw std::runtime_error("Safety velocity limit exceeded: " + config_.jointNames[i]
                    + ", dq=" + std::to_string(dq[i]) + " rad/s, limit="
                    + std::to_string(config_.velocityLimit[i]) + " rad/s; restart required");
            }
            require(q[i] >= config_.lower[i] && q[i] <= config_.upper[i], "Position exceeds joint limits");
            require(qDesired[i] >= config_.lower[i] && qDesired[i] <= config_.upper[i], "Target exceeds joint limits");
        }
    }

    ImpedanceConfig config_;
    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    Eigen::VectorXd command_, zeroVelocity_;
};

// 调试快照只保存一个周期的数据；控制线程使用的向量在上使能前分配。
struct DebugSample {
    double time = 0, cycleMs = 0;
    // LoadTorque 当前接口直接返回底层反馈，不擅自当作 Nm 或再次换算。
    Eigen::VectorXd q, dq, target, loadTorque, rigid, requested, sent;
    explicit DebugSample(int n)
        : q(n), dq(n), target(n), loadTorque(n), rigid(n), requested(n), sent(n) {}
};

// 终端可能阻塞：控制线程用 try_lock 投递快照，忙时丢弃；后台线程负责格式化和打印。
// 主程序先下使能，再停止打印线程，避免等待终端输出时机械臂仍在运行。
class DebugPrinter {
public:
    explicit DebugPrinter(const ImpedanceConfig &config)
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
            text << std::fixed << std::setprecision(6)
                 << "[阻抗调试] t=" << snapshot.time << " s, cycle=" << snapshot.cycleMs << " ms\n"
                 << "URDF理论力矩 = g(q)（动力学 dq=0）；阻尼和速度保护仍使用实测 dq。\n";
            LogTable table(text, {"joint", "q(rad)", "qd(rad)", "error(rad)", "dq(rad/s)",
                                 "urdf_theory(Nm)", "spring(Nm)", "damping(Nm)",
                                 "requested(Nm)", "sent(Nm)", "load(raw)"});
            for (int j = 0; j < snapshot.q.size(); ++j) {
                const double error = snapshot.target[j] - snapshot.q[j];
                table.row(config_.jointNames[j], snapshot.q[j], snapshot.target[j], error,
                          snapshot.dq[j], snapshot.rigid[j], config_.kp[j] * error,
                          -config_.kd[j] * snapshot.dq[j], snapshot.requested[j],
                          snapshot.sent[j], snapshot.loadTorque[j]);
            }
            std::cout << text.str() << std::flush;
        }
    }

    const ImpedanceConfig &config_;
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

// 动力学模型顺序不一定等于 Robot 顺序，用 URDF 硬件 ID 建立一一映射。
std::vector<int> mapRobotJoints(rocos::Robot &robot, const ImpedanceConfig &config) {
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
void limitTorque(const ImpedanceConfig &config, const Eigen::VectorXd &requested,
                 Eigen::VectorXd &previous) {
    for (int j = 0; j < requested.size(); ++j) {
        const double limit = config.torqueLimit[j]; // Nm。
        const double step = config.torqueRateLimit[j] * DELTA_T; // Nm/s × 控制周期。
        const double bounded = std::max(-limit, std::min(limit, requested[j]));
        previous[j] = std::max(previous[j] - step, std::min(previous[j] + step, bounded));
    }
}

// 启动时拒绝超限补偿，不直接截断；否则使能后可能无法托住当前姿态。
// 将诊断信息放入异常，主线程确认下使能后再打印；不受 debug_interval 影响。
void validateInitialTorque(const UrdfImpedanceController &controller, const Eigen::VectorXd &q,
                           const Eigen::VectorXd &dq, const Eigen::VectorXd &torque,
                           const std::vector<double> &ratedTorque) {
    const auto &config = controller.config();
    if ((torque.cwiseAbs().array() <= config.torqueLimit.array()).all()) return;

    std::ostringstream message;
    message << std::fixed << std::setprecision(6)
            << "Initial compensation exceeds torque limits\n"
            << "启动失败：补偿力矩绝对值超过 YAML 的 torque_limit（Nm），尚未切换 CST 或上使能。\n"
            << "YAML: " << FLAGS_config << "\nURDF: " << config.urdf << '\n'
            << "初始 qd=q，弹簧项为 0；preload = g(q) - kd*dq，动力学 dq=0，阻尼使用实测 dq，无摩擦项。\n";
    LogTable table(message, {"joint", "hardware_id", "q(rad)", "dq(rad/s)", "urdf_theory(Nm)",
                             "damping(Nm)", "preload(Nm)", "limit(Nm)", "status"});
    for (int j = 0; j < torque.size(); ++j)
        table.row(config.jointNames[j], config.hardwareIds[j], q[j], dq[j], controller.rigidTorque()[j],
                  -config.kd[j] * dq[j], torque[j], config.torqueLimit[j],
                  std::abs(torque[j]) > config.torqueLimit[j] ? "EXCEEDED" : "OK");
    for (int j = 0; j < torque.size(); ++j) {
        const double excess = std::abs(torque[j]) - config.torqueLimit[j];
        if (excess > 0) {
            message << "  [" << config.jointNames[j] << "] 超限 " << excess << " Nm，达到限值的 "
                    << 100.0 * std::abs(torque[j]) / config.torqueLimit[j] << "%"
                    << "；rated_torque=" << ratedTorque[j] << " Nm，预期驱动器指令="
                    << torque[j] / ratedTorque[j] * 1000.0 << " 千分比（未下发）。\n";
        }
    }
    throw std::runtime_error(message.str());
}

// 保留首周期和最近一次反馈，故障后不重新读数，避免把停机后的状态当作超速现场。
// 控制周期只复制预分配的数据；格式化和终端输出在下使能后执行。
struct ControlDiagnostics {
    using Clock = std::chrono::steady_clock;
    bool enableRequested = false, enableReturned = false;
    Clock::time_point enableRequest, enableReturn, firstRead, lastRead;
    size_t cycles = 0, lastSentCycle = 0; // 0 表示使能前的预置力矩。
    Eigen::VectorXd qStart, firstQ, firstDq, q, dq, lastSent, lastTheory;

    explicit ControlDiagnostics(int n)
        : qStart(n), firstQ(n), firstDq(n), q(n), dq(n), lastSent(n), lastTheory(n) {}

    void recordState(const Eigen::VectorXd &position, const Eigen::VectorXd &velocity) {
        lastRead = Clock::now();
        q = position;
        dq = velocity;
        if (++cycles == 1) {
            firstRead = lastRead;
            firstQ = q;
            firstDq = dq;
        }
    }

    void print(const ImpedanceConfig &config, const std::vector<double> &ratedTorque) const {
        if (!enableRequested) return;
        const auto ms = [](Clock::time_point end, Clock::time_point start) {
            return std::chrono::duration<double, std::milli>(end - start).count();
        };
        std::ostringstream text;
        text << std::fixed << std::setprecision(6) << "[使能诊断] ";
        if (enableReturned) text << "setEnabled 耗时=" << ms(enableReturn, enableRequest) << " ms\n";
        else text << "setEnabled 未正常返回\n";
        if (cycles == 0) {
            text << "尚未读取首周期反馈。\n";
            std::cerr << text.str() << std::flush;
            return;
        }
        text << "首周期读取距使能请求=" << ms(firstRead, enableRequest) << " ms，距使能返回="
             << ms(firstRead, enableReturn) << " ms\n";
        LogTable firstTable(text, {"joint", "q_start(rad)", "q_first(rad)", "delta_q_first(rad)", "dq_first(rad/s)"});
        for (int j = 0; j < q.size(); ++j)
            firstTable.row(config.jointNames[j], qStart[j], firstQ[j], firstQ[j] - qStart[j], firstDq[j]);

        const bool overspeed = (dq.cwiseAbs().array() > config.velocityLimit.array()).any();
        text << (overspeed ? "[超速现场] " : "[控制退出快照] ") << "cycle=" << cycles
             << "，反馈读取距使能请求=" << ms(lastRead, enableRequest)
             << " ms，距使能返回=" << ms(lastRead, enableReturn) << " ms\n"
             << "最后成功下发周期=" << lastSentCycle << "（0=使能前预置），last_cmd 为命令力矩，非实测力矩。\n"
             << "last_theory 为该下发周期的重力力矩 g(q)（动力学 dq=0），last_raw 单位为额定力矩千分比。\n";
        LogTable lastTable(text, {"joint", "hardware_id", "q(rad)", "delta_q_from_start(rad)", "dq(rad/s)",
                                 "velocity_limit(rad/s)", "last_theory(Nm)", "last_cmd(Nm)", "last_raw", "status"});
        for (int j = 0; j < q.size(); ++j) {
            // 与 Drive::setTorque 的乘法和 int16 截断一致，显示上次命令的实际换算值。
            const auto raw = static_cast<int16_t>(lastSent[j] * (1000.0 / ratedTorque[j]));
            lastTable.row(config.jointNames[j], config.hardwareIds[j], q[j], q[j] - qStart[j], dq[j],
                          config.velocityLimit[j], lastTheory[j], lastSent[j], raw,
                          std::abs(dq[j]) > config.velocityLimit[j] ? "EXCEEDED" : "OK");
        }
        std::cerr << text.str() << std::flush;
    }
};

void runController(rocos::Robot &robot, UrdfImpedanceController &controller, DebugPrinter &debug,
                   const std::vector<double> &ratedTorque, ControlDiagnostics &diagnostics) {
    const auto &config = controller.config();
    const auto indices = mapRobotJoints(robot, config);
    const int n = indices.size();

    // 先下使能，再读取起始姿态；模型首次计算也在上使能前完成。
    robot.setDisabled();
    configureTorqueUnits(robot, indices, ratedTorque);
    Eigen::VectorXd q(n), dq(n), qDesired(n), previous(n), command(n);
    DebugSample sample(n);
    readJointState(robot, indices, q, dq);
    require(q.allFinite() && dq.allFinite() && dq.cwiseAbs().maxCoeff() < stationarySpeed, "Robot must be stationary");
    qDesired = q; // 弹簧初始误差为零，保持当前位置。
    diagnostics.qStart = q;
    previous = controller.computeTorque(q, dq, qDesired);
    validateInitialTorque(controller, q, dq, previous, ratedTorque);

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
    std::ostringstream startup;
    startup << std::fixed << std::setprecision(6)
            << "Starting position hold; units: rad, rad/s, Nm, Nm/s\n"
            << "URDF理论力矩 = g(q)（动力学 dq=0），preload 另含实测速度的阻尼项。\n";
    LogTable startupTable(startup, {"joint", "hardware_id", "q_start", "kp", "kd", "velocity_limit",
                                    "torque_limit", "torque_rate_limit", "rated_torque", "urdf_theory(Nm)", "preload"});
    for (int j = 0; j < n; ++j) {
        startupTable.row(config.jointNames[j], config.hardwareIds[j], qDesired[j], config.kp[j], config.kd[j],
                         config.velocityLimit[j], config.torqueLimit[j], config.torqueRateLimit[j],
                         ratedTorque[j], controller.rigidTorque()[j], previous[j]);
    }
    startup << "超速故障将下使能并退出，需人工确认后重新启动。\n";
    std::cout << startup.str() << std::flush;

    // command 保持 Nm，Robot 按 1000/ratedTorque 换算后下发；预置和周期命令使用同一比例。
    // 下使能时先写入补偿力矩，上使能后即使用该值，而非从零力矩起步。
    require(!stopped && robot.setTorque(command) == 0, "Cannot preload CST torque");
    diagnostics.lastSent = previous;
    diagnostics.lastTheory = controller.rigidTorque();
    require(!stopped, "Interrupted before enabling");
    using Clock = std::chrono::steady_clock;
    diagnostics.enableRequest = Clock::now();
    diagnostics.enableRequested = true;
    robot.setEnabled();
    diagnostics.enableReturn = Clock::now();
    diagnostics.enableReturned = true;
    require(robot.isEnabled(), "Robot did not enable");

    const auto start = Clock::now();
    auto last = start;
    double nextDebug = 0;
    while (!stopped) {
        const auto now = Clock::now();
        const double elapsed = std::chrono::duration<double>(now - start).count();
        const double cycle = std::chrono::duration<double>(now - last).count();
        if (FLAGS_duration > 0 && elapsed >= FLAGS_duration) break;
        require(FLAGS_sim || cycle <= maxControlGap, "Control cycle exceeded 20 ms");
        last = now;
        require(robot.isEnabled(), "Robot was disabled during impedance control");
        readJointState(robot, indices, q, dq);
        diagnostics.recordState(q, dq); // 必须先保存，再由 computeTorque 检查速度并可能抛异常。
        // 外部轨迹接入点：更新 qDesired，保持位置连续且按 config.jointNames 排列。
        // 网络收发等可能阻塞的工作放在上层线程，不放进此循环。
        // computeTorque 首先检查自定义 velocity_limit，超速则抛异常，不再下发新的力矩。
        const auto &torque = controller.computeTorque(q, dq, qDesired);
        limitTorque(config, torque, previous);
        for (int j = 0; j < n; ++j) {
            command[indices[j]] = previous[j];
        }
        require(robot.setTorque(command) == 0, "Robot rejected CST torque"); // 内部等待一个总线周期。
        diagnostics.lastSent = previous; // 只记录成功下发值，不能被本周期未下发的计算值覆盖。
        diagnostics.lastTheory = controller.rigidTorque();
        diagnostics.lastSentCycle = diagnostics.cycles;
        if (FLAGS_debug_interval > 0 && elapsed >= nextDebug) {
            sample.time = elapsed;
            sample.cycleMs = cycle * 1000;
            sample.q = q;
            sample.dq = dq;
            sample.target = qDesired;
            sample.rigid = controller.rigidTorque();
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
        UrdfImpedanceController controller(FLAGS_config);
        const auto ratedTorque = loadRatedTorques(FLAGS_config, controller.config().torqueLimit);
        if (FLAGS_check_only) {
            std::cout << "URDF impedance configuration validated" << std::endl;
            return 0;
        }
        const auto &config = controller.config();
        DebugPrinter debug(config); // 打印线程及缓存先于 Robot 创建，初始化失败时尚未连接硬件。
        ControlDiagnostics diagnostics(config.jointNames.size());
        rocos::HardwareInterface *hw;
        if (FLAGS_sim)
            hw = new rocos::HardwareSim(std::max(20, *std::max_element(config.hardwareIds.begin(), config.hardwareIds.end()) + 1));
        else
            hw = new rocos::Hardware(config.urdf, FLAGS_id);
        rocos::Robot robot(hw, config.urdf, FLAGS_base, FLAGS_tip);
        int result = 0;
        try {
            runController(robot, controller, debug, ratedTorque, diagnostics);
            robot.setDisabled();
        } catch (const std::exception &error) {
            // 先停机再打印，避免日志输出阻塞而延迟下使能。
            robot.setDisabled();
            std::cerr << error.what() << std::endl;
            result = 1;
        }
        debug.stop(); // 此时已下使能，可以等待打印线程退出。
        diagnostics.print(config, ratedTorque); // 不受 debug_interval 影响，首周期超速也会保留现场。
        std::cout << "Impedance control stopped; robot disabled" << std::endl;
        // Robot 当前析构函数不会停止后台线程，显式下使能后保持对象至进程退出。
        std::exit(result);
    } catch (const std::exception &error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
