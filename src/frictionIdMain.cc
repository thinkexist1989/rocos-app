// 单关节摩擦采集：读取 Studio 协议 → MoveJ 到起点 → servoJ 往返 → 下使能后保存。
// 位置轨迹按函数计算；q/dq/tau 均从 Robot 读取，参考值与实测值分开保存。
// hardware 头文件需放在 robot.h 前，避免现有 RESET 宏与 EtherCAT 枚举冲突。
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <rocos_app/robot.h>
#include <gflags/gflags.h>
#include <tinyxml2.h>
#include <openssl/sha.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

DEFINE_string(urdf, "robot.urdf", "生成 Studio 协议时使用的同一份 ROCOS URDF");
DEFINE_string(base, "base_link", "运动链基座名称");
DEFINE_string(tip, "link_7", "运动链末端名称");
DEFINE_bool(sim, true, "使用 HardwareSim 核对接口；不模拟真实速度和摩擦");
DEFINE_int32(id, 0, "真机硬件实例编号");
DEFINE_string(trajectory, "friction_protocol.json", "Studio 协议路径，也可传同目录 friction_trajectory.csv");
DEFINE_string(output, "", "采集 CSV 路径，禁止覆盖已有文件");
DEFINE_double(time_scale, 1, "轨迹速度比例 (0,1]，同时缩放 target_velocity");
DEFINE_bool(check_only, false, "仅检查协议和 URDF，不连接硬件");

namespace {
constexpr size_t jointCount = 7; // Robot 当前使用七轴 SRS 求解器。
constexpr double settlingSeconds = 0.5;
constexpr double stationarySpeed = 0.02; // 起停时允许的实测速度，rad/s。
constexpr double maxTrackingError = 0.15; // 采集期间允许的跟踪误差，rad。
constexpr double maxServoGap = 0.02; // 真机周期超时阈值，s。
using Positions = std::array<double, jointCount>;
using Clock = std::chrono::steady_clock;
using File = std::unique_ptr<FILE, decltype(&std::fclose)>;
volatile std::sig_atomic_t interrupted = 0;

// 信号处理只置标志；下使能和文件保存由主流程完成。
void signalHandler(int) { interrupted = 1; }
void require(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}
void checkSignal() { require(!interrupted, "Interrupted by signal"); }
double elapsed(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
double readNumber(const YAML::Node &node) {
    const double value = node.as<double>();
    require(std::isfinite(value), "Protocol contains a nonfinite value");
    return value;
}

// 数组始终按 Studio 的 joint_names 排列，通过硬件 ID 映射到 Robot 索引。
struct Joint {
    std::string name;
    int hardwareId = -1;
    int robotIndex = -1;
    double lower, upper; // 位置限位，rad。
    double velocity, acceleration, jerk; // 对应 rad/s、rad/s²、rad/s³。
};
// 一次单向扫掠：加速、匀速、减速、端点停留；正反两次扫掠构成一对。
struct Sweep {
    double start; // 当前扫掠在协议中的开始时间，s。
    double speed; // 正的速度幅值，rad/s，方向单独记录。
    double ramp, steady, motion, dwell; // 加减速、匀速、单向运动及停留时长，s。
    int repeat, pair, id, direction; // repeat 为完整轮次，direction 为 +1 或 -1。
    double end() const { return start + motion + dwell; }
};
struct Protocol {
    std::string path;
    std::array<Joint, jointCount> joints;
    Positions initial;
    size_t activeJoint; // 唯一运动的关节，其余关节保持 initial。
    double lower, upper;
    std::vector<Sweep> sweeps;
    double duration() const { return sweeps.back().end(); }
};
struct Reference {
    double position;
    double targetVelocity; // 本次扫掠的目标匀速值；ramp 阶段也保留该值。
    const char *phase;
};
struct Measurement {
    double time; // 实际采集经过时间，s。
    double referenceTime; // 协议函数时间，随 time_scale 推进。
    double targetVelocity;
    int repeat, pair, sweep;
    const char *phase;
    Positions q, dq, tau;
};

size_t sampleCount(const Protocol &protocol) {
    const double steps = std::ceil(protocol.duration() / (DELTA_T * FLAGS_time_scale));
    require(std::isfinite(steps) && steps < std::vector<Measurement>().max_size() - 1, "Recording is too large");
    return static_cast<size_t>(steps) + 1; // 包含起点和最后一个精确终点。
}

// 核对协议是否对应当前 URDF，避免误用其他模型生成的轨迹。
std::string fileHash(const std::string &path) {
    std::ifstream file(path, std::ios::binary);
    require(bool(file), "Cannot read URDF");
    const std::string bytes{std::istreambuf_iterator<char>(file), {}};
    unsigned char hash[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char *>(bytes.data()), bytes.size(), hash);
    std::ostringstream text;
    for (auto byte : hash) text << std::hex << std::setw(2) << std::setfill('0') << int(byte);
    return text.str();
}

// 读取控制器使用的硬件 ID、单位和运动限制；此时还未连接真机。
void loadHardwareJoints(Protocol &protocol) {
    tinyxml2::XMLDocument xml;
    require(xml.LoadFile(FLAGS_urdf.c_str()) == tinyxml2::XML_SUCCESS, "Cannot load hardware URDF");
    const auto *robot = xml.FirstChildElement("robot");
    require(robot, "URDF has no robot element");
    std::set<int> ids;
    for (size_t j = 0; j < jointCount; ++j) {
        auto &joint = protocol.joints[j];
        const tinyxml2::XMLElement *hardware = nullptr;
        for (auto *element = robot->FirstChildElement("joint"); element; element = element->NextSiblingElement("joint")) {
            if (element->Attribute("name", joint.name.c_str())) hardware = element->FirstChildElement("hardware");
        }
        require(hardware && hardware->Attribute("type", "driver"), "Missing joint driver hardware");
        require(hardware->QueryIntAttribute("id", &joint.hardwareId) == tinyxml2::XML_SUCCESS, "Missing hardware ID");
        require(joint.hardwareId >= 0 && joint.hardwareId < 4096, "Invalid hardware ID");
        require(ids.insert(joint.hardwareId).second, "Duplicate hardware ID");
        const auto *limit = hardware->FirstChildElement("limit");
        const auto *transform = hardware->FirstChildElement("transform");
        require(limit && transform && transform->Attribute("user_unit_name", "rad"), "Missing hardware limits or rad units");
        double countsPerRad = 0;
        require(transform->QueryDoubleAttribute("cnt_per_unit", &countsPerRad) == tinyxml2::XML_SUCCESS,
                "Missing position conversion");
        require(std::isfinite(countsPerRad) && countsPerRad != 0, "Invalid position conversion");
        auto readLimit = [&](const char *key) {
            double value = 0;
            require(limit->QueryDoubleAttribute(key, &value) == tinyxml2::XML_SUCCESS && std::isfinite(value), "Invalid hardware limit");
            return value;
        };
        joint.lower = readLimit("lower");
        joint.upper = readLimit("upper");
        joint.velocity = readLimit("vel");
        joint.acceleration = readLimit("acc");
        joint.jerk = readLimit("jerk");
        require(joint.lower < joint.upper && joint.velocity > 0 && joint.acceleration > 0 && joint.jerk > 0, "Invalid joint limits");
        require(protocol.initial[j] >= joint.lower && protocol.initial[j] <= joint.upper, "Initial pose exceeds hardware limits");
    }
}

// 编排实验：每轮遍历全部速度，每档速度依次正转、反转，共用一个 pair。
void buildSweeps(Protocol &protocol, const YAML::Node &config) {
    const auto &joint = protocol.joints[protocol.activeJoint];
    const double acceleration = readNumber(config["acceleration"]);
    const double rate = readNumber(config["sample_rate"]);
    const double dwell = readNumber(config["dwell_seconds"]);
    const double minimumSteady = readNumber(config["minimum_steady_seconds"]);
    const int repeats = config["repeats"].as<int>();
    const auto speeds = config["speeds"].as<std::vector<double>>();
    require(acceleration > 0 && rate >= 10 && rate <= 2000 && dwell >= 0 && minimumSteady > 0, "Invalid timing configuration");
    require(repeats >= 1 && repeats <= 20 && speeds.size() >= 2 && speeds.size() <= 30, "Invalid repeats/speed count");
    for (double speed : speeds) {
        require(std::isfinite(speed) && speed > 0, "Speed must be finite and positive");
    }
    require(std::set<double>(speeds.begin(), speeds.end()).size() == speeds.size(), "Duplicate speed settings");
    const double scale = FLAGS_time_scale;
    const double endpointDwell = std::max(dwell, 1 / rate); // 即使停留为零，也保留一个参考采样周期的终点。
    double start = 0;
    int pair = 0;
    for (int repeat = 0; repeat < repeats; ++repeat) {
        for (double speed : speeds) {
            // 五次速度过渡的最大斜率为 1.875；匀速区两端各预留 0.15 s。
            const double ramp = 1.875 * speed / acceleration;
            const double steady = (protocol.upper - protocol.lower) / speed - ramp;
            require(std::isfinite(ramp) && std::isfinite(steady) && steady >= minimumSteady + 0.3, "Insufficient steady travel");
            require(speed * scale <= joint.velocity, "Sweep exceeds velocity limit");
            require(acceleration * scale * scale <= joint.acceleration, "Sweep exceeds acceleration limit");
            // jerk 峰值为 (10/√3)·v/ramp²，时间缩放后乘 scale³。
            const double peakJerk = 10 / std::sqrt(3.) * speed / ramp / ramp;
            require(peakJerk * scale * scale * scale <= joint.jerk, "Sweep exceeds jerk limit");
            for (int direction : {1, -1}) {
                Sweep sweep{start, speed, ramp, steady, 2 * ramp + steady, endpointDwell,
                            repeat, pair, int(protocol.sweeps.size()), direction};
                protocol.sweeps.push_back(sweep);
                start = sweep.end();
                require(std::isfinite(start), "Nonfinite experiment duration");
            }
            ++pair;
        }
    }
}

Protocol loadProtocol() {
    Protocol protocol;
    protocol.path = FLAGS_trajectory;
    // CSV 仅用于定位同目录协议；实际执行解析函数，避免参考采样率与控制周期混淆。
    if (protocol.path.size() >= 4 && protocol.path.substr(protocol.path.size() - 4) == ".csv") {
        const auto slash = protocol.path.find_last_of('/');
        protocol.path = (slash == std::string::npos ? "" : protocol.path.substr(0, slash + 1)) + "friction_protocol.json";
    }
    const auto data = YAML::LoadFile(protocol.path);
    const auto config = data["config"];
    const auto names = data["joint_names"].as<std::vector<std::string>>();
    require(names.size() == jointCount, "Expected seven protocol joints");
    require(std::set<std::string>(names.begin(), names.end()).size() == jointCount, "Duplicate protocol joint names");
    require(data["urdf_hash"].as<std::string>() == fileHash(FLAGS_urdf), "Protocol/URDF hash mismatch");
    require(data["initial_pose"].size() == jointCount, "Invalid initial pose size");
    for (size_t j = 0; j < jointCount; ++j) {
        protocol.joints[j].name = names[j];
        protocol.initial[j] = readNumber(data["initial_pose"][j]);
    }
    const auto active = std::find(names.begin(), names.end(), data["joint"].as<std::string>());
    require(active != names.end(), "Selected joint is missing");
    protocol.activeJoint = active - names.begin();
    require(data["joint_index"].as<size_t>() == protocol.activeJoint, "Selected joint index mismatch");
    require(config["joint"].as<std::string>() == *active, "Config joint mismatch");
    protocol.lower = readNumber(config["lower"]);
    protocol.upper = readNumber(config["upper"]);
    loadHardwareJoints(protocol);
    const auto &joint = protocol.joints[protocol.activeJoint];
    require(protocol.lower < protocol.upper && protocol.lower >= joint.lower && protocol.upper <= joint.upper, "Sweep exceeds position limits");
    require(std::abs(protocol.initial[protocol.activeJoint] - protocol.lower) < 1e-10, "Initial pose must start at sweep lower limit");

    buildSweeps(protocol, config);
    std::cout << "Protocol: " << *active << ", " << protocol.sweeps.size() / 2 << " pairs, "
              << protocol.duration() / FLAGS_time_scale << " s" << std::endl;
    return protocol;
}

// 对速度过渡 s(u)=10u³−15u⁴+6u⁵ 积分，u∈[0,1]。
// 加速位移为 v·ramp·F(u)，减速位移为 v·ramp·[u−F(u)]。
double rampIntegral(double u) {
    return u * u * u * u * (2.5 - 3 * u + u * u);
}

// 在当前单向扫掠内计算位置与阶段；该函数不读取硬件。
Reference evaluateReference(const Protocol &protocol, const Sweep &sweep, double time) {
    const double t = std::max(0., time - sweep.start);
    const double origin = sweep.direction == 1 ? protocol.lower : protocol.upper;
    if (t >= sweep.motion) {
        return {sweep.direction == 1 ? protocol.upper : protocol.lower, 0, "dwell"};
    }
    double distance;
    const char *phase = "ramp";
    if (t < sweep.ramp) {
        // 加速：从静止平滑过渡到目标速度。
        distance = sweep.speed * sweep.ramp * rampIntegral(t / sweep.ramp);
    } else if (t < sweep.ramp + sweep.steady) {
        // 匀速：加速阶段已经走过 v·ramp/2。
        distance = sweep.speed * (t - sweep.ramp / 2);
        phase = "steady";
    } else {
        // 减速：从目标速度平滑回到静止。
        const double u = (t - sweep.ramp - sweep.steady) / sweep.ramp;
        distance = sweep.speed * (sweep.ramp / 2 + sweep.steady + sweep.ramp * (u - rampIntegral(u)));
    }
    return {origin + sweep.direction * distance, sweep.direction * sweep.speed * FLAGS_time_scale, phase};
}

// 把协议顺序转换成 Robot 顺序，其余关节始终保持初始姿态。
KDL::JntArray makeRobotTarget(const Protocol &protocol, double activePosition) {
    KDL::JntArray target(jointCount);
    for (size_t j = 0; j < jointCount; ++j) target(protocol.joints[j].robotIndex) = protocol.initial[j];
    target(protocol.joints[protocol.activeJoint].robotIndex) = activePosition;
    return target;
}

// 实验首尾都位于下限；保持位置，等待反馈稳定，不记录这段额外等待。
void settle(rocos::Robot &robot, const Protocol &protocol) {
    const auto target = makeRobotTarget(protocol, protocol.lower);
    const auto start = Clock::now();
    do {
        checkSignal();
        require(robot.isEnabled() && robot.servoJ(target) == 0, "Robot rejected hold target");
    } while (elapsed(start) < settlingSeconds);
    for (const auto &joint : protocol.joints) {
        const int r = joint.robotIndex;
        require(std::abs(robot.getJointPosition(r) - target(r)) <= 0.01, "Robot did not reach initial pose");
        require(std::abs(robot.getJointVelocity(r)) <= stationarySpeed, "Robot did not settle");
    }
}

void moveToStart(rocos::Robot &robot, const Protocol &protocol) {
    for (const auto &joint : protocol.joints) {
        require(std::abs(robot.getJointVelocity(joint.robotIndex)) <= stationarySpeed, "Robot must be stationary before MoveJ");
    }
    checkSignal();
    const auto target = makeRobotTarget(protocol, protocol.lower);
    // 异步接近起点：速度 0.15 rad/s、加速度 0.3 rad/s²，等待期间仍响应中断。
    require(robot.MoveJ(target, 0.15, 0.3, 0, 0, true) == 0, "MoveJ rejected initial pose");
    const auto start = Clock::now();
    while (robot.getRunState() == rocos::Robot::RunState::Running) {
        checkSignal();
        require(robot.isEnabled() && elapsed(start) < 120, "MoveJ failed or timed out");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    settle(robot, protocol);
}

// 按协议顺序保存反馈，同时检查位置、速度和跟踪误差。
void readJointFeedback(rocos::Robot &robot, const Protocol &protocol,
                       const KDL::JntArray &target, Measurement &sample) {
    for (size_t j = 0; j < jointCount; ++j) {
        const auto &joint = protocol.joints[j];
        const int r = joint.robotIndex;
        sample.q[j] = robot.getJointPosition(r);
        // 采集保留实测速度，不在这里滤波，也不用参考速度替代反馈。
        sample.dq[j] = robot.getJointVelocity(r);
        // 原样保存 getter 返回值；输出侧 Nm 标定需由硬件约定确认。
        sample.tau[j] = robot.getJointLoadTorque(r);
        require(std::isfinite(sample.q[j]) && std::isfinite(sample.dq[j]) && std::isfinite(sample.tau[j]), "Nonfinite feedback");
        require(sample.q[j] >= joint.lower && sample.q[j] <= joint.upper, "Feedback exceeds position limits");
        require(std::abs(sample.q[j] - target(r)) <= maxTrackingError, "Position tracking error exceeds 0.15 rad");
        require(std::abs(sample.dq[j]) <= joint.velocity * 1.05, "Feedback exceeds velocity limit");
    }
}

// 每周期只更新目标、调用 servoJ、读取反馈并存入预分配缓冲。
void recordMeasurements(rocos::Robot &robot, const Protocol &protocol, std::vector<Measurement> &samples) {
    const auto start = Clock::now();
    auto previous = start;
    size_t sweepIndex = 0;
    const size_t steps = sampleCount(protocol) - 1;
    const double referenceStep = DELTA_T * FLAGS_time_scale;
    auto target = makeRobotTarget(protocol, protocol.lower);
    for (size_t step = 0; step <= steps; ++step) {
        checkSignal();
        const double t = std::min(protocol.duration(), step * referenceStep);
        while (sweepIndex + 1 < protocol.sweeps.size() && t >= protocol.sweeps[sweepIndex].end()) ++sweepIndex;
        const auto &sweep = protocol.sweeps[sweepIndex];
        const auto reference = evaluateReference(protocol, sweep, t);
        target(protocol.joints[protocol.activeJoint].robotIndex) = reference.position;
        // servoJ 已等待一个总线周期，无需额外 sleep。
        require(robot.isEnabled() && robot.servoJ(target) == 0, "Robot rejected friction trajectory target");
        const auto now = Clock::now();
        require(FLAGS_sim || std::chrono::duration<double>(now - previous).count() <= maxServoGap, "Servo cycle exceeded 20 ms");
        previous = now;
        Measurement sample;
        sample.time = std::chrono::duration<double>(now - start).count();
        sample.referenceTime = t;
        sample.repeat = sweep.repeat;
        sample.pair = sweep.pair;
        sample.sweep = sweep.id;
        sample.phase = reference.phase;
        sample.targetVelocity = reference.targetVelocity;
        readJointFeedback(robot, protocol, target, sample);
        samples.push_back(sample);
    }
}

File createOutput(const std::string &path) {
    // 独占创建，防止覆盖已有实验数据。
    File file(std::fopen(path.c_str(), "wx"), std::fclose);
    require(bool(file), "Cannot create output; choose a new writable filename");
    return file;
}

// 数据保持原始反馈值，列序号对应协议中的 joint_names。
void writeCsv(FILE *csv, const std::vector<Measurement> &samples) {
    std::fprintf(csv, "time,reference_time,repeat,pair,sweep,phase,target_velocity");
    for (size_t j = 1; j <= jointCount; ++j) std::fprintf(csv, ",q_%zu,dq_%zu,tau_%zu", j, j, j);
    std::fprintf(csv, "\n");
    for (const auto &sample : samples) {
        std::fprintf(csv, "%.17g,%.17g,%d,%d,%d,%s,%.17g", sample.time, sample.referenceTime,
                     sample.repeat, sample.pair, sample.sweep, sample.phase, sample.targetVelocity);
        for (size_t j = 0; j < jointCount; ++j) std::fprintf(csv, ",%.17g,%.17g,%.17g", sample.q[j], sample.dq[j], sample.tau[j]);
        std::fprintf(csv, "\n");
    }
    require(std::fflush(csv) == 0 && !std::ferror(csv), "CSV write failed");
}

// 记录完成状态、关节映射和反馈来源，避免仿真或中断文件被误用。
void writeMetadata(FILE *metadata, const Protocol &protocol,
                   size_t recordedSamples, const std::string &failure) {
    YAML::Node status;
    status["protocol"] = protocol.path;
    status["joint"] = protocol.joints[protocol.activeJoint].name;
    status["joint_index"] = protocol.activeJoint;
    status["simulation"] = FLAGS_sim;
    status["completed"] = failure.empty();
    status["failure"] = failure;
    status["samples"] = recordedSamples;
    status["time_scale"] = FLAGS_time_scale;
    status["servo_period_seconds"] = DELTA_T;
    status["torque_source"] = "Robot::getJointLoadTorque";
    status["torque_conversion"] = "none; confirm getter values are output-joint Nm before identification";
    status["temperature_recorded"] = false;
    for (const auto &joint : protocol.joints) {
        status["joint_names"].push_back(joint.name);
        status["robot_indices"].push_back(joint.robotIndex);
    }
    YAML::Emitter emitter;
    emitter << status;
    require(emitter.good(), "Cannot serialize recording status");
    std::fprintf(metadata, "%s\n", emitter.c_str());
    require(std::fflush(metadata) == 0 && !std::ferror(metadata), "Metadata write failed");
}

int runExperiment(rocos::Robot &robot, Protocol &protocol, FILE *csv, FILE *metadata) {
    std::vector<Measurement> samples;
    std::string failure;
    try {
        samples.reserve(sampleCount(protocol)); // 上使能前分配，周期内不扩容。
        require(robot.getJointNum() == jointCount, "Robot joint count mismatch");
        // 协议列顺序、URDF 文件顺序、Robot 索引和硬件编号可能不同。
        for (auto &joint : protocol.joints) {
            for (size_t r = 0; r < jointCount; ++r) {
                if (robot.getJointHardwareId(r) == joint.hardwareId) joint.robotIndex = r;
            }
            require(joint.robotIndex >= 0, "Protocol joint is absent from Robot");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        checkSignal();
        robot.setEnabled();
        moveToStart(robot, protocol);
        std::cout << "Recording friction sweeps" << (FLAGS_sim ? " [SIMULATION ONLY]" : "") << std::endl;
        recordMeasurements(robot, protocol, samples);
        settle(robot, protocol);
    } catch (const std::exception &error) {
        failure = error.what();
    }
    // 正常结束或捕获异常都先下使能，然后把已采集的完整行写盘。
    robot.setDisabled();
    writeCsv(csv, samples);
    writeMetadata(metadata, protocol, samples.size(), failure);
    std::cout << "Saved " << samples.size() << " samples to " << FLAGS_output << std::endl;
    if (failure.empty()) return 0;
    std::cerr << failure << std::endl;
    return interrupted ? 130 : 1;
}
} // namespace

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    try {
        require(std::isfinite(FLAGS_time_scale) && FLAGS_time_scale > 0 && FLAGS_time_scale <= 1, "time_scale must be in (0,1]");
        auto protocol = loadProtocol();
        sampleCount(protocol); // 预检也检查采样数，硬件连接前拒绝溢出。
        if (FLAGS_check_only) return 0;
        if (FLAGS_output.empty()) FLAGS_output = FLAGS_sim ? "friction.measurements.sim.csv" : "friction.measurements.csv";
        auto csv = createOutput(FLAGS_output);
        auto metadata = createOutput(FLAGS_output + ".metadata.yaml");
        rocos::HardwareInterface *hardware;
        if (FLAGS_sim) {
            int slaves = 20;
            for (const auto &joint : protocol.joints) slaves = std::max(slaves, joint.hardwareId + 1);
            hardware = new rocos::HardwareSim(slaves);
        } else {
            hardware = new rocos::Hardware(FLAGS_urdf, FLAGS_id);
        }
        rocos::Robot robot(hardware, FLAGS_urdf, FLAGS_base, FLAGS_tip);
        int exitCode = 1;
        try {
            exitCode = runExperiment(robot, protocol, csv.get(), metadata.get());
        } catch (const std::exception &error) {
            robot.setDisabled();
            std::cerr << error.what() << std::endl;
        }
        csv.reset();
        metadata.reset();
        // Robot 当前析构函数不会停止后台线程，保持对象存活至进程退出。
        // 此前已显式下使能，并通过 reset() 关闭输出文件。
        std::exit(exitCode);
    } catch (const std::exception &error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
