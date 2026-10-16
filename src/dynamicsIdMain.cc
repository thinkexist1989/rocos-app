// Execute a Studio excitation with Robot and save measured q, dq and LoadTorque.
// See docs/dynamics-id.md. Hardware headers precede Robot to avoid macro clashes.
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <rocos_app/robot.h>
#include <gflags/gflags.h>
#include <tinyxml2.h>

#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <set>
#include <thread>

DEFINE_string(urdf, "robot.urdf", "ROCOS hardware URDF");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");
DEFINE_bool(sim, true, "Use HardwareSim; its feedback is not identification data");
DEFINE_int32(id, 0, "Hardware instance ID");
DEFINE_string(trajectory, "trajectory.json", "Studio trajectory.json, or CSV path with trajectory.json beside it");
DEFINE_string(output, "", "New measured CSV; default measurements.csv or measurements.sim.csv");
DEFINE_int32(loops, 4, "Excitation repetitions");
DEFINE_double(time_scale, 1.0, "Trajectory speed factor (0, 1]; 0.1 advances 0.1 ms per 1 ms servo cycle");
DEFINE_bool(check_only, false, "Validate trajectory without connecting hardware");

namespace {
constexpr size_t jointCount = 7; // Current Robot initializes a seven-axis SRS solver.
constexpr double cycleTime = DELTA_T;
using Clock = std::chrono::steady_clock;
using Positions = std::array<double, jointCount>;
using File = std::unique_ptr<FILE, decltype(&std::fclose)>;
volatile std::sig_atomic_t interrupted = 0;

void signalHandler(int) { interrupted = 1; }
void require(bool ok, const std::string &message) {
    if (!ok) throw std::runtime_error(message);
}
void checkSignal() { require(!interrupted, "Interrupted by signal"); }
double seconds(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
File createOutput(const std::string &path) {
    File file(std::fopen(path.c_str(), "wx"), std::fclose);
    require(bool(file), "Cannot create output; use a new writable filename: " + path);
    return file;
}

struct Joint {
    std::string name;
    int hardwareId = -1, robotIndex = -1;
    double lower, upper, velocity, acceleration, jerk;
};
struct Trajectory {
    std::array<Joint, jointCount> joints; // Studio order, independent of Robot order.
    std::vector<Positions> points;        // Reference positions at Robot's 1 ms period.
};
struct Measurement {
    double time;
    int loop;
    Positions q, dq, tau;
};

// Studio's exact Fourier trajectory with sin^4 envelope and analytic derivatives.
std::array<double, 4> evaluate(const std::vector<double> &a, const std::vector<double> &b,
                              double center, double t, double duration) {
    const double w = std::acos(-1.0) / duration;
    const double s = std::sin(w * t), c = std::cos(w * t);
    const double e = std::pow(s, 4), de = 4 * w * s * s * s * c;
    const double dde = 4 * w * w * (3 * s * s * c * c - e);
    const double ddde = 4 * w * w * w * (6 * s * c * c * c - 10 * s * s * s * c);
    double f = 0, df = 0, ddf = 0, dddf = 0;
    for (size_t k = 0; k < a.size(); ++k) {
        const double frequency = 2 * w * (k + 1);
        const double p = a[k] * std::sin(frequency * t) + b[k] * std::cos(frequency * t);
        const double v = frequency * (a[k] * std::cos(frequency * t) - b[k] * std::sin(frequency * t));
        f += p; df += v; ddf -= frequency * frequency * p; dddf -= frequency * frequency * v;
    }
    return {{center + e * f, de * f + e * df, dde * f + 2 * de * df + e * ddf,
             ddde * f + 3 * dde * df + 3 * de * ddf + e * dddf}};
}

Trajectory loadTrajectory() {
    std::string path = FLAGS_trajectory;
    if (path.size() >= 4 && path.substr(path.size() - 4) == ".csv") {
        const auto slash = path.find_last_of('/');
        path = (slash == std::string::npos ? "" : path.substr(0, slash + 1)) + "trajectory.json";
    }
    std::cout << "Reading Studio trajectory parameters: " << path << std::endl;
    const auto data = YAML::LoadFile(path); // JSON can be read by yaml-cpp.
    const auto names = data["joint_names"].as<std::vector<std::string>>();
    const auto bounds = data["config"]["bounds"], coefficients = data["coefficients"];
    const double duration = data["config"]["duration"].as<double>();
    require(data["limits_verified"].as<bool>(false), "Studio trajectory limits are not verified");
    require(names.size() == jointCount && std::set<std::string>(names.begin(), names.end()).size() == jointCount &&
            bounds.IsSequence() && bounds.size() == jointCount && coefficients.IsSequence() && coefficients.size() == jointCount,
            "Expected seven unique joints with bounds and coefficients");
    require(std::isfinite(duration) && duration >= 2 && duration <= 600, "Invalid trajectory duration");

    tinyxml2::XMLDocument urdf;
    require(urdf.LoadFile(FLAGS_urdf.c_str()) == tinyxml2::XML_SUCCESS && urdf.FirstChildElement("robot"), "Cannot read hardware URDF");
    Trajectory trajectory;
    const double timeStep = cycleTime * FLAGS_time_scale;
    const double steps = std::ceil(duration / timeStep);
    require(std::isfinite(steps) && steps < trajectory.points.max_size(), "Time scale produces too many trajectory points");
    trajectory.points.resize(static_cast<size_t>(steps) + 1);
    std::cout << "Trajectory time step: " << timeStep << " s; nominal loop: "
              << duration / FLAGS_time_scale << " s" << std::endl;
    std::set<int> hardwareIds;
    for (size_t j = 0; j < jointCount; ++j) {
        auto &joint = trajectory.joints[j];
        joint.name = names[j];
        const tinyxml2::XMLElement *hardware = nullptr;
        for (auto *element = urdf.FirstChildElement("robot")->FirstChildElement("joint"); element; element = element->NextSiblingElement("joint"))
            if (element->Attribute("name", joint.name.c_str())) hardware = element->FirstChildElement("hardware");
        require(hardware && hardware->Attribute("type", "driver") && hardware->QueryIntAttribute("id", &joint.hardwareId) == tinyxml2::XML_SUCCESS &&
                joint.hardwareId >= 0 && joint.hardwareId < 4096 && hardwareIds.insert(joint.hardwareId).second, "Invalid hardware mapping: " + joint.name);
        const auto *limit = hardware->FirstChildElement("limit"), *transform = hardware->FirstChildElement("transform");
        require(limit && transform && transform->Attribute("user_unit_name", "rad"), "Missing limits or rad units: " + joint.name);
        auto readLimit = [&](const char *key) {
            double value = 0;
            require(limit->QueryDoubleAttribute(key, &value) == tinyxml2::XML_SUCCESS && std::isfinite(value), "Invalid hardware limit: " + joint.name);
            return value;
        };
        joint.lower = readLimit("lower"); joint.upper = readLimit("upper");
        joint.velocity = readLimit("vel"); joint.acceleration = readLimit("acc"); joint.jerk = readLimit("jerk");
        const double lower = bounds[j]["lower"].as<double>(), upper = bounds[j]["upper"].as<double>();
        const double velocity = bounds[j]["velocity"].as<double>(), acceleration = bounds[j]["acceleration"].as<double>();
        const double center = bounds[j]["center"].as<double>();
        require(joint.lower < joint.upper && lower < upper && std::isfinite(center) && velocity > 0 && acceleration > 0 &&
                joint.velocity > 0 && joint.acceleration > 0 && joint.jerk > 0, "Invalid limits: " + joint.name);
        require(coefficients[j].IsSequence() && coefficients[j].size() == 2, "Expected sine/cosine coefficients");
        const auto a = coefficients[j][0].as<std::vector<double>>(), b = coefficients[j][1].as<std::vector<double>>();
        require(!a.empty() && a.size() <= 10 && a.size() == b.size(), "Invalid Fourier coefficients");
        for (size_t k = 0; k < trajectory.points.size(); ++k) {
            auto state = evaluate(a, b, center, std::min(duration, k * timeStep), duration);
            // q(scale*t): velocity, acceleration and jerk scale by s, s^2, s^3.
            state[1] *= FLAGS_time_scale;
            state[2] *= FLAGS_time_scale * FLAGS_time_scale;
            state[3] *= FLAGS_time_scale * FLAGS_time_scale * FLAGS_time_scale;
            require(std::isfinite(state[0]) && state[0] >= joint.lower && state[0] <= joint.upper && state[0] >= lower - 1e-9 && state[0] <= upper + 1e-9 &&
                    std::abs(state[1]) <= std::min(joint.velocity, velocity) * (1 + 1e-8) &&
                    std::abs(state[2]) <= std::min(joint.acceleration, acceleration) * (1 + 1e-8) && std::abs(state[3]) <= joint.jerk * (1 + 1e-8),
                    "Trajectory exceeds limits: " + joint.name);
            trajectory.points[k][j] = state[0];
        }
    }
    return trajectory;
}

KDL::JntArray targetForRobot(const Trajectory &trajectory, const Positions &positions) {
    KDL::JntArray target(jointCount);
    for (size_t j = 0; j < jointCount; ++j) target(trajectory.joints[j].robotIndex) = positions[j];
    return target;
}

void settle(rocos::Robot &robot, const Trajectory &trajectory, const Positions &positions) {
    const auto target = targetForRobot(trajectory, positions);
    const auto start = Clock::now();
    do {
        checkSignal();
        require(robot.isEnabled() && robot.servoJ(target) == 0, "Robot rejected hold target");
    } while (seconds(start) < 0.5);
    for (const auto &joint : trajectory.joints)
        require(std::abs(robot.getJointPosition(joint.robotIndex) - target(joint.robotIndex)) <= 0.01 &&
                std::abs(robot.getJointVelocity(joint.robotIndex)) <= 0.02, "Robot did not settle: " + joint.name);
}

void moveToStart(rocos::Robot &robot, const Trajectory &trajectory) {
    checkSignal();
    for (const auto &joint : trajectory.joints)
        require(std::abs(robot.getJointVelocity(joint.robotIndex)) <= 0.02, "Robot must be stationary before MoveJ");
    require(robot.MoveJ(targetForRobot(trajectory, trajectory.points.front()), 0.15, 0.3, 0, 0, true) == 0, "MoveJ rejected start pose");
    const auto start = Clock::now();
    while (robot.getRunState() == rocos::Robot::RunState::Running) {
        checkSignal();
        require(robot.isEnabled() && seconds(start) < 120, "MoveJ failed or timed out");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    settle(robot, trajectory, trajectory.points.front());
}

void record(rocos::Robot &robot, const Trajectory &trajectory, std::vector<Measurement> &samples) {
    const auto start = Clock::now();
    auto previous = start;
    for (int loop = 1; loop <= FLAGS_loops; ++loop) {
        for (const auto &positions : trajectory.points) {
            checkSignal();
            require(robot.isEnabled() && robot.servoJ(targetForRobot(trajectory, positions)) == 0, "Robot rejected excitation target");
            const auto now = Clock::now();
            require(FLAGS_sim || std::chrono::duration<double>(now - previous).count() <= 0.02, "Servo cycle exceeded 20 ms");
            previous = now;
            Measurement sample;
            sample.time = std::chrono::duration<double>(now - start).count(); sample.loop = loop;
            for (size_t j = 0; j < jointCount; ++j) {
                const auto &joint = trajectory.joints[j];
                sample.q[j] = robot.getJointPosition(joint.robotIndex);
                sample.dq[j] = robot.getJointVelocity(joint.robotIndex);
                sample.tau[j] = robot.getJointLoadTorque(joint.robotIndex);
                require(std::isfinite(sample.q[j]) && std::isfinite(sample.dq[j]) && std::isfinite(sample.tau[j]) &&
                        sample.q[j] >= joint.lower && sample.q[j] <= joint.upper && std::abs(sample.q[j] - positions[j]) <= 0.15 &&
                        std::abs(sample.dq[j]) <= joint.velocity * 1.05, "Invalid feedback or tracking error: " + joint.name);
            }
            samples.push_back(sample);
        }
    }
    settle(robot, trajectory, trajectory.points.back());
}

void saveCsv(FILE *file, const std::vector<Measurement> &samples) {
    std::fprintf(file, "time,loop");
    for (size_t j = 1; j <= jointCount; ++j) std::fprintf(file, ",q_%zu,dq_%zu,tau_%zu", j, j, j);
    std::fprintf(file, "\n");
    for (const auto &sample : samples) {
        std::fprintf(file, "%.17g,%d", sample.time, sample.loop);
        for (size_t j = 0; j < jointCount; ++j) std::fprintf(file, ",%.17g,%.17g,%.17g", sample.q[j], sample.dq[j], sample.tau[j]);
        std::fprintf(file, "\n");
    }
    require(std::fflush(file) == 0 && !std::ferror(file), "CSV write failed");
}
int runExperiment(rocos::Robot &robot, Trajectory &trajectory, FILE *csv, FILE *metadata) {
    std::vector<Measurement> samples;
    samples.reserve(trajectory.points.size() * FLAGS_loops);
    YAML::Node result;
    result["simulation"] = FLAGS_sim;
    result["time_scale"] = FLAGS_time_scale;
    result["servo_period_seconds"] = cycleTime;
    result["torque_source"] = "Robot::getJointLoadTorque";
    result["torque_conversion"] = "none; save the Robot getter return value directly";
    result["completed"] = false;
    std::string failure;
    try {
        require(robot.getJointNum() == jointCount, "Robot joint count mismatch");
        for (auto &joint : trajectory.joints) {
            for (size_t r = 0; r < jointCount; ++r)
                if (robot.getJointHardwareId(r) == joint.hardwareId) joint.robotIndex = r;
            require(joint.robotIndex >= 0, "Joint is absent from Robot: " + joint.name);
            YAML::Node mapping;
            mapping["name"] = joint.name;
            mapping["robot_index"] = joint.robotIndex;
            result["joints"].push_back(mapping);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        checkSignal();
        robot.setEnabled();
        moveToStart(robot, trajectory);
        std::cout << "Recording " << FLAGS_loops << " excitation loop(s)"
                  << (FLAGS_sim ? " [SIMULATION ONLY]" : "") << std::endl;
        record(robot, trajectory, samples);
        result["completed"] = true;
    } catch (const std::exception &e) {
        failure = e.what();
    }
    robot.setDisabled();
    saveCsv(csv, samples);
    result["samples"] = samples.size();
    result["failure"] = failure;
    YAML::Emitter emitter;
    emitter << result;
    require(emitter.good(), "Cannot serialize recording status");
    std::fprintf(metadata, "%s\n", emitter.c_str());
    require(std::fflush(metadata) == 0 && !std::ferror(metadata), "Status write failed");
    std::cout << "Saved " << samples.size() << " samples to " << FLAGS_output << std::endl;
    if (!failure.empty()) {
        std::cerr << failure << std::endl;
        return interrupted ? 130 : 1;
    }
    return 0;
}
} // namespace

int main(int argc, char **argv) {
    using namespace rocos;
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    try {
        require(FLAGS_loops > 0, "Loops must be positive");
        require(std::isfinite(FLAGS_time_scale) && FLAGS_time_scale > 0 && FLAGS_time_scale <= 1,
                "time_scale must be finite and in (0, 1]");
        auto trajectory = loadTrajectory();
        if (FLAGS_check_only) return 0;

        if (FLAGS_output.empty()) FLAGS_output = FLAGS_sim ? "measurements.sim.csv" : "measurements.csv";
        auto csv = createOutput(FLAGS_output);
        auto metadata = createOutput(FLAGS_output + ".metadata.yaml");

        HardwareInterface *hw;
        if (FLAGS_sim) {
            int slaves = 20;
            for (const auto &joint : trajectory.joints) slaves = std::max(slaves, joint.hardwareId + 1);
            hw = new HardwareSim(slaves);
        } else {
            hw = new Hardware(FLAGS_urdf, FLAGS_id);
        }
        Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);

        int exitCode = 1;
        try {
            exitCode = runExperiment(robot, trajectory, csv.get(), metadata.get());
        } catch (const std::exception &e) {
            robot.setDisabled();
            std::cerr << e.what() << std::endl;
        }
        csv.reset();
        metadata.reset();
        // Robot's current destructor does not stop its workers. Exit after
        // disabling and closing files, keeping Robot alive until process exit.
        std::exit(exitCode);
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
