#include <rocos_app/joint_impedance.h>
#include <pinocchio/algorithm/regressor.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <yaml-cpp/yaml.h>
#include <tinyxml2.h>
#include <openssl/sha.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

namespace {
void require(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}
void require(bool ok, const std::string &message) {
    if (!ok) throw std::runtime_error(message);
}

std::string relativeTo(const std::string &file, const std::string &path) {
    if (!path.empty() && path.front() == '/') return path;
    const auto slash = file.find_last_of('/');
    return (slash == std::string::npos ? "" : file.substr(0, slash + 1)) + path;
}

std::string sha256(const std::string &path) {
    std::ifstream file(path, std::ios::binary);
    require(bool(file), "Cannot read URDF: " + path);
    const std::string bytes{std::istreambuf_iterator<char>(file), {}};
    unsigned char hash[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char *>(bytes.data()), bytes.size(), hash);
    std::ostringstream text;
    for (auto byte : hash) text << std::hex << std::setw(2) << std::setfill('0') << int(byte);
    return text.str();
}

Eigen::VectorXd readVector(const YAML::Node &node, int count, const std::string &name) {
    require(node.IsSequence() && node.size() == size_t(count), "Wrong length: " + name);
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

struct FrictionParameters {
    double coulomb;
    double loadCoefficient;
    double stribeckAmplitude;
    double viscousAtReference;
    double temperatureSlope;
    double referenceTemperature;
    double stribeckVelocity;
    double smoothVelocity;

    explicit FrictionParameters(const YAML::Node &node) {
        auto read = [&](const char *key) {
            const double value = node[key].as<double>();
            require(std::isfinite(value), "Nonfinite friction coefficient");
            return value;
        };
        coulomb = read("coulomb");
        loadCoefficient = read("load_coefficient");
        stribeckAmplitude = read("stribeck_amplitude");
        viscousAtReference = read("viscous_at_reference");
        temperatureSlope = read("viscous_temperature_slope");
        referenceTemperature = read("reference_temperature_c");
        stribeckVelocity = read("stribeck_velocity");
        smoothVelocity = read("smooth_velocity");

        require(coulomb >= 0 && loadCoefficient >= 0, "Negative Coulomb/load friction");
        require(stribeckAmplitude >= 0 && viscousAtReference >= 0, "Negative Stribeck/viscous friction");
        require(stribeckVelocity > 0 && smoothVelocity > 0, "Friction velocities must be positive");
    }

    double compute(double velocity, double rigidTorque, double temperature) const {
        const double loadFriction = loadCoefficient * std::abs(rigidTorque);
        const double stribeck = stribeckAmplitude * std::exp(-std::pow(velocity / stribeckVelocity, 2));
        const double direction = std::tanh(velocity / smoothVelocity);
        const double viscous = viscousAtReference + temperatureSlope * (temperature - referenceTemperature);
        require(std::isfinite(viscous) && viscous >= 0, "Temperature produces invalid viscous friction");
        return (coulomb + loadFriction + stribeck) * direction + viscous * velocity;
    }
};
} // namespace

namespace rocos {
struct JointImpedanceController::Impl {
    JointImpedanceConfig config;
    pinocchio::Model model;
    std::unique_ptr<pinocchio::Data> data;
    Eigen::VectorXd parameters; // P * beta, exported once from the Studio result.
    Eigen::VectorXd zeroAcceleration, rigidTorque, frictionTorque, command;
    std::vector<FrictionParameters> friction;

    explicit Impl(const std::string &path) {
        const auto yaml = YAML::LoadFile(path);
        auto modelPath = [&](const char *key) {
            return relativeTo(path, yaml[key].as<std::string>());
        };
        config.urdf = modelPath("urdf");
        loadRigidModel(modelPath("rigid_model"));
        loadGains(yaml);
        loadHardwareLimits();
        loadFrictionModel(modelPath("friction"));

        const int n = config.jointNames.size();
        zeroAcceleration = Eigen::VectorXd::Zero(n);
        rigidTorque.resize(n);
        frictionTorque.resize(n);
        command.resize(n);
    }

    void loadRigidModel(const std::string &path) {
        const auto result = YAML::LoadFile(path);
        config.jointNames = result["joint_names"].as<std::vector<std::string>>();
        const int n = config.jointNames.size();
        require(n == 7, "Current Robot requires seven joints");
        const std::set<std::string> uniqueNames(config.jointNames.begin(), config.jointNames.end());
        require(uniqueNames.size() == config.jointNames.size(), "Duplicate model joint names");
        require(result["schema"].as<std::string>() == "rocos_identified_rigid_v1", "Unknown rigid model schema");
        require(result["urdf_hash"].as<std::string>() == sha256(config.urdf), "Rigid model/URDF hash mismatch");
        // Y needs joint axes and link transforms. Torque parameters come from P * beta.
        pinocchio::urdf::buildModel(config.urdf, model);
        require(model.nq == n && model.nv == n && model.njoints == n + 1, "Expected fixed-base single-DOF model");
        for (int i = 0; i < n; ++i) {
            require(model.names[i + 1] == config.jointNames[i], "Model joint order mismatch");
            require(model.joints[i + 1].nq() == 1 && model.joints[i + 1].nv() == 1, "Unsupported joint type");
        }
        data.reset(new pinocchio::Data(model));
        parameters = readVector(result["rigid_parameters"], 10 * n, "P * beta");
    }

    void loadGains(const YAML::Node &yaml) {
        const int n = config.jointNames.size();
        config.kp = readVector(yaml["kp"], n, "kp");
        config.kd = readVector(yaml["kd"], n, "kd");
        config.torqueLimit = readVector(yaml["torque_limit"], n, "torque_limit");
        config.torqueRateLimit = readVector(yaml["torque_rate_limit"], n, "torque_rate_limit");
        require((config.kp.array() >= 0).all(), "kp must be nonnegative");
        require((config.kd.array() >= 0).all(), "kd must be nonnegative");
        require((config.torqueLimit.array() > 0).all(), "Torque limits must be positive");
        require((config.torqueRateLimit.array() > 0).all(), "Torque rate limits must be positive");
    }

    void loadFrictionModel(const std::string &path) {
        const auto document = YAML::LoadFile(path);
        require(document["model"].as<std::string>() == "stribeck_load_temperature_v1",
                "Unsupported friction model");
        const auto joints = document["joint_parameters"];
        require(joints.size() == config.jointNames.size(), "Friction joint count mismatch");
        for (const auto &name : config.jointNames) {
            friction.emplace_back(joints[name]); // Match by joint name, never by JSON order.
            config.needsTemperature |= friction.back().temperatureSlope != 0;
        }
    }

    void loadHardwareLimits() {
        config.lower = model.lowerPositionLimit;
        config.upper = model.upperPositionLimit;
        config.velocity = model.velocityLimit;
        require(config.lower.allFinite() && config.upper.allFinite(), "Nonfinite position limits");
        require(config.velocity.allFinite(), "Nonfinite velocity limits");
        tinyxml2::XMLDocument xml;
        require(xml.LoadFile(config.urdf.c_str()) == tinyxml2::XML_SUCCESS, "Cannot read hardware URDF");
        const auto *robot = xml.FirstChildElement("robot");
        require(robot, "URDF has no robot element");
        std::set<int> ids;
        for (size_t i = 0; i < config.jointNames.size(); ++i) {
            const auto *joint = findJoint(robot, config.jointNames[i]);
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
            config.hardwareIds.push_back(id);

            config.lower[i] = std::max(config.lower[i], readAttribute(limit, "lower"));
            config.upper[i] = std::min(config.upper[i], readAttribute(limit, "upper"));
            config.velocity[i] = std::min(config.velocity[i], readAttribute(limit, "vel"));
            require(config.lower[i] < config.upper[i], "Empty position limit interval");
            require(config.velocity[i] > 0, "Velocity limit must be positive");

            const double effort = std::min(model.effortLimit[i], readAttribute(limit, "effort"));
            require(std::isfinite(effort) && config.torqueLimit[i] <= effort, "Torque limit exceeds URDF effort");
            const double scale = readAttribute(transform, "torque_per_unit");
            require(scale != 0, "Torque scale must be nonzero");
            require(config.torqueLimit[i] * std::abs(scale) <= 32767, "Torque limit exceeds int16 drive range");
            require(readAttribute(transform, "cnt_per_unit") != 0, "Position scale must be nonzero");
        }
    }
};

JointImpedanceController::JointImpedanceController(const std::string &path) : impl_(new Impl(path)) {}
JointImpedanceController::~JointImpedanceController() = default;
const JointImpedanceConfig &JointImpedanceController::config() const { return impl_->config; }

const Eigen::VectorXd &JointImpedanceController::computeTorque(
        const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
        const Eigen::VectorXd &qDesired, const Eigen::VectorXd &temperature) {
    auto &state = *impl_;
    const auto &config = state.config;
    const int n = config.jointNames.size();
    require(q.size() == n && dq.size() == n && qDesired.size() == n, "Wrong state/target size");
    require(q.allFinite() && dq.allFinite() && qDesired.allFinite(), "Nonfinite state/target");
    for (int i = 0; i < n; ++i) {
        require(q[i] >= config.lower[i] && q[i] <= config.upper[i], "Position exceeds joint limits");
        require(qDesired[i] >= config.lower[i] && qDesired[i] <= config.upper[i], "Target exceeds joint limits");
        require(std::abs(dq[i]) <= config.velocity[i], "Velocity exceeds joint limits");
    }
    if (temperature.size() == 0) {
        require(!config.needsTemperature, "Friction model requires measured joint temperatures");
    } else {
        require(temperature.size() == n && temperature.allFinite(), "Invalid measured temperatures");
    }

    // Rigid-body compensation: Y(q, dq, 0) * P * beta, with no inertia cancellation.
    const auto &Y = pinocchio::computeJointTorqueRegressor(
            state.model, *state.data, q, dq, state.zeroAcceleration);
    state.rigidTorque.noalias() = Y * state.parameters;

    for (int i = 0; i < n; ++i) {
        const auto &friction = state.friction[i];
        const double jointTemperature = temperature.size() ? temperature[i] : friction.referenceTemperature;
        state.frictionTorque[i] = friction.compute(dq[i], state.rigidTorque[i], jointTemperature);
    }

    state.command = state.rigidTorque + state.frictionTorque
                  + config.kp.cwiseProduct(qDesired - q)
                  - config.kd.cwiseProduct(dq);
    require(state.command.allFinite(), "Nonfinite impedance torque");
    return state.command;
}
} // namespace rocos
