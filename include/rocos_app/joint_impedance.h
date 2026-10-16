#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

namespace rocos {

struct JointImpedanceConfig {
    std::string urdf;
    std::vector<std::string> jointNames;
    std::vector<int> hardwareIds;
    Eigen::VectorXd kp, kd;
    Eigen::VectorXd lower, upper, velocity;
    Eigen::VectorXd torqueLimit, torqueRateLimit;
    bool needsTemperature = false;
};

// All vectors use config().jointNames order, with rad, rad/s, Nm and degC.
// One instance belongs to one control thread; update qDesired in that thread.
class JointImpedanceController {
public:
    explicit JointImpedanceController(const std::string &yamlPath);
    ~JointImpedanceController();
    const JointImpedanceConfig &config() const;

    // Y(q,dq,0) P beta + friction + Kp(qDesired-q) - Kd*dq.
    // No saturation here: the caller applies torque and torque-rate limits.
    // Temperature may be empty only when all temperature slopes are zero.
    // The returned buffer is reused by the next call; no steady-state allocation.
    const Eigen::VectorXd &computeTorque(const Eigen::VectorXd &q,
                                        const Eigen::VectorXd &dq,
                                        const Eigen::VectorXd &qDesired,
                                        const Eigen::VectorXd &temperature = Eigen::VectorXd());

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rocos
