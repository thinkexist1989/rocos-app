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
    Eigen::VectorXd velocityLimit; // YAML 独立保护阈值，rad/s，不高于 URDF 速度限制。
    Eigen::VectorXd torqueLimit, torqueRateLimit;
    bool needsTemperature = false;
};

// 向量统一按 config().jointNames 排列，单位为 rad、rad/s、Nm 和 °C。
// 每个实例由一个控制线程使用，qDesired 也应在该线程更新。
class JointImpedanceController {
public:
    explicit JointImpedanceController(const std::string &yamlPath);
    ~JointImpedanceController();
    const JointImpedanceConfig &config() const;

    // 最近一次成功计算的补偿分量，供同一控制线程采样调试数据。
    const Eigen::VectorXd &rigidTorque() const;
    const Eigen::VectorXd &frictionTorque() const;

    // Y(q,dq,0) P beta + friction + Kp(qDesired-q) - Kd*dq.
    // 返回未限幅力矩，由调用方统一限制力矩幅值和变化率。
    // 仅当全部温度斜率为零时，temperature 才允许为空。
    // 返回值引用内部缓冲，下次调用会覆盖；模型及缓冲只在初始化时创建。
    const Eigen::VectorXd &computeTorque(const Eigen::VectorXd &q,
                                        const Eigen::VectorXd &dq,
                                        const Eigen::VectorXd &qDesired,
                                        const Eigen::VectorXd &temperature = Eigen::VectorXd());

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rocos
