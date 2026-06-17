#ifndef ROCOS_APP_MOTION_MODEL_PROVIDER_H
#define ROCOS_APP_MOTION_MODEL_PROVIDER_H

#include <functional>
#include <vector>

namespace rocos::motion {

struct KinematicsAdapter {
    std::function<bool(const std::vector<double>& q, std::vector<double>& pose_out)>
        forwardKinematics;
    std::function<bool(const std::vector<double>& q_seed,
                       const std::vector<double>& target_pose,
                       std::vector<double>& q_out)>
        inverseKinematics;
    std::function<int()> getDof;
};

struct ModelProvider {
    KinematicsAdapter kinematics;
};

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_MODEL_PROVIDER_H
