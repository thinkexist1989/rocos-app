#ifndef ROCOS_APP_MOTION_CARTESIAN_GEOMETRY_H
#define ROCOS_APP_MOTION_CARTESIAN_GEOMETRY_H

#include <rocos_app/motion/finite_motion_command.h>

#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace rocos::motion::cartesian {

constexpr double kEquivalentRadius = 0.1;
constexpr double kEpsilon = 1e-7;

struct PathMetrics {
    double translation_distance{0.0};
    double rotation_angle{0.0};
    double path_length{0.0};
};

inline double rotationAngle(const KDL::Rotation& r_start,
                            const KDL::Rotation& r_end) {
    KDL::Vector axis;
    const auto r_rel = r_start.Inverse() * r_end;
    return r_rel.GetRotAngle(axis);
}

inline PathMetrics computePathMetrics(const KDL::Frame& start,
                                      const KDL::Frame& end) {
    PathMetrics metrics;
    metrics.translation_distance = (end.p - start.p).Norm();

    KDL::Vector axis;
    const auto r_rel = start.M.Inverse() * end.M;
    metrics.rotation_angle = std::abs(r_rel.GetRotAngle(axis));

    const double r_length = kEquivalentRadius * metrics.rotation_angle;
    metrics.path_length = std::max(metrics.translation_distance, r_length);
    return metrics;
}

namespace detail {

inline std::vector<double> quaternionSlerp(
    const std::vector<double>& start,
    const std::vector<double>& end,
    double s) {
    double cos_theta = start[0] * end[0] + start[1] * end[1] +
                       start[2] * end[2] + start[3] * end[3];

    auto start_adj = start;
    if (cos_theta < 0) {
        for (auto& v : start_adj) { v *= -1; }
        cos_theta *= -1;
    }

    const double theta = std::acos(std::clamp(cos_theta, -1.0, 1.0));
    if (std::abs(theta) < kEpsilon || s == 0.0) {
        return start_adj;
    }

    const double sin_theta = std::sin(theta);
    if (sin_theta < kEpsilon) {
        return start_adj;
    }

    const double c1 = std::sin((1.0 - s) * theta) / sin_theta;
    const double c2 = std::sin(s * theta) / sin_theta;

    std::vector<double> result(4);
    for (int i = 0; i < 4; ++i) {
        result[i] = c1 * start_adj[i] + c2 * end[i];
        if (std::isnan(result[i])) {
            return start_adj;
        }
    }
    return result;
}

}  // namespace detail

inline KDL::Frame interpolateLinear(const KDL::Frame& start,
                                    const KDL::Frame& end,
                                    double s) {
    KDL::Frame result;
    result.p = start.p + (end.p - start.p) * s;

    std::vector<double> q_start(4), q_end(4);
    start.M.GetQuaternion(q_start[0], q_start[1], q_start[2], q_start[3]);
    end.M.GetQuaternion(q_end[0], q_end[1], q_end[2], q_end[3]);

    const auto q_interp = detail::quaternionSlerp(q_start, q_end, s);
    result.M = KDL::Rotation::Quaternion(q_interp[0], q_interp[1],
                                          q_interp[2], q_interp[3]);
    return result;
}

// 圆弧插值：直接使用预计算的 center_frame（含旋转矩阵和位置）。
// center_frame.M 的列向量为局部坐标系的轴，center_frame.p 为圆心。
// s ∈ [0,1]，arc_pos = (R*cos(s*θ), R*sin(s*θ), 0) 在局部坐标系中。
// 最终位置 = center_frame * arc_pos。orientation 保持 start 的不变。
inline KDL::Frame interpolateCircular(const KDL::Frame& start,
                                      const KDL::Frame& /*end*/,
                                      const KDL::Frame& center_frame,
                                      double theta,
                                      double s) {
    const double radius = (start.p - center_frame.p).Norm();
    if (radius < kEpsilon) { return start; }

    const KDL::Vector arc_pos(
        radius * std::cos(s * theta),
        radius * std::sin(s * theta),
        0.0);

    return KDL::Frame(start.M, center_frame * arc_pos);
}

inline bool computeCircleCenter(KDL::Frame& center_out,
                                const KDL::Frame& p1,
                                const KDL::Frame& p2,
                                const KDL::Frame& p3) {
    KDL::Vector v1 = p2.p - p1.p;
    KDL::Vector v2 = p3.p - p1.p;

    if (v1.Normalize() < kEpsilon) { return false; }
    if (v2.Normalize() < kEpsilon) { return false; }

    // axis_z = v2 × v1 (与 legacy circle_center 一致)
    KDL::Vector axis_z{v2 * v1};
    if (axis_z.Normalize() < kEpsilon) { return false; }

    KDL::Vector axis_x{v1};
    KDL::Vector axis_y{axis_z * axis_x};
    axis_y.Normalize();

    v1 = p2.p - p1.p;
    v2 = p3.p - p1.p;

    const double bx = dot(v1, axis_x);
    const double cx = dot(v2, axis_x);
    const double cy = dot(v2, axis_y);

    if (std::abs(cy) < kEpsilon) { return false; }

    const double h = ((cx - bx / 2) * (cx - bx / 2) + cy * cy -
                      (bx / 2) * (bx / 2)) / (2 * cy);
    center_out.p = p1.p + axis_x * (bx / 2) + axis_y * h;
    return true;
}

inline bool computeCircleArcParams(KDL::Frame& center_frame_out,
                                   double& theta_out,
                                   const KDL::Frame& start,
                                   const KDL::Frame& via,
                                   const KDL::Frame& end) {
    KDL::Frame center;
    if (!computeCircleCenter(center, start, via, end)) {
        return false;
    }

    KDL::Vector axis_x = start.p - center.p;
    const double radius = axis_x.Normalize();
    if (radius < kEpsilon) { return false; }

    KDL::Vector axis_tmp = via.p - center.p;
    axis_tmp.Normalize();

    KDL::Vector axis_z(axis_x * axis_tmp);
    if (axis_z.Normalize() < kEpsilon) { return false; }

    KDL::Vector axis_y{axis_z * axis_x};
    axis_y.Normalize();

    center_frame_out = KDL::Frame(KDL::Rotation{axis_x, axis_y, axis_z}, center.p);

    const KDL::Vector end_local = center_frame_out.Inverse() * end.p;
    double theta = std::atan2(end_local(1), end_local(0));
    if (theta < 0) {
        theta += 2.0 * M_PI;
    }
    theta_out = theta;
    return true;
}

inline MotionProfileLimits computeNormalizedLimits(
    double path_length,
    double max_cart_vel,
    double max_cart_acc) {
    if (path_length < kEpsilon) {
        return MotionProfileLimits{1.0, 1.0, 1.0};
    }
    return MotionProfileLimits{
        max_cart_vel / path_length,
        max_cart_acc / path_length,
        2.0 * max_cart_acc / path_length
    };
}

}  // namespace rocos::motion::cartesian

#endif  // ROCOS_APP_MOTION_CARTESIAN_GEOMETRY_H
