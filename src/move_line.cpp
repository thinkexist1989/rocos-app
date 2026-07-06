// Copyright 2026, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

#include "move_line.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rocos {

namespace {
// 路径长度阈值：低于此值视为已到位
constexpr double kPathEpsilon = 1e-7;
}  // namespace

// ============================================================================
// 构造 / 析构
// ============================================================================

MoveLine::MoveLine(const Frame& pose_start,
                   const Frame& pose_goal,
                   double v_limit,
                   double a_limit,
                   double j_limit,
                   double dt)
    : dt_(dt)
    , profile_(dt)
    , pose_start_(pose_start)
    , pose_goal_(pose_goal)
    , v_limit_(v_limit)
    , a_limit_(a_limit)
    , j_limit_(j_limit) {}

MoveLine::~MoveLine() = default;

// ============================================================================
// 参数校验
// ============================================================================

Result MoveLine::support() const {
    // 1. dt 合法性
    if (!std::isfinite(dt_) || dt_ <= 0.0) {
        return Result::ParameterNanOrInf;
    }

    // 2. 一维标量限制：有限性 + 正值性
    if (!std::isfinite(v_limit_) || !std::isfinite(a_limit_) || !std::isfinite(j_limit_)) {
        return Result::ParameterNanOrInf;
    }
    if (v_limit_ <= 0.0 || a_limit_ <= 0.0 || j_limit_ <= 0.0) {
        return Result::IllegalParameter;
    }

    // 3. 起止位姿：位置和旋转分量有限性检查
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(pose_start_.p(i)) || !std::isfinite(pose_goal_.p(i))) {
            return Result::ParameterNanOrInf;
        }
    }
    // 旋转矩阵 9 个分量
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (!std::isfinite(pose_start_.M(r, c)) ||
                !std::isfinite(pose_goal_.M(r, c))) {
                return Result::ParameterNanOrInf;
            }
        }
    }

    return Result::NoError;
}

// ============================================================================
// 归一化限制计算
// ============================================================================

bool MoveLine::computeNormalizedLimits() {
    // 平移距离
    const double translation = (pose_goal_.p - pose_start_.p).Norm();

    // 旋转角度（从相对旋转矩阵中提取）
    KDL::Vector axis;
    const auto r_rel = pose_start_.M.Inverse() * pose_goal_.M;
    const double rotation = std::abs(r_rel.GetRotAngle(axis));

    // 等效路径长度：max(平移, 等效半径 × 旋转弧度)
    const double r_length = kEquivalentRadius * rotation;
    path_length_ = std::max(translation, r_length);

    if (path_length_ < kPathEpsilon) {
        has_motion_ = false;
        norm_v_ = 1.0;
        norm_a_ = 1.0;
        norm_j_ = 1.0;
        return true;
    }

    has_motion_ = true;
    norm_v_ = v_limit_ / path_length_;
    norm_a_ = a_limit_ / path_length_;
    norm_j_ = j_limit_ / path_length_;

    return true;
}

// ============================================================================
// 四元数球面线性插值（Slerp）— 栈上 std::array，零堆分配
// ============================================================================

std::array<double, 4> MoveLine::slerpQuaternion(
    const std::array<double, 4>& q_start,
    const std::array<double, 4>& q_end,
    double s) {

    // s ≤ 0 → 返回精确起点
    if (s <= 0.0) {
        return q_start;
    }
    // s ≥ 1 → 返回精确终点（消除浮点累积误差导致的稳态偏差）
    if (s >= 1.0) {
        return q_end;
    }

    // 点积 = cos(θ)
    double cos_theta = q_start[0] * q_end[0] + q_start[1] * q_end[1] +
                       q_start[2] * q_end[2] + q_start[3] * q_end[3];

    // 保证走最短弧
    auto q1 = q_start;
    if (cos_theta < 0.0) {
        for (auto& v : q1) v *= -1.0;
        cos_theta = -cos_theta;
    }

    const double theta = std::acos(std::clamp(cos_theta, -1.0, 1.0));
    const double sin_theta = std::sin(theta);

    // 夹角极小或 sinθ ≈ 0 → 退化为 NLerp（归一化线性插值），
    // 不能返回 q1，否则 s→1 时旋转锁死在起点，产生不可消除的稳态误差
    if (sin_theta < kPathEpsilon) {
        std::array<double, 4> result{};
        double norm_sq = 0.0;
        for (int i = 0; i < 4; ++i) {
            result[i] = (1.0 - s) * q1[i] + s * q_end[i];
            norm_sq += result[i] * result[i];
        }
        const double inv_norm = 1.0 / std::sqrt(norm_sq);
        for (auto& v : result) v *= inv_norm;
        return result;
    }

    // 标准 Slerp
    const double c1 = std::sin((1.0 - s) * theta) / sin_theta;
    const double c2 = std::sin(s * theta) / sin_theta;

    std::array<double, 4> result{};
    for (int i = 0; i < 4; ++i) {
        result[i] = c1 * q1[i] + c2 * q_end[i];
        if (std::isnan(result[i])) {
            // 防御性回退：NaN → NLerp
            return slerpQuaternion(q_start, q_end, s < 0.5 ? 0.0 : 1.0);
        }
    }
    return result;
}

// ============================================================================
// 位姿插值：平移线性 + 旋转 Slerp
// ============================================================================

Frame MoveLine::interpolatePose(double s) const {
    Frame result;

    // 平移：线性
    result.p = pose_start_.p + (pose_goal_.p - pose_start_.p) * s;

    // 旋转：四元数 Slerp（栈上 std::array，零堆分配，实时安全）
    std::array<double, 4> q_start{}, q_end{};
    pose_start_.M.GetQuaternion(q_start[0], q_start[1], q_start[2], q_start[3]);
    pose_goal_.M.GetQuaternion(q_end[0], q_end[1], q_end[2], q_end[3]);

    const auto q_interp = slerpQuaternion(q_start, q_end, s);
    result.M = KDL::Rotation::Quaternion(
        q_interp[0], q_interp[1], q_interp[2], q_interp[3]);

    return result;
}

// ============================================================================
// 校验 + 初始化（合并 support + Reset，父类指针统一入口）
// ============================================================================

Result MoveLine::Reset() {
    const Result validation = support();
    if (validation != Result::NoError) {
        return validation;
    }

    if (!computeNormalizedLimits()) {
        return Result::PlanError;
    }

    if (!has_motion_) {
        return Result::PlanFinished;
    }

    if (!profile_.Start(norm_v_, norm_a_, norm_j_)) {
        return Result::PlanError;
    }

    return Result::NoError;
}

// ============================================================================
// 单步推进
// ============================================================================

Result MoveLine::Update() {
    if (!has_motion_) {
        return Result::PlanFinished;
    }

    const int rc = profile_.Update();

    if (rc < 0) {
        return Result::PlanError;
    }
    if (rc == 0) {
        return Result::PlanFinished;
    }

    return Result::NoError;
}

// ============================================================================
// 生成当前周期的笛卡尔参考位姿
// ============================================================================

Result MoveLine::GenerateRef(Reference& ref_out) {
    const double s = profile_.position();
    ref_out = interpolatePose(s);
    return Result::NoError;
}

// ============================================================================
// 暂停 / 继续 / 停止
// ============================================================================

Result MoveLine::Pause() {
    if (!has_motion_) {
        return Result::NoError;
    }
    if (!profile_.Pause(norm_a_, norm_j_)) {
        return Result::PlanError;
    }
    return Result::NoError;
}

Result MoveLine::Resume() {
    if (!has_motion_) {
        return Result::NoError;
    }
    profile_.Resume();
    return Result::NoError;
}

Result MoveLine::Stop() {
    if (!has_motion_) {
        return Result::NoError;
    }
    if (!profile_.Stop(norm_a_, norm_j_)) {
        return Result::PlanError;
    }
    return Result::NoError;
}

}  // namespace rocos
