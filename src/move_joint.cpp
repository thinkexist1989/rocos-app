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

#include "move_joint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rocos {

namespace {
// 已到位判断阈值：1e-6 低于任何实用编码器分辨率
// （24 位绝对值编码器 ≈ 3.7e-7 rad），确保机械意义上的零
constexpr double kDeltaEpsilon = 1e-6;
}  // namespace

// ============================================================================
// 构造 / 析构
// ============================================================================

MoveJoint::MoveJoint(const JntArray& q_start,
                     const JntArray& q_goal,
                     double v_limit,
                     double a_limit,
                     double j_limit,
                     double dt,
                     ModelInterface* model)
    : MotionInterface(model)
    , dt_(dt)
    , profile_(dt)
    , q_start_(q_start)
    , q_goal_(q_goal)
    , v_limit_(v_limit)
    , a_limit_(a_limit)
    , j_limit_(j_limit) {}

MoveJoint::~MoveJoint() = default;

// ============================================================================
// 参数校验
// ============================================================================

Result MoveJoint::support() const {
    const auto n = q_start_.rows();

    // 1. 维度检查
    if (n == 0 || q_goal_.rows() != n) {
        return Result::UnmatchedJointsNumber;
    }

    // 2. dt 合法性
    if (!std::isfinite(dt_) || dt_ <= 0.0) {
        return Result::ParameterNanOrInf;
    }

    // 3. 一维标量限制：有限性 + 正值性
    if (!std::isfinite(v_limit_) || !std::isfinite(a_limit_) || !std::isfinite(j_limit_)) {
        return Result::ParameterNanOrInf;
    }
    if (v_limit_ <= 0.0 || a_limit_ <= 0.0 || j_limit_ <= 0.0) {
        return Result::IllegalParameter;
    }

    // 4. 逐关节位置：有限性（不计算 delta，由 computeNormalizedLimits 统一处理）
    for (unsigned int i = 0; i < n; ++i) {
        if (!std::isfinite(q_start_(i)) || !std::isfinite(q_goal_(i))) {
            return Result::ParameterNanOrInf;
        }
    }

    return Result::NoError;
}

// ============================================================================
// 归一化限制计算
//
// delta_[i] = q_goal[i] - q_start[i]            （保留符号）
// delta_max = max_i |delta_[i]|                  （绝对值最大，决定运动时长）
// norm_v = v_limit / delta_max                   （一维标量归一化）
// ============================================================================

bool MoveJoint::computeNormalizedLimits() {
    const auto n = q_start_.rows();
    if (n == 0) return false;

    delta_.resize(n);

    double delta_max = 0.0;
    for (unsigned int i = 0; i < n; ++i) {
        delta_(i) = q_goal_(i) - q_start_(i);              // 保留符号
        delta_max = std::max(delta_max, std::abs(delta_(i)));
    }

    if (delta_max < kDeltaEpsilon) {
        has_motion_ = false;
        norm_v_ = 1.0;
        norm_a_ = 1.0;
        norm_j_ = 1.0;
        return true;
    }

    has_motion_ = true;
    norm_v_ = v_limit_ / delta_max;
    norm_a_ = a_limit_ / delta_max;
    norm_j_ = j_limit_ / delta_max;

    return true;
}

// ============================================================================
// 校验 + 初始化（合并 support + Reset，父类指针统一入口）
// ============================================================================

Result MoveJoint::Reset() {
    // 1. 纯参数校验（维度、有限性、正值性）
    const Result validation = support();
    if (validation != Result::NoError) {
        return validation;
    }

    // 2. 计算 delta_[i]、delta_max、归一化限制（唯一的 delta 计算入口）
    if (!computeNormalizedLimits()) {
        return Result::PlanError;
    }

    // 3. 已到位，无需运动（阈值 kDeltaEpsilon = 1e-6）
    if (!has_motion_) {
        return Result::PlanFinished;
    }

    // 4. 启动 UnitVelocityProfile
    if (!profile_.Start(norm_v_, norm_a_, norm_j_)) {
        return Result::PlanError;
    }

    return Result::NoError;
}

// ============================================================================
// 单步推进
// ============================================================================

Result MoveJoint::Update() {
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
// 生成当前周期的关节参考位置
// ============================================================================

Result MoveJoint::GenerateRef(Reference& ref_out) {
    const auto n = q_start_.rows();
    const double s = profile_.position();

    JntArray q_out(n);
    for (unsigned int i = 0; i < n; ++i) {
        q_out(i) = q_start_(i) + s * delta_(i);
    }

    ref_out = std::move(q_out);
    return Result::NoError;
}

// ============================================================================
// 暂停 / 继续 / 停止
// ============================================================================

Result MoveJoint::Pause() {
    if (!has_motion_) {
        return Result::NoError;
    }

    if (!profile_.Pause(norm_a_, norm_j_)) {
        return Result::PlanError;
    }

    return Result::NoError;
}

Result MoveJoint::Resume() {
    if (!has_motion_) {
        return Result::NoError;
    }

    profile_.Resume();
    return Result::NoError;
}

Result MoveJoint::Stop() {
    if (!has_motion_) {
        return Result::NoError;
    }

    if (!profile_.Stop(norm_a_, norm_j_)) {
        return Result::PlanError;
    }

    return Result::NoError;
}

}  // namespace rocos
