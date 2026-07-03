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

#include "move_circle_three_point.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rocos {

namespace {
constexpr double kArcEpsilon = 1e-7;
}  // namespace

// ============================================================================
// 三点解算圆弧：圆心 + 转角 → 正交标架
//
// 参考 cartesian_geometry.h 的 computeCircleCenter / computeCircleArcParams，
// 内部重建正交化标架，X 轴强制对齐起点。
// ============================================================================

bool MoveCircleThreePoint::solveThreePoints(const Frame& start,
                                             const Frame& via,
                                             const Frame& goal,
                                             Frame& arc_frame_out,
                                             double& radius_out,
                                             double& theta_out) {
    // ─── Step 1: 解圆心 ───
    KDL::Vector v1 = via.p - start.p;
    KDL::Vector v2 = goal.p - start.p;

    if (v1.Normalize() < kArcEpsilon) return false;
    if (v2.Normalize() < kArcEpsilon) return false;

    // 圆弧平面法向：Start → Via → Goal 右手螺旋
    KDL::Vector axis_z = v1 * v2;
    if (axis_z.Normalize() < kArcEpsilon) return false;

    // 局部坐标系
    KDL::Vector axis_x = v1;
    KDL::Vector axis_y = axis_z * axis_x;
    axis_y.Normalize();

    // 投影几何解 h = ((cx - bx/2)² + cy² - (bx/2)²) / (2·cy)
    const KDL::Vector d1 = via.p - start.p;
    const KDL::Vector d2 = goal.p - start.p;
    const double bx = KDL::dot(d1, axis_x);
    const double cx = KDL::dot(d2, axis_x);
    const double cy = KDL::dot(d2, axis_y);

    if (std::abs(cy) < kArcEpsilon) return false;

    const double h = ((cx - bx / 2.0) * (cx - bx / 2.0) + cy * cy -
                      (bx / 2.0) * (bx / 2.0)) / (2.0 * cy);
    const KDL::Vector center = start.p + axis_x * (bx / 2.0) + axis_y * h;

    // ─── Step 2: 构建圆弧局部标架（X 轴强制指向起点）───
    KDL::Vector arc_x = start.p - center;
    radius_out = arc_x.Normalize();
    if (radius_out < kArcEpsilon) return false;

    // 正交化 Z
    KDL::Vector arc_z = axis_z;
    arc_z -= KDL::dot(arc_z, arc_x) * arc_x;
    if (arc_z.Normalize() < kArcEpsilon) return false;

    KDL::Vector arc_y = arc_z * arc_x;
    arc_y.Normalize();

    arc_frame_out = KDL::Frame(KDL::Rotation(arc_x, arc_y, arc_z), center);

    // ─── Step 3: 计算转角 theta（起点 → 终点）───
    const KDL::Vector end_local = arc_frame_out.Inverse() * goal.p;
    double theta = std::atan2(end_local(1), end_local(0));
    if (theta < 0.0) {
        theta += 2.0 * M_PI;
    }
    theta_out = theta;

    return true;
}

// ============================================================================
// 构造 / 析构
// ============================================================================

MoveCircleThreePoint::MoveCircleThreePoint(const Frame& pose_start,
                                           const Frame& pose_via,
                                           const Frame& pose_goal,
                                           double v_limit,
                                           double a_limit,
                                           double j_limit,
                                           double dt)
    : dt_(dt)
    , profile_(dt)
    , pose_start_(pose_start)
    , v_limit_(v_limit)
    , a_limit_(a_limit)
    , j_limit_(j_limit) {
    if (!solveThreePoints(pose_start_, pose_via, pose_goal,
                          arc_frame_, radius_, theta_)) {
        // 解算失败：标记无运动，后续 support/Reset 返回错误
        has_motion_ = false;
        theta_ = 0.0;
        radius_ = 0.0;
        arc_frame_ = pose_start_;
    }
}

MoveCircleThreePoint::~MoveCircleThreePoint() = default;

// ============================================================================
// 参数校验
// ============================================================================

Result MoveCircleThreePoint::support() const {
    // 纯参数校验：仅检查有限性 + 正值性，不做几何阈值拦截。
    // 圆心解算失败 / 运动量太小 → 由 computeNormalizedLimits 设 has_motion_=false，
    // Reset() 返回 PlanFinished 优雅退出，不报错。
    if (!std::isfinite(dt_) || dt_ <= 0.0) {
        return Result::ParameterNanOrInf;
    }
    if (!std::isfinite(theta_)) {
        return Result::ParameterNanOrInf;
    }
    if (!std::isfinite(radius_)) {
        return Result::ParameterNanOrInf;
    }
    if (!std::isfinite(v_limit_) || !std::isfinite(a_limit_) || !std::isfinite(j_limit_)) {
        return Result::ParameterNanOrInf;
    }
    if (v_limit_ <= 0.0 || a_limit_ <= 0.0 || j_limit_ <= 0.0) {
        return Result::IllegalParameter;
    }
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(pose_start_.p(i)) || !std::isfinite(arc_frame_.p(i))) {
            return Result::ParameterNanOrInf;
        }
    }
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (!std::isfinite(pose_start_.M(r, c)) ||
                !std::isfinite(arc_frame_.M(r, c))) {
                return Result::ParameterNanOrInf;
            }
        }
    }
    return Result::NoError;
}

// ============================================================================
// 归一化限制计算
// ============================================================================

bool MoveCircleThreePoint::computeNormalizedLimits() {
    if (radius_ < kArcEpsilon || std::abs(theta_) < kArcEpsilon) {
        has_motion_ = false;
        norm_v_ = 1.0; norm_a_ = 1.0; norm_j_ = 1.0;
        return true;
    }

    path_length_ = radius_ * std::abs(theta_);

    if (path_length_ < kArcEpsilon) {
        has_motion_ = false;
        norm_v_ = 1.0; norm_a_ = 1.0; norm_j_ = 1.0;
        return true;
    }

    has_motion_ = true;
    norm_v_ = v_limit_ / path_length_;
    norm_a_ = a_limit_ / path_length_;
    norm_j_ = j_limit_ / path_length_;
    return true;
}

// ============================================================================
// 圆弧插值
// ============================================================================

Frame MoveCircleThreePoint::interpolateCircular(double s) const {
    const double angle = s * theta_;
    const KDL::Vector arc_pos(radius_ * std::cos(angle),
                               radius_ * std::sin(angle),
                               0.0);
    return KDL::Frame(pose_start_.M, arc_frame_ * arc_pos);
}

// ============================================================================
// 校验 + 初始化
// ============================================================================

Result MoveCircleThreePoint::Reset() {
    const Result validation = support();
    if (validation != Result::NoError) return validation;

    if (!computeNormalizedLimits()) return Result::PlanError;
    if (!has_motion_)             return Result::PlanFinished;

    if (!profile_.Start(norm_v_, norm_a_, norm_j_)) return Result::PlanError;
    return Result::NoError;
}

// ============================================================================
// 单步推进
// ============================================================================

Result MoveCircleThreePoint::Update() {
    if (!has_motion_) return Result::PlanFinished;

    const int rc = profile_.Update();
    if (rc < 0) return Result::PlanError;
    if (rc == 0) return Result::PlanFinished;
    return Result::NoError;
}

// ============================================================================
// 输出笛卡尔参考
// ============================================================================

Result MoveCircleThreePoint::GenerateRef(Reference& ref_out) {
    ref_out = interpolateCircular(profile_.position());
    return Result::NoError;
}

// ============================================================================
// 暂停 / 继续 / 停止
// ============================================================================

Result MoveCircleThreePoint::Pause() {
    if (!has_motion_) return Result::NoError;
    if (!profile_.Pause(norm_a_, norm_j_)) return Result::PlanError;
    return Result::NoError;
}

Result MoveCircleThreePoint::Resume() {
    if (!has_motion_) return Result::NoError;
    profile_.Resume();
    return Result::NoError;
}

Result MoveCircleThreePoint::Stop() {
    if (!has_motion_) return Result::NoError;
    if (!profile_.Stop(norm_a_, norm_j_)) return Result::PlanError;
    return Result::NoError;
}

}  // namespace rocos
