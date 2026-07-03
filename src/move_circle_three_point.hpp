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
#pragma once

#include "motion_interface.hpp"
#include "unit_velocity_profile.hpp"

namespace rocos {

/// @brief 三点圆弧运动 (MoveC Three-Point) — 基于 UnitVelocityProfile
///
/// 构造时由 pose_start / pose_via / pose_goal 三点解算圆心和转角，
/// 内部重建正交圆弧坐标系（X 轴强制对齐起始点），其余归一化流程与
/// MoveCircle 完全一致。
///
/// 归一化方案：
///   1. 三点解算圆心 → radius = |start.p - center|
///   2. 计算转角 theta（start → goal 绕圆心）
///   3. path_length = radius × |theta|
///   4. norm_v/a/j = v_limit/a_limit/j_limit / path_length
///   5. 反向映射：arc_pos = (R·cos(sθ), R·sin(sθ), 0) → world
///      姿态固定 = pose_start.M
///
/// 输出 Reference 类型为 Frame（笛卡尔位姿），不涉及逆运动学。
class MoveCircleThreePoint : public MotionInterface {
public:
    /// @brief 构造并解算三点圆弧
    /// @param pose_start  起始位姿
    /// @param pose_via    途经位姿（仅用位置，姿态忽略）
    /// @param pose_goal   目标位姿
    /// @param v_limit     速度限制（一维标量 m/s），默认 1.0
    /// @param a_limit     加速度限制（一维标量 m/s²），默认 2.0
    /// @param j_limit     jerk 限制（一维标量 m/s³），默认 10.0
    /// @param dt          控制周期 [秒]，默认 0.001
    explicit MoveCircleThreePoint(const Frame& pose_start,
                                  const Frame& pose_via,
                                  const Frame& pose_goal,
                                  double v_limit   = 1.0,
                                  double a_limit   = 2.0,
                                  double j_limit   = 10.0,
                                  double dt        = 0.001);
    ~MoveCircleThreePoint() override;

    // ─── MotionInterface 接口 ───

    Result support() const override;
    Result Reset() override;
    Result GenerateRef(Reference& ref_out) override;
    Result Update() override;

    bool supportsPause()  const override { return true; }
    bool supportsResume() const override { return true; }
    bool supportsStop()   const override { return true; }

    Result Pause() override;
    Result Resume() override;
    Result Stop() override;

private:
    /// @brief 三点解算圆心 + 转角
    static bool solveThreePoints(const Frame& start,
                                  const Frame& via,
                                  const Frame& goal,
                                  Frame& arc_frame_out,
                                  double& radius_out,
                                  double& theta_out);

    /// @brief 计算弧长和归一化限制
    bool computeNormalizedLimits();

    /// @brief 当前 s ∈ [0,1] 处圆弧插值位姿
    Frame interpolateCircular(double s) const;

    double dt_;
    UnitVelocityProfile profile_;

    Frame pose_start_;
    Frame arc_frame_;       // X=(start-center), Z=plane normal, Y=Z×X
    double theta_{0.0};
    double radius_{0.0};

    double path_length_{0.0};

    double v_limit_{0.0};
    double a_limit_{0.0};
    double j_limit_{0.0};

    double norm_v_{0.0};
    double norm_a_{0.0};
    double norm_j_{0.0};

    bool has_motion_{false};
};

}  // namespace rocos
