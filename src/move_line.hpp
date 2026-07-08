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

#include <Eigen/Geometry>

namespace rocos {

/// @brief 笛卡尔直线运动 (MoveL) — 基于 UnitVelocityProfile 的相位同步规划
///
/// 归一化方案（参考 cartesian_geometry.h / move_l_command.h）：
///   1. 计算等效路径长度：
///        translation = |p_end - p_start|
///        rotation    = |GetRotAngle(R_start^-1 * R_end)|
///        path_length = max(translation, kEquivalentRadius * rotation)
///      kEquivalentRadius = 0.1 m，将旋转弧度转为等效平移距离
///   2. 归一化到单位区间 [0,1]：
///        norm_v = v_limit / path_length
///        norm_a = a_limit / path_length
///        norm_j = j_limit / path_length
///   3. 反向映射（插值）：
///        平移：p(s) = p_start + s * (p_end - p_start)
///        旋转：R(s) = Slerp(R_start, R_end, s)
///
/// 输出 Reference 类型为 Frame（笛卡尔位姿），不涉及逆运动学。
///
/// 典型用法：
///   MoveLine move(pose_start, pose_goal);
///   Result r = move.Reset();
///   if (r == Result::NoError) {
///       while (true) {
///           r = move.Update();
///           if (r == Result::PlanFinished) break;
///           if (r < Result::NoError) { /* error */ }
///           Reference ref;
///           move.GenerateRef(ref);
///       }
///   }
class MoveLine : public MotionInterface {
public:
    /// @brief 构造并配置全部运动参数
    /// @param pose_start  起始笛卡尔位姿
    /// @param pose_goal   目标笛卡尔位姿
    /// @param v_limit     速度限制（一维标量 m/s），默认 1.0
    /// @param a_limit     加速度限制（一维标量 m/s²），默认 2.0
    /// @param j_limit     jerk 限制（一维标量 m/s³），默认 10.0
    /// @param dt          控制周期 [秒]，默认 0.001
    explicit MoveLine(const Frame& pose_start,
                      const Frame& pose_goal,
                      double v_limit   = 1.0,
                      double a_limit   = 2.0,
                      double j_limit   = 10.0,
                      double dt        = 0.001,
                      ModelInterface* model = nullptr);
    ~MoveLine() override;

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
    /// @brief 计算 path_length 和归一化限制（四元数点积得旋转角）
    bool computeNormalizedLimits();

    /// @brief 当前 s ∈ [0,1] 处插值位姿
    Frame interpolatePose(double s) const;

    double dt_;
    UnitVelocityProfile profile_;

    Frame pose_start_;
    Frame pose_goal_;
    Eigen::Quaterniond q_start_;   // 缓存，避免每周期 GetQuaternion
    Eigen::Quaterniond q_end_;

    double path_length_{0.0};

    // 一维标量限制（保证平移和转动同步）
    double v_limit_{0.0};
    double a_limit_{0.0};
    double j_limit_{0.0};

    // 归一化后的单位区间限制
    double norm_v_{0.0};
    double norm_a_{0.0};
    double norm_j_{0.0};

    // 等效半径 [m]
    static constexpr double kEquivalentRadius = 0.1;

    bool has_motion_{false};
};

}  // namespace rocos
