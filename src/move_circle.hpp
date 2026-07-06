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

/// @brief 笛卡尔圆弧运动 (MoveC) — 基于 UnitVelocityProfile 的相位同步规划
///
/// 构造时内部重建圆弧局部坐标系：强制 X 轴 = (pose_start.p - center) 方向，
/// 保证 s=0 时 arc_pos = (R,0,0) 精确映射到 pose_start_.p，无起始跳变。
/// 仅信任外部传入的圆心位置和平面法向（Z轴），不依赖其 X/Y 轴方向。
///
/// 归一化方案：
///   1. 弧长 path_length = radius × |theta|
///   2. 归一化到单位区间 [0,1]：
///        norm_v = v_limit / path_length
///        norm_a = a_limit / path_length
///        norm_j = j_limit / path_length
///   3. 反向映射（圆弧插值）：
///        局部坐标：arc_pos = (R·cos(s·θ), R·sin(s·θ), 0)
///        世界坐标：p(s) = arc_frame_ * arc_pos
///        姿态固定：M(s) = pose_start.M
///
/// 输出 Reference 类型为 Frame（笛卡尔位姿），不涉及逆运动学。
class MoveCircle : public MotionInterface {
public:
    /// @brief 圆心+角度模式
    /// @param pose_start   起始笛卡尔位姿
    /// @param center_frame 圆心信息（仅信任 p=圆心, M.UnitZ()=圆弧平面法向）
    /// @param theta        圆弧角度 [rad]
    explicit MoveCircle(const Frame& pose_start,
                        const Frame& center_frame,
                        double theta,
                        double v_limit   = 1.0,
                        double a_limit   = 2.0,
                        double j_limit   = 10.0,
                        double dt        = 0.001);

    /// @brief 三点圆弧模式（内部解算圆心+转角）
    /// @param pose_start  起始位姿
    /// @param pose_via    途经位姿（仅用位置）
    /// @param pose_goal   目标位姿
    explicit MoveCircle(const Frame& pose_start,
                        const Frame& pose_via,
                        const Frame& pose_goal,
                        double v_limit   = 1.0,
                        double a_limit   = 2.0,
                        double j_limit   = 10.0,
                        double dt        = 0.001);

    ~MoveCircle() override;

    // ─── MotionInterface 接口 ───

    [[nodiscard]] Result support() const override;
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
    /// @brief 内部重建正交化的圆弧局部坐标系（X 轴强制指向起始点）
    static bool buildArcFrame(const Frame& pose_start,
                              const Frame& center_frame,
                              Frame& arc_frame_out,
                              double& radius_out);

    /// @brief 三点解算圆心 + 转角 + 正交标架
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
    Frame arc_frame_;       // 内部重建：X=(start-center), Z=plane normal, Y=Z×X
    double theta_{0.0};
    double radius_{0.0};    // 缓存，避免重复计算

    // 弧长 = radius × |theta|
    double path_length_{0.0};

    // 一维标量限制
    double v_limit_{0.0};
    double a_limit_{0.0};
    double j_limit_{0.0};

    // 归一化后的单位区间限制
    double norm_v_{0.0};
    double norm_a_{0.0};
    double norm_j_{0.0};

    bool has_motion_{false};
};

}  // namespace rocos
