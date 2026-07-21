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
#include "model_interface.hpp"
#include "unit_velocity_profile.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <vector>

namespace rocos {

/// @brief 笛卡尔圆弧运动 (MoveC) — 离线规划 + 全轨迹 IK 验证
///
/// 与 MoveCircle（在线规划）的核心区别：
///   1. Reset() 中一次性跑完整条 S 曲线，生成全部笛卡尔路径点
///   2. 逐点逆运动学验证：任一点 IK 失败 → PlanError → 回到 Stopped
///   3. 关节空间轨迹存储，正常执行时按索引回放
///   4. 暂停/继续走 Jacobian 积分通道：
///        s_dot → 切向速度 (Twist) → J⁺ · Twist → 关节速度 → 积分
///      因为全轨迹 IK 已验证，Jacobian 沿路径非奇异，J⁺ 始终存在
///
/// 支持两种构造模式：
///   圆心+角度：MoveCircleOffline(pose_start, center_frame, theta, model, ...)
///   三点圆弧：MoveCircleOffline(pose_start, pose_via, pose_goal, model, ...)
///
/// 典型用法：
///   MoveCircleOffline move(pose_start, center_frame, theta, model);
///   move.SetInitialJointPosition(q_current);
///   Result r = move.Reset();                   // 离线规划 + 逐点 IK
///   if (r == Result::NoError) {
///       while (true) {
///           r = move.Update();
///           if (r == Result::PlanFinished) break;
///           if (r < Result::NoError) { /* error */ }
///           Reference ref;
///           move.GenerateRef(ref);             // JntArray
///       }
///   }
class MoveCircleOffline : public MotionInterface {
public:
    /// @brief 圆心+角度模式
    /// @param pose_start   起始笛卡尔位姿
    /// @param center_frame 圆心信息（仅信任 p=圆心, M.UnitZ()=圆弧平面法向）
    /// @param theta        圆弧角度 [rad]
    /// @param model        机器人模型（必需，提供 IK / Jacobian）
    explicit MoveCircleOffline(const Frame& pose_start,
                               const Frame& center_frame,
                               double theta,
                               ModelInterface* model,
                               double v_limit = 1.0,
                               double a_limit = 2.0,
                               double j_limit = 10.0,
                               double dt      = 0.001);

    /// @brief 三点圆弧模式（内部解算圆心+转角）
    /// @param pose_start 起始位姿
    /// @param pose_via   中间位姿（仅用位置，不含姿态）
    /// @param pose_goal  目标位姿
    explicit MoveCircleOffline(const Frame& pose_start,
                               const Frame& pose_via,
                               const Frame& pose_goal,
                               ModelInterface* model,
                               double v_limit = 1.0,
                               double a_limit = 2.0,
                               double j_limit = 10.0,
                               double dt      = 0.001);

    ~MoveCircleOffline() override;

    /// @brief 设置 IK 暖启动初始关节位置（Reset 前必须调用）
    void SetInitialJointPosition(const JntArray& q);

    // ─── MotionInterface 接口 ───

    Result ValidateParameters() const override;
    Result Reset() override;
    Result GenerateRef(Reference& ref_out) override;
    Result Update() override;

    bool CanPause()  const override { return true; }
    bool CanResume() const override { return true; }
    bool CanStop()   const override { return true; }

    Result Pause() override;
    Result Resume() override;
    Result Stop() override;

private:
    // ── 圆弧几何（复用 MoveCircle 内部实现）──

    /// @brief 内部重建正交化的圆弧局部坐标系（X 轴强制指向起始点）
    static bool buildArcFrame(const Frame& start,
                              const Frame& center,
                              Frame& arc_out,
                              double& radius);

    /// @brief 三点解算圆心 + 转角 + 正交标架
    static bool solveThreePoints(const Frame& start,
                                 const Frame& via,
                                 const Frame& goal,
                                 Frame& arc_out,
                                 double& radius,
                                 double& theta);

    /// @brief s ∈ [0,1] 处圆弧插值位姿（局部坐标→世界坐标）
    Frame interpolatePose(double s) const;

    /// @brief s_dot → 6 维切向 Twist（圆弧切线速度 + 角速度=0）
    Twist computeCartesianTwist(double s_dot) const;

    /// @brief J(q)⁺ · twist → q_dot，Eigen SVD 伪逆求解
    bool solveJointVelocity(const JntArray& q,
                            const Twist& twist,
                            JntArray& q_dot_out) const;

    /// @brief 归一化限制
    bool computeNormalizedLimits();

    /// @brief 从轨迹回放模式切换到 Jacobian 积分模式
    Result switchToIntegrateMode();

    // ─── 执行模式 ───
    enum class Mode { Normal, Integrate };

    double dt_;
    UnitVelocityProfile profile_;
    ModelInterface* model_;  // 必需，不拥有

    Frame pose_start_;
    Frame arc_frame_;        // 内部重建：X=(start-center), Z=plane normal, Y=Z×X
    double theta_{0.0};
    double radius_{0.0};

    // 弧长
    double path_length_{0.0};

    // 一维标量限制
    double v_limit_{0.0};
    double a_limit_{0.0};
    double j_limit_{0.0};

    // 归一化后的单位区间限制
    double norm_v_{0.0};
    double norm_a_{0.0};
    double norm_j_{0.0};

    static constexpr double kArcEpsilon = 1e-7;

    // ─── 离线轨迹 ───
    struct TrajectoryPoint {
        double s;
        double s_dot;
        JntArray q;
    };
    std::vector<TrajectoryPoint> trajectory_;
    size_t index_{0};

    // ─── IK 暖启动 ───
    JntArray q_init_;

    // ─── 积分模式状态 ───
    Mode mode_{Mode::Normal};
    JntArray q_current_;
    int n_joints_{0};

    bool has_motion_{false};
};

}  // namespace rocos
