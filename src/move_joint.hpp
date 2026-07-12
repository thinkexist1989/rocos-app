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

/// @brief 关节空间运动 (MoveJ) — 基于 UnitVelocityProfile 的相位同步规划
///
/// 归一化方案（保证所有关节同步启停）：
///   1. 计算各关节位移 delta_i = q_goal[i] - q_start[i]
///   2. 取最大绝对位移 delta_max = max_i( |delta_i| )
///   3. 用一维标量限制归一化到单位区间 [0,1]：
///        norm_v = v_limit / delta_max
///        norm_a = a_limit / delta_max
///        norm_j = j_limit / delta_max
///   4. 所有关节共享同一个 s(t) 曲线，保证相位同步
///   5. 输出反向映射：q[i] = q_start[i] + s * delta_i
///
/// 典型用法（父类指针统一调用 Reset 即可）：
///   MoveJoint move(q_start, q_goal);                      // 全部默认值
///   MoveJoint move(q_start, q_goal, 1.0, 2.0, 10.0, dt);  // 全部指定
///   Result r = move.Reset();                        // 校验 + 初始化一步完成
///   if (r == Result::NoError) {
///       while (true) {
///           r = move.Update();
///           if (r == Result::PlanFinished) break;
///           if (r < Result::NoError) { /* error */ }
///           Reference ref;
///           move.GenerateRef(ref);
///       }
///   }
class MoveJoint : public MotionInterface {
public:
    /// @brief 构造并配置全部运动参数
    /// @param q_start   起始关节位置
    /// @param q_goal    目标关节位置
    /// @param v_limit   速度限制（一维标量），默认 1.0
    /// @param a_limit   加速度限制（一维标量），默认 2.0
    /// @param j_limit   jerk 限制（一维标量），默认 10.0
    /// @param dt        控制周期 [秒]，默认 0.001
    explicit MoveJoint(const JntArray& q_start,
                       const JntArray& q_goal,
                       double v_limit   = 1.0,
                       double a_limit   = 2.0,
                       double j_limit   = 10.0,
                       double dt        = 0.001,
                       ModelInterface* model = nullptr);
    ~MoveJoint() override;

    // ─── MotionInterface 接口 ───

    /// @brief 纯参数校验：维度匹配、有限性、正值性（不涉及 delta 计算）
    /// @return NoError 通过，<0=错误码
    Result ValidateParameters() const override;

    /// @brief 校验参数并初始化规划（合并 support + Reset）
    /// @return NoError=成功启动, PlanFinished=已到位无需运动, <0=错误码
    Result Reset() override;

    /// @brief 生成当前周期的关节参考位置
    Result GenerateRef(Reference& ref_out) override;

    /// @brief 推进 OTG 一步
    /// @return NoError=仍在运行, PlanFinished=完成, <0=错误
    Result Update() override;

    bool CanPause()  const override { return true; }
    bool CanResume() const override { return true; }
    bool CanStop()   const override { return true; }

    Result Pause() override;
    Result Resume() override;
    Result Stop() override;



private:
    /// @brief 计算 delta_[i] 和归一化限制 norm_v/a/j
    bool computeNormalizedLimits();

    double dt_;
    UnitVelocityProfile profile_;

    JntArray q_start_;
    JntArray q_goal_;
    JntArray delta_;

    // 一维标量限制（所有关节共享，保证相位同步）
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
