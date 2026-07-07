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

#include <chrono>
#include <mutex>

namespace rocos {

/// @brief 连续点动 (Jog) —— 关节空间或笛卡尔空间
///
/// 活性保持 (Liveness / Watchdog) 机制：
///   1. 上位机必须按周期 <= timeout_sec 调用 FeedJog() 喂入新方向向量
///   2. 每次 Update() 内部把 elapsed_since_feed_ += dt_
///   3. 若 elapsed_since_feed_ > timeout_sec_，触发自减速：
///        - 当前速度向量按加速度上限逐步归零
///        - 归零后 Update() 返回 Result::PlanFinished
///   4. FeedJog() 清零 elapsed_since_feed_，watchdog 重新计时
///   5. 若新方向与旧方向单位化点积 < threshold_，先自减速再切到新方向
///      （避免方向跳变引起的冲击）
///
/// 生命周期契约（对齐 motion_fsm_executor_design.md 场景 8-11）：
///   supportsPause()  = false
///   supportsResume() = false
///   supportsStop()   = true
///   IsLivenessRequired() = true
///
/// 线程模型：
///   FeedJog() 由 HTTP 线程调用；Update()/GenerateRef() 由控制线程调用。
///   所有可变状态由 mtx_ 保护。
class MoveJog : public MotionInterface {
public:
    /// @param jogvec       初始点动方向向量 (JntArray=关节, Twist=笛卡尔)
    /// @param timeout_sec  喂饭超时秒数；超过该时长未 FeedJog 则自减速
    /// @param threshold    方向切换阈值 [-1,1]；新旧方向余弦 < 该值触发平滑切换
    /// @param dt           控制周期(秒)，需与 Executor 周期一致
    MoveJog(JogVec jogvec,
            double timeout_sec = 0.1,
            double threshold   = 0.9,
            double dt          = 0.001);

    ~MoveJog() override;

    // ─── MotionInterface 接口 ───
    Result GenerateRef(Reference &ref_out) override;
    Result Reset() override;
    Result Update() override;
    Result support() const override;

    bool supportsPause()  const override { return false; }
    bool supportsResume() const override { return false; }
    bool supportsStop()   const override { return true;  }

    Result Pause()  override;
    Result Resume() override;
    Result Stop()   override;

    // ─── Jog 特有：活性保持入口 ───

    /// @brief 上位机周期调用，喂入新方向向量并清零 watchdog
    /// @param new_vec 新方向向量（variant 类型必须与构造时一致）
    /// @return NoError       接受
    /// @return IllegalParameter    variant 类型/维度不匹配
    /// @return PlanFinished  已进入终止态，需重建 MoveJog 实例
    /// @note  线程安全
    Result FeedJog(const JogVec &new_vec);

    // ─── 诊断接口（线程安全） ───
    double GetTimeSinceLastFeed() const noexcept;
    bool   IsDecelerating() const noexcept;
    bool   IsFinished() const noexcept;

private:
    /// @brief 归一化点积检测；返回 true 表示方向连续可平滑跟随
    bool checkDirectionContinuityLocked(const JogVec &new_vec) const;

    /// @brief 触发自减速（超时 / Stop / 方向跳变共用）
    void startDecelerationLocked() noexcept;

    /// @brief 单周期减速积分：把 current_vec_ 按 a_limit_ 向 0 收敛
    /// @return true 表示已完全归零
    bool stepDecelerationLocked() noexcept;

    // ─── 配置参数（构造后只读） ───
    const double dt_;
    const double timeout_sec_;
    const double threshold_;
    const double a_limit_{2.0};   ///< 自减速加速度上限，TODO: 从配置读取

    // ─── 运行时状态（mtx_ 保护） ───
    mutable std::mutex mtx_;
    JogVec current_vec_;
    JogVec target_vec_;
    double elapsed_since_feed_{0.0};
    bool   decelerating_{false};
    bool   finished_{false};
    std::chrono::steady_clock::time_point last_feed_wall_{};
};

} // namespace rocos
