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

#include <ruckig/ruckig.hpp>

#include <atomic>
#include <chrono>
#include <mutex>

namespace rocos {

/// @brief 零空间点动 (Null-Space Jogging) — 基于投影矩阵法
///
/// 允许用户在关节空间输入期望速度，系统会自动滤除会引起末端运动的分量。
/// 生命周期三态：Active → [超时/Stop] → Decelerating → [speed=0] → Stopped
/// 线程模型与 MoveJog 保持完全一致。
class MoveNullJog : public MotionInterface {
public:
    enum class State { Active, Decelerating, Stopped };

    MoveNullJog(double dt = 0.001,
                double timeout = 0.1,
                ModelInterface* model = nullptr,
                double dir_threshold = 0.99);

    ~MoveNullJog() override;

    // ─── MotionInterface ───
    Result ValidateParameters() const override;
    Result Reset() override;
    Result Update() override;
    Result GenerateRef(Reference& ref_out) override;

    bool CanPause()  const override { return false; }
    bool CanResume() const override { return false; }
    bool CanStop()   const override { return true; }

    Result Pause() override;
    Result Resume() override;
    Result Stop() override;

    // ─── Null-Space Jog 特有 ───
    void setInitialPosition(const JntArray& q);

    /// @brief 喂入关节空间的点动意图方向 + 速度模长
    /// @param intent_direction 用户的物理期望
    /// @param speed 期望运动的速度
    Result FeedNullJog(const JntArray& intent_direction, double speed);

    bool IsStopped() const { return state_.load() == State::Stopped; }

private:
    static double normalizeDirection(const JntArray& src, JntArray& dst);
    static double directionCosine(const JntArray& a, const JntArray& b);

    void reconfigureSpeedOtg(double target_speed);
    JntArray computeJointVelocity();

    const double dt_;
    const double timeout_;
    const double dir_threshold_;

    // 速度 OTG（一维，控制线程独占）
    ruckig::Ruckig<1>          speed_otg_;
    ruckig::InputParameter<1>  speed_input_;
    ruckig::OutputParameter<1> speed_output_;
    double speed_otg_current_{0.0};

    // 限制参数
    double v_max_{1.0};
    double a_max_{3.0};
    double j_max_{5.0};

    // ─── mtx_ 保护（HTTP 写 / 控制读） ───
    mutable std::mutex mtx_;
    JntArray intent_direction_;
    double target_speed_{0.0};
    double elapsed_since_feed_{0.0};

    // ─── 原子变量（无锁读） ───
    std::atomic<State> state_{State::Active};
    std::atomic<bool>  has_started_{false};

    // ─── 控制线程独享 ───
    JntArray  q_integral_;
    int       joint_count_{0};
    Jacobian  jacobian_;
};

} // namespace rocos
