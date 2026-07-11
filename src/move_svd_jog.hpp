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
#include <Eigen/Dense>

#include <atomic>
#include <chrono>
#include <mutex>
#include <vector>

namespace rocos {

/// @brief 基于 SVD 的零空间维度点动 (SVD Null-Space Jogging)
///
/// 允许用户直接在抽象的"零空间维度"上输入期望速度。
/// 系统自动进行雅可比矩阵的 SVD 分解，提取正交基，并执行严格的基向量跟踪
/// 以防止运动过程中的突变和翻转。最后通过 1D OTG 和全局安全缩放保证平滑与安全。
class MoveSvdJog : public MotionInterface {
public:
    enum class State { Active, Decelerating, Stopped };

    MoveSvdJog(double dt = 0.001,
               double timeout = 0.1,
               ModelInterface* model = nullptr,
               double dir_threshold = 0.99);

    ~MoveSvdJog() override;

    // ─── MotionInterface ───
    Result support() const override;
    Result Reset() override;
    Result Update() override;
    Result GenerateRef(Reference& ref_out) override;

    bool supportsPause()  const override { return false; }
    bool supportsResume() const override { return false; }
    bool supportsStop()   const override { return true; }

    Result Pause() override;
    Result Resume() override;
    Result Stop() override;

    // ─── SVD Null-Space Jog 特有 ───
    void setInitialPosition(const JntArray& q);

    /// @brief 喂入抽象零空间维度的点动速度
    /// @param dim_speeds 长度必须为 n-6，如7轴机械臂传入 {0.5}
    Result FeedSvdJog(const std::vector<double>& dim_speeds);

    bool IsStopped() const { return state_.load() == State::Stopped; }

private:
    void reconfigureSpeedOtg(double target_speed);
    JntArray computeJointVelocity();

    /// 跟踪并对齐 SVD 分解出的零空间基向量，防止翻转和乱序
    void alignNullspaceBasis(Eigen::MatrixXd& current_basis);

    const double dt_;
    const double timeout_;
    const double dir_threshold_;

    ruckig::Ruckig<1>          speed_otg_;
    ruckig::InputParameter<1>  speed_input_;
    ruckig::OutputParameter<1> speed_output_;
    double speed_otg_current_{0.0};

    double v_max_{1.0};
    double a_max_{3.0};
    double j_max_{5.0};

    // ─── mtx_ 保护（HTTP 写 / 控制读） ───
    mutable std::mutex mtx_;
    std::vector<double> unit_dim_direction_;
    double target_speed_{0.0};
    double elapsed_since_feed_{0.0};

    // ─── 原子变量 ───
    std::atomic<State> state_{State::Active};
    std::atomic<bool>  has_started_{false};

    // ─── 控制线程独享 ───
    JntArray  q_integral_;
    int       joint_count_{0};
    int       null_dim_count_{0};
    Jacobian  jacobian_;
    Eigen::MatrixXd prev_nullspace_basis_;
};

} // namespace rocos
