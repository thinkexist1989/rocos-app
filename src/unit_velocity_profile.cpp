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

#include "unit_velocity_profile.hpp"

#include <ruckig/ruckig.hpp>

#include <algorithm>
#include <cmath>

namespace rocos {

struct UnitVelocityProfile::Impl {
    explicit Impl(double dt) : otg(dt) {}

    // ─── 辅助：位置钳位到 [0, 1] ───
    static double ClampToUnit(double val) {
        return std::clamp(val, 0.0, 1.0);
    }

    // ─── 辅助：限制参数是否合法 ───
    static bool Valid(double v) {
        return std::isfinite(v) && v > 0.0;
    }
    static bool ValidLimits(double v, double a, double j) {
        return Valid(v) && Valid(a) && Valid(j);
    }

    // ─── 辅助：将当前物理状态同步到 OTG 输入 ───
    void SyncStateToInput() {
        input.current_position     = {position};
        input.current_velocity     = {velocity};
        input.current_acceleration = {acceleration};
    }

    // ─── 辅助：将限制参数同步到 OTG 输入 ───
    void SyncLimitsToInput(double v, double a, double j) {
        input.max_velocity     = {v};
        input.max_acceleration = {a};
        input.max_jerk         = {j};
    }

    // ========== 数据成员 ==========

    ruckig::Ruckig<1> otg;
    ruckig::InputParameter<1> input;
    ruckig::OutputParameter<1> output;

    // 当前物理状态
    double position     = 0.0;
    double velocity     = 0.0;
    double acceleration = 0.0;

    // Start 时保存的限制，Resume / Pause / Stop 复用
    double saved_vel_  = 0.0;
    double saved_acc_  = 0.0;
    double saved_jerk_ = 0.0;
};

// ============================================================================
// 构造 / 析构 / 移动
// ============================================================================

UnitVelocityProfile::UnitVelocityProfile(double dt)
    : impl_(std::make_unique<Impl>(dt)) {}

UnitVelocityProfile::~UnitVelocityProfile() = default;

UnitVelocityProfile::UnitVelocityProfile(UnitVelocityProfile&&) noexcept = default;
UnitVelocityProfile& UnitVelocityProfile::operator=(UnitVelocityProfile&&) noexcept = default;

// ============================================================================
// 配置接口 — 只做 OTG 参数配置，不存储语义状态
// ============================================================================

void UnitVelocityProfile::Reset(double position, double velocity, double acceleration) {
    impl_->position     = Impl::ClampToUnit(position);
    impl_->velocity     = velocity;
    impl_->acceleration = acceleration;

    impl_->input  = {};
    impl_->output = {};
    impl_->SyncStateToInput();
}

bool UnitVelocityProfile::Start(double v, double a, double j) {
    if (!Impl::ValidLimits(v, a, j)) return false;

    impl_->saved_vel_  = v;
    impl_->saved_acc_  = a;
    impl_->saved_jerk_ = j;

    // 从 s=0 开始
    impl_->position     = 0.0;
    impl_->velocity     = 0.0;
    impl_->acceleration = 0.0;
    impl_->input  = {};
    impl_->output = {};

    impl_->input.control_interface = ruckig::ControlInterface::Position;
    impl_->SyncStateToInput();
    impl_->input.target_position     = {1.0};
    impl_->input.target_velocity     = {0.0};
    impl_->input.target_acceleration = {0.0};
    impl_->SyncLimitsToInput(v, a, j);

    return true;
}

bool UnitVelocityProfile::Pause(double acc, double jerk) {
    if (!Impl::ValidLimits(impl_->saved_vel_, acc, jerk)) return false;

    impl_->input.control_interface = ruckig::ControlInterface::Velocity;
    impl_->SyncStateToInput();
    impl_->input.target_velocity     = {0.0};
    impl_->input.target_acceleration = {0.0};
    impl_->SyncLimitsToInput(impl_->saved_vel_, acc, jerk);
    return true;
}

void UnitVelocityProfile::Resume() {
    impl_->input.control_interface = ruckig::ControlInterface::Position;
    impl_->SyncStateToInput();                       // 从当前 s 继续，不从 0 开始
    impl_->input.target_position     = {1.0};
    impl_->input.target_velocity     = {0.0};
    impl_->input.target_acceleration = {0.0};
    impl_->SyncLimitsToInput(impl_->saved_vel_, impl_->saved_acc_, impl_->saved_jerk_);
}

bool UnitVelocityProfile::Stop(double acc, double jerk) {
    return Pause(acc, jerk);
}

// ============================================================================
// 单步推进 — 只做数学计算，不做语义判断
// ============================================================================

int UnitVelocityProfile::Update() {
    // 推进 OTG 一步
    const auto result = impl_->otg.update(impl_->input, impl_->output);

    if (result == ruckig::Result::Error) {
        return -1;
    }

    // 从 OTG 输出回读物理量，防御性过滤 NaN
    double raw_pos = impl_->output.new_position[0];
    double raw_vel = impl_->output.new_velocity[0];
    double raw_acc = impl_->output.new_acceleration[0];

    if (std::isnan(raw_pos) || std::isnan(raw_vel) || std::isnan(raw_acc)) {
        return -1;
    }

    impl_->position     = Impl::ClampToUnit(raw_pos);
    impl_->velocity     = raw_vel;
    impl_->acceleration = raw_acc;

    // 将本周期输出同步为下周期输入
    impl_->output.pass_to_input(impl_->input);

    return (result == ruckig::Result::Working) ? 1 : 0;
}

// ============================================================================
// 纯物理量查询
// ============================================================================

double UnitVelocityProfile::position()     const { return impl_->position; }
double UnitVelocityProfile::velocity()     const { return impl_->velocity; }
double UnitVelocityProfile::acceleration() const { return impl_->acceleration; }

}  // namespace rocos
