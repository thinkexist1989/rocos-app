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

#include "move_null_jog.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace rocos {

namespace {
constexpr double kSpeedEpsilon = 1e-6;

double vecNorm(const JntArray& q) noexcept {
    double s = 0.0;
    for (unsigned int i = 0; i < q.rows(); ++i) s += q(i) * q(i);
    return std::sqrt(s);
}

bool vecIsFinite(const JntArray& q) noexcept {
    for (unsigned int i = 0; i < q.rows(); ++i)
        if (!std::isfinite(q(i))) return false;
    return true;
}
} // namespace

// ============================================================================
// 归一化方向与相似度
// ============================================================================

double MoveNullJog::normalizeDirection(const JntArray& src, JntArray& dst) {
    const double norm = vecNorm(src);
    if (norm < kSpeedEpsilon) return 0.0;

    JntArray dir(src.rows());
    for (unsigned int i = 0; i < src.rows(); ++i) dir(i) = src(i) / norm;
    dst = std::move(dir);
    return norm;
}

double MoveNullJog::directionCosine(const JntArray& a, const JntArray& b) {
    double s = 0.0;
    for (unsigned int i = 0; i < a.rows(); ++i) s += a(i) * b(i);
    return s;
}

// ============================================================================
// 构造 / 析构
// ============================================================================

MoveNullJog::MoveNullJog(double dt, double timeout, ModelInterface* model, double dir_threshold)
    : MotionInterface(model)
    , dt_(dt)
    , timeout_(timeout)
    , dir_threshold_(dir_threshold)
    , speed_otg_(dt) {}

MoveNullJog::~MoveNullJog() = default;

// ============================================================================
// 参数校验
// ============================================================================

Result MoveNullJog::ValidateParameters() const {
    if (!model_) return Result::Fatal;
    if (!std::isfinite(dt_) || dt_ <= 0.0) return Result::ParameterNanOrInf;
    if (!std::isfinite(timeout_) || timeout_ <= 0.0) return Result::ParameterNanOrInf;
    return Result::NoError;
}

// ============================================================================
// 初始化
// ============================================================================

Result MoveNullJog::Reset() {
    std::lock_guard<std::mutex> lock(mtx_);

    if (q_integral_.rows() == 0) {
        return Result::IllegalParameter;
    }

    speed_otg_current_ = 0.0;
    speed_input_.control_interface     = ruckig::ControlInterface::Velocity;
    speed_input_.current_velocity      = {0.0};
    speed_input_.current_acceleration  = {0.0};
    speed_input_.target_velocity       = {target_speed_};
    speed_input_.max_velocity          = {v_max_};
    speed_input_.max_acceleration      = {a_max_};
    speed_input_.max_jerk              = {j_max_};

    intent_direction_    = JntArray(static_cast<unsigned int>(joint_count_));
    target_speed_        = 0.0;
    elapsed_since_feed_  = 0.0;
    state_.store(State::Active);
    has_started_.store(false);

    return Result::NoError;
}

void MoveNullJog::setInitialPosition(const JntArray& q) {
    joint_count_ = static_cast<int>(q.rows());
    q_integral_ = q;
    jacobian_.resize(static_cast<unsigned int>(joint_count_));
}

// ============================================================================
// FeedNullJog — HTTP 线程旁路写入意图
// ============================================================================

Result MoveNullJog::FeedNullJog(const JntArray& intent_direction, double speed) {
    if (state_.load() != State::Active) return Result::PlanError;
    if (!std::isfinite(speed)) return Result::ParameterNanOrInf;
    if (!vecIsFinite(intent_direction)) return Result::ParameterNanOrInf;

    JntArray unit_dir;
    if (normalizeDirection(intent_direction, unit_dir) < kSpeedEpsilon)
        return Result::IllegalParameter;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (state_.load() != State::Active) return Result::PlanError;

        const bool is_first = (vecNorm(intent_direction_) < kSpeedEpsilon);

        if (!is_first && directionCosine(intent_direction_, unit_dir) < dir_threshold_) {
            return Result::PlanError;
        }

        intent_direction_ = std::move(unit_dir);
        target_speed_ = std::abs(speed);
        elapsed_since_feed_ = 0.0;
        has_started_.store(true);
    }
    return Result::NoError;
}

// ============================================================================
// 速度 OTG 配置
// ============================================================================

void MoveNullJog::reconfigureSpeedOtg(double target_speed) {
    speed_input_.control_interface = ruckig::ControlInterface::Velocity;
    speed_input_.target_velocity   = {target_speed};
    speed_input_.max_velocity      = {v_max_};
    speed_input_.max_acceleration  = {a_max_};
    speed_input_.max_jerk          = {j_max_};
}

// ============================================================================
// Update
// ============================================================================

Result MoveNullJog::Update() {
    if (state_.load() == State::Stopped) return Result::PlanFinished;
    if (!has_started_.load()) return Result::NoError;

    double target;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        elapsed_since_feed_ += dt_;
        if (elapsed_since_feed_ > timeout_ && state_.load() == State::Active) {
            state_.store(State::Decelerating);
            target_speed_ = 0.0;
        }
        target = target_speed_;
    }

    reconfigureSpeedOtg(target);
    const auto res = speed_otg_.update(speed_input_, speed_output_);

    if (res == ruckig::Result::Error) {
        state_.store(State::Stopped);
        return Result::PlanError;
    }

    speed_otg_current_ = speed_output_.new_velocity[0];
    speed_output_.pass_to_input(speed_input_);

    if (state_.load() == State::Decelerating &&
        std::abs(speed_otg_current_) < kSpeedEpsilon) {
        state_.store(State::Stopped);
        return Result::PlanFinished;
    }

    return Result::NoError;
}

// ============================================================================
// computeJointVelocity — 零空间投影
// ============================================================================

JntArray MoveNullJog::computeJointVelocity() {
    std::lock_guard<std::mutex> lock(mtx_);
    const double s = speed_otg_current_;
    JntArray q_dot(static_cast<unsigned int>(joint_count_));

    if (model_) {
        Eigen::VectorXd q_dot_raw(joint_count_);
        for (int i = 0; i < joint_count_; ++i)
            q_dot_raw(i) = intent_direction_(i) * s;

        model_->GetJacobian(q_integral_, jacobian_);
        const auto& J = jacobian_.data;

        Eigen::VectorXd v_task = J * q_dot_raw;

        const double lambda = 0.01;
        Eigen::MatrixXd JJt = J * J.transpose();
        JJt.diagonal().array() += lambda * lambda;

        Eigen::VectorXd q_dot_cancel = J.transpose() * JJt.llt().solve(v_task);
        Eigen::VectorXd q_dot_cmd = q_dot_raw - q_dot_cancel;

        // ── 等比例饱和缩放：保留方向，不超单关节限速 ──
        double max_scale = 1.0;
        const double joint_v_limit = v_max_;
        for (int i = 0; i < joint_count_; ++i) {
            const double scale = std::abs(q_dot_cmd(i)) / joint_v_limit;
            if (scale > max_scale) max_scale = scale;
        }
        if (max_scale > 1.0) q_dot_cmd /= max_scale;

        for (unsigned int i = 0; i < q_dot.rows(); ++i)
            q_dot(i) = q_dot_cmd(static_cast<int>(i));
    }
    return q_dot;
}

// ============================================================================
// GenerateRef
// ============================================================================

Result MoveNullJog::GenerateRef(Reference& ref_out) {
    if (!has_started_.load()) {
        ref_out = q_integral_;
        return Result::NoError;
    }

    JntArray q_dot = computeJointVelocity();

    for (unsigned int i = 0; i < q_integral_.rows(); ++i)
        q_integral_(i) += q_dot(i) * dt_;

    ref_out = q_integral_;
    return Result::NoError;
}

// ============================================================================
// Pause / Resume / Stop
// ============================================================================

Result MoveNullJog::Pause()  { return Result::PlanError; }
Result MoveNullJog::Resume() { return Result::PlanError; }

Result MoveNullJog::Stop() {
    state_.store(State::Decelerating);
    std::lock_guard<std::mutex> lock(mtx_);
    target_speed_ = 0.0;
    return Result::NoError;
}

} // namespace rocos
