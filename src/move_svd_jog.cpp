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

#include "move_svd_jog.hpp"

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

namespace rocos {

namespace {
constexpr double kSpeedEpsilon = 1e-6;
} // namespace

// ============================================================================
// 构造 / 析构
// ============================================================================

MoveSvdJog::MoveSvdJog(double dt, double timeout, ModelInterface* model, double dir_threshold)
    : MotionInterface(model)
    , dt_(dt)
    , timeout_(timeout)
    , dir_threshold_(dir_threshold)
    , speed_otg_(dt) {}

MoveSvdJog::~MoveSvdJog() = default;

// ============================================================================
// 参数校验
// ============================================================================

Result MoveSvdJog::support() const {
    if (!model_) return Result::Fatal;
    if (!std::isfinite(dt_) || dt_ <= 0.0) return Result::ParameterNanOrInf;
    if (!std::isfinite(timeout_) || timeout_ <= 0.0) return Result::ParameterNanOrInf;
    if (joint_count_ <= 6 && joint_count_ > 0) return Result::IllegalParameter;
    return Result::NoError;
}

// ============================================================================
// 初始化
// ============================================================================

Result MoveSvdJog::Reset() {
    std::lock_guard<std::mutex> lock(mtx_);

    if (q_integral_.rows() == 0) return Result::IllegalParameter;

    joint_count_    = static_cast<int>(q_integral_.rows());
    null_dim_count_ = std::max(0, joint_count_ - 6);
    if (null_dim_count_ == 0) return Result::IllegalParameter;

    speed_otg_current_ = 0.0;
    speed_input_.control_interface     = ruckig::ControlInterface::Velocity;
    speed_input_.current_velocity      = {0.0};
    speed_input_.current_acceleration  = {0.0};
    speed_input_.target_velocity       = {target_speed_};
    speed_input_.max_velocity          = {v_max_};
    speed_input_.max_acceleration      = {a_max_};
    speed_input_.max_jerk              = {j_max_};

    unit_dim_direction_.assign(null_dim_count_, 0.0);
    target_speed_        = 0.0;
    elapsed_since_feed_  = 0.0;
    prev_nullspace_basis_.resize(0, 0);

    state_.store(State::Active);
    has_started_.store(false);

    return Result::NoError;
}

void MoveSvdJog::setInitialPosition(const JntArray& q) {
    joint_count_ = static_cast<int>(q.rows());
    q_integral_ = q;
    jacobian_.resize(static_cast<unsigned int>(joint_count_));
}

// ============================================================================
// FeedSvdJog — 抽象零空间维度速度输入
// ============================================================================

Result MoveSvdJog::FeedSvdJog(const std::vector<double>& dim_speeds) {
    if (state_.load() != State::Active) return Result::PlanError;
    if (dim_speeds.size() != static_cast<size_t>(null_dim_count_))
        return Result::IllegalParameter;

    double norm_sq = 0.0;
    for (double v : dim_speeds) {
        if (!std::isfinite(v)) return Result::ParameterNanOrInf;
        norm_sq += v * v;
    }
    double norm = std::sqrt(norm_sq);
    if (norm < kSpeedEpsilon) return Result::IllegalParameter;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (state_.load() != State::Active) return Result::PlanError;

        for (size_t i = 0; i < dim_speeds.size(); ++i)
            unit_dim_direction_[i] = dim_speeds[i] / norm;

        target_speed_ = norm;
        elapsed_since_feed_ = 0.0;
        has_started_.store(true);
    }
    return Result::NoError;
}

// ============================================================================
// 速度 OTG
// ============================================================================

void MoveSvdJog::reconfigureSpeedOtg(double target_speed) {
    speed_input_.control_interface = ruckig::ControlInterface::Velocity;
    speed_input_.target_velocity   = {target_speed};
    speed_input_.max_velocity      = {v_max_};
    speed_input_.max_acceleration  = {a_max_};
    speed_input_.max_jerk          = {j_max_};
}

Result MoveSvdJog::Update() {
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
// alignNullspaceBasis — 跟踪对齐零空间基，防止 SVD 翻转/乱序
// ============================================================================

void MoveSvdJog::alignNullspaceBasis(Eigen::MatrixXd& current_basis) {
    if (prev_nullspace_basis_.rows() != joint_count_ ||
        prev_nullspace_basis_.cols() != null_dim_count_) {
        prev_nullspace_basis_ = current_basis;
        return;
    }

    Eigen::MatrixXd aligned_basis(joint_count_, null_dim_count_);
    std::vector<bool> used(null_dim_count_, false);

    for (int i = 0; i < null_dim_count_; ++i) {
        double best_dot = -1.0;
        int best_idx = -1;
        double sign = 1.0;

        for (int j = 0; j < null_dim_count_; ++j) {
            if (used[j]) continue;
            double dot_val = prev_nullspace_basis_.col(i).dot(current_basis.col(j));
            double abs_dot = std::abs(dot_val);
            if (abs_dot > best_dot) {
                best_dot = abs_dot;
                best_idx = j;
                sign = (dot_val < 0.0) ? -1.0 : 1.0;
            }
        }

        if (best_idx != -1) {
            aligned_basis.col(i) = sign * current_basis.col(best_idx);
            used[best_idx] = true;
        } else {
            // 兜底：从尚未被分配列中取第一个可用列，避免复用导致降秩
            for (int k = 0; k < null_dim_count_; ++k) {
                if (!used[k]) {
                    aligned_basis.col(i) = current_basis.col(k);
                    used[k] = true;
                    break;
                }
            }
        }
    }

    current_basis = aligned_basis;
    prev_nullspace_basis_ = aligned_basis;
}

// ============================================================================
// computeJointVelocity — SVD 提取基 → 映射意图 → 安全限幅
// ============================================================================

JntArray MoveSvdJog::computeJointVelocity() {
    std::lock_guard<std::mutex> lock(mtx_);
    const double s = speed_otg_current_;
    JntArray q_dot(static_cast<unsigned int>(joint_count_));

    if (model_) {
        model_->GetJacobian(q_integral_, jacobian_);

        Eigen::MatrixXd J = jacobian_.data.topRows(6);

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullV);
        Eigen::MatrixXd V = svd.matrixV();

        Eigen::MatrixXd current_basis(joint_count_, null_dim_count_);
        for (int i = 0; i < null_dim_count_; ++i)
            current_basis.col(i) = V.col(6 + i);

        alignNullspaceBasis(current_basis);

        Eigen::VectorXd q_dot_cmd = Eigen::VectorXd::Zero(joint_count_);
        for (int i = 0; i < null_dim_count_; ++i)
            q_dot_cmd += (unit_dim_direction_[i] * s) * current_basis.col(i);

        // ── 等比例饱和缩放 ──
        double max_scale = 1.0;
        for (int i = 0; i < joint_count_; ++i) {
            double scale = std::abs(q_dot_cmd(i)) / v_max_;
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

Result MoveSvdJog::GenerateRef(Reference& ref_out) {
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

Result MoveSvdJog::Pause()  { return Result::PlanError; }
Result MoveSvdJog::Resume() { return Result::PlanError; }

Result MoveSvdJog::Stop() {
    state_.store(State::Decelerating);
    std::lock_guard<std::mutex> lock(mtx_);
    target_speed_ = 0.0;
    return Result::NoError;
}

} // namespace rocos
