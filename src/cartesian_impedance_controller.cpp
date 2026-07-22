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

#include "cartesian_impedance_controller.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <variant>

namespace rocos {

CartesianImpedanceController::~CartesianImpedanceController() {
    if (hardware_ != nullptr) {
        // 安全兜底：先同步硬件，再将目标位置设为当前位置，切换到 CSP 模式，防止析构时飞车
        hardware_->WaitForSignal();
        hardware_->SetPosition(hardware_->GetPosition());
        hardware_->SetMode(8);
    }
}

// ==========================================================================
// SetReady — 初始化硬件并发送重力补偿力矩
// ==========================================================================

Result CartesianImpedanceController::SetReady() {
    if (hardware_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }

    hardware_->WaitForSignal();

    const JntArray q_act = hardware_->GetPosition();
    const unsigned int n = q_act.rows();

    // 读取控制周期（微秒→秒）
    dt_ = static_cast<double>(hardware_->GetDt()) / 1'000'000.0;

    // 计算重力补偿力矩 τ_grav = InverseDynamics(q_act, 0, 0, ∅)
    JntArray tau_grav(n);
    if (model_ != nullptr) {
        JntArray zero(n);
        for (unsigned int i = 0; i < n; ++i) {
            zero(i) = 0.0;
        }
        Wrenches f_ext;
        Result res = model_->InverseDynamics(q_act, zero, zero, f_ext, tau_grav);
        if (res != Result::NoError) {
            return res;
        }
    } else {
        for (unsigned int i = 0; i < n; ++i) {
            tau_grav(i) = 0.0;
        }
    }

    // 将当前实际关节位置作为零空间期望位置的初始值
    q_nullspace_des_ = q_act;
    q_nullspace_valid_ = true;
    q_nullspace_user_set_ = false;  // 系统自动设置，允许 GenerateCmd 后续更新

    // 初始化上一周期力矩为重力补偿值，使变化率限制从安全起点开始
    tau_prev_ = tau_grav;
    tau_prev_valid_ = true;

    // 下发重力补偿力矩作为初始指令，切换到 CST 模式
    hardware_->SetTorque(tau_grav);
    hardware_->SetMode(CST_MODE);
    mode_set_ = true;
    return Result::NoError;
}

// ==========================================================================
// 基础接口
// ==========================================================================

bool CartesianImpedanceController::Reset() {
    mode_set_ = false;
    x_des_valid_ = false;
    q_nullspace_valid_ = false;
    q_nullspace_user_set_ = false;
    tau_prev_valid_ = false;
    return true;
}

Result CartesianImpedanceController::SetHardware(HardwareInterface* hardware) {
    if (hardware == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    hardware_ = hardware;
    return Result::NoError;
}

Result CartesianImpedanceController::SetModel(ModelInterface* model) {
    if (model == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    model_ = model;
    return Result::NoError;
}

// ==========================================================================
// GenerateCmd — 将 Reference 转为期望笛卡尔位姿 x_des_
// ==========================================================================

Result CartesianImpedanceController::GenerateCmd(const Reference& ref_in,
                                                  JntArray& q_cmd) {
    // 分支1: 关节空间参考值 — 通过 FK 转为笛卡尔位姿
    if (auto* q_ref = std::get_if<JntArray>(&ref_in)) {
        if (q_ref->rows() == 0) {
            return Result::MoveInput;
        }

        for (unsigned int i = 0; i < q_ref->rows(); ++i) {
            if (!std::isfinite((*q_ref)(i))) {
                return Result::ParameterNanOrInf;
            }
        }

        if (model_ == nullptr) {
            return Result::ParameterPointerEqualsNullptr;
        }

        Frame x_fk;
        Result fk_res = model_->ForwardKinematics(*q_ref, x_fk);
        if (fk_res != Result::NoError) {
            return fk_res;
        }

        x_des_ = x_fk;
        x_des_valid_ = true;

        // 自动更新零空间期望位置为参考关节角（仅当用户未显式设置时）
        if (!q_nullspace_user_set_) {
            q_nullspace_des_ = *q_ref;
            q_nullspace_valid_ = true;
        }

        q_cmd = *q_ref;
        return Result::NoError;
    }

    // 分支2: 笛卡尔空间参考值 — 直接存储
    if (auto* frame_ref = std::get_if<Frame>(&ref_in)) {
        x_des_ = *frame_ref;
        x_des_valid_ = true;

        // 通过 IK 将期望位姿转为关节角
        if (hardware_ != nullptr && model_ != nullptr) {
            JntArray q_curr = hardware_->GetPosition();
            if (q_curr.rows() > 0) {
                JntArray q_out;
                q_out.resize(q_curr.rows());
                Result ik_res = model_->InverseKinematics(q_curr, *frame_ref, q_out);
                if (ik_res == Result::NoError) {
                    q_cmd = q_out;

                    // 自动将 IK 解作为零空间期望位置（仅当用户未显式设置时）
                    if (!q_nullspace_user_set_) {
                        q_nullspace_des_ = q_out;
                        q_nullspace_valid_ = true;
                    }
                } else {
                    // IK 失败时保持当前关节角
                    q_cmd = q_curr;
                }
            }
        }
        return Result::NoError;
    }

    return Result::MoveUnknown;
}

// ==========================================================================
// UpdateCmd — 笛卡尔空间阻抗控制 + 零空间阻抗 + 力矩安全保护
//
//   核心公式：
//     Δx     = diff(x_cur, x_des)          ← 6D 位姿误差（工具系）
//     v_cur  = R_cur^T · (J · q̇_act)       ← 末端速度（工具系）
//     F_cur  = K_p·Δx - K_d·v_cur          ← 阻抗力/力矩（工具系）
//     F_base = R_cur · F_cur               ← 转回基坐标系
//     τ_imp  = J^T · F_base                ← Jacobian 转置映射到关节力矩
//     τ_null = N · (K_p_null·Δq - K_d_null·q̇)  ← 零空间阻抗
//     τ_raw  = τ_imp + τ_null + τ_grav
//     经过变化率限制和饱和后下发
// ==========================================================================

Result CartesianImpedanceController::UpdateCmd(const JntArray& q_cmd) {
    (void)q_cmd;  // 笛卡尔阻抗控制器使用内部 x_des_，不使用 q_cmd

    if (hardware_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    if (model_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    if (!x_des_valid_) {
        return Result::MoveInput;  // 尚未收到期望位姿
    }

    const unsigned int n = hardware_->GetPosition().rows();
    if (n == 0) {
        return Result::JointStateError;
    }

    // ---- 读取实际关节状态 ----
    JntArray q_act = hardware_->GetPosition();
    JntArray q_dot_act = hardware_->GetVelocity();

    if (q_act.rows() != n) {
        return Result::JointStateError;
    }

    const bool has_velocity = (q_dot_act.rows() == n);

    // ---- 1. FK → 当前笛卡尔位姿 x_cur ----
    Frame x_cur;
    Result fk_res = model_->ForwardKinematics(q_act, x_cur);
    if (fk_res != Result::NoError) {
        return fk_res;
    }

    // ---- 2. Jacobian J(q) (6×n) ----
    Jacobian J_out;
    Result jac_res = model_->GetJacobian(q_act, J_out);
    if (jac_res != Result::NoError) {
        return jac_res;
    }

    // ---- 3. 6D 位姿误差 Δx = diff(x_cur, x_des)（工具系下表达） ----
    Twist x_err = KDL::diff(x_cur, x_des_);

    // ---- 4. 末端速度 v_base = J · q̇（基坐标系） ----
    Twist v_base;
    if (has_velocity) {
        Eigen::VectorXd qd(n);
        for (unsigned int i = 0; i < n; ++i) {
            qd(i) = q_dot_act(i);
        }
        Eigen::Matrix<double, 6, 1> v_ee = J_out.data * qd;
        v_base.vel = Vector(v_ee(0), v_ee(1), v_ee(2));
        v_base.rot = Vector(v_ee(3), v_ee(4), v_ee(5));
    }

    // ---- 5. 速度变换到工具系：v_cur = R_cur^T · v_base ----
    Twist v_cur;
    if (has_velocity) {
        const KDL::Rotation& R_cur = x_cur.M;
        v_cur.vel = R_cur.Inverse() * v_base.vel;
        v_cur.rot = R_cur.Inverse() * v_base.rot;
    }

    // ---- 6. 阻抗力/力矩（工具系下） ----
    Wrench F_cur;
    F_cur.force  = Vector(
        K_p_lin_ * x_err.vel.x() - K_d_lin_ * v_cur.vel.x(),
        K_p_lin_ * x_err.vel.y() - K_d_lin_ * v_cur.vel.y(),
        K_p_lin_ * x_err.vel.z() - K_d_lin_ * v_cur.vel.z());
    F_cur.torque = Vector(
        K_p_ang_ * x_err.rot.x() - K_d_ang_ * v_cur.rot.x(),
        K_p_ang_ * x_err.rot.y() - K_d_ang_ * v_cur.rot.y(),
        K_p_ang_ * x_err.rot.z() - K_d_ang_ * v_cur.rot.z());

    // ---- 7. 力/力矩变换到基坐标系：F_base = R_cur · F_cur ----
    const KDL::Rotation& R_cur = x_cur.M;
    Wrench F_base;
    F_base.force  = R_cur * F_cur.force;
    F_base.torque = R_cur * F_cur.torque;

    // ---- 8. Jacobian 转置映射到关节力矩：τ_imp = J^T · F_base ----
    Eigen::Matrix<double, 6, 1> F_vec;
    F_vec(0) = F_base.force.x();  F_vec(1) = F_base.force.y();  F_vec(2) = F_base.force.z();
    F_vec(3) = F_base.torque.x(); F_vec(4) = F_base.torque.y(); F_vec(5) = F_base.torque.z();

    Eigen::VectorXd tau_imp_eig = J_out.data.transpose() * F_vec;

    // ---- 9. 零空间阻抗 τ_null = N · (K_p_null·Δq - K_d_null·q̇) ----
    //        N = I - J#·J  是阻尼伪逆零空间投影矩阵
    JntArray tau_null(n);
    {
        for (unsigned int i = 0; i < n; ++i) {
            tau_null(i) = 0.0;
        }
    }

    if (q_nullspace_valid_ && q_nullspace_des_.rows() == n) {
        // 构建零空间投影矩阵 N = I - J^T·(J·J^T + λ²I)^(-1)·J
        const auto& J = J_out.data;  // 6×n
        const double lambda = kNullspaceDampingLambda;

        Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
        JJt.diagonal().array() += lambda * lambda;

        // 阻尼伪逆: J# = J^T · (J·J^T + λ²I)^(-1)   (n×6)
        Eigen::MatrixXd J_pinv = J.transpose() * JJt.llt().solve(Eigen::Matrix<double, 6, 6>::Identity());

        // 零空间投影: N = I - J# · J   (n×n)
        Eigen::MatrixXd N_mat = Eigen::MatrixXd::Identity(n, n) - J_pinv * J;

        // 关节空间零空间阻抗: τ_null_raw = K_p_null·(q_des - q) - K_d_null·q̇
        Eigen::VectorXd tau_null_raw(n);
        for (unsigned int i = 0; i < n; ++i) {
            const double pos_err = q_nullspace_des_(i) - q_act(i);
            const double vel_damp = has_velocity ? K_d_null_ * q_dot_act(i) : 0.0;
            tau_null_raw(i) = K_p_null_ * pos_err - vel_damp;
        }

        // 投影到零空间
        Eigen::VectorXd tau_null_eig = N_mat * tau_null_raw;
        for (unsigned int i = 0; i < n; ++i) {
            tau_null(i) = tau_null_eig(i);
        }
    }

    // ---- 10. 重力补偿：τ_grav = InverseDynamics(q, 0, 0, ∅) ----
    JntArray tau_grav(n);
    {
        JntArray zero(n);
        for (unsigned int i = 0; i < n; ++i) {
            zero(i) = 0.0;
        }
        Wrenches f_ext;
        Result id_res = model_->InverseDynamics(q_act, zero, zero, f_ext, tau_grav);
        if (id_res != Result::NoError) {
            return id_res;
        }
    }

    // ---- 11. 合成原始力矩 τ_raw = τ_imp + τ_null + τ_grav ----
    JntArray tau_raw(n);
    for (unsigned int i = 0; i < n; ++i) {
        tau_raw(i) = tau_imp_eig(i) + tau_null(i) + tau_grav(i);
    }

    // ---- 12. 力矩变化率限制 ----
    JntArray tau(n);
    if (tau_rate_limit_ > 0.0 && tau_prev_valid_ && tau_prev_.rows() == n) {
        const double max_delta = tau_rate_limit_ * dt_;
        for (unsigned int i = 0; i < n; ++i) {
            double delta = tau_raw(i) - tau_prev_(i);
            delta = std::max(-max_delta, std::min(max_delta, delta));
            tau(i) = tau_prev_(i) + delta;
        }
    } else {
        tau = tau_raw;
    }

    // ---- 13. 力矩饱和 ----
    if (tau_max_ > 0.0) {
        for (unsigned int i = 0; i < n; ++i) {
            tau(i) = std::max(-tau_max_, std::min(tau_max_, tau(i)));
        }
    }

    // ---- 14. 保存当前力矩供下一周期变化率限制 ----
    tau_prev_ = tau;
    tau_prev_valid_ = true;

    // ---- 15. 下发力矩指令（CST 模式） ----
    if (!mode_set_) {
        hardware_->SetMode(CST_MODE);
        mode_set_ = true;
    }

    hardware_->SetTorque(tau);
    return Result::NoError;
}

// ==========================================================================
// 笛卡尔阻抗参数
// ==========================================================================

Result CartesianImpedanceController::SetTranslationalStiffness(double K) {
    if (!std::isfinite(K) || K < 0.0) {
        return Result::ParameterNanOrInf;
    }
    K_p_lin_ = K;
    return Result::NoError;
}

Result CartesianImpedanceController::SetRotationalStiffness(double K) {
    if (!std::isfinite(K) || K < 0.0) {
        return Result::ParameterNanOrInf;
    }
    K_p_ang_ = K;
    return Result::NoError;
}

Result CartesianImpedanceController::SetTranslationalDamping(double D) {
    if (!std::isfinite(D) || D < 0.0) {
        return Result::ParameterNanOrInf;
    }
    K_d_lin_ = D;
    return Result::NoError;
}

Result CartesianImpedanceController::SetRotationalDamping(double D) {
    if (!std::isfinite(D) || D < 0.0) {
        return Result::ParameterNanOrInf;
    }
    K_d_ang_ = D;
    return Result::NoError;
}

// ==========================================================================
// 零空间阻抗参数
// ==========================================================================

Result CartesianImpedanceController::SetNullspaceStiffness(double K) {
    if (!std::isfinite(K) || K < 0.0) {
        return Result::ParameterNanOrInf;
    }
    K_p_null_ = K;
    return Result::NoError;
}

Result CartesianImpedanceController::SetNullspaceDamping(double D) {
    if (!std::isfinite(D) || D < 0.0) {
        return Result::ParameterNanOrInf;
    }
    K_d_null_ = D;
    return Result::NoError;
}

Result CartesianImpedanceController::SetNullspaceReference(const JntArray& q_nullspace) {
    if (q_nullspace.rows() == 0) {
        return Result::MoveInput;
    }
    for (unsigned int i = 0; i < q_nullspace.rows(); ++i) {
        if (!std::isfinite(q_nullspace(i))) {
            return Result::ParameterNanOrInf;
        }
    }
    q_nullspace_des_ = q_nullspace;
    q_nullspace_valid_ = true;
    q_nullspace_user_set_ = true;  // 用户显式设置，禁止 GenerateCmd 覆盖
    return Result::NoError;
}

// ==========================================================================
// 力矩安全参数
// ==========================================================================

Result CartesianImpedanceController::SetTorqueRateLimit(double limit) {
    if (!std::isfinite(limit) || limit < 0.0) {
        return Result::ParameterNanOrInf;
    }
    tau_rate_limit_ = limit;
    return Result::NoError;
}

Result CartesianImpedanceController::SetTorqueSaturation(double limit) {
    if (!std::isfinite(limit) || limit < 0.0) {
        return Result::ParameterNanOrInf;
    }
    tau_max_ = limit;
    return Result::NoError;
}

} // namespace rocos
