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

#include "controller_interface.hpp"

namespace rocos {

/// @brief 笛卡尔空间阻抗控制器（工具系下表达阻抗，经 Jacobian 转置映射到关节力矩）
///
/// 控制律（每个周期执行）：
///   1. 获取当前 q, q̇
///   2. FK → 当前笛卡尔位姿 x_cur
///   3. J = GetJacobian(q)
///   4. Δx_base = diff(x_cur, x_des)          ← 6D 误差 twist（基坐标系下，KDL 定义）
///   5. v_ee_base = J · q̇                     ← 末端速度 twist（基坐标系）
///   6. Δx_tool = Ad_{g^{-1}} · Δx_base       ← 误差变换到工具系
///   7. v_ee_cur  = Ad_{g^{-1}} · v_ee_base   ← 速度变换到工具系
///   8. F_cur = K_p·Δx_tool - K_d·v_ee_cur   ← 阻抗力/力矩（工具系）
///   9. F_base = Ad_g · F_cur                 ← 力/力矩变换回基坐标系
///  10. τ_imp = J^T · F_base                  ← Jacobian 转置映射到关节力矩
///  11. τ_raw = τ_imp + τ_grav
///  12. Rate limit: |τ_new - τ_prev| ≤ τ_rate_limit · dt
///  13. Saturation: clamp(τ_new, -τ_max, +τ_max)
///  14. SetTorque(τ_cmd)，CST 模式 (mode=10)
///
/// 阻抗参数在工具系（末端当前坐标系）下定义，使刚度方向对操作者直观。
class CartesianImpedanceController : public ControllerInterface {
public:
    CartesianImpedanceController() = default;
    ~CartesianImpedanceController() override;

    bool Reset() override;
    Result SetReady() override;
    Result SetHardware(HardwareInterface* hardware) override;
    Result SetModel(ModelInterface* model) override;
    Result GenerateCmd(const Reference& ref_in, JntArray& q_cmd) override;
    Result UpdateCmd(const JntArray& q_cmd) override;

    // ---- 笛卡尔阻抗参数 ----
    /// @brief 设置笛卡尔平动刚度 [N/m]
    Result SetTranslationalStiffness(double K);
    /// @brief 设置笛卡尔转动刚度 [Nm/rad]
    Result SetRotationalStiffness(double K);
    /// @brief 设置笛卡尔平动阻尼 [N·s/m]
    Result SetTranslationalDamping(double D);
    /// @brief 设置笛卡尔转动阻尼 [Nm·s/rad]
    Result SetRotationalDamping(double D);

    [[nodiscard]] double GetTranslationalStiffness()  const { return K_p_lin_; }
    [[nodiscard]] double GetRotationalStiffness()      const { return K_p_ang_; }
    [[nodiscard]] double GetTranslationalDamping()     const { return K_d_lin_; }
    [[nodiscard]] double GetRotationalDamping()        const { return K_d_ang_; }

    // ---- 力矩安全参数 ----
    /// @brief 设置力矩变化率限制 [Nm/s]，0 表示不限制，默认 0
    Result SetTorqueRateLimit(double limit);
    /// @brief 设置关节力矩饱和上限 [Nm]，0 表示不限制，默认 0
    Result SetTorqueSaturation(double limit);

private:
    Result ValidateTorqueCommand(const JntArray& tau_cmd);

    HardwareInterface* hardware_{nullptr};
    ModelInterface* model_{nullptr};
    bool mode_set_{false};

    // 笛卡尔位姿
    Frame x_des_;              ///< 期望笛卡尔位姿（每周期由 GenerateCmd 更新）
    bool x_des_valid_{false};  ///< x_des_ 是否已被设置
    /// 当 x_des_ 来自 Frame（笛卡尔运动）时设为 true，
    /// 阻止后续 JntArray（hold_position_）覆盖，避免运动结束后放弃剩余跟踪误差
    bool x_des_from_frame_{false};

    // 笛卡尔阻抗增益（工具系）
    double K_p_lin_{1500.0};    ///< 平动刚度 [N/m]（需足够大以克服关节静摩擦）
    double K_p_ang_{120.0};     ///< 转动刚度 [Nm/rad]
    double K_d_lin_{120.0};     ///< 平动阻尼 [N·s/m]
    double K_d_ang_{15.0};      ///< 转动阻尼 [Nm·s/rad]

    // 力矩安全
    double tau_rate_limit_{200.0};  ///< 力矩变化率限制 [Nm/s]，0=不限制
    double tau_max_{0.0};          ///< 关节力矩饱和值 [Nm]，0=不限制
    JntArray tau_prev_;            ///< 上一周期输出的力矩（用于变化率限制）
    bool tau_prev_valid_{false};   ///< tau_prev_ 是否有效
    double dt_{0.001};             ///< 控制周期 [s]

    static constexpr int8_t CST_MODE = 10;  // Cyclic Synchronous Torque
};

} // namespace rocos
