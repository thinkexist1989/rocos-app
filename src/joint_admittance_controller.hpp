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

/// @brief 关节空间导纳控制器
///
/// 控制律（在每个控制周期执行）：
///   1. τ_ext = τ_act - τ_grav - τ_offset
///   2. M·q̈_adm + B·q̇_adm = τ_ext   →   q̈_adm = (τ_ext - B·q̇_adm) / M
///   3. 半隐式欧拉积分:  q̇_adm += q̈_adm·dt ,  q_adm += q̇_adm·dt
///   4. q_out = q_des + q_adm
///   5. 以 CSP 模式下发位置指令到硬件
///
/// 注意: 最终下发的始终是关节角度（SetPosition），不是力矩。
class JointImpedanceController : public ControllerInterface {
public:
    JointImpedanceController() = default;
    ~JointImpedanceController() override = default;

    bool Reset() override;
    Result SetHardware(HardwareInterface* hardware) override;
    Result SetModel(ModelInterface* model) override;
    Result GenerateCmd(const Reference& ref_in, JntArray& q_cmd) override;
    Result UpdateCmd(const JntArray& q_cmd) override;

    /// @brief 设置导纳虚拟惯量 M [kg·m²]，需与关节数一致
    Result SetInertia(const JntArray& M);
    /// @brief 设置导纳阻尼系数 B [Nm·s/rad]，需与关节数一致
    Result SetDamping(const JntArray& B);
    /// @brief 设置关节力矩零飘 τ_offset [Nm]，需与关节数一致
    Result SetTorqueOffset(const JntArray& tau_offset);
    /// @brief 设置控制周期 dt [s]（默认 0.001 = 1ms）
    Result SetDt(double dt);

private:
    HardwareInterface* hardware_{nullptr};
    ModelInterface* model_{nullptr};
    bool mode_set_{false};

    // 导纳参数
    JntArray M_;            // 虚拟惯量 [kg·m²]
    JntArray B_;            // 阻尼系数 [Nm·s/rad]
    JntArray tau_offset_;   // 关节力矩零飘 [Nm]
    double dt_{0.001};      // 控制周期 [s]

    // 导纳积分状态
    JntArray q_adm_;        // 导纳位置偏移
    JntArray q_dot_adm_;    // 导纳速度
    bool adm_initialized_{false};

    // 默认值
    static constexpr double kDefaultInertia = 1.0;    // kg·m²
    static constexpr double kDefaultDamping = 20.0;   // Nm·s/rad
    static constexpr double kDefaultDt = 0.001;       // s
    static constexpr int8_t CSP_MODE = 8;             // Cyclic Synchronous Position
};

} // namespace rocos
