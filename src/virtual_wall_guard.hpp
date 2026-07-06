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
#include "virtual_wall.hpp"

#include <memory>
#include <vector>

namespace rocos {

/// @brief 虚拟墙守护器 — 以装饰器模式包装任意 ControllerInterface
///
/// 基于运动方向判断（非绝对位置）进行虚拟墙检测：
///
///   对每个墙，计算当前距离 d_curr、目标距离 d_target、禁止侧法向量 n：
///     - d_target ≥ 0 且 d_curr ≥ 0 且 v_n > 0  → ❌ 硬拦截
///     - d_target ≥ 0 但 (d_curr < 0 或 v_n ≤ 0) → ✂️ 投影到边界（允许退回/沿墙滑动）
///     - -wd < d_target < 0 且 v_n > 0           → ⚠️ 只缩减法向分量，切向全速
///     - 其余情况                                → ✅ 放行
///
/// 典型用法：
///   auto posCtrl = std::make_unique<PositionController>();
///   auto guard   = std::make_unique<VirtualWallGuard>(
///                      std::move(posCtrl), model.get(), hardware.get());
///   guard->AddWall(PlaneWall{...});
///   executor->SwitchController(guard.get());
class VirtualWallGuard : public ControllerInterface {
public:
    /// @param inner    被装饰的真实控制器（所有权转移）
    /// @param model    用于 FK 计算 TCP 位姿
    /// @param hardware 用于读取当前关节位置，可为 nullptr（此时退化为纯绝对位置检查）
    VirtualWallGuard(std::unique_ptr<ControllerInterface> inner,
                     ModelInterface* model,
                     HardwareInterface* hardware = nullptr);
    ~VirtualWallGuard() override = default;

    // ─── ControllerInterface 接口 ───
    bool Reset() override;
    Result SetHardware(HardwareInterface* hardware) override;
    Result SetModel(ModelInterface* model) override;
    Result GenerateCmd(const Reference& ref_in, JntArray& q_cmd) override;
    Result UpdateCmd(const JntArray& q_cmd) override;

    // ─── 虚拟墙管理 ───
    void AddWall(const WallVariant& wall);
    void ClearWalls();
    size_t GetWallCount() const;

    /// @brief 启用/禁用虚拟墙检测（默认启用）
    void SetEnabled(bool enabled);
    [[nodiscard]] bool IsEnabled() const;

private:
    /// @brief 从 Reference 计算目标 TCP 位置
    Result computeTargetTcp(const Reference& ref_in, Vector& tcp_out) const;

    /// @brief 通过 FK 计算当前 TCP 位置
    Result computeCurrentTcp(Vector& tcp_out) const;

    /// @brief 将修改后的 TCP 位置转回 Reference
    /// @param new_pos 修改后的目标位置
    /// @param original_ref 原始 Reference（JntArray 时需通过 IK 反算）
    Reference targetPosToRef(const Vector& new_pos,
                             const Reference& original_ref) const;

    /// @brief 无当前位姿时的兜底逻辑（仅基于绝对位置检查）
    Result fallbackGenerateCmd(const Reference& ref_in, JntArray& q_cmd);

    std::unique_ptr<ControllerInterface> inner_;
    ModelInterface* model_{nullptr};
    HardwareInterface* hardware_{nullptr};
    std::vector<WallVariant> walls_;
    bool enabled_{true};
};

}  // namespace rocos
