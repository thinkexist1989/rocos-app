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

#include <memory>

namespace rocos {

/// @brief 一维单位区间 [0,1] 速度规划器（无生命周期状态）
///
/// 本类只负责推进 OTG，不持有任何语义状态（不区分 running/paused/stopped）。
/// 所有 Start/Pause/Resume/Stop 仅仅是对 OTG 做不同的参数配置，
/// "暂停"还是"停止"完全由上层调用者根据自身状态来解释。
///
/// 典型用法：
///   profile.Start(v, a, j);     // 配置位置模式，目标 s=1.0
///   while (true) {
///       int rc = profile.Update();   // 推进 OTG，返回 1=Working / 0=Finished / <0=Error
///       double s = profile.position();
///       if (rc <= 0) break;
///   }
///
///   profile.Pause();            // 切换到速度模式，目标 v=0
///   while (profile.Update() > 0) {}
///   // ... 外部决定何时恢复 ...
///   profile.Resume();           // 切回位置模式，目标 s=1.0，从当前 s 继续
class UnitVelocityProfile {
public:
    /// @brief 构造，传入控制周期 [秒]
    explicit UnitVelocityProfile(double dt);
    ~UnitVelocityProfile();

    UnitVelocityProfile(const UnitVelocityProfile&) = delete;
    UnitVelocityProfile& operator=(const UnitVelocityProfile&) = delete;
    UnitVelocityProfile(UnitVelocityProfile&&) noexcept;
    UnitVelocityProfile& operator=(UnitVelocityProfile&&) noexcept;

    // ─── 配置接口（仅做 OTG 参数配置，不存储语义状态）───

    /// @brief 重置到指定初始状态
    void Reset(double position = 0.0, double velocity = 0.0, double acceleration = 0.0);

    /// @brief 启动规划：位置模式，从 s=0 规划到 s=1.0
    /// @return true 成功 / false 限制参数非法
    bool Start(double max_velocity, double max_acceleration, double max_jerk);

    /// @brief 暂停：切换到速度模式，目标速度 0，平滑减速到停
    /// @param max_acceleration  减速阶段最大加速度，必须 > 0
    /// @param max_jerk          减速阶段最大 jerk，必须 > 0
    /// @return true 成功 / false 限制参数非法
    bool Pause(double max_acceleration, double max_jerk);

    /// @brief 继续：切回位置模式，从当前 s 继续规划到 1.0
    void Resume();

    /// @brief 停止：切换到速度模式，目标速度 0，平滑减速到停
    /// @param max_acceleration  减速阶段最大加速度，必须 > 0
    /// @param max_jerk          减速阶段最大 jerk，必须 > 0
    /// @return true 成功 / false 限制参数非法
    bool Stop(double max_acceleration, double max_jerk);

    // ─── 单步推进 ───

    /// @brief 推进 OTG 一步
    /// @return  1 = 仍在运行 (Ruckig::Working)
    ///          0 = 本周期刚好完成 (Ruckig::Finished)
    ///         -1 = OTG 计算错误
    int Update();

    // ─── 纯物理量查询 ───

    double position()     const;
    double velocity()     const;
    double acceleration() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rocos
