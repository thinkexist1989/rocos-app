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

#include "types.hpp"

#include <string>
#include <variant>
#include <vector>

namespace rocos {

/// @brief 虚拟墙响应策略
enum class WallAction {
    HardStop,   ///< 硬停止：到达边界立即报错
    SoftLimit   ///< 软限位：进入减速区后逐步降低速度
};

/// @brief 虚拟墙限制模式
enum class WallMode {
    InsideOnly,   ///< 只允许在墙内运动（禁止外出）
    OutsideOnly   ///< 只允许在墙外运动（禁止进入）
};

// ============================================================================
// 虚拟墙几何基元
// ============================================================================

/// @brief 平面墙：由一个点和法向量定义，法向量指向禁止侧
struct PlaneWall {
    Vector point{0, 0, 0};
    Vector normal{1, 0, 0};        ///< 单位法向量，指向禁止侧
    double warning_distance{0.05}; ///< 减速区宽度 [m]
    WallAction action{WallAction::HardStop};
    std::string name;
};

/// @brief 圆柱墙：由中心线 + 半径定义
struct CylinderWall {
    Vector center{0, 0, 0};
    Vector axis{0, 0, 1};          ///< 中心轴方向（单位向量）
    double radius{1.0};            ///< 半径 [m]
    WallMode mode{WallMode::InsideOnly};
    double warning_distance{0.05}; ///< 减速区宽度 [m]
    WallAction action{WallAction::HardStop};
    std::string name;
};

/// @brief 球墙
struct SphereWall {
    Vector center{0, 0, 0};
    double radius{1.0};            ///< 半径 [m]
    WallMode mode{WallMode::InsideOnly};
    double warning_distance{0.05}; ///< 减速区宽度 [m]
    WallAction action{WallAction::HardStop};
    std::string name;
};

/// @brief 轴对齐立方体墙
struct BoxWall {
    Vector min_corner{0, 0, 0};
    Vector max_corner{1, 1, 1};
    WallMode mode{WallMode::InsideOnly};
    double warning_distance{0.05}; ///< 减速区宽度 [m]
    WallAction action{WallAction::HardStop};
    std::string name;
};

/// @brief 统一墙类型
using WallVariant = std::variant<PlaneWall, CylinderWall, SphereWall, BoxWall>;

// ============================================================================
// 符号距离函数（Signed Distance）
//
// 返回值约定（与 WallMode 无关，已内化）：
//   d < 0 → 在安全区内
//   d = 0 → 恰好在边界上
//   d > 0 → 已穿透墙（违规）
// ============================================================================

double signedDistance(const PlaneWall& wall, const Vector& point);
double signedDistance(const CylinderWall& wall, const Vector& point);
double signedDistance(const SphereWall& wall, const Vector& point);
double signedDistance(const BoxWall& wall, const Vector& point);

/// @brief 统一调度
inline double signedDistance(const WallVariant& wall, const Vector& point) {
    return std::visit(
        [&](const auto& w) { return signedDistance(w, point); }, wall);
}

// ============================================================================
// 禁止侧法向量（Wall Normal / Gradient Direction）
//
// 返回 signedDistance 的梯度方向 ∇d，即随着点移动 d 增大的方向。
// 等价于"指向禁止区的单位向量"。已内化 WallMode。
//
// 与 signedDistance 配合使用：
//   法向速度分量 v_n = dot(P_target - P_current, wallNormal(wall, P_current))
//   v_n > 0 → 正在向禁止侧移动
// ============================================================================

Vector wallNormal(const PlaneWall& wall, const Vector& point);
Vector wallNormal(const CylinderWall& wall, const Vector& point);
Vector wallNormal(const SphereWall& wall, const Vector& point);
Vector wallNormal(const BoxWall& wall, const Vector& point);

/// @brief 统一调度
inline Vector wallNormal(const WallVariant& wall, const Vector& point) {
    return std::visit(
        [&](const auto& w) { return wallNormal(w, point); }, wall);
}

// ============================================================================
// 边界投影（Project Onto Boundary）
//
// 将点投影到墙的几何边界面上（最近点）。
// 纯几何操作，不涉及 WallMode。
// ============================================================================

Vector projectOntoBoundary(const PlaneWall& wall, const Vector& point);
Vector projectOntoBoundary(const CylinderWall& wall, const Vector& point);
Vector projectOntoBoundary(const SphereWall& wall, const Vector& point);
Vector projectOntoBoundary(const BoxWall& wall, const Vector& point);

/// @brief 统一调度
inline Vector projectOntoBoundary(const WallVariant& wall, const Vector& point) {
    return std::visit(
        [&](const auto& w) { return projectOntoBoundary(w, point); }, wall);
}

/// @brief 获取虚拟墙的减速区宽度
inline double getWarningDistance(const WallVariant& wall) {
    return std::visit(
        [](const auto& w) -> double { return w.warning_distance; }, wall);
}

/// @brief 获取虚拟墙的响应策略
inline WallAction getWallAction(const WallVariant& wall) {
    return std::visit(
        [](const auto& w) -> WallAction { return w.action; }, wall);
}

}  // namespace rocos
