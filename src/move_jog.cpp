//
// Created by think on 2026/7/7.
//

#include "move_jog.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rocos {

namespace {
// 视为“已归零”的速度阈值：小于此值直接钳零并结束
constexpr double kVelocityEpsilon = 1e-6;

// ─── variant 辅助 ─────────────────────────────────────────────
// 因为 JogVec = std::variant<JntArray, Twist>，两种类型的“逐元素”
// 操作方式不同（JntArray 用 (i) 访问，Twist 有 vel/rot 两个 Vector），
// 这里集中封装，避免逻辑散落到主流程里。

/// @brief 计算方向向量的 L2 范数
double vecNorm(const JogVec &v) noexcept {
    if (std::holds_alternative<JntArray>(v)) {
        const auto &q = std::get<JntArray>(v);
        double s = 0.0;
        for (unsigned int i = 0; i < q.rows(); ++i) s += q(i) * q(i);
        return std::sqrt(s);
    }
    const auto &tw = std::get<Twist>(v);
    double s = 0.0;
    for (int i = 0; i < 3; ++i) s += tw.vel(i) * tw.vel(i) + tw.rot(i) * tw.rot(i);
    return std::sqrt(s);
}

/// @brief 计算两向量点积（要求 variant 类型相同、维度相同）
double vecDot(const JogVec &a, const JogVec &b) noexcept {
    if (std::holds_alternative<JntArray>(a)) {
        const auto &qa = std::get<JntArray>(a);
        const auto &qb = std::get<JntArray>(b);
        if (qa.rows() != qb.rows()) return 0.0;
        double s = 0.0;
        for (unsigned int i = 0; i < qa.rows(); ++i) s += qa(i) * qb(i);
        return s;
    }
    const auto &ta = std::get<Twist>(a);
    const auto &tb = std::get<Twist>(b);
    double s = 0.0;
    for (int i = 0; i < 3; ++i) s += ta.vel(i) * tb.vel(i) + ta.rot(i) * tb.rot(i);
    return s;
}

/// @brief 逐元素向 0 收敛：v <- sign(v) * max(|v| - step, 0)
void shrinkTowardZero(JogVec &v, double step) noexcept {
    auto shrink = [step](double x) {
        const double mag = std::max(std::abs(x) - step, 0.0);
        return std::copysign(mag, x);
    };
    if (std::holds_alternative<JntArray>(v)) {
        auto &q = std::get<JntArray>(v);
        for (unsigned int i = 0; i < q.rows(); ++i) q(i) = shrink(q(i));
        return;
    }
    auto &tw = std::get<Twist>(v);
    for (int i = 0; i < 3; ++i) {
        tw.vel(i) = shrink(tw.vel(i));
        tw.rot(i) = shrink(tw.rot(i));
    }
}

/// @brief 检查所有元素是否有限
bool vecIsFinite(const JogVec &v) noexcept {
    if (std::holds_alternative<JntArray>(v)) {
        const auto &q = std::get<JntArray>(v);
        for (unsigned int i = 0; i < q.rows(); ++i)
            if (!std::isfinite(q(i))) return false;
        return true;
    }
    const auto &tw = std::get<Twist>(v);
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(tw.vel(i)) || !std::isfinite(tw.rot(i))) return false;
    }
    return true;
}
} // namespace

// ============================================================================
// 构造 / 析构
// ============================================================================

