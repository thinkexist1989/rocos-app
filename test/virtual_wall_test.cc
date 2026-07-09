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

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <cmath>
#include <limits>
#include <memory>
#include <variant>

#include <test/doctest.h>

#include "src/virtual_wall.hpp"
#include "src/virtual_wall_guard.hpp"
#include "src/position_controller.hpp"

// ==========================================================================
// Fake 实现
// ==========================================================================

namespace {

class FakeHardware : public rocos::HardwareInterface {
public:
    rocos::JntArray fake_position_;
    rocos::JntArray last_set_position_;
    int8_t last_set_mode_{-1};
    int set_mode_count_{0};

    rocos::JntArray GetPosition() override { return fake_position_; }
    rocos::JntArray GetVelocity() override { return {}; }
    rocos::JntArray GetTorque() override { return {}; }
    rocos::JntArray GetLoadTorque() override { return {}; }
    void SetPosition(const rocos::JntArray& q) override { last_set_position_ = q; }
    void SetVelocity(const rocos::JntArray&) override {}
    void SetTorque(const rocos::JntArray&) override {}
    void SetMode(int8_t mode) override {
        last_set_mode_ = mode;
        ++set_mode_count_;
    }
    void SetEnabled() override {}
    void SetDisabled() override {}

    double GetJointPosition(int32_t) override { return 0.0; }
    double GetJointVelocity(int32_t) override { return 0.0; }
    double GetJointTorque(int32_t) override { return 0.0; }
    double GetJointLoadTorque(int32_t) override { return 0.0; }
    void SetJointPosition(int32_t, double) override {}
    void SetJointVelocity(int32_t, double) override {}
    void SetJointTorque(int32_t, double) override {}
    void SetJointMode(int32_t, int8_t) override {}
    void SetJointEnabled(int32_t) override {}
    void SetJointDisabled(int32_t) override {}
    std::string getJointName(int32_t) override { return ""; }

    rocos::Wrench GetWrench() override { return rocos::Wrench::Zero(); }

    bool GetDigitalInput(int, int) override { return false; }
    void SetDigitalOutput(int, int, bool) override {}
    double GetAnalogInput(int, int) override { return 0.0; }
    void SetAnalogOutput(int, int, double) override {}

    bool Reset() override { return true; }
};

/// @brief FakeFKModel: FK 将前 3 个关节直映射为 XYZ
///        IK 反之，便于测试 Cartesian ↔ Joint 互转
class FakeFKModel : public rocos::ModelInterface {
public:
    bool fk_should_fail_{false};
    bool ik_should_fail_{false};

    rocos::Result GetJacobian(const rocos::JntArray &q, rocos::Jacobian &J_out) override {
        return rocos::Result::NoError;
    }

    rocos::Result ForwardKinematics(const rocos::JntArray& q_in,
                                    rocos::Frame& p_out) override {
        if (fk_should_fail_) return rocos::Result::FkCalcFail;
        if (q_in.rows() >= 3)
            p_out.p = rocos::Vector(q_in(0), q_in(1), q_in(2));
        else if (q_in.rows() == 2)
            p_out.p = rocos::Vector(q_in(0), q_in(1), 0.0);
        else if (q_in.rows() == 1)
            p_out.p = rocos::Vector(q_in(0), 0.0, 0.0);
        else
            p_out.p = rocos::Vector(0.0, 0.0, 0.0);
        return rocos::Result::NoError;
    }

    rocos::Result InverseKinematics(const rocos::JntArray& q_in,
                                     const rocos::Frame& p_in,
                                     rocos::JntArray& q_out) override {
        if (ik_should_fail_) return rocos::Result::IkCalcFail;
        // 将 TCP 的 XYZ 映射回前 3 个关节
        q_out.resize(q_in.rows());
        if (q_in.rows() >= 1) q_out(0) = p_in.p.x();
        if (q_in.rows() >= 2) q_out(1) = p_in.p.y();
        if (q_in.rows() >= 3) q_out(2) = p_in.p.z();
        for (unsigned int i = 3; i < q_in.rows(); ++i)
            q_out(i) = q_in(i);
        return rocos::Result::NoError;
    }

    rocos::Result ForwardDynamics(const rocos::JntArray&,
                                   const rocos::JntArray&,
                                   const rocos::JntArray&,
                                   const rocos::Wrenches&,
                                   rocos::JntArray&) override {
        return rocos::Result::NoError;
    }

    rocos::Result InverseDynamics(const rocos::JntArray&,
                                   const rocos::JntArray&,
                                   const rocos::JntArray&,
                                   const rocos::Wrenches&,
                                   rocos::JntArray&) override {
        return rocos::Result::NoError;
    }
};

class FakeController : public rocos::ControllerInterface {
public:
    rocos::Reference last_ref_;
    rocos::JntArray   last_q_cmd_;
    int generate_count_{0};
    int update_count_{0};

    bool Reset() override { generate_count_ = 0; update_count_ = 0; return true; }
    rocos::Result SetHardware(rocos::HardwareInterface*) override { return rocos::Result::NoError; }
    rocos::Result SetModel(rocos::ModelInterface*) override { return rocos::Result::NoError; }

    rocos::Result GenerateCmd(const rocos::Reference& ref_in,
                               rocos::JntArray& q_cmd) override {
        ++generate_count_;
        last_ref_ = ref_in;
        if (auto* jnt = std::get_if<rocos::JntArray>(&ref_in)) {
            q_cmd = *jnt;
            last_q_cmd_ = *jnt;
        } else if (auto* frame = std::get_if<rocos::Frame>(&ref_in)) {
            q_cmd.resize(3);
            q_cmd(0) = frame->p.x();
            q_cmd(1) = frame->p.y();
            q_cmd(2) = frame->p.z();
            last_q_cmd_ = q_cmd;
        }
        return rocos::Result::NoError;
    }

    rocos::Result UpdateCmd(const rocos::JntArray& q_cmd) override {
        ++update_count_;
        last_q_cmd_ = q_cmd;
        return rocos::Result::NoError;
    }
};

rocos::JntArray makeJnt(unsigned int n, double offset = 0.0) {
    rocos::JntArray q(n);
    for (unsigned int i = 0; i < n; ++i)
        q(i) = static_cast<double>(i + 1) + offset;
    return q;
}

/// 创建一个 6 关节位置，前 3 个映射到给定 XYZ
rocos::JntArray makeJntAt(double x, double y, double z, unsigned int n = 6) {
    rocos::JntArray q(n);
    q(0) = x; q(1) = y; q(2) = z;
    for (unsigned int i = 3; i < n; ++i) q(i) = 0.0;
    return q;
}

}  // namespace

// ============================================================================
// 1. signedDistance — 纯数学测试（不变）
// ============================================================================

TEST_CASE("signedDistance - PlaneWall") {
    rocos::PlaneWall wall;
    wall.point  = rocos::Vector(0, 0, 0);
    wall.normal = rocos::Vector(1, 0, 0);

    SUBCASE("安全侧 d < 0") {
        CHECK(rocos::signedDistance(wall, rocos::Vector(-1.0, 0, 0)) < 0.0);
    }
    SUBCASE("边界上 d = 0") {
        CHECK(rocos::signedDistance(wall, rocos::Vector(0, 0, 0)) == doctest::Approx(0.0));
    }
    SUBCASE("禁止侧 d > 0") {
        CHECK(rocos::signedDistance(wall, rocos::Vector(1.0, 0, 0)) > 0.0);
    }
}

TEST_CASE("signedDistance - CylinderWall mode") {
    rocos::CylinderWall wall;
    wall.center = rocos::Vector(0, 0, 0);
    wall.axis   = rocos::Vector(0, 0, 1);
    wall.radius = 1.0;

    SUBCASE("InsideOnly: 圆柱内 d<0, 外 d>0") {
        wall.mode = rocos::WallMode::InsideOnly;
        CHECK(rocos::signedDistance(wall, rocos::Vector(0, 0, 0)) < 0.0);
        CHECK(rocos::signedDistance(wall, rocos::Vector(2.0, 0, 0)) > 0.0);
    }
    SUBCASE("OutsideOnly: 圆柱外 d<0, 内 d>0") {
        wall.mode = rocos::WallMode::OutsideOnly;
        wall.radius = 2.0;
        CHECK(rocos::signedDistance(wall, rocos::Vector(5.0, 0, 0)) < 0.0);
        CHECK(rocos::signedDistance(wall, rocos::Vector(0, 0, 0)) > 0.0);
    }
}

TEST_CASE("signedDistance - SphereWall") {
    rocos::SphereWall wall;
    wall.center = rocos::Vector(0, 0, 0);
    wall.radius = 1.0;
    wall.mode   = rocos::WallMode::InsideOnly;

    CHECK(rocos::signedDistance(wall, rocos::Vector(0, 0, 0)) < 0.0);
    CHECK(rocos::signedDistance(wall, rocos::Vector(2.0, 0, 0)) > 0.0);
}

TEST_CASE("signedDistance - BoxWall") {
    rocos::BoxWall wall;
    wall.min_corner = rocos::Vector(0, 0, 0);
    wall.max_corner = rocos::Vector(2, 2, 2);
    wall.mode       = rocos::WallMode::InsideOnly;

    CHECK(rocos::signedDistance(wall, rocos::Vector(1, 1, 1)) < 0.0);
    CHECK(rocos::signedDistance(wall, rocos::Vector(3, 1, 1)) > 0.0);
}

TEST_CASE("signedDistance - WallVariant 统一调度") {
    rocos::WallVariant wall = rocos::PlaneWall{
        rocos::Vector(1, 0, 0), rocos::Vector(1, 0, 0)};
    CHECK(rocos::signedDistance(wall, rocos::Vector(0, 0, 0)) < 0.0);
    CHECK(rocos::signedDistance(wall, rocos::Vector(2, 0, 0)) > 0.0);
}

// ============================================================================
// 2. wallNormal — 禁止侧法向量（内化 WallMode）
// ============================================================================

TEST_CASE("wallNormal - PlaneWall") {
    rocos::PlaneWall wall;
    wall.point  = rocos::Vector(5, 0, 0);
    wall.normal = rocos::Vector(1, 0, 0);  // +X 为禁止侧

    auto n = rocos::wallNormal(wall, rocos::Vector(1, 2, 3));
    CHECK(n.x() == doctest::Approx(1.0));
    CHECK(n.y() == doctest::Approx(0.0));
}

TEST_CASE("wallNormal - CylinderWall InsideOnly → 朝外") {
    rocos::CylinderWall wall;
    wall.center = rocos::Vector(0, 0, 0);
    wall.axis   = rocos::Vector(0, 0, 1);
    wall.radius = 1.0;
    wall.mode   = rocos::WallMode::InsideOnly;  // 禁止外出 → 法向量朝外

    // 在 X 轴上，法向量应朝 +X
    auto n = rocos::wallNormal(wall, rocos::Vector(0.5, 0, 0));
    CHECK(n.x() > 0.0);
    CHECK(std::abs(n.y()) < 1e-9);
}

TEST_CASE("wallNormal - CylinderWall OutsideOnly → 朝内") {
    rocos::CylinderWall wall;
    wall.center = rocos::Vector(0, 0, 0);
    wall.axis   = rocos::Vector(0, 0, 1);
    wall.radius = 2.0;
    wall.mode   = rocos::WallMode::OutsideOnly;  // 禁止进入 → 法向量朝内

    // 在 X 轴上圆柱外远点，法向量应朝内（-X）
    auto n = rocos::wallNormal(wall, rocos::Vector(5.0, 0, 0));
    CHECK(n.x() < 0.0);  // 指向圆心
}

TEST_CASE("wallNormal - SphereWall InsideOnly → 朝外") {
    rocos::SphereWall wall;
    wall.center = rocos::Vector(0, 0, 0);
    wall.radius = 1.0;
    wall.mode   = rocos::WallMode::InsideOnly;

    auto n = rocos::wallNormal(wall, rocos::Vector(0.5, 0, 0));
    CHECK(n.x() > 0.0);  // 球内 → 朝外 = 禁止外出
}

TEST_CASE("wallNormal - BoxWall InsideOnly") {
    rocos::BoxWall wall;
    wall.min_corner = rocos::Vector(0, 0, 0);
    wall.max_corner = rocos::Vector(2, 2, 2);
    wall.mode       = rocos::WallMode::InsideOnly;

    // 在立方体内靠近 +X 面 → 法向量应朝 +X（禁止方向=朝外）
    auto n = rocos::wallNormal(wall, rocos::Vector(1.9, 1, 1));
    CHECK(n.x() > 0.0);
}

// ============================================================================
// 3. projectOntoBoundary — 投影到几何边界
// ============================================================================

TEST_CASE("projectOntoBoundary - PlaneWall") {
    rocos::PlaneWall wall;
    wall.point  = rocos::Vector(5, 0, 0);
    wall.normal = rocos::Vector(1, 0, 0);

    // 点 (10, 2, 3) → 投影到 x=5
    auto proj = rocos::projectOntoBoundary(wall, rocos::Vector(10, 2, 3));
    CHECK(proj.x() == doctest::Approx(5.0));
    CHECK(proj.y() == doctest::Approx(2.0));
    CHECK(proj.z() == doctest::Approx(3.0));
}

TEST_CASE("projectOntoBoundary - CylinderWall") {
    rocos::CylinderWall wall;
    wall.center = rocos::Vector(0, 0, 0);
    wall.axis   = rocos::Vector(0, 0, 1);
    wall.radius = 1.0;

    // 点 (2, 0, 0) → 投影到 (1, 0, 0)
    auto proj = rocos::projectOntoBoundary(wall, rocos::Vector(2, 0, 0));
    CHECK(proj.x() == doctest::Approx(1.0));
    CHECK(proj.y() == doctest::Approx(0.0));
    CHECK(proj.z() == doctest::Approx(0.0));
}

TEST_CASE("projectOntoBoundary - SphereWall") {
    rocos::SphereWall wall;
    wall.center = rocos::Vector(0, 0, 0);
    wall.radius = 2.0;

    auto proj = rocos::projectOntoBoundary(wall, rocos::Vector(4, 0, 0));
    CHECK(proj.x() == doctest::Approx(2.0));
}

TEST_CASE("projectOntoBoundary - BoxWall 外 → clamp 到面") {
    rocos::BoxWall wall;
    wall.min_corner = rocos::Vector(0, 0, 0);
    wall.max_corner = rocos::Vector(2, 2, 2);

    auto proj = rocos::projectOntoBoundary(wall, rocos::Vector(5, 1, 1));
    CHECK(proj.x() == doctest::Approx(2.0));
    CHECK(proj.y() == doctest::Approx(1.0));
}

TEST_CASE("projectOntoBoundary - BoxWall 内 → 推到最近面") {
    rocos::BoxWall wall;
    wall.min_corner = rocos::Vector(0, 0, 0);
    wall.max_corner = rocos::Vector(2, 2, 2);

    // 内点 (1.9, 1, 1) → 最近面是 x=2
    auto proj = rocos::projectOntoBoundary(wall, rocos::Vector(1.9, 1, 1));
    CHECK(proj.x() == doctest::Approx(2.0));
}

// ============================================================================
// 4. VirtualWallGuard — 基于运动方向的检测
// ============================================================================

TEST_CASE("VW - 无墙透传") {
    FakeFKModel model;
    FakeHardware hw;
    hw.fake_position_ = makeJnt(6);

    rocos::VirtualWallGuard guard(std::make_unique<FakeController>(), &model, &hw);

    auto ref = rocos::Reference{makeJntAt(10, 2, 3)};
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
}

TEST_CASE("VW - 安全区全速放行") {
    FakeFKModel model;
    FakeHardware hw;
    hw.fake_position_ = makeJntAt(1, 1, 1);   // current TCP (1,1,1)

    rocos::VirtualWallGuard guard(std::make_unique<FakeController>(), &model, &hw);

    rocos::PlaneWall wall;
    wall.point  = rocos::Vector(5, 0, 0);
    wall.normal = rocos::Vector(1, 0, 0);      // x>5 禁止
    guard.AddWall(wall);

    // 目标 (3, 2, 3) 在安全区 → 全速
    auto ref = rocos::Reference{makeJntAt(3, 2, 3)};
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
    // 未修改
    CHECK(q_cmd(0) == doctest::Approx(3.0));
}

TEST_CASE("VW - 从安全区冲向墙外 → 投影到边界（不硬拦截）") {
    // d_curr < 0, d_target ≥ 0 → 投影，不拦截
    FakeFKModel model;
    FakeHardware hw;
    hw.fake_position_ = makeJntAt(1, 1, 1);   // current TCP (1,1,1), safe

    auto fakeCtrl = std::make_unique<FakeController>();
    auto* fPtr = fakeCtrl.get();

    rocos::VirtualWallGuard guard(std::move(fakeCtrl), &model, &hw);

    rocos::PlaneWall wall;
    wall.point  = rocos::Vector(5, 0, 0);
    wall.normal = rocos::Vector(1, 0, 0);      // x>5 禁止
    guard.AddWall(wall);

    // 目标 (10, 2, 3) → d_target > 0, 但 d_curr < 0 → 投影而非拦截
    auto ref = rocos::Reference{makeJntAt(10, 2, 3)};
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
    CHECK(fPtr->generate_count_ == 1);

    // 投影后 TCP.x 应为 5
    CHECK(q_cmd(0) == doctest::Approx(5.0));
    CHECK(q_cmd(1) == doctest::Approx(2.0));    // y 不变
    CHECK(q_cmd(2) == doctest::Approx(3.0));    // z 不变
}

TEST_CASE("VW - 已在墙外继续往外 → 硬拦截") {
    // d_curr ≥ 0, d_target ≥ 0, v_n > 0 → 硬拦截
    FakeFKModel model;
    FakeHardware hw;
    hw.fake_position_ = makeJntAt(6, 1, 1);   // current TCP (6,1,1), 已在墙外

    rocos::VirtualWallGuard guard(std::make_unique<FakeController>(), &model, &hw);

    rocos::PlaneWall wall;
    wall.point  = rocos::Vector(5, 0, 0);
    wall.normal = rocos::Vector(1, 0, 0);      // x>5 禁止
    guard.AddWall(wall);

    // 目标 (10, 1, 1) → 继续往外
    auto ref = rocos::Reference{makeJntAt(10, 1, 1)};
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::SlopoverVirtualWall);
}

TEST_CASE("VW - 已在墙外往回走 → 投影到边界（允许退回）") {
    // d_curr ≥ 0, d_target ≥ 0, v_n ≤ 0 → 投影到边界
    FakeFKModel model;
    FakeHardware hw;
    hw.fake_position_ = makeJntAt(6, 1, 1);   // current TCP (6,1,1), 已在墙外

    rocos::VirtualWallGuard guard(std::make_unique<FakeController>(), &model, &hw);

    rocos::PlaneWall wall;
    wall.point  = rocos::Vector(5, 0, 0);
    wall.normal = rocos::Vector(1, 0, 0);      // x>5 禁止
    guard.AddWall(wall);

    // 目标 (4, 1, 1) → 往回走，d_target 仍>0(in this test x=4 < 5 so d < 0)
    // 用目标(4,1,1): d_target = 4-5 = -1 < 0, 这个例子不对
    // 应该：目标 (5.5, 1, 1) → d_target > 0, d_curr > 0, v_n = -0.5 < 0 → 投影
    auto ref = rocos::Reference{makeJntAt(5.5, 1, 1)};
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
    // 投影到 x=5
    CHECK(q_cmd(0) == doctest::Approx(5.0));
}

TEST_CASE("VW - 沿墙切向运动 → 全速放行") {
    // 在边界上，沿切向运动 → v_n ≈ 0 → 无干扰
    FakeFKModel model;
    FakeHardware hw;
    hw.fake_position_ = makeJntAt(5, 1, 1);   // current TCP (5,1,1), 恰好在墙边界

    auto fakeCtrl = std::make_unique<FakeController>();
    auto* fPtr = fakeCtrl.get();

    rocos::VirtualWallGuard guard(std::move(fakeCtrl), &model, &hw);

    rocos::PlaneWall wall;
    wall.point  = rocos::Vector(5, 0, 0);
    wall.normal = rocos::Vector(1, 0, 0);      // x>5 禁止
    guard.AddWall(wall);

    // 目标 (5, 10, 1) → 纯切向（沿 Y 轴），d_target = 0, v_n = 0 → 投影后应回到边界
    auto ref = rocos::Reference{makeJntAt(5, 10, 1)};
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
    CHECK(fPtr->generate_count_ == 1);
    // x 被约束在边界上，y 全速
    CHECK(q_cmd(0) == doctest::Approx(5.0));
    CHECK(q_cmd(1) == doctest::Approx(10.0));
}

TEST_CASE("VW - 减速区只缩减法向、切向全速") {
    // 向墙移动进入减速区 → 只缩减法向量，切向保持全速
    FakeFKModel model;
    FakeHardware hw;
    hw.fake_position_ = makeJntAt(1, 1, 1);   // current (1,1,1)

    rocos::VirtualWallGuard guard(std::make_unique<FakeController>(), &model, &hw);

    rocos::PlaneWall wall;
    wall.point            = rocos::Vector(10, 0, 0);
    wall.normal           = rocos::Vector(1, 0, 0);
    wall.warning_distance = 5.0;               // 减速区 x ∈ [5, 10)
    guard.AddWall(wall);

    // 目标 (7, 5, 1): d_target = 7-10 = -3, |d|/wd = 0.6
    // 法向 delta: (7-1, 0, 0) * 0.6 = (3.6, 0, 0)
    // 切向 delta: (0, 5-1, 0) = (0, 4, 0) 全速
    // target: (1+3.6, 1+4, 1) = (4.6, 5, 1)
    auto ref = rocos::Reference{makeJntAt(7, 5, 1)};
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
    CHECK(q_cmd(0) == doctest::Approx(4.6));
    CHECK(q_cmd(1) == doctest::Approx(5.0));    // 切向全速！
}

TEST_CASE("VW - 离开墙方向不减速") {
    FakeFKModel model;
    FakeHardware hw;
    hw.fake_position_ = makeJntAt(8, 1, 1);   // current (8,1,1), 在减速区内

    auto fakeCtrl = std::make_unique<FakeController>();
    auto* fPtr = fakeCtrl.get();

    rocos::VirtualWallGuard guard(std::move(fakeCtrl), &model, &hw);

    rocos::PlaneWall wall;
    wall.point            = rocos::Vector(10, 0, 0);
    wall.normal           = rocos::Vector(1, 0, 0);
    wall.warning_distance = 5.0;
    guard.AddWall(wall);

    // 目标 (3, 1, 1): d_target = -7, 在减速区内, 但 v_n < 0 (离开墙) → 全速
    auto ref = rocos::Reference{makeJntAt(3, 1, 1)};
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
    CHECK(q_cmd(0) == doctest::Approx(3.0));    // 全速通过，未缩放
}

TEST_CASE("VW - OutsideOnly 球：从外进入 → 投影") {
    // SphereWall OutsideOnly r=5: 禁止进入球内
    FakeFKModel model;
    FakeHardware hw;
    hw.fake_position_ = makeJntAt(10, 0, 0);  // 球外安全

    rocos::VirtualWallGuard guard(std::make_unique<FakeController>(), &model, &hw);

    rocos::SphereWall sphere;
    sphere.center = rocos::Vector(0, 0, 0);
    sphere.radius = 5.0;
    sphere.mode   = rocos::WallMode::OutsideOnly;  // 禁止进入
    guard.AddWall(sphere);

    // 目标 (0,0,0): 球心，d_target > 0(违规), 但 d_curr < 0(安全区) → 投影到球面
    auto ref = rocos::Reference{makeJntAt(0, 0, 0)};
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
    // 应投影到球面 r=5 在 X 轴上
    CHECK(q_cmd(0) == doctest::Approx(5.0));
}

TEST_CASE("VW - OutsideOnly 球：已在球内继续深入 → 硬拦截") {
    FakeFKModel model;
    FakeHardware hw;
    hw.fake_position_ = makeJntAt(1, 0, 0);   // 已在球内（违规区）

    rocos::VirtualWallGuard guard(std::make_unique<FakeController>(), &model, &hw);

    rocos::SphereWall sphere;
    sphere.center = rocos::Vector(0, 0, 0);
    sphere.radius = 5.0;
    sphere.mode   = rocos::WallMode::OutsideOnly;
    guard.AddWall(sphere);

    // 目标 (0, 0, 0): 继续深入 → d_curr>0, d_target>0, v_n>0 → 硬拦截
    auto ref = rocos::Reference{makeJntAt(0, 0, 0)};
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::SlopoverVirtualWall);
}

TEST_CASE("VW - OutsideOnly 球：已在球内往外退 → 投影到球面") {
    FakeFKModel model;
    FakeHardware hw;
    hw.fake_position_ = makeJntAt(1, 0, 0);   // 球内（违规区）

    rocos::VirtualWallGuard guard(std::make_unique<FakeController>(), &model, &hw);

    rocos::SphereWall sphere;
    sphere.center = rocos::Vector(0, 0, 0);
    sphere.radius = 5.0;
    sphere.mode   = rocos::WallMode::OutsideOnly;
    guard.AddWall(sphere);

    // 目标 (10, 0, 0): 往外走回安全区 → d_target<0?
    // No: at (10,0,0), d_target = 5 - |10| = -5 < 0, that's safe!
    // Better: 目标从(1,0,0)到(2,0,0): d_target = 5-2 = 3 > 0, d_curr = 5-1 > 0, v_n>0
    // Actually we want d_curr > 0, d_target > 0, v_n <= 0
    // 目标 (4, 0, 0): d_target = 5-4 = 1 > 0, v_n = dot((3,0,0),(-1,0,0)) = -3 < 0
    //   → d_curr>0, d_target>0, v_n<0 → 投影
    auto ref = rocos::Reference{makeJntAt(4, 0, 0)};
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
    CHECK(q_cmd(0) == doctest::Approx(5.0));    // 投影回球面
}

TEST_CASE("VW - 多墙同时作用") {
    FakeFKModel model;
    FakeHardware hw;
    hw.fake_position_ = makeJntAt(1, 1, 1);

    rocos::VirtualWallGuard guard(std::make_unique<FakeController>(), &model, &hw);

    // 墙1: x>10 禁止
    rocos::PlaneWall wall1;
    wall1.point  = rocos::Vector(10, 0, 0);
    wall1.normal = rocos::Vector(1, 0, 0);
    guard.AddWall(wall1);

    // 墙2: y>8 禁止
    rocos::PlaneWall wall2;
    wall2.point  = rocos::Vector(0, 8, 0);
    wall2.normal = rocos::Vector(0, 1, 0);
    guard.AddWall(wall2);

    // 目标 (15, 10, 1): 同时超出两个墙 → 分别投影
    auto ref = rocos::Reference{makeJntAt(15, 10, 1)};
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
    // x 被墙1约束到 10, y 被墙2约束到 8
    CHECK(q_cmd(0) == doctest::Approx(10.0));
    CHECK(q_cmd(1) == doctest::Approx(8.0));
}

// ============================================================================
// 5. 基础功能（不变）
// ============================================================================

TEST_CASE("VW - 禁用后透传违规目标") {
    FakeFKModel model;
    FakeHardware hw;
    hw.fake_position_ = makeJntAt(6, 1, 1);   // 已在墙外

    auto fakeCtrl = std::make_unique<FakeController>();
    auto* fPtr = fakeCtrl.get();

    rocos::VirtualWallGuard guard(std::move(fakeCtrl), &model, &hw);

    rocos::PlaneWall wall;
    wall.point  = rocos::Vector(5, 0, 0);
    wall.normal = rocos::Vector(1, 0, 0);
    guard.AddWall(wall);

    guard.SetEnabled(false);
    auto ref = rocos::Reference{makeJntAt(10, 1, 1)};   // 违规范目标
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
    CHECK(fPtr->generate_count_ == 1);
}

TEST_CASE("VW - AddWall / ClearWalls / GetWallCount") {
    FakeFKModel model;
    rocos::VirtualWallGuard guard(std::make_unique<FakeController>(), &model);
    CHECK(guard.GetWallCount() == 0u);
    guard.AddWall(rocos::PlaneWall{});
    guard.AddWall(rocos::SphereWall{});
    CHECK(guard.GetWallCount() == 2u);
    guard.ClearWalls();
    CHECK(guard.GetWallCount() == 0u);
}

TEST_CASE("VW - SetEnabled / IsEnabled") {
    FakeFKModel model;
    rocos::VirtualWallGuard guard(std::make_unique<FakeController>(), &model);
    CHECK(guard.IsEnabled());
    guard.SetEnabled(false);
    CHECK(!guard.IsEnabled());
}

TEST_CASE("VW - 包装 PositionController") {
    auto posCtrl = std::make_unique<rocos::PositionController>();
    FakeFKModel model;
    FakeHardware hw;
    hw.fake_position_ = makeJntAt(1, 2, 3);

    posCtrl->SetHardware(&hw);
    posCtrl->SetModel(&model);

    rocos::VirtualWallGuard guard(std::move(posCtrl), &model, &hw);

    auto ref = rocos::Reference{makeJntAt(2, 3, 4)};
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
    CHECK(guard.UpdateCmd(q_cmd) == rocos::Result::NoError);
    CHECK(hw.set_mode_count_ == 1);
}

TEST_CASE("VW - FK 失败时不拦截") {
    auto fakeCtrl = std::make_unique<FakeController>();
    auto* fPtr = fakeCtrl.get();
    FakeFKModel model;
    model.fk_should_fail_ = true;
    FakeHardware hw;
    hw.fake_position_ = makeJnt(6);

    rocos::VirtualWallGuard guard(std::move(fakeCtrl), &model, &hw);
    rocos::PlaneWall wall;
    wall.point  = rocos::Vector(5, 0, 0);
    wall.normal = rocos::Vector(1, 0, 0);
    guard.AddWall(wall);

    auto ref = rocos::Reference{makeJntAt(10, 2, 3)};
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
    CHECK(fPtr->generate_count_ == 1);
}

TEST_CASE("VW - 无硬件时退化到绝对位置检查") {
    FakeFKModel model;
    rocos::VirtualWallGuard guard(std::make_unique<FakeController>(), &model,
                                   nullptr);  // 无 hardware

    rocos::PlaneWall wall;
    wall.point  = rocos::Vector(5, 0, 0);
    wall.normal = rocos::Vector(1, 0, 0);
    guard.AddWall(wall);

    // 无当前位置 → 目标 x=10 直接硬拦截
    rocos::JntArray q_in(3);
    q_in(0) = 10.0; q_in(1) = 2.0; q_in(2) = 3.0;
    rocos::Reference ref = q_in;
    rocos::JntArray q_cmd;
    CHECK(guard.GenerateCmd(ref, q_cmd) == rocos::Result::SlopoverVirtualWall);
}

TEST_CASE("VW - Reset / UpdateCmd 透传") {
    FakeFKModel model;
    auto fakeCtrl = std::make_unique<FakeController>();
    auto* fPtr = fakeCtrl.get();
    rocos::VirtualWallGuard guard(std::move(fakeCtrl), &model);

    fPtr->generate_count_ = 5;
    guard.Reset();
    CHECK(fPtr->generate_count_ == 0);

    rocos::JntArray q(3);
    q(0) = 1; q(1) = 2; q(2) = 3;
    guard.UpdateCmd(q);
    CHECK(fPtr->update_count_ == 1);
}
