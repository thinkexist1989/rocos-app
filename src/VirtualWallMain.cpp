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
//
// VirtualWallMain — 虚拟墙功能演示程序
//
// 演示场景（不需真实硬件）：
//   1. 平面墙：从安全区冲向墙 → 自动投影到边界
//   2. 平面墙：在墙外继续往外 → 硬拦截
//   3. 平面墙：沿墙切向滑动 → 全速放行
//   4. 平面墙：从墙外退回安全区 → 投影通过
//   5. 球墙 InsideOnly：超出球面 → 投影
//   6. 圆柱墙 InsideOnly：超半径 → 投影
//   7. 圆柱墙 OutsideOnly：试图进入 → 投影
//   8. 减速区：只缩减法向分量，切向全速
//   9. 多墙同时约束 → 分别投影

#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "src/virtual_wall.hpp"
#include "src/virtual_wall_guard.hpp"

// ============================================================================
// 轻量 Fake 实现（无需真实硬件/URDF）
// ============================================================================

namespace {

class DemoHardware : public rocos::HardwareInterface {
public:
    rocos::JntArray position_;

    explicit DemoHardware(double x, double y, double z) {
        position_.resize(6);
        position_(0) = x; position_(1) = y; position_(2) = z;
        position_(3) = 0; position_(4) = 0; position_(5) = 0;
    }

    rocos::JntArray GetPosition() override { return position_; }
    rocos::JntArray GetVelocity() override { return {}; }
    rocos::JntArray GetTorque() override { return {}; }
    rocos::JntArray GetLoadTorque() override { return {}; }
    void SetPosition(const rocos::JntArray&) override {}
    void SetVelocity(const rocos::JntArray&) override {}
    void SetTorque(const rocos::JntArray&) override {}
    void SetMode(int8_t) override {}
    void SetEnabled() override {}
    void SetDisabled() override {}

    double GetJointPosition(int32_t) override { return 0; }
    double GetJointVelocity(int32_t) override { return 0; }
    double GetJointTorque(int32_t) override { return 0; }
    double GetJointLoadTorque(int32_t) override { return 0; }
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
    double GetAnalogInput(int, int) override { return 0; }
    void SetAnalogOutput(int, int, double) override {}
    bool Reset() override { return true; }
};

/// FK: 前3关节 → TCP(x,y,z) 直接映射
class DemoModel : public rocos::ModelInterface {
public:
    rocos::Result ForwardKinematics(const rocos::JntArray& q,
                                     rocos::Frame& p) override {
        if (q.rows() >= 3)
            p.p = rocos::Vector(q(0), q(1), q(2));
        else if (q.rows() == 2)
            p.p = rocos::Vector(q(0), q(1), 0);
        else
            p.p = rocos::Vector(0, 0, 0);
        return rocos::Result::NoError;
    }

    rocos::Result InverseKinematics(const rocos::JntArray& seed,
                                     const rocos::Frame& target,
                                     rocos::JntArray& q_out) override {
        q_out.resize(seed.rows());
        if (seed.rows() >= 1) q_out(0) = target.p.x();
        if (seed.rows() >= 2) q_out(1) = target.p.y();
        if (seed.rows() >= 3) q_out(2) = target.p.z();
        for (uint i = 3; i < seed.rows(); ++i) q_out(i) = seed(i);
        return rocos::Result::NoError;
    }

    rocos::Result ForwardDynamics(const rocos::JntArray&, const rocos::JntArray&,
                                   const rocos::JntArray&, const rocos::Wrenches&,
                                   rocos::JntArray&) override {
        return rocos::Result::NoError;
    }

    rocos::Result GetJacobian(const rocos::JntArray &q, rocos::Jacobian &J_out) override {
        return rocos::Result::NoError;
    }

    rocos::Result InverseDynamics(const rocos::JntArray&, const rocos::JntArray&,
                                  const rocos::JntArray&, const rocos::Wrenches&,
                                  rocos::JntArray&) override {
        return rocos::Result::NoError;
    }
};

/// 透传控制器：GenerateCmd 直接将 Reference 转为 JntArray
class PassThroughCtrl : public rocos::ControllerInterface {
public:
    bool Reset() override { return true; }
    rocos::Result SetHardware(rocos::HardwareInterface*) override { return rocos::Result::NoError; }
    rocos::Result SetModel(rocos::ModelInterface*) override { return rocos::Result::NoError; }

    rocos::Result GenerateCmd(const rocos::Reference& ref,
                               rocos::JntArray& q_cmd) override {
        if (auto* jnt = std::get_if<rocos::JntArray>(&ref)) {
            q_cmd = *jnt;
        } else if (auto* f = std::get_if<rocos::Frame>(&ref)) {
            q_cmd.resize(6);
            q_cmd(0) = f->p.x(); q_cmd(1) = f->p.y(); q_cmd(2) = f->p.z();
            q_cmd(3) = 0; q_cmd(4) = 0; q_cmd(5) = 0;
        }
        return rocos::Result::NoError;
    }

    rocos::Result UpdateCmd(const rocos::JntArray&) override { return rocos::Result::NoError; }
};

rocos::JntArray makeTarget(double x, double y, double z) {
    rocos::JntArray q(6);
    q(0) = x; q(1) = y; q(2) = z; q(3) = 0; q(4) = 0; q(5) = 0;
    return q;
}

}  // namespace

// ============================================================================
// 工具函数
// ============================================================================

static void printHeader(const std::string& title) {
    std::cout << "\n\033[1;36m" << std::string(60, '=') << "\033[0m\n";
    std::cout << "\033[1;36m  " << title << "\033[0m\n";
    std::cout << "\033[1;36m" << std::string(60, '=') << "\033[0m\n";
}

static void printResult(const std::string& label, rocos::Result r,
                         const rocos::JntArray& q_cmd) {
    std::cout << "  " << std::left << std::setw(28) << label << " → ";
    if (r == rocos::Result::NoError) {
        std::cout << "\033[1;32mPASS \033[0m";
        std::cout << "TCP=(" << std::fixed << std::setprecision(2)
                  << q_cmd(0) << ", " << q_cmd(1) << ", " << q_cmd(2) << ")";
    } else if (r == rocos::Result::SlopoverVirtualWall) {
        std::cout << "\033[1;31mSTOP \033[0m(硬拦截: SlopoverVirtualWall)";
    } else {
        std::cout << "\033[1;33mERR  \033[0m(" << rocos::to_string(r) << ")";
    }
    std::cout << std::endl;
}

static void describe(const std::string& text) {
    std::cout << "\033[90m    // " << text << "\033[0m\n";
}

// ============================================================================
// main
// ============================================================================

int main() {
    using namespace rocos;

    DemoModel model;

    printHeader("场景 1: 平面墙 — 从安全区冲向墙外 → 自动投影到边界");

    {
        DemoHardware hw(1.0, 1.0, 1.0);       // 当前位置 TCP=(1,1,1)
        auto guard = std::make_unique<VirtualWallGuard>(
            std::make_unique<PassThroughCtrl>(), &model, &hw);

        PlaneWall wall;
        wall.point  = Vector(5, 0, 0);         // 墙在 x=5
        wall.normal = Vector(1, 0, 0);         // x>5 为禁止侧
        guard->AddWall(wall);

        describe("平面墙 x=5 禁止侧 +X，当前 TCP(1,1,1)，冲向 (10,2,3)");

        JntArray q_cmd;
        auto ref = Reference{makeTarget(10, 2, 3)};
        auto res = guard->GenerateCmd(ref, q_cmd);
        printResult("冲向墙外", res, q_cmd);
        // 预期: PASS, TCP=(5.0, 2.0, 3.0) — x被投影到5，y/z不变
    }

    printHeader("场景 2: 平面墙 — 已在墙外，继续往外 → 硬拦截");

    {
        DemoHardware hw(6.0, 1.0, 1.0);       // 当前 x=6 已在墙外
        auto guard = std::make_unique<VirtualWallGuard>(
            std::make_unique<PassThroughCtrl>(), &model, &hw);

        PlaneWall wall;
        wall.point  = Vector(5, 0, 0);
        wall.normal = Vector(1, 0, 0);
        guard->AddWall(wall);

        describe("平面墙 x=5，当前已在禁止区 x=6，继续冲向 x=10");

        JntArray q_cmd;
        auto ref = Reference{makeTarget(10, 1, 1)};
        auto res = guard->GenerateCmd(ref, q_cmd);
        printResult("继续往外", res, q_cmd);
        // 预期: STOP (硬拦截)
    }

    printHeader("场景 3: 平面墙 — 从墙外往回走 → 投影通过");

    {
        DemoHardware hw(6.0, 1.0, 1.0);       // 当前 x=6 在墙外
        auto guard = std::make_unique<VirtualWallGuard>(
            std::make_unique<PassThroughCtrl>(), &model, &hw);

        PlaneWall wall;
        wall.point  = Vector(5, 0, 0);
        wall.normal = Vector(1, 0, 0);
        guard->AddWall(wall);

        describe("平面墙 x=5，当前在墙外 x=6，目标是 x=4 安全区");

        JntArray q_cmd;
        auto ref = Reference{makeTarget(4, 1, 1)};
        auto res = guard->GenerateCmd(ref, q_cmd);
        printResult("退回安全区", res, q_cmd);
        // 预期: PASS, TCP=(4.0, 1.0, 1.0) — d_target<0 在安全区，直接放行
    }

    printHeader("场景 4: 平面墙 — 沿墙切向滑动 → 全速放行");

    {
        DemoHardware hw(5.0, 1.0, 1.0);       // 当前恰好在墙边界 x=5
        auto guard = std::make_unique<VirtualWallGuard>(
            std::make_unique<PassThroughCtrl>(), &model, &hw);

        PlaneWall wall;
        wall.point  = Vector(5, 0, 0);
        wall.normal = Vector(1, 0, 0);
        guard->AddWall(wall);

        describe("平面墙 x=5，当前恰好在边界 (5,1,1)，沿 Y 轴滑动到 (5,8,1)");

        JntArray q_cmd;
        auto ref = Reference{makeTarget(5, 8, 1)};
        auto res = guard->GenerateCmd(ref, q_cmd);
        printResult("切向滑动", res, q_cmd);
        // 预期: PASS, 投影后 x 保持在 5，y 全速到 8
    }

    printHeader("场景 5: 球墙 InsideOnly — 超出球面 → 投影");

    {
        DemoHardware hw(2.0, 2.0, 2.0);       // 球内，距原点 ~3.46
        auto guard = std::make_unique<VirtualWallGuard>(
            std::make_unique<PassThroughCtrl>(), &model, &hw);

        SphereWall sphere;
        sphere.center = Vector(0, 0, 0);
        sphere.radius = 5.0;
        sphere.mode   = WallMode::InsideOnly;  // 禁止超出球面
        guard->AddWall(sphere);

        describe("球心原点 r=5 InsideOnly，当前在球内 (2,2,2)，冲向 (10,0,0)");

        JntArray q_cmd;
        auto ref = Reference{makeTarget(10, 0, 0)};
        auto res = guard->GenerateCmd(ref, q_cmd);
        printResult("超出球面", res, q_cmd);
        // 预期: PASS, 投影到球面 r=5 在+X方向: (5,0,0)
    }

    printHeader("场景 6: 圆柱墙 InsideOnly — 超半径 → 投影");

    {
        DemoHardware hw(1.0, 1.0, 1.0);
        auto guard = std::make_unique<VirtualWallGuard>(
            std::make_unique<PassThroughCtrl>(), &model, &hw);

        CylinderWall cyl;
        cyl.center = Vector(0, 0, 0);
        cyl.axis   = Vector(0, 0, 1);
        cyl.radius = 3.0;
        cyl.mode   = WallMode::InsideOnly;     // 禁止超出圆柱
        guard->AddWall(cyl);

        describe("圆柱 Z轴 r=3 InsideOnly，当前 (1,1,1) |radial|=1.41，冲向 (5,0,0)");

        JntArray q_cmd;
        auto ref = Reference{makeTarget(5, 0, 0)};   // |radial|=5 超出圆柱
        auto res = guard->GenerateCmd(ref, q_cmd);
        printResult("超出圆柱", res, q_cmd);
        // 预期: PASS, 投影到圆柱面: (3,0,0)
    }

    printHeader("场景 7: 圆柱墙 OutsideOnly — 试图进入 → 投影");

    {
        DemoHardware hw(10.0, 0.0, 0.0);      // 圆柱外安全区
        auto guard = std::make_unique<VirtualWallGuard>(
            std::make_unique<PassThroughCtrl>(), &model, &hw);

        CylinderWall cyl;
        cyl.center = Vector(0, 0, 0);
        cyl.axis   = Vector(0, 0, 1);
        cyl.radius = 5.0;
        cyl.mode   = WallMode::OutsideOnly;    // 禁止进入圆柱内
        guard->AddWall(cyl);

        describe("圆柱 Z轴 r=5 OutsideOnly，当前在外 (10,0,0)，冲向圆柱内 (2,0,0)");

        JntArray q_cmd;
        auto ref = Reference{makeTarget(2, 0, 0)};
        auto res = guard->GenerateCmd(ref, q_cmd);
        printResult("试图进入", res, q_cmd);
        // 预期: PASS, 投影到圆柱面: (5,0,0)
    }

    printHeader("场景 8: 减速区 — 法向减速，切向全速");

    {
        DemoHardware hw(1.0, 1.0, 1.0);       // 当前 (1,1,1)
        auto guard = std::make_unique<VirtualWallGuard>(
            std::make_unique<PassThroughCtrl>(), &model, &hw);

        PlaneWall wall;
        wall.point            = Vector(10, 0, 0);
        wall.normal           = Vector(1, 0, 0);
        wall.warning_distance = 5.0;           // 减速区 x∈[5,10)
        guard->AddWall(wall);

        describe("墙 x=10 wd=5，当前 (1,1,1)，冲向 (8,6,1)");
        describe("d_target = -2, |d|/wd = 0.4 减速法向；切向(Y)全速");

        JntArray q_cmd;
        auto ref = Reference{makeTarget(8, 6, 1)};
        auto res = guard->GenerateCmd(ref, q_cmd);
        printResult("减速区", res, q_cmd);
        // 预期: PASS
        //  法向 x: 1 + 0.4*(8-1) = 3.8
        //  切向 y: 1 + 1.0*(6-1) = 6.0 (全速!)
    }

    printHeader("场景 9: 多墙 + 拐角 — 分别投影到两面墙的交线");

    {
        DemoHardware hw(1.0, 1.0, 1.0);
        auto guard = std::make_unique<VirtualWallGuard>(
            std::make_unique<PassThroughCtrl>(), &model, &hw);

        PlaneWall wallX;
        wallX.point  = Vector(5, 0, 0);
        wallX.normal = Vector(1, 0, 0);        // x>5 禁止
        guard->AddWall(wallX);

        PlaneWall wallY;
        wallY.point  = Vector(0, 5, 0);
        wallY.normal = Vector(0, 1, 0);        // y>5 禁止
        guard->AddWall(wallY);

        describe("墙 x=5 + 墙 y=5，当前 (1,1,1)，冲向 (10,10,1)");

        JntArray q_cmd;
        auto ref = Reference{makeTarget(10, 10, 1)};
        auto res = guard->GenerateCmd(ref, q_cmd);
        printResult("拐角投影", res, q_cmd);
        // 预期: PASS, x→5, y→5，投影到两面墙交线
    }

    printHeader("场景 10: 禁用虚拟墙 → 违规目标透传");

    {
        DemoHardware hw(6.0, 1.0, 1.0);
        auto guard = std::make_unique<VirtualWallGuard>(
            std::make_unique<PassThroughCtrl>(), &model, &hw);

        PlaneWall wall;
        wall.point  = Vector(5, 0, 0);
        wall.normal = Vector(1, 0, 0);
        guard->AddWall(wall);

        guard->SetEnabled(false);              // 禁用！

        describe("禁用虚拟墙后，即使已在墙外继续往外也不拦截");

        JntArray q_cmd;
        auto ref = Reference{makeTarget(10, 1, 1)};
        auto res = guard->GenerateCmd(ref, q_cmd);
        printResult("禁用后透传", res, q_cmd);
        // 预期: PASS, TCP=(10,1,1) 原样透传
    }

    std::cout << "\n\033[1;32m所有演示场景完成。\033[0m\n" << std::endl;
    return 0;
}
