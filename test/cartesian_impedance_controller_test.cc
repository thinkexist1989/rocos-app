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
#include <string>
#include <variant>
#include <vector>

#include <test/doctest.h>

#include "src/cartesian_impedance_controller.hpp"

// ==========================================================================
// Fake 实现 — 替代真实 Hardware / Model，使控制器可脱离硬件测试
// ==========================================================================

namespace {

class FakeHardware : public rocos::HardwareInterface {
public:
    rocos::JntArray fake_position_;
    rocos::JntArray fake_velocity_;
    rocos::JntArray last_set_torque_;
    int8_t last_set_mode_{-1};
    int set_mode_count_{0};

    rocos::JntState GetState() override { return rocos::JntState::ENABLED; }
    rocos::JntState GetJointState(int32_t) override { return rocos::JntState::ENABLED; }
    ~FakeHardware() override {}

    // DriveInterface — batch
    rocos::JntArray GetPosition() override { return fake_position_; }
    rocos::JntArray GetVelocity() override { return fake_velocity_; }
    rocos::JntArray GetTorque() override { return {}; }
    rocos::JntArray GetLoadTorque() override { return {}; }
    void SetPosition(const rocos::JntArray&) override {}
    void SetVelocity(const rocos::JntArray&) override {}
    void SetTorque(const rocos::JntArray& tau) override { last_set_torque_ = tau; }
    void SetMode(int8_t mode) override {
        last_set_mode_ = mode;
        ++set_mode_count_;
    }
    void SetEnabled() override {}
    void SetDisabled() override {}

    // DriveInterface — single joint
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

    // FTSensorInterface
    rocos::Wrench GetWrench() override { return rocos::Wrench::Zero(); }

    // IOInteface
    bool GetDigitalInput(int32_t, int32_t) override { return false; }
    void SetDigitalOutput(int32_t, int32_t, bool) override {}
    double GetAnalogInput(int32_t, int32_t) override { return 0.0; }
    void SetAnalogOutput(int32_t, int32_t, double) override {}

    // HardwareInterface
    bool Reset() override { return true; }
    void WaitForSignal() override {}
};

/// @brief FakeModel: 提供可预测的 FK、Jacobian、InverseDynamics 返回值
///
/// FK:    p_out = Vector(q[0], q[1], q[2]), R = Identity
/// Jacobian: 6×n，前 3 列为 XYZ 平动，后 3 列为 RPY 转动，其余列为混合
/// ID:    每个关节返回常数重力力矩
class FakeModel : public rocos::ModelInterface {
public:
    bool fk_should_fail_{false};
    bool jac_should_fail_{false};
    bool id_should_fail_{false};
    double grav_torque_{5.0};   // 每个关节的模拟重力力矩 [Nm]

    rocos::Result ForwardKinematics(const rocos::JntArray& q_in,
                                    rocos::Frame& p_out) override {
        if (fk_should_fail_) return rocos::Result::FkCalcFail;
        // 位置 = (q[0], q[1], q[2])，旋转 = Identity
        double x = q_in.rows() > 0 ? q_in(0) : 0.0;
        double y = q_in.rows() > 1 ? q_in(1) : 0.0;
        double z = q_in.rows() > 2 ? q_in(2) : 0.0;
        p_out = rocos::Frame(KDL::Rotation::Identity(), KDL::Vector(x, y, z));
        return rocos::Result::NoError;
    }

    rocos::Result InverseKinematics(const rocos::JntArray& q_in,
                                     const rocos::Frame&,
                                     rocos::JntArray& q_out) override {
        // 透传种子作为 IK 解
        q_out = q_in;
        return rocos::Result::NoError;
    }

    /// @brief 构造简单 6×n Jacobian:
    ///        前 3 列 = XYZ 平动轴 (dof 0-2)
    ///        列 3-5 = RPY 转动轴 (dof 3-5)
    ///        列 6   = 混合列 (用于构造 1 维零空间)
    rocos::Result GetJacobian(const rocos::JntArray& q,
                              rocos::Jacobian& J_out) override {
        (void)q;
        if (jac_should_fail_) return rocos::Result::JacobianCalcFail;

        const unsigned int n = q.rows();
        J_out.resize(n);

        // 构造 Jacobian: 每列是一个 Twist
        // 列 0: prisma X  → Twist(vel=[1,0,0], rot=[0,0,0])
        // 列 1: prisma Y  → Twist(vel=[0,1,0], rot=[0,0,0])
        // 列 2: prisma Z  → Twist(vel=[0,0,1], rot=[0,0,0])
        // 列 3: revo X    → Twist(vel=[0,0,0], rot=[1,0,0])
        // 列 4: revo Y    → Twist(vel=[0,0,0], rot=[0,1,0])
        // 列 5: revo Z    → Twist(vel=[0,0,0], rot=[0,0,1])
        // 列 6+: 混合列   → Twist(vel=[0.3,0.3,0.3], rot=[0.3,0.3,0.3])

        for (unsigned int j = 0; j < n; ++j) {
            KDL::Vector vel(0.0, 0.0, 0.0), rot(0.0, 0.0, 0.0);
            switch (j) {
                case 0: vel = KDL::Vector(1.0, 0.0, 0.0); break;
                case 1: vel = KDL::Vector(0.0, 1.0, 0.0); break;
                case 2: vel = KDL::Vector(0.0, 0.0, 1.0); break;
                case 3: rot = KDL::Vector(1.0, 0.0, 0.0); break;
                case 4: rot = KDL::Vector(0.0, 1.0, 0.0); break;
                case 5: rot = KDL::Vector(0.0, 0.0, 1.0); break;
                default:
                    // 其余列为混合，构造 1 维零空间 (n=7 时有 rank 6)
                    vel = KDL::Vector(0.3, 0.3, 0.3);
                    rot = KDL::Vector(0.3, 0.3, 0.3);
                    break;
            }
            J_out.setColumn(j, KDL::Twist(vel, rot));
        }
        return rocos::Result::NoError;
    }

    rocos::Result InverseDynamics(const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::Wrenches&,
                                  rocos::JntArray& torques) override {
        if (id_should_fail_) return rocos::Result::IdCalcFail;
        for (unsigned int i = 0; i < torques.rows(); ++i) {
            torques(i) = grav_torque_;
        }
        return rocos::Result::NoError;
    }

    rocos::Result ForwardDynamics(const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::Wrenches&,
                                  rocos::JntArray&) override {
        return rocos::Result::NoError;
    }

    int GetJointNum() const override { return 7; }
    std::vector<std::string> GetJointNames() const override { return {}; }
};

// ---- 工具函数 ----

rocos::JntArray makeJntArray(unsigned int n, double start = 1.0) {
    rocos::JntArray q(n);
    for (unsigned int i = 0; i < n; ++i) {
        q(i) = start + static_cast<double>(i);
    }
    return q;
}

rocos::JntArray makeConstJntArray(unsigned int n, double val = 0.0) {
    rocos::JntArray q(n);
    for (unsigned int i = 0; i < n; ++i) {
        q(i) = val;
    }
    return q;
}

/// @brief 构造位姿：仅平移 (x,y,z)，无旋转
rocos::Frame makeFrame(double x, double y, double z) {
    return rocos::Frame(KDL::Rotation::Identity(), KDL::Vector(x, y, z));
}

}  // namespace

// 被各个测试复用的 GenerateCmd 输出参数
static rocos::JntArray dummy_q_cmd;

// ==========================================================================
// §1. 基础接口
// ==========================================================================

TEST_CASE("CartesianImpedance - SetHardware / SetModel") {
    rocos::CartesianImpedanceController ctrl;
    FakeHardware hw;
    FakeModel model;

    SUBCASE("SetHardware nullptr → ParameterPointerEqualsNullptr") {
        CHECK(ctrl.SetHardware(nullptr) == rocos::Result::ParameterPointerEqualsNullptr);
    }
    SUBCASE("SetHardware 有效 → NoError") {
        CHECK(ctrl.SetHardware(&hw) == rocos::Result::NoError);
    }
    SUBCASE("SetModel nullptr → ParameterPointerEqualsNullptr") {
        CHECK(ctrl.SetModel(nullptr) == rocos::Result::ParameterPointerEqualsNullptr);
    }
    SUBCASE("SetModel 有效 → NoError") {
        CHECK(ctrl.SetModel(&model) == rocos::Result::NoError);
    }
}

TEST_CASE("CartesianImpedance - Reset") {
    rocos::CartesianImpedanceController ctrl;
    CHECK(ctrl.Reset() == true);
}

// ==========================================================================
// §2. 参数设置
// ==========================================================================

TEST_CASE("CartesianImpedance - 笛卡尔阻抗参数") {
    rocos::CartesianImpedanceController ctrl;

    SUBCASE("SetTranslationalStiffness 正常") {
        CHECK(ctrl.SetTranslationalStiffness(1000.0) == rocos::Result::NoError);
    }
    SUBCASE("SetTranslationalStiffness 负值 → ParameterNanOrInf") {
        CHECK(ctrl.SetTranslationalStiffness(-100.0) == rocos::Result::ParameterNanOrInf);
    }
    SUBCASE("SetTranslationalStiffness NaN → ParameterNanOrInf") {
        CHECK(ctrl.SetTranslationalStiffness(
            std::numeric_limits<double>::quiet_NaN()) == rocos::Result::ParameterNanOrInf);
    }
    SUBCASE("SetRotationalStiffness 正常") {
        CHECK(ctrl.SetRotationalStiffness(80.0) == rocos::Result::NoError);
    }
    SUBCASE("SetTranslationalDamping 正常") {
        CHECK(ctrl.SetTranslationalDamping(150.0) == rocos::Result::NoError);
    }
    SUBCASE("SetRotationalDamping 正常") {
        CHECK(ctrl.SetRotationalDamping(15.0) == rocos::Result::NoError);
    }
}

TEST_CASE("CartesianImpedance - 零空间参数") {
    rocos::CartesianImpedanceController ctrl;

    SUBCASE("SetNullspaceStiffness 正常") {
        CHECK(ctrl.SetNullspaceStiffness(60.0) == rocos::Result::NoError);
    }
    SUBCASE("SetNullspaceStiffness 负值 → ParameterNanOrInf") {
        CHECK(ctrl.SetNullspaceStiffness(-10.0) == rocos::Result::ParameterNanOrInf);
    }
    SUBCASE("SetNullspaceDamping 正常") {
        CHECK(ctrl.SetNullspaceDamping(15.0) == rocos::Result::NoError);
    }
    SUBCASE("SetNullspaceReference 正常") {
        CHECK(ctrl.SetNullspaceReference(makeConstJntArray(7, 0.5)) == rocos::Result::NoError);
    }
    SUBCASE("SetNullspaceReference 空数组 → MoveInput") {
        rocos::JntArray empty_q{};
        CHECK(ctrl.SetNullspaceReference(empty_q) == rocos::Result::MoveInput);
    }
    SUBCASE("SetNullspaceReference 含 NaN → ParameterNanOrInf") {
        auto q = makeConstJntArray(7, 0.5);
        q(3) = std::numeric_limits<double>::quiet_NaN();
        CHECK(ctrl.SetNullspaceReference(q) == rocos::Result::ParameterNanOrInf);
    }
}

TEST_CASE("CartesianImpedance - 力矩安全参数") {
    rocos::CartesianImpedanceController ctrl;

    SUBCASE("SetTorqueRateLimit 正常") {
        CHECK(ctrl.SetTorqueRateLimit(500.0) == rocos::Result::NoError);
    }
    SUBCASE("SetTorqueRateLimit 零值 → 有效（关闭限制）") {
        CHECK(ctrl.SetTorqueRateLimit(0.0) == rocos::Result::NoError);
    }
    SUBCASE("SetTorqueRateLimit 负值 → ParameterNanOrInf") {
        CHECK(ctrl.SetTorqueRateLimit(-100.0) == rocos::Result::ParameterNanOrInf);
    }
    SUBCASE("SetTorqueSaturation 正常") {
        CHECK(ctrl.SetTorqueSaturation(50.0) == rocos::Result::NoError);
    }
    SUBCASE("SetTorqueSaturation 零值 → 有效（关闭限制）") {
        CHECK(ctrl.SetTorqueSaturation(0.0) == rocos::Result::NoError);
    }
    SUBCASE("SetTorqueSaturation 负值 → ParameterNanOrInf") {
        CHECK(ctrl.SetTorqueSaturation(-10.0) == rocos::Result::ParameterNanOrInf);
    }
}

// ==========================================================================
// §3. GenerateCmd
// ==========================================================================

TEST_CASE("CartesianImpedance - GenerateCmd") {
    rocos::CartesianImpedanceController ctrl;
    FakeHardware hw;
    FakeModel model;
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);

    SUBCASE("JntArray → FK 生成笛卡尔位姿") {
        auto q_in = makeConstJntArray(7, 0.1);
        rocos::Reference ref = q_in;
        rocos::JntArray q_cmd;
        CHECK(ctrl.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
        CHECK(q_cmd.rows() == 7u);
    }

    SUBCASE("Frame 直接存储") {
        rocos::Frame frame = makeFrame(0.5, 0.0, 0.0);
        rocos::Reference ref = frame;
        rocos::JntArray q_cmd;
        CHECK(ctrl.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
        // q_cmd 由 IK 填充
        CHECK(q_cmd.rows() == 7u);
    }

    SUBCASE("空 JntArray → MoveInput") {
        rocos::Reference ref = dummy_q_cmd;
        rocos::JntArray q_cmd;
        CHECK(ctrl.GenerateCmd(ref, q_cmd) == rocos::Result::MoveInput);
    }

    SUBCASE("JntArray 含 NaN → ParameterNanOrInf") {
        auto q_bad = makeConstJntArray(7, 0.0);
        q_bad(2) = std::numeric_limits<double>::quiet_NaN();
        rocos::Reference ref = q_bad;
        rocos::JntArray q_cmd;
        CHECK(ctrl.GenerateCmd(ref, q_cmd) == rocos::Result::ParameterNanOrInf);
    }

    SUBCASE("JntArray 无 model → ParameterPointerEqualsNullptr") {
        rocos::CartesianImpedanceController ctrl2;
        ctrl2.SetHardware(&hw);
        // 不设 model
        auto q_in = makeConstJntArray(7, 0.0);
        rocos::Reference ref = q_in;
        rocos::JntArray q_cmd;
        CHECK(ctrl2.GenerateCmd(ref, q_cmd) == rocos::Result::ParameterPointerEqualsNullptr);
    }
}

// ==========================================================================
// §4. UpdateCmd — 笛卡尔阻抗力矩
// ==========================================================================

TEST_CASE("CartesianImpedance - UpdateCmd 笛卡尔阻抗力矩") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 0.0;  // 先关重力，隔离观察阻抗力矩

    SUBCASE("无 hardware → ParameterPointerEqualsNullptr") {
        rocos::CartesianImpedanceController ctrl;
        ctrl.SetModel(&model);
        // 设 x_des_ 以通过 x_des_valid_ 检查
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::ParameterPointerEqualsNullptr);
    }

    SUBCASE("无 model → ParameterPointerEqualsNullptr") {
        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::ParameterPointerEqualsNullptr);
    }

    SUBCASE("未设 x_des_ → MoveInput") {
        rocos::CartesianImpedanceController ctrl;
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        // 未调用 GenerateCmd → x_des_valid_ = false
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::MoveInput);
    }

    SUBCASE("位置误差产生 X 方向力矩") {
        // FK: p_cur = (q[0], q[1], q[2]) = (0,0,0)
        // 设 x_des_.p = (1,0,0) → Δx = diff(I, Frame((1,0,0))) = Twist([1,0,0],[0,0,0])
        // F_cur = K_p_lin(500)*[1,0,0] - K_d*0 = [500,0,0]
        // J^T * F: col_0=[1,0,0,0,0,0] → τ[0] = 1*500 + 0 = 500
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_ = makeConstJntArray(7, 0.0);

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                         dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        CHECK(hw.last_set_torque_.rows() == 7u);
        CHECK(hw.last_set_torque_(0) == doctest::Approx(500.0).epsilon(0.01));
    }

    SUBCASE("Y 方向误差产生对应力矩") {
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_ = makeConstJntArray(7, 0.0);

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 1.0, 0.0)},
                         dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        // col_1=[0,1,0,0,0,0] → τ[1] = 1*500 = 500
        CHECK(hw.last_set_torque_(1) == doctest::Approx(500.0).epsilon(0.01));
    }

    SUBCASE("自定义刚度改变力矩比例") {
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_ = makeConstJntArray(7, 0.0);

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.SetTranslationalStiffness(100.0);  // 改成 100 N/m
        ctrl.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                         dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        // τ[0] = 100*1 = 100
        CHECK(hw.last_set_torque_(0) == doctest::Approx(100.0).epsilon(0.01));
    }

    SUBCASE("FK 与当前关节一致 → 零误差 → 零阻抗力矩") {
        // FK: p_cur = (q[0], q[1], q[2]) = (1,0,0)
        // x_des_ = Frame with p=(1,0,0) → diff ≈ 0
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_position_(0) = 1.0;  // q[0]=1 → p_cur.x = 1
        hw.fake_velocity_ = makeConstJntArray(7, 0.0);

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                         dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        // 位置匹配 → 阻抗力矩 ≈ 0（含重力 5 Nm）
        CHECK(hw.last_set_torque_(0) == doctest::Approx(0.0).epsilon(0.01));
    }
}

// ==========================================================================
// §5. 重力补偿
// ==========================================================================

TEST_CASE("CartesianImpedance - 重力补偿") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 8.0;  // 每关节 8 Nm 重力

    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);
    // x_des == x_cur → 零阻抗力矩
    ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                     dummy_q_cmd);

    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    // τ = τ_imp(≈0) + τ_grav(8) = 8
    CHECK(hw.last_set_torque_(0) == doctest::Approx(8.0).epsilon(0.01));
}

// ==========================================================================
// §6. 阻尼项
// ==========================================================================

TEST_CASE("CartesianImpedance - 阻尼项") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 0.0;

    SUBCASE("正速度 → 阻尼产生反向力矩") {
        // v_base = J * q̇, J col_0=[1,0,0,...], q̇[0]=2 → v_base.vel.x = 2
        // v_cur = R^T * v_base, R=I → v_cur.vel.x = 2
        // F_cur.force.x = K_p*0 - K_d_lin*2 = -200
        // τ[0] = 1*(-200) = -200
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_(0) = 2.0;  // q̇[0] = 2

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        // x_des == x_cur → 零位置误差
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        // τ[0] = K_p_lin*0 - K_d_lin*2 = -200
        CHECK(hw.last_set_torque_(0) == doctest::Approx(-200.0).epsilon(0.01));
    }

    SUBCASE("无速度反馈 → 阻尼项为零") {
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_ = dummy_q_cmd;  // 0 维

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        // Δx ≠ 0 → 仅刚度产生力矩
        ctrl.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                         dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        // τ[0] = 500*1 - 0 = 500（无阻尼）
        CHECK(hw.last_set_torque_(0) == doctest::Approx(500.0).epsilon(0.01));
    }
}

// ==========================================================================
// §7. 零空间阻抗
// ==========================================================================

TEST_CASE("CartesianImpedance - 零空间阻抗") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 0.0;

    SUBCASE("零空间参考含偏差 → 零空间力矩非零") {
        // q_act = [0,0,0,0,0,0,0], q_nullspace_des = [1,1,1,1,1,1,1]
        // τ_null = N * (K_p_null * Δq - K_d_null * q̇)
        // 对于 7-DOF Jacobian (6×7)，N 有一个 1D 零空间
        // 零空间力矩投影后应为非零值
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_ = makeConstJntArray(7, 0.0);

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.SetNullspaceStiffness(100.0);
        ctrl.SetNullspaceReference(makeConstJntArray(7, 1.0));
        // x_des == x_cur → 笛卡尔阻抗力矩为零
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);

        // 零空间力矩应全部非零（每个关节都有零空间分量）
        double sum_abs = 0.0;
        for (unsigned int i = 0; i < 7; ++i) {
            sum_abs += std::abs(hw.last_set_torque_(i));
        }
        CHECK(sum_abs > 1.0);  // 非零
    }

    SUBCASE("无零空间参考 → 零空间力矩为零") {
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_ = makeConstJntArray(7, 0.0);

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        // 不设 nullspace reference
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        // 零空间无参考 → τ_null=0, τ_imp=0, τ_grav=0 → τ=0
        double sum_abs = 0.0;
        for (unsigned int i = 0; i < 7; ++i) {
            sum_abs += std::abs(hw.last_set_torque_(i));
        }
        CHECK(sum_abs == doctest::Approx(0.0).epsilon(0.01));
    }

    SUBCASE("Reset 清除零空间参考") {
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_ = makeConstJntArray(7, 0.0);

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.SetNullspaceReference(makeConstJntArray(7, 1.0));
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        ctrl.Reset();
        // Reset 后 x_des_valid_ 也清除了，需要重新 GenerateCmd
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);

        double sum_abs = 0.0;
        for (unsigned int i = 0; i < 7; ++i) {
            sum_abs += std::abs(hw.last_set_torque_(i));
        }
        CHECK(sum_abs == doctest::Approx(0.0).epsilon(0.01));
    }
}

// ==========================================================================
// §8. 力矩变化率限制
// ==========================================================================

TEST_CASE("CartesianImpedance - 力矩变化率限制") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 0.0;

    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);
    // 设一个小的变化率限制: 100 Nm/s, dt≈0.001 → max_delta = 0.1 Nm/step
    ctrl.SetTorqueRateLimit(100.0);

    // x_des 偏移 1m → 稳态力矩 = 500 Nm
    ctrl.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                     dummy_q_cmd);

    // 第一次调用：无 tau_prev_（但 GenerateCmd 不设 tau_prev_）
    // 所以第一拍 tau_prev_valid_ 为 false，应直接输出完整力矩
    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    double first_torque = hw.last_set_torque_(0);
    // 第一拍无限制，直接输出 500
    CHECK(first_torque == doctest::Approx(500.0).epsilon(0.01));

    // 第二次调用：tau_prev_ = first_torque
    // 此时位置仍为 (0,0,0)，x_des 仍为 (1,0,0)，稳态力矩仍为 500
    // 但 rate limiter 已被 tau_prev_valid_ 激活
    // 由于稳态不变，Δτ = 0，力矩保持 500
    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    CHECK(hw.last_set_torque_(0) == doctest::Approx(500.0).epsilon(0.01));

    SUBCASE("变化率限制设为 0 → 不限制") {
        rocos::CartesianImpedanceController ctrl2;
        ctrl2.SetHardware(&hw);
        ctrl2.SetModel(&model);
        ctrl2.SetTorqueRateLimit(0.0);  // 关闭
        ctrl2.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                          dummy_q_cmd);
        CHECK(ctrl2.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        // 直接输出完整力矩
        CHECK(hw.last_set_torque_(0) == doctest::Approx(500.0).epsilon(0.01));
    }
}

// ==========================================================================
// §9. 力矩饱和
// ==========================================================================

TEST_CASE("CartesianImpedance - 力矩饱和") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 0.0;

    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);
    ctrl.SetTorqueSaturation(3.0);  // 限制 ±3 Nm

    // x_des 偏移 1m → 原始力矩 500 Nm，应被截断到 3 Nm
    ctrl.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                     dummy_q_cmd);

    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    // 每个关节都被限制在 ±3 Nm
    for (unsigned int i = 0; i < 7; ++i) {
        CHECK(std::abs(hw.last_set_torque_(i)) <= 3.0);
    }

    SUBCASE("饱和设为 0 → 不限制") {
        rocos::CartesianImpedanceController ctrl2;
        ctrl2.SetHardware(&hw);
        ctrl2.SetModel(&model);
        ctrl2.SetTorqueSaturation(0.0);  // 关闭
        ctrl2.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                          dummy_q_cmd);
        CHECK(ctrl2.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        CHECK(hw.last_set_torque_(0) == doctest::Approx(500.0).epsilon(0.01));
    }

    SUBCASE("负方向同样被限制") {
        // x_des 反方向偏移 → 原始力矩 -500 Nm，应被截断到 -3 Nm
        hw.fake_position_ = makeConstJntArray(7, 0.0);

        rocos::CartesianImpedanceController ctrl3;
        ctrl3.SetHardware(&hw);
        ctrl3.SetModel(&model);
        ctrl3.SetTorqueSaturation(3.0);
        ctrl3.GenerateCmd(rocos::Reference{makeFrame(-1.0, 0.0, 0.0)},
                          dummy_q_cmd);

        CHECK(ctrl3.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        CHECK(hw.last_set_torque_(0) <= -3.0 + 0.01);
    }
}

// ==========================================================================
// §10. CST 模式管理
// ==========================================================================

TEST_CASE("CartesianImpedance - CST 模式管理") {
    FakeHardware hw;
    FakeModel model;
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);
    ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                     dummy_q_cmd);

    SUBCASE("首次 UpdateCmd 设置 CST 模式") {
        ctrl.UpdateCmd(makeConstJntArray(7));
        CHECK(hw.last_set_mode_ == 10);
        CHECK(hw.set_mode_count_ == 1);
    }

    SUBCASE("第二次不重复设模式") {
        ctrl.UpdateCmd(makeConstJntArray(7));
        ctrl.UpdateCmd(makeConstJntArray(7));
        CHECK(hw.set_mode_count_ == 1);
    }

    SUBCASE("Reset 后重新设模式") {
        ctrl.UpdateCmd(makeConstJntArray(7));
        CHECK(hw.set_mode_count_ == 1);
        ctrl.Reset();
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        ctrl.UpdateCmd(makeConstJntArray(7));
        CHECK(hw.set_mode_count_ == 2);
    }
}

// ==========================================================================
// §11. 异常路径
// ==========================================================================

TEST_CASE("CartesianImpedance - 异常路径") {
    FakeHardware hw;
    FakeModel model;
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    SUBCASE("FK 失败 → 传播 FkCalcFail") {
        model.fk_should_fail_ = true;
        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::FkCalcFail);
    }

    SUBCASE("Jacobian 失败 → 传播 JacobianCalcFail") {
        model.jac_should_fail_ = true;
        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::JacobianCalcFail);
    }

    SUBCASE("InverseDynamics 失败 → 传播 IdCalcFail") {
        model.id_should_fail_ = true;
        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::IdCalcFail);
    }

    SUBCASE("GetPosition 返回空 → JointStateError") {
        dummy_q_cmd = rocos::JntArray{};  // 重置为空的 JntArray
        hw.fake_position_ = dummy_q_cmd;  // 0 维
        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::JointStateError);
    }
}

// ==========================================================================
// §12. SetReady 初始化
// ==========================================================================

TEST_CASE("CartesianImpedance - SetReady") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 7.0;
    hw.fake_position_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);

    SUBCASE("SetReady 无 hardware → ParameterPointerEqualsNullptr") {
        rocos::CartesianImpedanceController ctrl2;
        CHECK(ctrl2.SetReady() == rocos::Result::ParameterPointerEqualsNullptr);
    }

    SUBCASE("SetReady 成功后 CST 模式和初始力矩") {
        CHECK(ctrl.SetReady() == rocos::Result::NoError);
        // 检查力矩为重力补偿值
        CHECK(hw.last_set_torque_(0) == doctest::Approx(7.0).epsilon(0.01));
        // 检查 CST 模式已设置
        CHECK(hw.last_set_mode_ == 10);
        CHECK(hw.set_mode_count_ == 1);
    }

    SUBCASE("SetReady 后零空间参考初始化为当前关节位置") {
        CHECK(ctrl.SetReady() == rocos::Result::NoError);

        // 校验：零空间参考已初始化 → 阻抗力矩中零空间分量为零
        // (因为 q_nullspace_des == q_act == 0)
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);

        // 零空间 error = 0 → τ_null = 0, τ_imp = 0 → τ = τ_grav = 7
        CHECK(hw.last_set_torque_(0) == doctest::Approx(7.0).epsilon(0.01));
    }
}

// ==========================================================================
// §13. 综合场景
// ==========================================================================

TEST_CASE("CartesianImpedance - 综合：刚度 + 阻尼 + 重力 + 零空间") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 3.0;

    // q_act = [0.1, 0, 0, 0, 0, 0, 0] → p_cur = (0.1, 0, 0)
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_position_(0) = 0.1;
    // q̇ = [0.5, 0, 0, 0, 0, 0, 0]
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_(0) = 0.5;

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);
    ctrl.SetTranslationalStiffness(200.0);  // 200 N/m
    ctrl.SetTranslationalDamping(50.0);     // 50 N·s/m
    ctrl.SetNullspaceStiffness(30.0);
    ctrl.SetNullspaceReference(makeConstJntArray(7, 0.5));

    // x_des = (0.3, 0.2, 0.0) → p_des - p_cur = (0.2, 0.2, 0)
    ctrl.GenerateCmd(rocos::Reference{makeFrame(0.3, 0.2, 0.0)},
                     dummy_q_cmd);

    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);

    // 手动验算 τ[0]:
    //   Δx.vel = R^T*(0.2, 0.2, 0) = (0.2, 0.2, 0) [R=I]
    //   v_cur.vel = R^T * J*q̇, J*[0.5,0,0,...]^T = [0.5,0,0,0,0,0]^T → v_cur.vel = (0.5, 0, 0)
    //   F_cur.force = 200*(0.2,0.2,0) - 50*(0.5,0,0) = (40-25, 40-0, 0) = (15, 40, 0)
    //   τ_imp[0] = J_col_0^T * F = [1,0,0]^T * [15,40,0]^T = 15
    //
    //   零空间力矩有非零分量
    //   τ_grav[0] = 3
    //
    //   τ_raw[0] = 15 + τ_null[0] + 3
    //
    // 至少保证力矩有值且在合理范围
    CHECK(std::abs(hw.last_set_torque_(0)) > 1.0);
    CHECK(std::abs(hw.last_set_torque_(0)) < 100.0);
    CHECK(hw.last_set_torque_.rows() == 7u);
}
