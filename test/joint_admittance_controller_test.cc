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
#include <variant>

#include <test/doctest.h>

#include "src/joint_impedance_controller.hpp"

// ==========================================================================
// Fake 实现
// ==========================================================================

namespace {

class FakeHardware : public rocos::HardwareInterface {
public:
    rocos::JntArray fake_position_;
    rocos::JntArray fake_velocity_;
    rocos::JntArray last_set_torque_;
    int8_t last_set_mode_{-1};
    int set_mode_count_{0};

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

/// @brief FakeModel: IK 透传种子; ID 返回预设重力力矩
class FakeModel : public rocos::ModelInterface {
public:
    bool ik_should_fail_{false};
    bool id_should_fail_{false};
    double grav_torque_{5.0};

    rocos::Result GetJacobian(const rocos::JntArray &q, rocos::Jacobian &J_out) override {
        return rocos::Result::NoError;
    }

    rocos::Result ForwardKinematics(const rocos::JntArray&,
                                    rocos::Frame&) override {
        return rocos::Result::NoError;
    }

    rocos::Result InverseKinematics(const rocos::JntArray& q_in,
                                     const rocos::Frame&,
                                     rocos::JntArray& q_out) override {
        if (ik_should_fail_) return rocos::Result::IkCalcFail;
        q_out = q_in;
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
};

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

}  // namespace

// ==========================================================================
// 1. 基础接口
// ==========================================================================

TEST_CASE("JointImpedanceController - SetHardware / SetModel") {
    rocos::JointImpedanceController ctrl;
    FakeHardware hw;
    FakeModel model;

    CHECK(ctrl.SetHardware(nullptr) == rocos::Result::ParameterPointerEqualsNullptr);
    CHECK(ctrl.SetHardware(&hw) == rocos::Result::NoError);
    CHECK(ctrl.SetModel(nullptr) == rocos::Result::ParameterPointerEqualsNullptr);
    CHECK(ctrl.SetModel(&model) == rocos::Result::NoError);
}

TEST_CASE("JointImpedanceController - Reset") {
    rocos::JointImpedanceController ctrl;
    CHECK(ctrl.Reset() == true);
}

// ==========================================================================
// 2. 增益设置
// ==========================================================================

TEST_CASE("JointImpedanceController - 增益设置") {
    rocos::JointImpedanceController ctrl;

    SUBCASE("SetStiffness 正常") {
        CHECK(ctrl.SetStiffness(makeConstJntArray(3, 100.0)) == rocos::Result::NoError);
    }
    SUBCASE("SetStiffness 负值 → ParameterNanOrInf") {
        CHECK(ctrl.SetStiffness(makeConstJntArray(3, -10.0)) == rocos::Result::ParameterNanOrInf);
    }
    SUBCASE("SetStiffness NaN → ParameterNanOrInf") {
        auto K = makeConstJntArray(3, 100.0);
        K(1) = std::numeric_limits<double>::quiet_NaN();
        CHECK(ctrl.SetStiffness(K) == rocos::Result::ParameterNanOrInf);
    }
    SUBCASE("SetDamping 正常") {
        CHECK(ctrl.SetDamping(makeConstJntArray(3, 20.0)) == rocos::Result::NoError);
    }
    SUBCASE("SetDamping 负值 → ParameterNanOrInf") {
        CHECK(ctrl.SetDamping(makeConstJntArray(3, -5.0)) == rocos::Result::ParameterNanOrInf);
    }
}

// ==========================================================================
// 3. GenerateCmd
// ==========================================================================

TEST_CASE("JointImpedanceController - GenerateCmd") {
    rocos::JointImpedanceController ctrl;
    FakeHardware hw;
    hw.fake_position_ = makeJntArray(3);
    ctrl.SetHardware(&hw);

    SUBCASE("JntArray 透传") {
        auto q_in = makeJntArray(3);
        rocos::Reference ref = q_in;
        rocos::JntArray q_cmd;
        CHECK(ctrl.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
        CHECK(q_cmd.rows() == 3u);
    }

    SUBCASE("Frame → IK") {
        FakeModel model;
        ctrl.SetModel(&model);
        rocos::Frame frame;
        rocos::Reference ref = frame;
        rocos::JntArray q_cmd;
        CHECK(ctrl.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
    }
}

// ==========================================================================
// 4. UpdateCmd — 阻抗控制律: τ = K_p·Δq - K_d·q̇ + τ_grav
// ==========================================================================

TEST_CASE("JointImpedanceController - UpdateCmd 力矩计算") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 5.0;

    SUBCASE("静止 + 位置误差 + 重力补偿") {
        hw.fake_position_ = makeConstJntArray(3, 0.0);
        hw.fake_velocity_ = makeConstJntArray(3, 0.0);

        rocos::JointImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);

        // q_des = [1, 1, 1], q_act = [0, 0, 0], q̇ = [0, 0, 0]
        // τ = 100*(1-0) - 20*0 + 5 = 105
        auto q_des = makeConstJntArray(3, 1.0);
        auto res = ctrl.UpdateCmd(q_des);
        CHECK(res == rocos::Result::NoError);
        CHECK(hw.last_set_torque_(0) == doctest::Approx(105.0));
    }

    SUBCASE("有速度阻尼") {
        hw.fake_position_ = makeConstJntArray(3, 0.0);
        hw.fake_velocity_ = makeConstJntArray(3, 2.0);

        rocos::JointImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);

        // Δq = 0, q̇ = 2 → τ = 100*0 - 20*2 + 5 = -35
        auto q_des = makeConstJntArray(3, 0.0);
        ctrl.UpdateCmd(q_des);
        CHECK(hw.last_set_torque_(0) == doctest::Approx(-35.0));
    }

    SUBCASE("无模型时 τ_grav = 0") {
        hw.fake_position_ = makeConstJntArray(3, 0.0);
        hw.fake_velocity_ = makeConstJntArray(3, 0.0);

        rocos::JointImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        // 不设 model → τ_grav = 0

        auto q_des = makeConstJntArray(3, 1.0);
        ctrl.UpdateCmd(q_des);
        // τ = 100*(1-0) + 0 = 100
        CHECK(hw.last_set_torque_(0) == doctest::Approx(100.0));
    }

    SUBCASE("显式增益") {
        hw.fake_position_ = makeConstJntArray(3, 0.5);
        hw.fake_velocity_ = makeConstJntArray(3, 0.1);

        rocos::JointImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.SetStiffness(makeConstJntArray(3, 50.0));
        ctrl.SetDamping(makeConstJntArray(3, 10.0));

        // τ = 50*(1-0.5) - 10*0.1 + 5 = 25 - 1 + 5 = 29
        auto q_des = makeConstJntArray(3, 1.0);
        ctrl.UpdateCmd(q_des);
        CHECK(hw.last_set_torque_(0) == doctest::Approx(29.0));
    }
}

// ==========================================================================
// 5. CST 模式
// ==========================================================================

TEST_CASE("JointImpedanceController - CST 模式管理") {
    FakeHardware hw;
    hw.fake_position_ = makeConstJntArray(3, 0.0);
    hw.fake_velocity_ = makeConstJntArray(3, 0.0);

    rocos::JointImpedanceController ctrl;
    ctrl.SetHardware(&hw);

    SUBCASE("首次 UpdateCmd 设置 CST 模式") {
        ctrl.UpdateCmd(makeJntArray(3));
        CHECK(hw.last_set_mode_ == 10);
        CHECK(hw.set_mode_count_ == 1);
    }

    SUBCASE("第二次不重复设模式") {
        ctrl.UpdateCmd(makeJntArray(3));
        ctrl.UpdateCmd(makeJntArray(3));
        CHECK(hw.set_mode_count_ == 1);
    }

    SUBCASE("Reset 后重新设模式") {
        ctrl.UpdateCmd(makeJntArray(3));
        ctrl.Reset();
        ctrl.UpdateCmd(makeJntArray(3));
        CHECK(hw.set_mode_count_ == 2);
    }
}

// ==========================================================================
// 6. 异常路径
// ==========================================================================

TEST_CASE("JointImpedanceController - 异常路径") {
    FakeHardware hw;
    FakeModel model;
    hw.fake_position_ = makeConstJntArray(3, 0.0);
    hw.fake_velocity_ = makeConstJntArray(3, 0.0);

    SUBCASE("InverseDynamics 失败 → IdCalcFail") {
        model.id_should_fail_ = true;
        rocos::JointImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        CHECK(ctrl.UpdateCmd(makeJntArray(3)) == rocos::Result::IdCalcFail);
    }

    SUBCASE("q_des 维度与增益不匹配") {
        rocos::JointImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetStiffness(makeConstJntArray(5, 100.0));
        ctrl.SetDamping(makeConstJntArray(5, 20.0));
        CHECK(ctrl.UpdateCmd(makeJntArray(3)) == rocos::Result::UnmatchedJointsNumber);
    }

    SUBCASE("GetPosition 返回 0 轴 → JointStateError") {
        hw.fake_position_ = rocos::JntArray{};
        rocos::JointImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        CHECK(ctrl.UpdateCmd(makeJntArray(3)) == rocos::Result::JointStateError);
    }
}
