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

#include "src/joint_admittance_controller.hpp"

// ==========================================================================
// Fake 实现
// ==========================================================================

namespace {

class FakeHardware : public rocos::HardwareInterface {
public:
    rocos::JntArray fake_position_;
    rocos::JntArray fake_velocity_;
    rocos::JntArray fake_torque_;
    rocos::JntArray last_set_torque_;
    rocos::JntArray last_set_position_;
    int8_t last_set_mode_{-1};
    int set_mode_count_{0};
    int set_position_count_{0};
    int wait_for_signal_count_{0};
    uint32_t dt_us_{1000};

    rocos::JntState GetState() override { return rocos::JntState::ENABLED; }

    rocos::JntState GetJointState(int32_t id) override { return rocos::JntState::ENABLED; }

    ~FakeHardware() override {}

    // DriveInterface — batch
    rocos::JntArray GetPosition() override { return fake_position_; }
    rocos::JntArray GetVelocity() override { return fake_velocity_; }
    rocos::JntArray GetTorque() override { return fake_torque_; }
    rocos::JntArray GetLoadTorque() override { return {}; }
    void SetPosition(const rocos::JntArray& q) override {
        last_set_position_ = q;
        ++set_position_count_;
    }
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
    void WaitForSignal() override { ++wait_for_signal_count_; }
    uint32_t GetDt() const override { return dt_us_; }
};

/// @brief FakeModel: IK 透传种子; ID 返回预设重力力矩
class FakeModel : public rocos::ModelInterface {
public:
    FakeModel() { SetLimits(3, -10.0, 10.0, 10000.0, 1000.0); }

    bool ik_should_fail_{false};
    bool id_should_fail_{false};
    double grav_torque_{5.0};
    rocos::JntArray lower_limit_;
    rocos::JntArray upper_limit_;
    rocos::JntArray velocity_limit_;
    rocos::JntArray effort_limit_;

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

    int GetJointNum() const override { return static_cast<int>(lower_limit_.rows()); }
    std::vector<std::string> GetJointNames() const override { return {}; }

    const rocos::JntArray& GetPosLowerLimit() const override { return lower_limit_; }
    const rocos::JntArray& GetPosUpperLimit() const override { return upper_limit_; }
    const rocos::JntArray& GetVelocityLimit() const override { return velocity_limit_; }
    const rocos::JntArray& GetEffortLimit() const override { return effort_limit_; }

    void SetLimits(unsigned int n, double lower, double upper,
                   double velocity, double effort) {
        lower_limit_.resize(n);
        upper_limit_.resize(n);
        velocity_limit_.resize(n);
        effort_limit_.resize(n);
        for (unsigned int i = 0; i < n; ++i) {
            lower_limit_(i) = lower;
            upper_limit_(i) = upper;
            velocity_limit_(i) = velocity;
            effort_limit_(i) = effort;
        }
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

TEST_CASE("JointAdmittanceController - SetHardware / SetModel") {
    rocos::JointAdmittanceController ctrl;
    FakeHardware hw;
    FakeModel model;

    CHECK(ctrl.SetHardware(nullptr) == rocos::Result::ParameterPointerEqualsNullptr);
    CHECK(ctrl.SetHardware(&hw) == rocos::Result::NoError);
    CHECK(ctrl.SetModel(nullptr) == rocos::Result::ParameterPointerEqualsNullptr);
    CHECK(ctrl.SetModel(&model) == rocos::Result::NoError);
}

TEST_CASE("JointAdmittanceController - Reset") {
    rocos::JointAdmittanceController ctrl;
    CHECK(ctrl.Reset() == true);
}

// ==========================================================================
// 2. 增益设置
// ==========================================================================

TEST_CASE("JointAdmittanceController - 增益设置") {
    rocos::JointAdmittanceController ctrl;

    SUBCASE("SetInertia 正常") {
        CHECK(ctrl.SetInertia(makeConstJntArray(3, 100.0)) == rocos::Result::NoError);
    }
    SUBCASE("SetInertia 负值 → ParameterNanOrInf") {
        CHECK(ctrl.SetInertia(makeConstJntArray(3, -10.0)) == rocos::Result::ParameterNanOrInf);
    }
    SUBCASE("SetInertia NaN → ParameterNanOrInf") {
        auto K = makeConstJntArray(3, 100.0);
        K(1) = std::numeric_limits<double>::quiet_NaN();
        CHECK(ctrl.SetInertia(K) == rocos::Result::ParameterNanOrInf);
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

TEST_CASE("JointAdmittanceController - GenerateCmd") {
    rocos::JointAdmittanceController ctrl;
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

TEST_CASE("JointAdmittanceController - UpdateCmd 导纳输出") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 5.0;

    SUBCASE("外力为零时 q_out ≈ q_des") {
        hw.fake_position_ = makeConstJntArray(3, 0.0);
        hw.fake_velocity_ = makeConstJntArray(3, 0.0);
        hw.fake_torque_    = makeConstJntArray(3, 5.0);  // τ_act = τ_grav → τ_ext = 0

        rocos::JointAdmittanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);

        auto q_des = makeConstJntArray(3, 1.0);
        CHECK(ctrl.UpdateCmd(q_des) == rocos::Result::NoError);

        // τ_ext = 0 时导纳偏移应为 0，q_out = q_des
        CHECK(hw.last_set_position_.rows() == 3);
        CHECK(hw.last_set_position_(0) == doctest::Approx(1.0));
        CHECK(hw.last_set_mode_ == 8);  // CSP 模式
    }

    SUBCASE("有外力时产生导纳位置偏移") {
        hw.fake_position_ = makeConstJntArray(3, 0.0);
        hw.fake_velocity_ = makeConstJntArray(3, 0.0);
        hw.fake_torque_    = makeConstJntArray(3, 15.0);  // τ_ext = 15 - 5 = 10 Nm

        rocos::JointAdmittanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);

        auto q_des = makeConstJntArray(3, 0.0);
        CHECK(ctrl.UpdateCmd(q_des) == rocos::Result::NoError);

        // τ_ext > 0，q_out 应有正向偏移
        CHECK(hw.last_set_position_(0) > 0.0);
    }

    SUBCASE("缺少 model 时拒绝下发") {
        hw.fake_position_ = makeConstJntArray(3, 0.0);
        hw.fake_velocity_ = makeConstJntArray(3, 0.0);
        hw.fake_torque_    = makeConstJntArray(3, 0.0);

        rocos::JointAdmittanceController ctrl;
        ctrl.SetHardware(&hw);

        auto q_des = makeConstJntArray(3, 1.0);
        CHECK(ctrl.UpdateCmd(q_des) == rocos::Result::ParameterPointerEqualsNullptr);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_position_count_ == 0);
    }

    SUBCASE("输出位置超过 URDF 上限时拒绝下发") {
        hw.fake_position_ = makeConstJntArray(3, 0.0);
        hw.fake_velocity_ = makeConstJntArray(3, 0.0);
        hw.fake_torque_    = makeConstJntArray(3, 5.0);
        model.SetLimits(3, -1.0, 1.0, 10000.0, 1000.0);

        rocos::JointAdmittanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(3, 2.0)) == rocos::Result::PosLimit);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_position_count_ == 0);
    }

    SUBCASE("当前位置到输出位置所需速度超过 URDF velocity 时拒绝下发") {
        hw.fake_position_ = makeConstJntArray(3, 0.0);
        hw.fake_velocity_ = makeConstJntArray(3, 0.0);
        hw.fake_torque_    = makeConstJntArray(3, 5.0);
        hw.dt_us_ = 1000;
        model.SetLimits(3, -10.0, 10.0, 0.5, 1000.0);

        rocos::JointAdmittanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(3, 1.0)) == rocos::Result::SpeedLimit);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_position_count_ == 0);
    }

    SUBCASE("显式增益") {
        hw.fake_position_ = makeConstJntArray(3, 0.0);
        hw.fake_velocity_ = makeConstJntArray(3, 0.0);
        hw.fake_torque_    = makeConstJntArray(3, 5.0);

        rocos::JointAdmittanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.SetInertia(makeConstJntArray(3, 1.0));
        ctrl.SetDamping(makeConstJntArray(3, 10.0));

        auto q_des = makeConstJntArray(3, 1.0);
        CHECK(ctrl.UpdateCmd(q_des) == rocos::Result::NoError);
        CHECK(hw.last_set_position_(0) == doctest::Approx(1.0));
    }
}

// ==========================================================================
// 5. CSP 模式
// ==========================================================================

TEST_CASE("JointAdmittanceController - CSP 模式管理") {
    FakeHardware hw;
    hw.fake_position_ = makeConstJntArray(3, 0.0);
    hw.fake_velocity_ = makeConstJntArray(3, 0.0);
    hw.fake_torque_    = makeConstJntArray(3, 0.0);

    rocos::JointAdmittanceController ctrl;
    ctrl.SetHardware(&hw);
    FakeModel model;
    ctrl.SetModel(&model);

    SUBCASE("首次 UpdateCmd 设置 CSP 模式") {
        ctrl.UpdateCmd(makeJntArray(3));
        CHECK(hw.last_set_mode_ == 8);
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

TEST_CASE("JointAdmittanceController - SetReady routes through UpdateCmd") {
    FakeHardware hw;
    FakeModel model;
    hw.fake_position_ = makeConstJntArray(3, 0.25);
    hw.fake_velocity_ = makeConstJntArray(3, 0.0);
    hw.fake_torque_ = makeConstJntArray(3, 5.0);

    SUBCASE("缺少 model 时不直接写硬件") {
        rocos::JointAdmittanceController ctrl;
        ctrl.SetHardware(&hw);

        CHECK(ctrl.SetReady() == rocos::Result::ParameterPointerEqualsNullptr);
        CHECK(hw.wait_for_signal_count_ == 1);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_position_count_ == 0);
    }

    SUBCASE("ready 通过 UpdateCmd 锁定当前位置") {
        rocos::JointAdmittanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);

        CHECK(ctrl.SetReady() == rocos::Result::NoError);
        CHECK(hw.wait_for_signal_count_ == 1);
        CHECK(hw.set_mode_count_ == 1);
        CHECK(hw.set_position_count_ == 1);
        CHECK(hw.last_set_position_(0) == doctest::Approx(0.25));
    }
}

TEST_CASE("JointAdmittanceController - destructor routes through UpdateCmd") {
    FakeHardware hw;
    FakeModel model;
    hw.fake_position_ = makeConstJntArray(3, 0.25);
    hw.fake_velocity_ = makeConstJntArray(3, 0.0);
    hw.fake_torque_ = makeConstJntArray(3, 5.0);

    {
        rocos::JointAdmittanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
    }

    CHECK(hw.wait_for_signal_count_ == 1);
    CHECK(hw.set_mode_count_ == 1);
    CHECK(hw.set_position_count_ == 1);
    CHECK(hw.last_set_position_(0) == doctest::Approx(0.25));
}

// ==========================================================================
// 6. 异常路径
// ==========================================================================

TEST_CASE("JointAdmittanceController - 异常路径") {
    FakeHardware hw;
    FakeModel model;
    hw.fake_position_ = makeConstJntArray(3, 0.0);
    hw.fake_velocity_ = makeConstJntArray(3, 0.0);
    hw.fake_torque_    = makeConstJntArray(3, 0.0);

    SUBCASE("InverseDynamics 失败 → IdCalcFail") {
        model.id_should_fail_ = true;
        rocos::JointAdmittanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        CHECK(ctrl.UpdateCmd(makeJntArray(3)) == rocos::Result::IdCalcFail);
    }

    SUBCASE("q_des 维度与增益不匹配") {
        rocos::JointAdmittanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.SetInertia(makeConstJntArray(5, 1.0));
        ctrl.SetDamping(makeConstJntArray(5, 20.0));
        CHECK(ctrl.UpdateCmd(makeJntArray(3)) == rocos::Result::UnmatchedJointsNumber);
    }

    SUBCASE("GetPosition 返回 0 轴 → JointStateError") {
        hw.fake_position_ = rocos::JntArray{};
        rocos::JointAdmittanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        CHECK(ctrl.UpdateCmd(makeJntArray(3)) == rocos::Result::JointStateError);
    }
}
