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
    rocos::JntArray fake_torque_;
    rocos::JntArray last_set_position_;
    int8_t last_set_mode_{-1};
    int set_mode_count_{0};

    // DriveInterface — batch
    rocos::JntArray GetPosition() override { return fake_position_; }
    rocos::JntArray GetVelocity() override { return fake_velocity_; }
    rocos::JntArray GetTorque() override { return fake_torque_; }
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
    bool GetDigitalInput(int, int) override { return false; }
    void SetDigitalOutput(int, int, bool) override {}
    double GetAnalogInput(int, int) override { return 0.0; }
    void SetAnalogOutput(int, int, double) override {}

    // HardwareInterface
    bool Reset() override { return true; }
};

/// @brief FakeModel: IK 透传种子; ID 返回零力矩（模拟无重力场景）
class FakeModel : public rocos::ModelInterface {
public:
    bool ik_should_fail_{false};
    bool id_should_fail_{false};

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
        // 返回零力矩 — 模拟无重力环境
        for (unsigned int i = 0; i < torques.rows(); ++i) {
            torques(i) = 0.0;
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

TEST_CASE("JointImpedanceController - Reset") {
    rocos::JointImpedanceController ctrl;
    CHECK(ctrl.Reset() == true);
}

// ==========================================================================
// 2. 参数设置
// ==========================================================================

TEST_CASE("JointImpedanceController - 导纳参数设置") {
    rocos::JointImpedanceController ctrl;

    SUBCASE("SetInertia 正常") {
        CHECK(ctrl.SetInertia(makeConstJntArray(3, 1.0)) == rocos::Result::NoError);
    }
    SUBCASE("SetInertia 非正 → ParameterNanOrInf") {
        auto M = makeConstJntArray(3, 0.0);
        CHECK(ctrl.SetInertia(M) == rocos::Result::ParameterNanOrInf);
    }
    SUBCASE("SetDamping 正常") {
        CHECK(ctrl.SetDamping(makeConstJntArray(3, 10.0)) == rocos::Result::NoError);
    }
    SUBCASE("SetDamping 负值 → ParameterNanOrInf") {
        auto B = makeConstJntArray(3, -5.0);
        CHECK(ctrl.SetDamping(B) == rocos::Result::ParameterNanOrInf);
    }
    SUBCASE("SetTorqueOffset 正常") {
        CHECK(ctrl.SetTorqueOffset(makeConstJntArray(3, 0.5)) == rocos::Result::NoError);
    }
    SUBCASE("SetDt 正常") {
        CHECK(ctrl.SetDt(0.002) == rocos::Result::NoError);
    }
    SUBCASE("SetDt 非正 → IllegalParameter") {
        CHECK(ctrl.SetDt(0.0) == rocos::Result::IllegalParameter);
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
// 4. UpdateCmd — 导纳控制核心
// ==========================================================================

TEST_CASE("JointImpedanceController - 导纳控制 UpdateCmd") {
    rocos::JointImpedanceController ctrl;
    FakeHardware hw;
    FakeModel model;

    // 设置硬件状态: 静止, 有外力矩
    hw.fake_position_ = makeConstJntArray(3, 0.0);
    hw.fake_velocity_ = makeConstJntArray(3, 0.0);
    hw.fake_torque_   = makeConstJntArray(3, 10.0);  // τ_act = 10 Nm per joint
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);

    SUBCASE("无 hardware → 报错") {
        rocos::JointImpedanceController ctrl2;
        rocos::JntArray q(1);
        CHECK(ctrl2.UpdateCmd(q) == rocos::Result::ParameterPointerEqualsNullptr);
    }

    SUBCASE("q_des 空 → MoveInput") {
        CHECK(ctrl.UpdateCmd(rocos::JntArray{}) == rocos::Result::MoveInput);
    }

    SUBCASE("外力矩产生正向位置偏移") {
        // τ_ext = 10 - 0 - 0 = 10 Nm
        // q̈ = (10 - B*q̇) / M = (10 - 20*0) / 1 = 10 rad/s²
        // q̇ += 10*0.001 = 0.01,  q += 0.01*0.001 = 0.00001
        auto q_des = makeConstJntArray(3, 0.0);
        auto res = ctrl.UpdateCmd(q_des);
        CHECK(res == rocos::Result::NoError);
        CHECK(hw.last_set_mode_ == 8);                    // CSP 模式

        // q_out = q_des + q_adm ≈ 0.0 + 0.00001
        CHECK(hw.last_set_position_(0) > 0.0);
        CHECK(hw.last_set_position_(0) < 0.001);
    }

    SUBCASE("导纳积分累加 — 多次调用位置持续偏移") {
        // 无模型时 τ_grav = 0
        auto q_des = makeConstJntArray(3, 0.0);

        ctrl.UpdateCmd(q_des);
        double first_out = hw.last_set_position_(0);

        ctrl.UpdateCmd(q_des);
        double second_out = hw.last_set_position_(0);

        // 第二次输出的偏移应大于第一次（积分累加）
        CHECK(second_out > first_out);
    }

    SUBCASE("Reset 后积分归零") {
        auto q_des = makeConstJntArray(3, 0.0);

        ctrl.UpdateCmd(q_des);
        CHECK(hw.last_set_position_(0) > 0.0);   // 有导纳偏移

        ctrl.Reset();
        ctrl.UpdateCmd(q_des);
        // Reset 后状态归零，第一次调用的偏移应回到初始
        CHECK(hw.last_set_position_(0) > 0.0);
        CHECK(hw.last_set_position_(0) < 0.001);  // 接近 0
    }

    SUBCASE("无外力矩时输出等于 q_des") {
        hw.fake_torque_ = makeConstJntArray(3, 0.0);  // τ_act = 0

        // 需要先重置控制器以清零之前的积分
        rocos::JointImpedanceController ctrl2;
        ctrl2.SetHardware(&hw);
        ctrl2.SetModel(&model);

        auto q_des = makeConstJntArray(3, 1.0);
        ctrl2.UpdateCmd(q_des);
        // τ_ext = 0, 积分不累积, q_out = q_des + 0
        CHECK(hw.last_set_position_(0) == doctest::Approx(1.0));
    }

    SUBCASE("负向外力矩产生负向位置偏移") {
        hw.fake_torque_ = makeConstJntArray(3, -10.0);  // τ_act = -10 Nm

        rocos::JointImpedanceController ctrl2;
        ctrl2.SetHardware(&hw);
        ctrl2.SetModel(&model);

        auto q_des = makeConstJntArray(3, 0.0);
        ctrl2.UpdateCmd(q_des);
        CHECK(hw.last_set_position_(0) < 0.0);           // 负向偏移
    }
}

// ==========================================================================
// 5. 重力补偿与零飘
// ==========================================================================

TEST_CASE("JointImpedanceController - 重力补偿与零飘") {
    FakeHardware hw;
    hw.fake_position_ = makeConstJntArray(3, 0.0);
    hw.fake_velocity_ = makeConstJntArray(3, 0.0);

    SUBCASE("有模型时 τ_grav 参与计算") {
        hw.fake_torque_ = makeConstJntArray(3, 15.0);
        FakeModel model;                      // ID 返回零（无重力）

        rocos::JointImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);

        auto q_des = makeConstJntArray(3, 0.0);
        auto res = ctrl.UpdateCmd(q_des);
        CHECK(res == rocos::Result::NoError);
        // τ_ext = 15 - 0 - 0 = 15, 应有正向偏移
        CHECK(hw.last_set_position_(0) > 0.0);
    }

    SUBCASE("无模型时跳过重力补偿仍正常工作") {
        hw.fake_torque_ = makeConstJntArray(3, 10.0);

        rocos::JointImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        // 不设 model

        auto q_des = makeConstJntArray(3, 0.0);
        auto res = ctrl.UpdateCmd(q_des);
        CHECK(res == rocos::Result::NoError);
        CHECK(hw.last_set_position_(0) > 0.0);
    }

    SUBCASE("设置零飘 — τ_offset 抵消部分外力矩") {
        hw.fake_torque_ = makeConstJntArray(3, 10.0);
        FakeModel model;

        rocos::JointImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        // τ_offset = 10 → τ_ext = 10 - 0 - 10 = 0, 无偏移
        ctrl.SetTorqueOffset(makeConstJntArray(3, 10.0));

        auto q_des = makeConstJntArray(3, 1.0);
        ctrl.UpdateCmd(q_des);
        CHECK(hw.last_set_position_(0) == doctest::Approx(1.0));
    }
}

// ==========================================================================
// 6. 阻尼减速
// ==========================================================================

TEST_CASE("JointImpedanceController - 阻尼减速") {
    FakeHardware hw;
    hw.fake_position_ = makeConstJntArray(3, 0.0);
    hw.fake_velocity_ = makeConstJntArray(3, 0.0);
    hw.fake_torque_   = makeConstJntArray(3, 10.0);
    FakeModel model;

    rocos::JointImpedanceController ctrl_high;
    ctrl_high.SetHardware(&hw);
    ctrl_high.SetModel(&model);
    ctrl_high.SetInertia(makeConstJntArray(3, 0.5));
    ctrl_high.SetDamping(makeConstJntArray(3, 50.0));   // 大阻尼

    rocos::JointImpedanceController ctrl_low;
    ctrl_low.SetHardware(&hw);
    ctrl_low.SetModel(&model);
    ctrl_low.SetInertia(makeConstJntArray(3, 0.5));
    ctrl_low.SetDamping(makeConstJntArray(3, 1.0));    // 小阻尼

    auto q_des = makeConstJntArray(3, 0.0);

    // 第一步：阻尼项 B*q̇_adm 均为 0，位移相同
    ctrl_high.UpdateCmd(q_des);
    double first = hw.last_set_position_(0);
    ctrl_low.UpdateCmd(q_des);
    CHECK(hw.last_set_position_(0) == doctest::Approx(first));

    // 多步后：大阻尼速度增长慢 → 位移增量小
    for (int i = 0; i < 50; ++i) {
        ctrl_high.UpdateCmd(q_des);
    }
    double pos_high = hw.last_set_position_(0);

    for (int i = 0; i < 50; ++i) {
        ctrl_low.UpdateCmd(q_des);
    }
    double pos_low = hw.last_set_position_(0);

    CHECK(pos_low > pos_high);
}

// ==========================================================================
// 7. CSP 模式与 Reset
// ==========================================================================

TEST_CASE("JointImpedanceController - CSP 模式管理") {
    rocos::JointImpedanceController ctrl;
    FakeHardware hw;
    hw.fake_position_ = makeConstJntArray(3, 0.0);
    hw.fake_velocity_ = makeConstJntArray(3, 0.0);
    hw.fake_torque_   = makeConstJntArray(3, 0.0);
    ctrl.SetHardware(&hw);

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
        CHECK(hw.set_mode_count_ == 1);
        ctrl.Reset();
        ctrl.UpdateCmd(makeJntArray(3));
        CHECK(hw.set_mode_count_ == 2);
    }
}

// ==========================================================================
// 8. 逆动力学失败
// ==========================================================================

TEST_CASE("JointImpedanceController - 逆动力学失败") {
    FakeHardware hw;
    hw.fake_position_ = makeConstJntArray(3, 0.0);
    hw.fake_torque_   = makeConstJntArray(3, 5.0);
    FakeModel model;
    model.id_should_fail_ = true;

    rocos::JointImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);

    auto q_des = makeJntArray(3);
    CHECK(ctrl.UpdateCmd(q_des) == rocos::Result::IdCalcFail);
}
