// Copyright 2026, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <cmath>
#include <string>
#include <variant>
#include <vector>

#include <test/doctest.h>

#include "src/move_servo.hpp"

namespace {

class FakeHardware : public rocos::HardwareInterface {
 public:
    explicit FakeHardware(const rocos::JntArray& position)
        : fake_position_(position) {}

    rocos::JntArray GetPosition() override { return fake_position_; }
    rocos::JntArray GetVelocity() override { return rocos::JntArray(fake_position_.rows()); }
    rocos::JntArray GetTorque() override { return rocos::JntArray(fake_position_.rows()); }
    rocos::JntArray GetLoadTorque() override { return rocos::JntArray(fake_position_.rows()); }
    void SetPosition(const rocos::JntArray&) override {}
    void SetVelocity(const rocos::JntArray&) override {}
    void SetTorque(const rocos::JntArray&) override {}
    void SetMode(int8_t) override {}
    void SetEnabled() override {}
    void SetDisabled() override {}
    rocos::JntState GetState() override { return rocos::JntState::ENABLED; }
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
    rocos::JntState GetJointState(int32_t) override { return rocos::JntState::ENABLED; }
    std::string getJointName(int32_t) override { return ""; }
    rocos::Wrench GetWrench() override { return rocos::Wrench::Zero(); }
    bool GetDigitalInput(int, int) override { return false; }
    void SetDigitalOutput(int, int, bool) override {}
    double GetAnalogInput(int, int) override { return 0.0; }
    void SetAnalogOutput(int, int, double) override {}
    bool Reset() override { return true; }
    void WaitForSignal() override {}
    int GetDriveNum() const override { return static_cast<int>(fake_position_.rows()); }

 private:
    rocos::JntArray fake_position_;
};

class FakeModel : public rocos::ModelInterface {
 public:
    rocos::Result ForwardKinematics(const rocos::JntArray& q_in,
                                    rocos::Frame& p_out) override {
        p_out = rocos::Frame(rocos::Vector(q_in(0), q_in(1), q_in(2)));
        return rocos::Result::NoError;
    }

    rocos::Result InverseKinematics(const rocos::JntArray&,
                                    const rocos::Frame&,
                                    rocos::JntArray&) override {
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

    rocos::Result GetJacobian(const rocos::JntArray&,
                              rocos::Jacobian&) override {
        return rocos::Result::NoError;
    }

    int GetJointNum() const override { return 3; }
    std::vector<std::string> GetJointNames() const override { return {}; }
};

rocos::JntArray makeJntArray(double q0, double q1, double q2) {
    rocos::JntArray q(3);
    q(0) = q0;
    q(1) = q1;
    q(2) = q2;
    return q;
}

}  // namespace

TEST_CASE("MoveServo GenerateRef fills current joint position when no command is available") {
    const auto current_position = makeJntArray(1.0, 2.0, 3.0);
    FakeHardware hw(current_position);
    rocos::MoveServo move(&hw, nullptr, 0);
    move.SetMode(MotionMode::kJointPosition);

    rocos::Reference ref = rocos::Frame::Identity();

    CHECK(move.GenerateRef(ref) == rocos::Result::NoError);
    REQUIRE(std::holds_alternative<rocos::JntArray>(ref));

    const auto& q_ref = std::get<rocos::JntArray>(ref);
    REQUIRE(q_ref.rows() == current_position.rows());
    CHECK(q_ref(0) == doctest::Approx(1.0));
    CHECK(q_ref(1) == doctest::Approx(2.0));
    CHECK(q_ref(2) == doctest::Approx(3.0));
}

TEST_CASE("MoveServo GenerateRef fills current cartesian pose when no command is available") {
    FakeHardware hw(makeJntArray(4.0, 5.0, 6.0));
    FakeModel model;
    rocos::MoveServo move(&hw, &model, 0);
    move.SetMode(MotionMode::kCartesianPosition);

    rocos::Reference ref = rocos::JntArray(3);

    CHECK(move.GenerateRef(ref) == rocos::Result::NoError);
    REQUIRE(std::holds_alternative<rocos::Frame>(ref));

    const auto& frame_ref = std::get<rocos::Frame>(ref);
    CHECK(frame_ref.p.x() == doctest::Approx(4.0));
    CHECK(frame_ref.p.y() == doctest::Approx(5.0));
    CHECK(frame_ref.p.z() == doctest::Approx(6.0));
}
