#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>

#include <rocos_app/motion/position_controller.h>

#include <limits>
#include <vector>

namespace {

rocos::motion::MotionReference jointReference(std::vector<double> position,
                                              std::vector<double> velocity = {}) {
    rocos::motion::MotionReference reference;
    reference.space = rocos::motion::ReferenceSpace::Joint;
    reference.joint.position = std::move(position);
    reference.joint.velocity = std::move(velocity);
    return reference;
}

}  // namespace

TEST_CASE("PositionController converts joint reference to low level position command") {
    rocos::motion::PositionController controller;
    rocos::motion::LowLevelCommand output;

    const auto result =
        controller.update(jointReference({0.1, -0.2, 0.3}), output);

    REQUIRE(result.success);
    CHECK(controller.type() == rocos::motion::ControllerType::Position);
    CHECK(controller.acceptedReferenceSpace() ==
          rocos::motion::ReferenceSpace::Joint);
    CHECK(controller.complianceSpace() ==
          rocos::motion::ComplianceSpace::None);
    CHECK(output.target_position == std::vector<double>{0.1, -0.2, 0.3});
    CHECK(output.target_velocity.empty());
    CHECK(output.target_torque.empty());
}

TEST_CASE("PositionController forwards optional joint velocity reference") {
    rocos::motion::PositionController controller;
    rocos::motion::LowLevelCommand output;

    const auto result =
        controller.update(jointReference({0.1, -0.2}, {0.01, -0.02}), output);

    REQUIRE(result.success);
    CHECK(output.target_position == std::vector<double>{0.1, -0.2});
    CHECK(output.target_velocity == std::vector<double>{0.01, -0.02});
}

TEST_CASE("PositionController rejects invalid joint numbers with Diana API code") {
    rocos::motion::PositionController controller;
    rocos::motion::LowLevelCommand output;

    const auto result = controller.update(
        jointReference({0.1, std::numeric_limits<double>::quiet_NaN()}),
        output);

    CHECK_FALSE(result.success);
    CHECK(result.result == rocos::motion::MotionResultCode::InvalidNumber);
    CHECK(result.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::ParameterNanOrInf));
}

TEST_CASE("PositionController rejects incompatible reference space with Diana API code") {
    rocos::motion::PositionController controller;
    rocos::motion::MotionReference reference;
    reference.space = rocos::motion::ReferenceSpace::Cartesian;
    rocos::motion::LowLevelCommand output;

    const auto result = controller.update(reference, output);

    CHECK_FALSE(result.success);
    CHECK(result.result == rocos::motion::MotionResultCode::Unsupported);
    CHECK(result.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::MoveUnknown));
}
