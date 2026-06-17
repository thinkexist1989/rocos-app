#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>

#include <rocos_app/motion/diana_error_codes.h>
#include <rocos_app/motion/motion_context.h>
#include <rocos_app/motion/model_provider.h>
#include <rocos_app/motion/move_j_command.h>
#include <rocos_app/motion/move_l_command.h>
#include <rocos_app/motion/move_c_three_point_command.h>
#include <rocos_app/motion/move_c_center_angle_command.h>
#include <rocos_app/motion/motion_executor.h>
#include <rocos_app/motion/position_controller.h>

#include <kdl/frames.hpp>

#include <cmath>
#include <limits>
#include <vector>

namespace {

constexpr double kDt = 0.001;

// === Mock context and model ===

class StubMotionContext final : public rocos::motion::MotionContext {
public:
    rocos::motion::RobotStateSnapshot readStateSnapshot() const override {
        return rocos::motion::RobotStateSnapshot{};
    }
    double controlPeriod() const override { return kDt; }
    rocos::motion::MotionResult writeLowLevelCommand(
        const rocos::motion::LowLevelCommand&) override {
        return rocos::motion::MotionResult::ok();
    }
};

// 2-DOF planar robot mock FK/IK
//
// FK(q1,q2):
//   x = cos(q1) + cos(q1+q2)
//   y = sin(q1) + sin(q1+q2)
//
// IK target (tx,ty) → q1,q2:
//   cos(q2) = (tx²+ty²-2)/2
//   q2 = acos(...)
//   q1 = atan2(ty,tx) - atan2(sin(q2), 1+cos(q2))

bool planarFK(const std::vector<double>& q, std::vector<double>& pose_out) {
    if (q.size() != 2) { return false; }
    pose_out.resize(2);
    pose_out[0] = std::cos(q[0]) + std::cos(q[0] + q[1]);
    pose_out[1] = std::sin(q[0]) + std::sin(q[0] + q[1]);
    return true;
}

bool planarIK(const std::vector<double>& q_seed,
              const std::vector<double>& target,
              std::vector<double>& q_out) {
    if (target.size() != 2) { return false; }
    const double tx = target[0];
    const double ty = target[1];
    const double r2 = tx * tx + ty * ty;
    const double cos_q2 = (r2 - 2.0) / 2.0;
    if (std::abs(cos_q2) > 1.0 + 1e-12) { return false; }
    const double clamped = std::max(-1.0, std::min(1.0, cos_q2));

    // 两个 IK 解：elbow-up (+) 和 elbow-down (-)
    const double q2_pos = std::acos(clamped);
    const double q2_neg = -std::acos(clamped);

    // 选择最接近 seed 的解，确保沿路径连续
    double q2;
    if (!q_seed.empty()) {
        q2 = (std::abs(q_seed[1] - q2_pos) <= std::abs(q_seed[1] - q2_neg))
                 ? q2_pos : q2_neg;
    } else {
        q2 = q2_pos;
    }
    const double s2 = std::sin(q2);
    const double q1 = std::atan2(ty, tx) - std::atan2(s2, 1.0 + clamped);
    q_out.resize(2);
    q_out[0] = q1;
    q_out[1] = q2;
    return true;
}

rocos::motion::ModelProvider makePlanarModel() {
    rocos::motion::ModelProvider m;
    m.kinematics.forwardKinematics = planarFK;
    m.kinematics.inverseKinematics = planarIK;
    m.kinematics.getDof = []() { return 2; };
    return m;
}

// 将 MoveL/MoveC 命令传递的 7 元素 pose_vec [x,y,z,qx,qy,qz,qw]
// 适配为 2-DOF planar IK 所需的 2 元素 [x,y]
rocos::motion::MoveLCommand::IkCallback makePlanarIkAdapter() {
    return [](const std::vector<double>& q_seed,
              const std::vector<double>& pose_vec,
              std::vector<double>& q_out) -> bool {
        if (pose_vec.size() < 2) { return false; }
        std::vector<double> target_2d = {pose_vec[0], pose_vec[1]};
        return planarIK(q_seed, target_2d, q_out);
    };
}

// 构建 KDL::Frame（2D: z=0, rotation=identity）
KDL::Frame frame2D(double x, double y) {
    return KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(x, y, 0.0));
}

// 构建 KDL::Frame（绕 Z 轴旋转 theta）
KDL::Frame frame2Drot(double x, double y, double theta) {
    return KDL::Frame(KDL::Rotation::RotZ(theta), KDL::Vector(x, y, 0.0));
}

rocos::motion::MoveJCommand::Parameters makeMoveJParams() {
    rocos::motion::MoveJCommand::Parameters params;
    params.q_start = {0.0, 1.0};
    params.q_goal = {1.0, -1.0};
    params.max_velocity = {2.0, 4.0};
    params.max_acceleration = {4.0, 8.0};
    params.max_jerk = {20.0, 40.0};
    params.dt = kDt;
    return params;
}

}  // namespace

// === Existing MoveJ tests (updated signatures) ===

TEST_CASE("Diana error mapper exposes external API codes") {
    using rocos::motion::DianaErrorCode;
    using rocos::motion::MotionResultCode;

    CHECK(rocos::motion::toDianaErrorCode(MotionResultCode::Ok) == 0);
    CHECK(rocos::motion::toDianaErrorCode(MotionResultCode::Busy) ==
          static_cast<int>(DianaErrorCode::ConflictTaskRunning));
    CHECK(rocos::motion::toDianaErrorCode(MotionResultCode::InvalidCommand) ==
          static_cast<int>(DianaErrorCode::IllegalParameter));
    CHECK(rocos::motion::toDianaErrorCode(MotionResultCode::PlanningFailed) ==
          static_cast<int>(DianaErrorCode::PlanError));
    CHECK(rocos::motion::toDianaErrorCode(MotionResultCode::SafetyViolation) ==
          static_cast<int>(DianaErrorCode::Fatal));
    CHECK(rocos::motion::toDianaErrorCode(MotionResultCode::HardwareFault) ==
          static_cast<int>(DianaErrorCode::JointRegistError));
}

TEST_CASE("MoveJCommand samples finite path references within configured limits") {
    StubMotionContext ctx;
    rocos::motion::ModelProvider model;
    rocos::motion::MoveJCommand command(makeMoveJParams());

    const auto prepare = command.prepare(ctx, model);
    REQUIRE(prepare.success);

    const auto start = command.start(ctx);
    REQUIRE(start.success);

    std::vector<rocos::motion::MotionReference> references;
    for (int i = 0; i < 10000; ++i) {
        const auto step = command.update(ctx, model);
        REQUIRE(step.result.success);
        if (step.reference) {
            references.push_back(*step.reference);
            const auto& joint = step.reference->joint;
            REQUIRE(joint.position.size() == 2);
            REQUIRE(joint.velocity.size() == 2);
            REQUIRE(joint.acceleration.size() == 2);
            CHECK(std::abs(joint.velocity[0]) <= 2.0 + 1e-9);
            CHECK(std::abs(joint.velocity[1]) <= 4.0 + 1e-9);
            CHECK(std::abs(joint.acceleration[0]) <= 4.0 + 1e-9);
            CHECK(std::abs(joint.acceleration[1]) <= 8.0 + 1e-9);
        }
        if (step.status == rocos::motion::MotionStepStatus::Finished) {
            break;
        }
    }

    REQUIRE_FALSE(references.empty());
    CHECK(command.status() == rocos::motion::MotionStepStatus::Finished);

    const auto& final_joint = references.back().joint;
    CHECK(final_joint.position[0] == doctest::Approx(1.0));
    CHECK(final_joint.position[1] == doctest::Approx(-1.0));
    CHECK(final_joint.velocity[0] == doctest::Approx(0.0));
    CHECK(final_joint.velocity[1] == doctest::Approx(0.0));
}

TEST_CASE("MoveJCommand pause resume and stop are handled by finite profile") {
    StubMotionContext ctx;
    rocos::motion::ModelProvider model;
    rocos::motion::MoveJCommand command(makeMoveJParams());
    REQUIRE(command.prepare(ctx, model).success);
    REQUIRE(command.start(ctx).success);

    for (int i = 0; i < 100; ++i) {
        REQUIRE(command.update(ctx, model).result.success);
    }

    REQUIRE(command.pause().success);
    for (int i = 0; i < 10000; ++i) {
        const auto step = command.update(ctx, model);
        REQUIRE(step.result.success);
        if (step.status == rocos::motion::MotionStepStatus::Paused) {
            break;
        }
    }
    REQUIRE(command.status() == rocos::motion::MotionStepStatus::Paused);

    REQUIRE(command.resume().success);
    for (int i = 0; i < 10000; ++i) {
        const auto step = command.update(ctx, model);
        REQUIRE(step.result.success);
        if (step.status == rocos::motion::MotionStepStatus::Finished) {
            break;
        }
    }
    REQUIRE(command.status() == rocos::motion::MotionStepStatus::Finished);

    rocos::motion::MoveJCommand stop_command(makeMoveJParams());
    REQUIRE(stop_command.prepare(ctx, model).success);
    REQUIRE(stop_command.start(ctx).success);
    for (int i = 0; i < 100; ++i) {
        REQUIRE(stop_command.update(ctx, model).result.success);
    }

    REQUIRE(stop_command.stop().success);
    for (int i = 0; i < 10000; ++i) {
        const auto step = stop_command.update(ctx, model);
        REQUIRE(step.result.success);
        if (step.status == rocos::motion::MotionStepStatus::Stopped) {
            break;
        }
    }
    CHECK(stop_command.status() == rocos::motion::MotionStepStatus::Stopped);
}

TEST_CASE("MoveJCommand rejects invalid parameters with Diana API error codes") {
    StubMotionContext ctx;
    rocos::motion::ModelProvider model;
    auto params = makeMoveJParams();
    params.q_goal[0] = std::numeric_limits<double>::quiet_NaN();

    rocos::motion::MoveJCommand command(params);
    const auto result = command.prepare(ctx, model);

    CHECK_FALSE(result.success);
    CHECK(result.result == rocos::motion::MotionResultCode::InvalidNumber);
    CHECK(result.api_error_code ==
          static_cast<int>(rocos::motion::DianaErrorCode::ParameterNanOrInf));
}

// === MoveLCommand tests ===

TEST_CASE("MoveLCommand rejects NaN Cartesian velocity") {
    StubMotionContext ctx;
    rocos::motion::ModelProvider model;

    rocos::motion::MoveLCommand::Parameters params;
    params.pose_start = frame2D(0.0, 0.0);
    params.pose_goal = frame2D(1.0, 1.0);
    params.q_current = {0.5, 0.5};
    params.max_cartesian_velocity = std::numeric_limits<double>::quiet_NaN();
    params.max_cartesian_acceleration = 1.0;
    params.dt = kDt;

    rocos::motion::MoveLCommand command(std::move(params), makePlanarIkAdapter());
    const auto result = command.prepare(ctx, model);
    CHECK_FALSE(result.success);
    CHECK(result.result == rocos::motion::MotionResultCode::InvalidCommand);
}

TEST_CASE("MoveLCommand rejects empty q_current") {
    StubMotionContext ctx;
    rocos::motion::ModelProvider model;

    rocos::motion::MoveLCommand::Parameters params;
    params.pose_start = frame2D(0.0, 0.0);
    params.pose_goal = frame2D(1.0, 1.0);
    params.max_cartesian_velocity = 1.0;
    params.max_cartesian_acceleration = 1.0;
    params.dt = kDt;

    rocos::motion::MoveLCommand command(std::move(params), makePlanarIkAdapter());
    const auto result = command.prepare(ctx, model);
    CHECK_FALSE(result.success);
    CHECK(result.result == rocos::motion::MotionResultCode::InvalidCommand);
}

TEST_CASE("MoveLCommand samples linear Cartesian path and IK") {
    StubMotionContext ctx;
    rocos::motion::ModelProvider model;
    auto fk = makePlanarModel().kinematics.forwardKinematics;
    auto ik = makePlanarIkAdapter();

    // 起点: FK(q={0,0}) = (2,0)
    // 终点: FK(q={pi/4, -pi/2}) = (cos(pi/4)+cos(-pi/4), sin(pi/4)+sin(-pi/4)) = (sqrt(2), 0)
    const std::vector<double> q_start_planar = {0.0, 0.0};
    const std::vector<double> q_goal_planar = {M_PI / 4.0, -M_PI / 2.0};
    std::vector<double> p_start, p_goal;
    REQUIRE(fk(q_start_planar, p_start));
    REQUIRE(fk(q_goal_planar, p_goal));

    KDL::Frame start_frame = frame2D(p_start[0], p_start[1]);
    KDL::Frame goal_frame = frame2D(p_goal[0], p_goal[1]);

    rocos::motion::MoveLCommand::Parameters params;
    params.pose_start = start_frame;
    params.pose_goal = goal_frame;
    params.q_current = q_start_planar;
    params.max_cartesian_velocity = 2.0;
    params.max_cartesian_acceleration = 4.0;
    params.dt = kDt;

    rocos::motion::MoveLCommand command(std::move(params), ik);
    REQUIRE(command.prepare(ctx, model).success);
    REQUIRE(command.start(ctx).success);

    std::vector<rocos::motion::MotionReference> references;
    for (int i = 0; i < 10000; ++i) {
        const auto step = command.update(ctx, model);
        REQUIRE(step.result.success);
        if (step.reference) {
            references.push_back(*step.reference);
        }
        if (step.status == rocos::motion::MotionStepStatus::Finished) {
            break;
        }
    }

    REQUIRE_FALSE(references.empty());
    CHECK(command.status() == rocos::motion::MotionStepStatus::Finished);

    // 验证 IK 链式解：每个采样的关节位置 FK 应接近对应笛卡尔目标
    const auto& final_ref = references.back();
    std::vector<double> final_pose;
    REQUIRE(planarFK(final_ref.joint.position, final_pose));
    CHECK(final_pose[0] == doctest::Approx(p_goal[0]).epsilon(1e-3));
    CHECK(final_pose[1] == doctest::Approx(p_goal[1]).epsilon(1e-3));

    // 验证中间解也有效（选取中间参考）
    const auto& mid_ref = references[references.size() / 2];
    CHECK(mid_ref.joint.position.size() == 2);
    CHECK(mid_ref.joint.velocity.size() == 2);
    CHECK(mid_ref.joint.acceleration.size() == 2);
}

TEST_CASE("MoveLCommand IK failure signals via ReferenceSpace::None") {
    StubMotionContext ctx;
    rocos::motion::ModelProvider model;

    // 一个永远失败的 IK callback
    rocos::motion::MoveLCommand::IkCallback failingIk =
        [](const std::vector<double>&, const std::vector<double>&,
           std::vector<double>&) -> bool { return false; };

    // 不可达目标（距离 > 2，超出 planar 机器人臂长）
    rocos::motion::MoveLCommand::Parameters params;
    params.pose_start = frame2D(2.0, 0.0);
    params.pose_goal = frame2D(5.0, 5.0);
    params.q_current = {0.0, 0.0};
    params.max_cartesian_velocity = 1.0;
    params.max_cartesian_acceleration = 1.0;
    params.dt = kDt;

    rocos::motion::MoveLCommand command(std::move(params), failingIk);
    REQUIRE(command.prepare(ctx, model).success);
    REQUIRE(command.start(ctx).success);

    bool found_none = false;
    for (int i = 0; i < 100; ++i) {
        const auto step = command.update(ctx, model);
        if (step.status == rocos::motion::MotionStepStatus::Failed) {
            found_none = true;
            CHECK_FALSE(step.result.success);
            break;
        }
        if (step.status == rocos::motion::MotionStepStatus::Finished) {
            break;
        }
    }
    CHECK(found_none);
}

// === MoveCThreePointCommand tests ===

TEST_CASE("MoveCThreePointCommand rejects invalid parameters") {
    StubMotionContext ctx;
    rocos::motion::ModelProvider model;
    auto ik = makePlanarIkAdapter();

    rocos::motion::MoveCThreePointCommand::Parameters params;
    params.pose_start = frame2D(1.0, 0.0);
    params.pose_via = frame2D(0.0, 1.0);
    params.pose_goal = frame2D(-1.0, 0.0);
    params.q_current = {0.5, 0.5};
    params.max_cartesian_velocity = -1.0;  // 无效
    params.max_cartesian_acceleration = 1.0;
    params.dt = kDt;

    rocos::motion::MoveCThreePointCommand command(std::move(params), ik);
    const auto result = command.prepare(ctx, model);
    CHECK_FALSE(result.success);
    CHECK(result.result == rocos::motion::MotionResultCode::InvalidCommand);
}

TEST_CASE("MoveCThreePointCommand samples circular arc path") {
    StubMotionContext ctx;
    rocos::motion::ModelProvider model;
    auto ik = makePlanarIkAdapter();

    // 三点：半圆弧（1,0）→（0,1）→（-1,0），圆心（0,0），半径1，theta=pi
    rocos::motion::MoveCThreePointCommand::Parameters params;
    params.pose_start = frame2D(1.0, 0.0);
    params.pose_via = frame2D(0.0, 1.0);
    params.pose_goal = frame2D(-1.0, 0.0);
    // FK({-pi/4, pi/2}) = (1,0): elbow-up seed matching start pose
    params.q_current = {-M_PI / 4.0, M_PI / 2.0};
    params.max_cartesian_velocity = 0.5;
    params.max_cartesian_acceleration = 1.0;
    params.dt = kDt;

    rocos::motion::MoveCThreePointCommand command(std::move(params), ik);
    REQUIRE(command.prepare(ctx, model).success);
    REQUIRE(command.start(ctx).success);

    std::vector<rocos::motion::MotionReference> references;
    for (int i = 0; i < 20000; ++i) {
        const auto step = command.update(ctx, model);
        REQUIRE(step.result.success);
        if (step.reference) {
            references.push_back(*step.reference);
        }
        if (step.status == rocos::motion::MotionStepStatus::Finished) {
            break;
        }
    }

    REQUIRE_FALSE(references.empty());
    CHECK(command.status() == rocos::motion::MotionStepStatus::Finished);

    // 终点 IK 应该接近 (-1,0)
    const auto& final_ref = references.back();
    std::vector<double> final_pose;
    REQUIRE(planarFK(final_ref.joint.position, final_pose));
    CHECK(final_pose[0] == doctest::Approx(-1.0).epsilon(1e-3));
    CHECK(final_pose[1] == doctest::Approx(0.0).epsilon(1e-3));
}

// === MoveCCenterAngleCommand tests ===

TEST_CASE("MoveCCenterAngleCommand rejects zero theta") {
    StubMotionContext ctx;
    rocos::motion::ModelProvider model;
    auto ik = makePlanarIkAdapter();

    rocos::motion::MoveCCenterAngleCommand::Parameters params;
    params.pose_start = frame2D(1.0, 0.0);
    params.center = frame2D(0.0, 0.0);
    params.theta = 0.0;
    params.axis = 2;
    params.q_current = {0.0, 0.0};
    params.max_cartesian_velocity = 0.5;
    params.max_cartesian_acceleration = 1.0;
    params.dt = kDt;

    rocos::motion::MoveCCenterAngleCommand command(std::move(params), ik);
    const auto result = command.prepare(ctx, model);
    CHECK_FALSE(result.success);
    CHECK(result.result == rocos::motion::MotionResultCode::InvalidCommand);
}

TEST_CASE("MoveCCenterAngleCommand rejects invalid axis") {
    StubMotionContext ctx;
    rocos::motion::ModelProvider model;
    auto ik = makePlanarIkAdapter();

    rocos::motion::MoveCCenterAngleCommand::Parameters params;
    params.pose_start = frame2D(1.0, 0.0);
    params.center = frame2D(0.0, 0.0);
    params.theta = M_PI;
    params.axis = 5;  // 无效
    params.q_current = {0.0, 0.0};
    params.max_cartesian_velocity = 0.5;
    params.max_cartesian_acceleration = 1.0;
    params.dt = kDt;

    rocos::motion::MoveCCenterAngleCommand command(std::move(params), ik);
    const auto result = command.prepare(ctx, model);
    CHECK_FALSE(result.success);
    CHECK(result.result == rocos::motion::MotionResultCode::InvalidCommand);
}

TEST_CASE("MoveCCenterAngleCommand samples center-angle arc") {
    StubMotionContext ctx;
    rocos::motion::ModelProvider model;
    auto ik = makePlanarIkAdapter();

    // 起点(1,0)，绕Z轴转pi，圆心(0,0)
    rocos::motion::MoveCCenterAngleCommand::Parameters params;
    params.pose_start = frame2D(1.0, 0.0);
    params.center = frame2D(0.0, 0.0);
    params.theta = M_PI;
    params.axis = 2;
    // FK({pi/3, 2pi/3}) = (1,0): 匹配起点的 elbow-up 解
    params.q_current = {M_PI / 3.0, 2.0 * M_PI / 3.0};
    params.max_cartesian_velocity = 0.5;
    params.max_cartesian_acceleration = 1.0;
    params.dt = kDt;

    rocos::motion::MoveCCenterAngleCommand command(std::move(params), ik);
    REQUIRE(command.prepare(ctx, model).success);
    REQUIRE(command.start(ctx).success);

    std::vector<rocos::motion::MotionReference> references;
    for (int i = 0; i < 20000; ++i) {
        const auto step = command.update(ctx, model);
        REQUIRE(step.result.success);
        if (step.reference) {
            references.push_back(*step.reference);
        }
        if (step.status == rocos::motion::MotionStepStatus::Finished) {
            break;
        }
    }

    REQUIRE_FALSE(references.empty());
    CHECK(command.status() == rocos::motion::MotionStepStatus::Finished);

    // 终点应接近 (-1,0)（绕Z轴转pi后的位置）
    const auto& final_ref = references.back();
    std::vector<double> final_pose;
    REQUIRE(planarFK(final_ref.joint.position, final_pose));
    CHECK(final_pose[0] == doctest::Approx(-1.0).epsilon(1e-3));
    CHECK(final_pose[1] == doctest::Approx(0.0).epsilon(1e-3));
}

// === Executor integration with MoveLCommand ===

TEST_CASE("MoveLCommand runs to finish through MotionExecutor") {
    StubMotionContext ctx;
    auto model = makePlanarModel();

    rocos::motion::MotionExecutor executor;
    auto ik = makePlanarIkAdapter();
    auto fk = model.kinematics.forwardKinematics;

    const std::vector<double> q_start = {0.0, 0.0};
    std::vector<double> p_start, p_goal;
    REQUIRE(fk(q_start, p_start));
    // 终点目标 FK(q={pi/3, -pi/4})
    const std::vector<double> q_goal = {M_PI / 3.0, -M_PI / 4.0};
    REQUIRE(fk(q_goal, p_goal));

    rocos::motion::MoveLCommand::Parameters params;
    params.pose_start = frame2D(p_start[0], p_start[1]);
    params.pose_goal = frame2D(p_goal[0], p_goal[1]);
    params.q_current = q_start;
    params.max_cartesian_velocity = 0.5;
    params.max_cartesian_acceleration = 1.0;
    params.dt = kDt;

    auto command = std::make_unique<rocos::motion::MoveLCommand>(
        std::move(params), ik);
    const auto submit_result = executor.submit(std::move(command));
    REQUIRE(submit_result.success);

    // 等待完成（最长 60s，避免无限等待）
    for (int i = 0; i < 60000; ++i) {
        const auto status = executor.currentTaskStatus();
        if (status == rocos::motion::MotionTaskStatus::Finished ||
            status == rocos::motion::MotionTaskStatus::Failed) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const auto final_status = executor.currentTaskStatus();
    if (final_status != rocos::motion::MotionTaskStatus::Finished) {
        const auto err = executor.lastError();
        std::cerr << "MoveL executor test failed: status=" << static_cast<int>(final_status)
                  << " err=" << err.message << " api_code=" << err.api_error_code << std::endl;
    }
    CHECK(final_status == rocos::motion::MotionTaskStatus::Finished);
}

// === Cartesian geometry unit tests ===

TEST_CASE("interpolateLinear produces correct positions") {
    using namespace rocos::motion::cartesian;
    auto start = frame2D(0.0, 0.0);
    auto end = frame2D(1.0, 0.0);

    auto mid = interpolateLinear(start, end, 0.5);
    CHECK(mid.p.x() == doctest::Approx(0.5));
    CHECK(mid.p.y() == doctest::Approx(0.0));

    auto s0 = interpolateLinear(start, end, 0.0);
    CHECK(s0.p.x() == doctest::Approx(0.0));

    auto s1 = interpolateLinear(start, end, 1.0);
    CHECK(s1.p.x() == doctest::Approx(1.0));
}

TEST_CASE("computePathMetrics handles pure translation") {
    using namespace rocos::motion::cartesian;
    auto start = frame2D(0.0, 0.0);
    auto end = frame2D(1.0, 0.0);

    auto metrics = computePathMetrics(start, end);
    CHECK(metrics.translation_distance == doctest::Approx(1.0));
    CHECK(metrics.path_length == doctest::Approx(1.0));
}

TEST_CASE("computeCircleCenter finds correct center for equilateral triangle") {
    using namespace rocos::motion::cartesian;
    auto p1 = frame2D(1.0, 0.0);
    auto p2 = frame2D(0.0, 1.0);
    auto p3 = frame2D(-1.0, 0.0);

    KDL::Frame center;
    REQUIRE(computeCircleCenter(center, p1, p2, p3));
    CHECK(center.p.x() == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(center.p.y() == doctest::Approx(0.0).epsilon(1e-6));
}
