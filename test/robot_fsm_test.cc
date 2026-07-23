#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <memory>

#include <test/doctest.h>

#include "src/robot.hpp"

// =========================================================
// Robot FSM 单元测试
//
// 当前 FSM 关键行为（见 src/robot.cpp 中的 StateMachine 定义）：
//   - 构造器发出 EventResetReq: ERROR_STATE → RESETTING
//     on_fsm_reset() 使用 randomBool() 模拟硬件检测：
//       true  → EventEnabled  → STOPPED
//       false → EventDisabled → IDLE
//   - IDLE + EventEnableReq → ENABLING
//     on_fsm_enable() 使用 randomBool()：
//       true  → EventEnabled       → STOPPED
//       false → EventErrorOccurred → ERROR_STATE
//   - STOPPED + EventDisableReq → DISABLING
//     on_fsm_disable() 使用 randomBool()：
//       true  → EventDisabled      → IDLE
//       false → EventErrorOccurred → ERROR_STATE
//   - ERROR_STATE 仅接受 EventResetReq / EventStopReq。
//     SetEnabled/SetDisabled 会被拒绝，StopMotion 可触发停止请求。
//   - Start 目前是占位实现，不向 FSM 派发事件。
//   - IsRunning() 覆盖 RUNNING || STOPPING || PAUSING || RESUMING。
// 由于 randomBool()，所有涉及状态迁移的用例都需容忍两种结果，
// 部分用例通过重复构造收敛到目标初始状态。
// =========================================================

namespace {

constexpr int kMaxRetry = 64;  // randomBool() 命中目标状态所需的最大重试次数

std::unique_ptr<rocos::Robot> makeRobotInState(const std::string& target) {
    for (int i = 0; i < kMaxRetry; ++i) {
        auto robot = std::make_unique<rocos::Robot>();
        if (robot->GetStateString() == target) {
            return robot;
        }
        if (target == "STOPPED" && robot->GetStateString() == "IDLE") {
            (void)robot->SetEnabled();
            if (robot->GetStateString() == target) {
                return robot;
            }
        }
    }
    return nullptr;
}

std::unique_ptr<rocos::Robot> makeRobotInErrorState() {
    // ERROR_STATE 仅能通过 ENABLING/DISABLING 时 on_fsm_* 失败到达。
    for (int i = 0; i < 256; ++i) {
        auto robot = std::make_unique<rocos::Robot>();
        const auto state = robot->GetStateString();

        if (state == "IDLE") {
            robot->SetEnabled();   // IDLE → ENABLING → (STOPPED | ERROR_STATE)
        } else if (state == "STOPPED") {
            robot->SetDisabled();  // STOPPED → DISABLING → (IDLE | ERROR_STATE)
        }

        if (robot->GetStateString() == "ERROR_STATE") {
            return robot;
        }
    }

    return nullptr;
}

}  // namespace

TEST_CASE("Robot FSM - 构造后经 RESETTING 进入 IDLE") {
    // 构造器发出 EventResetReq，但启动初始化只建立 disabled 空闲态；
    // 故障复位仍由 ResetFault() 负责重新上使能并回到 STOPPED。
    rocos::Robot robot;
    const auto state = robot.GetStateString();
    CHECK(state == "IDLE");
    CHECK_FALSE(robot.IsRunning());
}

TEST_CASE("Robot FSM - IDLE 下 SetEnabled 进入 STOPPED 或 ERROR_STATE") {
    // IDLE + EventEnableReq → ENABLING
    // on_fsm_enable() randomBool():
    //   true  → EventEnabled       → STOPPED
    //   false → EventErrorOccurred → ERROR_STATE
    auto robot = makeRobotInState("IDLE");
    REQUIRE(robot != nullptr);
    REQUIRE(robot->GetStateString() == "IDLE");

    const auto rc = robot->SetEnabled();
    CHECK(rc == rocos::Result::NoError);

    const auto next = robot->GetStateString();
    std::cout << "IDLE.SetEnabled → " << next << std::endl;
    CHECK((next == "STOPPED" || next == "ERROR_STATE"));
}

TEST_CASE("Robot FSM - STOPPED 下 SetEnabled 无效") {
    // 转换表中仅 IDLE 接受 EventEnableReq，STOPPED 无匹配转换。
    auto robot = makeRobotInState("STOPPED");
    REQUIRE(robot != nullptr);

    const auto rc = robot->SetEnabled();
    CHECK(rc == rocos::Result::JointStateError);
    CHECK(robot->GetStateString() == "STOPPED");
}

TEST_CASE("Robot FSM - IDLE 下 SetDisabled 无效") {
    // 转换表中仅 STOPPED 接受 EventDisableReq，IDLE 无匹配转换。
    auto robot = makeRobotInState("IDLE");
    REQUIRE(robot != nullptr);

    const auto rc = robot->SetDisabled();
    CHECK(rc == rocos::Result::JointStateError);
    CHECK(robot->GetStateString() == "IDLE");
}

TEST_CASE("Robot FSM - STOPPED 下 SetDisabled 进入 IDLE 或 ERROR_STATE") {
    // STOPPED + EventDisableReq → DISABLING
    // on_fsm_disable() randomBool():
    //   true  → EventDisabled      → IDLE
    //   false → EventErrorOccurred → ERROR_STATE
    auto robot = makeRobotInState("STOPPED");
    REQUIRE(robot != nullptr);

    const auto rc = robot->SetDisabled();
    CHECK(rc == rocos::Result::NoError);

    const auto next = robot->GetStateString();
    std::cout << "STOPPED.SetDisabled → " << next << std::endl;
    CHECK((next == "IDLE" || next == "ERROR_STATE"));
}

TEST_CASE("Robot FSM - 初始状态下 IsRunning 为 false 且无运动控制请求不启动运行") {
    // IsRunning() 覆盖 RUNNING || STOPPING || PAUSING || RESUMING。
    // 初始 STOPPED/IDLE 均不在其中；Start 是占位实现，无活动 motion 时
    // PauseMotion/ResumeMotion/StopMotion 不会把机器人带入 RUNNING。
    rocos::Robot robot;
    const auto initial = robot.GetStateString();
    CHECK_FALSE(robot.IsRunning());

    robot.Start();
    CHECK(robot.GetStateString() == initial);
    CHECK_FALSE(robot.IsRunning());

    (void)robot.PauseMotion();
    CHECK_FALSE(robot.IsRunning());

    (void)robot.ResumeMotion();
    CHECK_FALSE(robot.IsRunning());

    (void)robot.StopMotion();
    CHECK_FALSE(robot.IsRunning());
}

TEST_CASE("Robot FSM - ERROR_STATE 拒绝 SetEnabled/SetDisabled 且 Start 不改变状态") {
    // ERROR_STATE 仅接受 EventResetReq / EventStopReq，
    // SetEnabled/SetDisabled 事件会被 FSM 拒绝并返回 JointStateError。
    // Start 是 stub，也不会改变状态。
    auto robot = makeRobotInErrorState();
    REQUIRE(robot != nullptr);
    REQUIRE(robot->GetStateString() == "ERROR_STATE");

    CHECK(robot->SetEnabled() == rocos::Result::JointStateError);
    CHECK(robot->GetStateString() == "ERROR_STATE");

    CHECK(robot->SetDisabled() == rocos::Result::JointStateError);
    CHECK(robot->GetStateString() == "ERROR_STATE");

    robot->Start();
    CHECK(robot->GetStateString() == "ERROR_STATE");

    CHECK_FALSE(robot->IsRunning());
}
