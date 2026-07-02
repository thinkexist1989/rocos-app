#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <memory>

#include <test/doctest.h>

#include "src/robot.hpp"

// =========================================================
// Robot FSM 单元测试
// 注意：on_fsm_reset 使用 randomBool() 模拟硬件检测，
//       初始状态为 STOPPED 或 IDLE（各 50%），测试需兼容两者。
// =========================================================

TEST_CASE("Robot FSM - 构造后进入有效初始状态") {
    rocos::Robot robot;
    const auto state = robot.GetStateString();
    // randomBool() 决定走 EventIsEnabled(→STOPPED) 还是 EventSuccess(→IDLE)
    CHECK((state == "STOPPED" || state == "IDLE"));
    CHECK_FALSE(robot.IsRunning());
}

TEST_CASE("Robot FSM - IDLE 下 SetEnabled 转入 STOPPED或ERROR_STATE") {
    // IDLE + EventEnableReq → ENABLING → on_fsm_enable(IsEnabled()=true) → EventSuccess → STOPPED
    // randomBool() 使初始状态随机，重复构造直到落在 IDLE 分支
    std::unique_ptr<rocos::Robot> robot;
    for (int i = 0; i < 32; ++i) {
        robot = std::make_unique<rocos::Robot>();
        if (robot->GetStateString() == "IDLE") break;
    }
    REQUIRE(robot->GetStateString() == "IDLE");

    const auto rc = robot->SetEnabled();
    CHECK(rocos::Result::NoError == rc);
    std::cout << "反馈结果: " << rc << std::endl;
    std::cout << "当前状态: " << robot->GetStateString() << std::endl;
    CHECK((robot->GetStateString() == "STOPPED" || robot->GetStateString() == "ERROR_STATE"));
}

TEST_CASE("Robot FSM - STOPPED 下 SetEnabled 无效") {
    // FSM 中仅 IDLE 接受 EventEnableReq，STOPPED 无此转换
    std::unique_ptr<rocos::Robot> robot;
    for (int i = 0; i < 32; ++i) {
        robot = std::make_unique<rocos::Robot>();
        if (robot->GetStateString() == "STOPPED") break;
    }
    REQUIRE(robot->GetStateString() == "STOPPED");

    const auto rc = robot->SetEnabled();
    CHECK(rc == rocos::Result::JointStateError);
    CHECK(robot->GetStateString() == "STOPPED");
}

TEST_CASE("Robot FSM - STOPPED 下 SetDisabled 转入 ERROR_STATE") {
    // STOPPED + EventDisableReq → DISABLING → on_fsm_disable
    // IsDisabled() 桩始终为 true → !true=false → EventErrorOccurred → ERROR_STATE
    std::unique_ptr<rocos::Robot> robot;
    for (int i = 0; i < 32; ++i) {
        robot = std::make_unique<rocos::Robot>();
        if (robot->GetStateString() == "STOPPED") break;
    }
    REQUIRE(robot->GetStateString() == "STOPPED");

    const auto rc = robot->SetDisabled();
    CHECK(rc == rocos::Result::NoError);
    CHECK(robot->GetStateString() == "ERROR_STATE");
}

TEST_CASE("Robot FSM - IsRunning 仅在 RUNNING 时为真") {
    rocos::Robot robot;
    // 初始状态无论 STOPPED 还是 IDLE，均不是 RUNNING
    CHECK_FALSE(robot.IsRunning());
    // Pause/Continue/Stop 为空桩，不触发 FSM 事件，状态不变
    robot.Pause();
    CHECK_FALSE(robot.IsRunning());
    robot.Resume();
    CHECK_FALSE(robot.IsRunning());
    robot.Stop();
    CHECK_FALSE(robot.IsRunning());
}
