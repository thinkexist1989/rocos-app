// Copyright 2021, Yang Luo"
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

//#define DOCTEST_CONFIG_IMPLEMENT
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>

#include "src/model.hpp"
#include "src/robot.hpp"

TEST_CASE("Hello World") {
    std::cout << "hello world!" << std::endl;
}

// talon URDF 的限位在自定义 <hardware>/<limit> 标签内，不是标准 URDF <limit>，
// 因此 urdf::parseURDF 解析不到，所有关节应退回默认 [-PI, PI]
TEST_CASE("Model - ParseUrdf with talon robot") {
    const std::string urdf_path = "talon/robot.urdf"; //TODO： 将talon文件夹复制到bin目录下
    const std::string base_link = "base_link";
    const std::string tip_link  = "link_7";

    rocos::Model model(urdf_path, base_link, tip_link);

    SUBCASE("关节链包含 7 个活动关节") {
        CHECK(model.GetChain().getNrOfJoints() == 7u);
    }

    SUBCASE("未提供标准 limit 时，关节限位默认为 [-PI, PI]") {
        const auto& q_min = model.GetPosLowerLimit();
        const auto& q_max = model.GetPosUpperLimit();
        REQUIRE(q_min.rows() == 7u);
        REQUIRE(q_max.rows() == 7u);
        for (unsigned int i = 0; i < 7u; ++i) {
            CHECK(q_min(i) == doctest::Approx(-KDL::PI).epsilon(1e-9));
            CHECK(q_max(i) == doctest::Approx( KDL::PI).epsilon(1e-9));
        }
    }

    SUBCASE("零位 FK 返回有效位姿") {
        KDL::JntArray q_zero(7);
        KDL::SetToZero(q_zero);
        KDL::Frame p_out;

        const int rc = static_cast<int>(model.ForwardKinematics(q_zero, p_out));
        CHECK(rc == 0);  // NoError = 0
        // 验证旋转矩阵正交性：X·X = 1, X·Y = 0
        const KDL::Vector col_x = p_out.M.UnitX();
        const KDL::Vector col_y = p_out.M.UnitY();
        CHECK(KDL::dot(col_x, col_x) == doctest::Approx(1.0).epsilon(1e-9));
        CHECK(KDL::dot(col_x, col_y) == doctest::Approx(0.0).epsilon(1e-9));
    }

    SUBCASE("IK 与 FK 互为逆运算") {
        KDL::JntArray q_init(7);
        KDL::SetToZero(q_init);

        KDL::Frame target;
        REQUIRE(static_cast<int>(model.ForwardKinematics(q_init, target)) == 0);

        KDL::JntArray q_solution(7);
        const int ik_rc = static_cast<int>(model.InverseKinematics(q_init, target, q_solution));
        CHECK(ik_rc == 0);

        // IK 解代回 FK，末端位置应与目标吻合
        KDL::Frame p_verify;
        REQUIRE(static_cast<int>(model.ForwardKinematics(q_solution, p_verify)) == 0);
        for (int r = 0; r < 3; ++r) {
            CHECK(p_verify.p(r) == doctest::Approx(target.p(r)).epsilon(1e-4));
        }
    }
}

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
