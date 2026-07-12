#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <rocos_app/lua_script_engine.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

#include <test/doctest.h>

#include "src/robot.hpp"

namespace {

bool waitForState(rocos::LuaScriptEngine& engine,
                  rocos::LuaScriptEngine::State expected,
                  std::chrono::milliseconds timeout =
                      std::chrono::seconds(2)) {
    const auto deadline_ = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline_) {
        if (engine.GetStatus().state == expected) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

}  // namespace

TEST_CASE("LuaScriptEngine LoadFile 执行安全脚本") {
    rocos::Robot robot_;
    rocos::LuaScriptEngine engine_(robot_, "scripts");

    REQUIRE(engine_.LoadFile("engine_test.lua") == rocos::Result::NoError);
    REQUIRE(engine_.Run() == rocos::Result::NoError);
    REQUIRE(waitForState(
        engine_, rocos::LuaScriptEngine::State::Completed));
    CHECK(engine_.GetStatus().error.empty());
}

TEST_CASE("LuaScriptEngine 拒绝 scripts 根目录之外的文件") {
    rocos::Robot robot_;
    rocos::LuaScriptEngine engine_(robot_, "scripts");

    CHECK(engine_.LoadFile("../robot.urdf") ==
          rocos::Result::LuaFileError);
    CHECK(engine_.LoadFile("/tmp/script.lua") ==
          rocos::Result::LuaFileError);
}

TEST_CASE("LuaScriptEngine 可停止 tight loop") {
    rocos::Robot robot_;
    rocos::LuaScriptEngine engine_(robot_, "scripts");

    REQUIRE(engine_.LoadSource(
                "local value = 0\n"
                "while true do value = value + 1 end\n",
                "loop.lua") == rocos::Result::NoError);
    REQUIRE(engine_.Run() == rocos::Result::NoError);
    REQUIRE(waitForState(
        engine_, rocos::LuaScriptEngine::State::Running));
    REQUIRE(engine_.Stop() == rocos::Result::NoError);
    REQUIRE(waitForState(
        engine_, rocos::LuaScriptEngine::State::Stopped));
}

TEST_CASE("LuaScriptEngine 在断点处暂停并继续") {
    rocos::Robot robot_;
    rocos::LuaScriptEngine engine_(robot_, "scripts");

    REQUIRE(engine_.LoadSource(
                "local first = 1\n"
                "local second = 2\n"
                "assert(first + second == 3)\n",
                "breakpoint.lua") == rocos::Result::NoError);
    REQUIRE(engine_.AddBreakpoint("breakpoint.lua", 2) ==
            rocos::Result::NoError);
    REQUIRE(engine_.Run() == rocos::Result::NoError);
    REQUIRE(waitForState(
        engine_, rocos::LuaScriptEngine::State::Paused));
    CHECK(engine_.GetStatus().location.line == 2);
    REQUIRE(engine_.Resume() == rocos::Result::NoError);
    REQUIRE(waitForState(
        engine_, rocos::LuaScriptEngine::State::Completed));
}

TEST_CASE("LuaScriptEngine 注册 Robot 运动 binding 并校验参数") {
    rocos::Robot robot_;
    rocos::LuaScriptEngine engine_(robot_, "scripts");

    REQUIRE(engine_.LoadSource(
                "robot.MoveJ({0.0}, 1.0, 2.0, 10.0)\n",
                "invalid_motion.lua") == rocos::Result::NoError);
    REQUIRE(engine_.Run() == rocos::Result::NoError);
    REQUIRE(waitForState(
        engine_, rocos::LuaScriptEngine::State::Failed));
    CHECK(engine_.GetStatus().error.find("MoveJ") != std::string::npos);
}
