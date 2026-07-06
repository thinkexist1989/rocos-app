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
#include <fstream>
#include <string>
#include <variant>
#include <vector>

#include <test/doctest.h>

#include "src/move_joint.hpp"

namespace {

struct DataPoint {
    int    step;
    double time;
    double q0;
    double q1;
    double q2;
};

// 控制周期
constexpr double kDt = 0.001;

// 运动限制
constexpr double kVLimit = 1.0;
constexpr double kALimit = 2.0;
constexpr double kJLimit = 10.0;

constexpr int kMaxSteps   = 10000;
constexpr int kPauseAtStep = 200;

const std::string kCsvDir = "/tmp/move_joint_csv/";

// ─── 辅助 ───

rocos::JntArray makeJntArray(const std::vector<double>& v) {
    rocos::JntArray arr(static_cast<unsigned int>(v.size()));
    for (unsigned int i = 0; i < v.size(); ++i) arr(i) = v[i];
    return arr;
}

std::vector<double> extractJointPos(rocos::MoveJoint& move) {
    rocos::Reference ref;
    move.GenerateRef(ref);
    const auto& q = std::get<rocos::JntArray>(ref);
    return {q(0), q(1), q(2)};
}

void writeCsv(const std::string& filename, const std::vector<DataPoint>& data) {
    std::string full_path = kCsvDir + filename;
    std::ofstream ofs(full_path);
    ofs << "step,time,q0,q1,q2\n";
    for (const auto& dp : data) {
        ofs << dp.step << "," << dp.time << ","
            << dp.q0 << "," << dp.q1 << "," << dp.q2 << "\n";
    }
}

}  // namespace

// ==========================================================================
// 测试辅助宏：三个场景 × 两个方向
// ==========================================================================

static void runNormal(rocos::MoveJoint& move,
                      std::vector<DataPoint>& data,
                      const std::string& csv_name) {
    data.clear();
    int step = 0;
    double time = 0.0;

    while (step < kMaxSteps) {
        auto q = extractJointPos(move);
        data.push_back({step, time, q[0], q[1], q[2]});

        rocos::Result r = move.Update();
        if (r != rocos::Result::NoError) break;

        ++step;
        time += kDt;
    }

    // 最后一步记录终点
    auto q = extractJointPos(move);
    data.push_back({step, time, q[0], q[1], q[2]});

    writeCsv(csv_name, data);
    MESSAGE(csv_name, " — ", data.size(), " data points");
}

static void runPauseResume(rocos::MoveJoint& move,
                           std::vector<DataPoint>& data,
                           const std::string& csv_name) {
    data.clear();
    int step = 0;
    double time = 0.0;

    // 阶段 1：正常运行到暂停点
    while (step < kPauseAtStep && step < kMaxSteps) {
        auto q = extractJointPos(move);
        data.push_back({step, time, q[0], q[1], q[2]});

        rocos::Result r = move.Update();
        if (r != rocos::Result::NoError) break;

        ++step;
        time += kDt;
    }

    // 阶段 2：Pause → 减速到停
    move.Pause();
    while (step < kMaxSteps) {
        auto q = extractJointPos(move);
        data.push_back({step, time, q[0], q[1], q[2]});

        rocos::Result r = move.Update();
        if (r == rocos::Result::PlanFinished) break;

        ++step;
        time += kDt;
    }

    // 阶段 3：Resume → 继续到目标
    move.Resume();
    ++step;
    time += kDt;
    while (step < kMaxSteps) {
        auto q = extractJointPos(move);
        data.push_back({step, time, q[0], q[1], q[2]});

        rocos::Result r = move.Update();
        if (r != rocos::Result::NoError) break;

        ++step;
        time += kDt;
    }

    // 终点
    auto q = extractJointPos(move);
    data.push_back({step, time, q[0], q[1], q[2]});

    writeCsv(csv_name, data);
    MESSAGE(csv_name, " — ", data.size(), " data points");
}

static void runPauseStop(rocos::MoveJoint& move,
                         std::vector<DataPoint>& data,
                         const std::string& csv_name) {
    data.clear();
    int step = 0;
    double time = 0.0;

    // 阶段 1：正常运行到暂停点
    while (step < kPauseAtStep && step < kMaxSteps) {
        auto q = extractJointPos(move);
        data.push_back({step, time, q[0], q[1], q[2]});

        rocos::Result r = move.Update();
        if (r != rocos::Result::NoError) break;

        ++step;
        time += kDt;
    }

    // 阶段 2：Pause → 减速到停
    move.Pause();
    while (step < kMaxSteps) {
        auto q = extractJointPos(move);
        data.push_back({step, time, q[0], q[1], q[2]});

        rocos::Result r = move.Update();
        if (r == rocos::Result::PlanFinished) break;

        ++step;
        time += kDt;
    }

    // 阶段 3：Stop 确认终止
    move.Stop();
    // Stop 后可能还需几步减速
    while (step < kMaxSteps) {
        auto q = extractJointPos(move);
        data.push_back({step, time, q[0], q[1], q[2]});

        rocos::Result r = move.Update();
        if (r == rocos::Result::PlanFinished) break;

        ++step;
        time += kDt;
    }

    writeCsv(csv_name, data);
    MESSAGE(csv_name, " — ", data.size(), " data points");
}

// ==========================================================================
// 正向：q_start=[0,1,2] → q_goal=[1,2,4] (delta_max=2.0)
// ==========================================================================

TEST_CASE("MoveJoint 正向 — 正常执行") {
    rocos::MoveJoint move(makeJntArray({0, 1, 2}), makeJntArray({1, 2, 4}),
                          kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runNormal(move, data, "forward_normal.csv");

    auto q = extractJointPos(move);
    CHECK(q[0] == doctest::Approx(1.0).epsilon(1e-3));
    CHECK(q[1] == doctest::Approx(2.0).epsilon(1e-3));
    CHECK(q[2] == doctest::Approx(4.0).epsilon(1e-3));
}

TEST_CASE("MoveJoint 正向 — 暂停继续") {
    rocos::MoveJoint move(makeJntArray({0, 1, 2}), makeJntArray({1, 2, 4}),
                          kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runPauseResume(move, data, "forward_pause_resume.csv");

    auto q = extractJointPos(move);
    CHECK(q[0] == doctest::Approx(1.0).epsilon(1e-3));
    CHECK(q[1] == doctest::Approx(2.0).epsilon(1e-3));
    CHECK(q[2] == doctest::Approx(4.0).epsilon(1e-3));
}

TEST_CASE("MoveJoint 正向 — 暂停后Stop") {
    rocos::MoveJoint move(makeJntArray({0, 1, 2}), makeJntArray({1, 2, 4}),
                          kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runPauseStop(move, data, "forward_pause_stop.csv");

    // 暂停后 stop，位置应停在中间，未到达目标
    auto q = extractJointPos(move);
    CHECK(q[2] < 4.0);
}

// ==========================================================================
// 反向：q_start=[1,2,4] → q_goal=[0,1,2] (delta_max=2.0)
// ==========================================================================

TEST_CASE("MoveJoint 反向 — 正常执行") {
    rocos::MoveJoint move(makeJntArray({1, 2, 4}), makeJntArray({0, 1, 2}),
                          kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runNormal(move, data, "reverse_normal.csv");

    auto q = extractJointPos(move);
    CHECK(q[0] == doctest::Approx(0.0).epsilon(1e-3));
    CHECK(q[1] == doctest::Approx(1.0).epsilon(1e-3));
    CHECK(q[2] == doctest::Approx(2.0).epsilon(1e-3));
}

TEST_CASE("MoveJoint 反向 — 暂停继续") {
    rocos::MoveJoint move(makeJntArray({1, 2, 4}), makeJntArray({0, 1, 2}),
                          kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runPauseResume(move, data, "reverse_pause_resume.csv");

    auto q = extractJointPos(move);
    CHECK(q[0] == doctest::Approx(0.0).epsilon(1e-3));
    CHECK(q[1] == doctest::Approx(1.0).epsilon(1e-3));
    CHECK(q[2] == doctest::Approx(2.0).epsilon(1e-3));
}

TEST_CASE("MoveJoint 反向 — 暂停后Stop") {
    rocos::MoveJoint move(makeJntArray({1, 2, 4}), makeJntArray({0, 1, 2}),
                          kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runPauseStop(move, data, "reverse_pause_stop.csv");

    auto q = extractJointPos(move);
    CHECK(q[2] > 2.0);
}
