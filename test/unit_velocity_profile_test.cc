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
#include <vector>

#include <test/doctest.h>

#include "src/unit_velocity_profile.hpp"

namespace {

// 数据点结构：记录每一步的物理量
struct DataPoint {
    int    step;
    double time;
    double position;
    double velocity;
    double acceleration;
};

// 控制周期 [秒]
constexpr double kDt = 0.001;

// 运动限制参数
constexpr double kVMax = 0.5;   // 最大速度 [单位/s]
constexpr double kAMax = 1.0;   // 最大加速度 [单位/s²]
constexpr double kJMax = 5.0;   // 最大 jerk [单位/s³]

// 最大步数保护，避免死循环
constexpr int kMaxSteps = 10000;

// Pause/Stop 触发步数
constexpr int kPauseAtStep  = 400;   // 约 0.4s 处暂停
constexpr int kStopAtStep   = 400;   // 约 0.4s 处停止

// CSV 输出目录
const std::string kCsvDir = "/tmp/velocity_profile_csv/";

/// @brief 将数据点写入 CSV 文件
void writeCsv(const std::string& filename, const std::vector<DataPoint>& data) {
    std::string full_path = kCsvDir + filename;
    std::ofstream ofs(full_path);
    ofs << "step,time,position,velocity,acceleration\n";
    for (const auto& dp : data) {
        ofs << dp.step << ","
            << dp.time << ","
            << dp.position << ","
            << dp.velocity << ","
            << dp.acceleration << "\n";
    }
    ofs.close();
}

}  // namespace

// ==========================================================================
// 测试 1：正常执行（Start → 连续 Update 到完成）
// ==========================================================================

TEST_CASE("UnitVelocityProfile - 正常执行") {
    rocos::UnitVelocityProfile profile(kDt);

    REQUIRE(profile.Start(kVMax, kAMax, kJMax));

    std::vector<DataPoint> data;
    data.reserve(kMaxSteps);

    int    step = 0;
    double time = 0.0;

    while (step < kMaxSteps) {
        int rc = profile.Update();

        data.push_back({step, time,
                        profile.position(),
                        profile.velocity(),
                        profile.acceleration()});

        if (rc <= 0) break;  // Finished 或 Error

        ++step;
        time += kDt;
    }

    // 验证最终到达目标位置
    CHECK(profile.position() == doctest::Approx(1.0).epsilon(1e-6));
    CHECK(profile.velocity() == doctest::Approx(0.0).epsilon(1e-6));

    writeCsv("normal_profile.csv", data);
    MESSAGE("正常执行 CSV 已写入: ", kCsvDir + "normal_profile.csv",
            " (", data.size(), " 个数据点)");
}

// ==========================================================================
// 测试 2：暂停和继续（Start → 运行 → Pause → Resume → 完成）
// ==========================================================================

TEST_CASE("UnitVelocityProfile - 暂停与继续") {
    rocos::UnitVelocityProfile profile(kDt);

    REQUIRE(profile.Start(kVMax, kAMax, kJMax));

    std::vector<DataPoint> data;
    data.reserve(kMaxSteps);

    int    step = 0;
    double time = 0.0;

    // 阶段 1：正常运行到暂停点
    while (step < kPauseAtStep && step < kMaxSteps) {
        int rc = profile.Update();

        data.push_back({step, time,
                        profile.position(),
                        profile.velocity(),
                        profile.acceleration()});

        if (rc <= 0) break;

        ++step;
        time += kDt;
    }

    // 记录暂停时的位置（应在 0 和 1 之间，说明尚未完成）
    double paused_position = profile.position();
    CHECK(paused_position > 0.0);
    CHECK(paused_position < 1.0);

    // 阶段 2：发出 Pause 指令，减速到停
    profile.Pause(kAMax, kJMax);

    while (step < kMaxSteps) {
        int rc = profile.Update();

        data.push_back({step, time,
                        profile.position(),
                        profile.velocity(),
                        profile.acceleration()});

        if (rc <= 0) break;  // 速度减到 0，Finished

        ++step;
        time += kDt;
    }

    // 验证速度已降为 0，位置未到 1.0
    CHECK(profile.velocity() == doctest::Approx(0.0).epsilon(1e-6));
    double stopped_position = profile.position();
    CHECK(stopped_position < 1.0);

    // 阶段 3：Resume 继续向目标运行
    profile.Resume();

    ++step;
    time += kDt;

    while (step < kMaxSteps) {
        int rc = profile.Update();

        data.push_back({step, time,
                        profile.position(),
                        profile.velocity(),
                        profile.acceleration()});

        if (rc <= 0) break;

        ++step;
        time += kDt;
    }

    // 验证最终到达目标
    CHECK(profile.position() == doctest::Approx(1.0).epsilon(1e-6));
    CHECK(profile.velocity() == doctest::Approx(0.0).epsilon(1e-6));

    writeCsv("pause_resume_profile.csv", data);
    MESSAGE("暂停继续 CSV 已写入: ", kCsvDir + "pause_resume_profile.csv",
            " (", data.size(), " 个数据点)");
}

// ==========================================================================
// 测试 3：直接停止（Start → 运行 → Stop → 减速到停）
// ==========================================================================

TEST_CASE("UnitVelocityProfile - 直接停止") {
    rocos::UnitVelocityProfile profile(kDt);

    REQUIRE(profile.Start(kVMax, kAMax, kJMax));

    std::vector<DataPoint> data;
    data.reserve(kMaxSteps);

    int    step = 0;
    double time = 0.0;

    // 阶段 1：正常运行到停止点
    while (step < kStopAtStep && step < kMaxSteps) {
        int rc = profile.Update();

        data.push_back({step, time,
                        profile.position(),
                        profile.velocity(),
                        profile.acceleration()});

        if (rc <= 0) break;

        ++step;
        time += kDt;
    }

    // 记录停止前的位置
    double stop_trigger_pos = profile.position();
    CHECK(stop_trigger_pos > 0.0);
    CHECK(stop_trigger_pos < 1.0);

    // 阶段 2：发出 Stop 指令，减速到停
    profile.Stop(kAMax, kJMax);

    while (step < kMaxSteps) {
        int rc = profile.Update();

        data.push_back({step, time,
                        profile.position(),
                        profile.velocity(),
                        profile.acceleration()});

        if (rc <= 0) break;  // 速度减到 0

        ++step;
        time += kDt;
    }

    // 验证速度已降为 0，但位置未到达目标
    CHECK(profile.velocity() == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(profile.position() < 1.0);

    writeCsv("stop_profile.csv", data);
    MESSAGE("直接停止 CSV 已写入: ", kCsvDir + "stop_profile.csv",
            " (", data.size(), " 个数据点)");
}
