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

#include <chrono>
#include <thread>
#include <vector>

#include <test/doctest.h>

#include "src/performance_profiler.hpp"

namespace {

void sleepForProfilerSample() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

}  // namespace

TEST_CASE("PerformanceProfiler - 统计单个通道") {
    rocos::PerformanceProfiler profiler({{1, "GenerateRef"}});

    profiler.MeasureStart(1);
    sleepForProfilerSample();
    profiler.MeasureEnd(1);

    rocos::PerformanceProfiler::ChannelMeasurement stats;
    REQUIRE(profiler.GetMeasurementStats(1, stats));
    CHECK(stats.channel == 1);
    CHECK(stats.name == "GenerateRef");
    CHECK(stats.count == 1);
    CHECK(stats.last_us > 0.0);
    CHECK(stats.min_us > 0.0);
    CHECK(stats.max_us >= stats.min_us);
    CHECK(stats.avg_us >= stats.min_us);
}

TEST_CASE("PerformanceProfiler - 多次采样更新 Min Max Avg") {
    rocos::PerformanceProfiler profiler({{2, "GenerateCmd"}});

    profiler.MeasureStart(2);
    sleepForProfilerSample();
    profiler.MeasureEnd(2);

    profiler.MeasureStart(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    profiler.MeasureEnd(2);

    const auto count = profiler.GetCount(2);
    const auto min_us = profiler.GetMinUs(2);
    const auto max_us = profiler.GetMaxUs(2);
    const auto avg_us = profiler.GetAvgUs(2);
    REQUIRE(count.has_value());
    REQUIRE(min_us.has_value());
    REQUIRE(max_us.has_value());
    REQUIRE(avg_us.has_value());
    CHECK(*count == 2);
    CHECK(*min_us > 0.0);
    CHECK(*max_us >= *min_us);
    CHECK(*avg_us >= *min_us);
    CHECK(*avg_us <= *max_us);
}

TEST_CASE("PerformanceProfiler - Reset 清空统计") {
    rocos::PerformanceProfiler profiler({{3, "UpdateCmd"}});

    profiler.MeasureStart(3);
    sleepForProfilerSample();
    profiler.MeasureEnd(3);
    const auto count_before_reset = profiler.GetCount(3);
    REQUIRE(count_before_reset.has_value());
    REQUIRE(*count_before_reset == 1);

    profiler.Reset(3);

    const auto count_after_reset = profiler.GetCount(3);
    const auto min_after_reset = profiler.GetMinUs(3);
    const auto max_after_reset = profiler.GetMaxUs(3);
    const auto avg_after_reset = profiler.GetAvgUs(3);
    REQUIRE(count_after_reset.has_value());
    REQUIRE(min_after_reset.has_value());
    REQUIRE(max_after_reset.has_value());
    REQUIRE(avg_after_reset.has_value());
    CHECK(*count_after_reset == 0);
    CHECK(*min_after_reset == doctest::Approx(0.0));
    CHECK(*max_after_reset == doctest::Approx(0.0));
    CHECK(*avg_after_reset == doctest::Approx(0.0));
}

TEST_CASE("PerformanceProfiler - 未知通道返回空统计") {
    rocos::PerformanceProfiler profiler({{4, "UpdateTotal"}});

    rocos::PerformanceProfiler::ChannelMeasurement stats;
    CHECK_FALSE(profiler.GetMeasurementStats(99, stats));
    CHECK_FALSE(profiler.GetCount(99).has_value());
    CHECK_FALSE(profiler.GetMinUs(99).has_value());
    CHECK_FALSE(profiler.GetMaxUs(99).has_value());
    CHECK_FALSE(profiler.GetAvgUs(99).has_value());
}

TEST_CASE("PerformanceProfiler - 打印接口可调用") {
    rocos::PerformanceProfiler profiler({{5, "PrintOne"}, {6, "PrintAll"}});

    profiler.MeasureStart(5);
    sleepForProfilerSample();
    profiler.MeasureEnd(5);

    profiler.PrintMeasurement(5);
    profiler.PrintMeasurements();
}
