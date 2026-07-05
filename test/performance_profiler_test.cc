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

    rocos::PerformanceMeasurementStats stats;
    REQUIRE(profiler.GetMeasurementStats(1, stats));
    CHECK(stats.channel == 1);
    CHECK(stats.name == "GenerateRef");
    CHECK(stats.count == 1);
    CHECK(stats.last_us > 0.0);
    CHECK(stats.min_us > 0.0);
    CHECK(stats.max_us >= stats.min_us);
    CHECK(stats.avg_us >= stats.min_us);
    CHECK(stats.total_us == doctest::Approx(stats.last_us));
}

TEST_CASE("PerformanceProfiler - 多次采样更新 Min Max Avg") {
    rocos::PerformanceProfiler profiler({{2, "GenerateCmd"}});

    profiler.MeasureStart(2);
    sleepForProfilerSample();
    profiler.MeasureEnd(2);

    profiler.MeasureStart(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    profiler.MeasureEnd(2);

    CHECK(profiler.GetCount(2) == 2);
    CHECK(profiler.GetMinUs(2) > 0.0);
    CHECK(profiler.GetMaxUs(2) >= profiler.GetMinUs(2));
    CHECK(profiler.GetAvgUs(2) >= profiler.GetMinUs(2));
    CHECK(profiler.GetAvgUs(2) <= profiler.GetMaxUs(2));
}

TEST_CASE("PerformanceProfiler - Reset 清空统计") {
    rocos::PerformanceProfiler profiler({{3, "UpdateCmd"}});

    profiler.MeasureStart(3);
    sleepForProfilerSample();
    profiler.MeasureEnd(3);
    REQUIRE(profiler.GetCount(3) == 1);

    profiler.Reset(3);

    CHECK(profiler.GetCount(3) == 0);
    CHECK(profiler.GetMinUs(3) == doctest::Approx(0.0));
    CHECK(profiler.GetMaxUs(3) == doctest::Approx(0.0));
    CHECK(profiler.GetAvgUs(3) == doctest::Approx(0.0));
}

TEST_CASE("PerformanceProfiler - 未知通道返回空统计") {
    rocos::PerformanceProfiler profiler({{4, "UpdateTotal"}});

    rocos::PerformanceMeasurementStats stats;
    CHECK_FALSE(profiler.GetMeasurementStats(99, stats));
    CHECK(profiler.GetCount(99) == 0);
    CHECK(profiler.GetMinUs(99) == doctest::Approx(0.0));
    CHECK(profiler.GetMaxUs(99) == doctest::Approx(0.0));
    CHECK(profiler.GetAvgUs(99) == doctest::Approx(0.0));
}

TEST_CASE("PerformanceProfiler - 打印接口可调用") {
    rocos::PerformanceProfiler profiler({{5, "PrintOne"}, {6, "PrintAll"}});

    profiler.MeasureStart(5);
    sleepForProfilerSample();
    profiler.MeasureEnd(5);

    profiler.PrintMeasurement(5);
    profiler.PrintMeasurements();
}

