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
#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rocos {

struct PerformanceMeasureInfo {
  int32_t channel;
  std::string name;
};

class PerformanceProfiler {
 public:
  struct ChannelMeasurement {
    int32_t channel{0};
    std::string name;
    uint64_t count{0};
    double last_us{0.0};
    double min_us{0.0};
    double max_us{0.0};
    double avg_us{0.0};
    std::chrono::steady_clock::time_point start_time{};
    bool is_running{false};
  };

  explicit PerformanceProfiler(const std::vector<PerformanceMeasureInfo>& measure_infos);

  void MeasureStart(int32_t channel);
  void MeasureEnd(int32_t channel);

  bool GetMeasurementStats(int32_t channel,
                           ChannelMeasurement& measurement) const;
  std::optional<double> GetMinUs(int32_t channel) const;
  std::optional<double> GetMaxUs(int32_t channel) const;
  std::optional<double> GetAvgUs(int32_t channel) const;
  std::optional<double> GetLastUs(int32_t channel) const;
  std::optional<uint64_t> GetCount(int32_t channel) const;

  void Reset();
  void Reset(int32_t channel);

  void PrintMeasurement(int32_t channel) const;
  void PrintMeasurements() const;
 private:
  using Clock = std::chrono::steady_clock;
  ChannelMeasurement* findMeasurement(int32_t channel);
  const ChannelMeasurement* findMeasurement(int32_t channel) const;
  static void printStats(const ChannelMeasurement& measurement);

  std::vector<int32_t> channels_;
  std::vector<int32_t> channel_to_index_;
  std::vector<ChannelMeasurement> measurements_;
};

}  // namespace rocos
