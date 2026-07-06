#include "performance_profiler.hpp"

#include <iomanip>
#include <iostream>

namespace rocos {
PerformanceProfiler::PerformanceProfiler(
    const std::vector<PerformanceMeasureInfo>& measure_infos) {
  channels_.reserve(measure_infos.size());
  measurements_.reserve(measure_infos.size());

  int32_t max_channel = -1;
  for (const auto& measure_info : measure_infos) {
    if (measure_info.channel < 0) {
      std::cerr << "PerformanceProfiler invalid channel: " << measure_info.channel
                << std::endl;
      continue;
    }
    if (measure_info.channel > max_channel) {
      max_channel = measure_info.channel;
    }
  }

  if (max_channel >= 0) {
    channel_to_index_.assign(static_cast<size_t>(max_channel) + 1, -1);
  }

  for (const auto& measure_info : measure_infos) {
    if (measure_info.channel < 0) {
      continue;
    }

    auto& index = channel_to_index_[static_cast<size_t>(measure_info.channel)];
    if (index >= 0) {
      std::cerr << "PerformanceProfiler duplicate channel: "
                << measure_info.channel << std::endl;
      continue;
    }

    ChannelMeasurement measurement;
    measurement.name_ = measure_info.name;
    measurements_.push_back(measurement);
    channels_.push_back(measure_info.channel);
    index = static_cast<int32_t>(measurements_.size() - 1);
  }
}

void PerformanceProfiler::MeasureStart(int32_t channel) {
  auto* measurement = findMeasurement(channel);
  if (measurement == nullptr) {
    return;
  }

  measurement->start_time_ = Clock::now();
  measurement->is_running_ = true;
}

void PerformanceProfiler::MeasureEnd(int32_t channel) {
  auto* measurement = findMeasurement(channel);
  if (measurement == nullptr) {
    return;
  }

  if (!measurement->is_running_) {
    return;
  }

  const auto end_time = Clock::now();
  const auto elapsed_us = std::chrono::duration<double, std::micro>(
                              end_time - measurement->start_time_)
                              .count();

  measurement->last_us_ = elapsed_us;
  measurement->total_us_ += elapsed_us;
  ++measurement->count_;

  if (measurement->count_ == 1) {
    measurement->min_us_ = elapsed_us;
    measurement->max_us_ = elapsed_us;
  } else {
    if (elapsed_us < measurement->min_us_) {
      measurement->min_us_ = elapsed_us;
    }
    if (elapsed_us > measurement->max_us_) {
      measurement->max_us_ = elapsed_us;
    }
  }

  measurement->is_running_ = false;
}

bool PerformanceProfiler::GetMeasurementStats(
    int32_t channel, PerformanceMeasurementStats& stats) const {
  const auto* measurement = findMeasurement(channel);
  if (measurement == nullptr) {
    return false;
  }

  stats = makeStats(channel, *measurement);
  return true;
}

double PerformanceProfiler::GetMinUs(int32_t channel) const {
  PerformanceMeasurementStats stats;
  return GetMeasurementStats(channel, stats) ? stats.min_us : 0.0;
}

double PerformanceProfiler::GetMaxUs(int32_t channel) const {
  PerformanceMeasurementStats stats;
  return GetMeasurementStats(channel, stats) ? stats.max_us : 0.0;
}

double PerformanceProfiler::GetAvgUs(int32_t channel) const {
  PerformanceMeasurementStats stats;
  return GetMeasurementStats(channel, stats) ? stats.avg_us : 0.0;
}

double PerformanceProfiler::GetLastUs(int32_t channel) const {
  PerformanceMeasurementStats stats;
  return GetMeasurementStats(channel, stats) ? stats.last_us : 0.0;
}

uint64_t PerformanceProfiler::GetCount(int32_t channel) const {
  PerformanceMeasurementStats stats;
  return GetMeasurementStats(channel, stats) ? stats.count : 0;
}

void PerformanceProfiler::Reset() {
  for (auto& measurement : measurements_) {
    measurement.count_ = 0;
    measurement.last_us_ = 0.0;
    measurement.min_us_ = 0.0;
    measurement.max_us_ = 0.0;
    measurement.total_us_ = 0.0;
    measurement.is_running_ = false;
  }
}

void PerformanceProfiler::Reset(int32_t channel) {
  auto* measurement = findMeasurement(channel);
  if (measurement == nullptr) {
    std::cerr << "PerformanceProfiler unknown channel in Reset: " << channel
              << std::endl;
    return;
  }

  measurement->count_ = 0;
  measurement->last_us_ = 0.0;
  measurement->min_us_ = 0.0;
  measurement->max_us_ = 0.0;
  measurement->total_us_ = 0.0;
  measurement->is_running_ = false;
}

void PerformanceProfiler::PrintMeasurement(int32_t channel) const {
  PerformanceMeasurementStats stats;
  if (!GetMeasurementStats(channel, stats)) {
    std::cerr << "PerformanceProfiler unknown channel in PrintMeasurement: "
              << channel << std::endl;
    return;
  }

  printStats(stats);
}

void PerformanceProfiler::PrintMeasurements() const {
  std::cout << std::left << std::setw(10) << "channel" << std::setw(24)
            << "name" << std::right << std::setw(12) << "count"
            << std::setw(14) << "last(us)" << std::setw(14) << "min(us)"
            << std::setw(14) << "max(us)" << std::setw(14) << "avg(us)"
            << std::setw(14) << "total(us)" << std::endl;

  for (const auto channel : channels_) {
    PrintMeasurement(channel);
  }
}

PerformanceProfiler::ChannelMeasurement* PerformanceProfiler::findMeasurement(
    int32_t channel) {
  if (channel < 0 || static_cast<size_t>(channel) >= channel_to_index_.size()) {
    return nullptr;
  }

  const auto index = channel_to_index_[static_cast<size_t>(channel)];
  if (index < 0) {
    return nullptr;
  }

  return &measurements_[static_cast<size_t>(index)];
}

const PerformanceProfiler::ChannelMeasurement* PerformanceProfiler::findMeasurement(
    int32_t channel) const {
  if (channel < 0 || static_cast<size_t>(channel) >= channel_to_index_.size()) {
    return nullptr;
  }

  const auto index = channel_to_index_[static_cast<size_t>(channel)];
  if (index < 0) {
    return nullptr;
  }

  return &measurements_[static_cast<size_t>(index)];
}

PerformanceMeasurementStats PerformanceProfiler::makeStats(
    int32_t channel, const ChannelMeasurement& measurement) {
  PerformanceMeasurementStats stats;
  stats.channel = channel;
  stats.name = measurement.name_;
  stats.count = measurement.count_;
  stats.last_us = measurement.last_us_;
  stats.min_us = measurement.count_ > 0 ? measurement.min_us_ : 0.0;
  stats.max_us = measurement.count_ > 0 ? measurement.max_us_ : 0.0;
  stats.avg_us = measurement.count_ > 0
                     ? measurement.total_us_ / static_cast<double>(measurement.count_)
                     : 0.0;
  stats.total_us = measurement.total_us_;
  return stats;
}

void PerformanceProfiler::printStats(const PerformanceMeasurementStats& stats) {
  std::cout << std::left << std::setw(10) << stats.channel << std::setw(24)
            << stats.name << std::right << std::setw(12) << stats.count
            << std::setw(14) << std::fixed << std::setprecision(3)
            << stats.last_us << std::setw(14) << stats.min_us << std::setw(14)
            << stats.max_us << std::setw(14) << stats.avg_us << std::setw(14)
            << stats.total_us << std::endl;
}

}  // namespace rocos
