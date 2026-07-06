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
    measurement.channel = measure_info.channel;
    measurement.name = measure_info.name;
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

  measurement->start_time = Clock::now();
  measurement->is_running = true;
}

void PerformanceProfiler::MeasureEnd(int32_t channel) {
  auto* measurement = findMeasurement(channel);
  if (measurement == nullptr) {
    return;
  }

  if (!measurement->is_running) {
    return;
  }

  const auto end_time = Clock::now();
  const auto elapsed_us = std::chrono::duration<double, std::micro>(
                              end_time - measurement->start_time)
                              .count();

  measurement->last_us = elapsed_us;
  ++measurement->count;
  measurement->avg_us +=
      (elapsed_us - measurement->avg_us) / static_cast<double>(measurement->count);

  if (measurement->count == 1) {
    measurement->min_us = elapsed_us;
    measurement->max_us = elapsed_us;
  } else {
    if (elapsed_us < measurement->min_us) {
      measurement->min_us = elapsed_us;
    }
    if (elapsed_us > measurement->max_us) {
      measurement->max_us = elapsed_us;
    }
  }

  measurement->is_running = false;
}

bool PerformanceProfiler::GetMeasurementStats(
    int32_t channel, ChannelMeasurement& measurement_out) const {
  const auto* measurement = findMeasurement(channel);
  if (measurement == nullptr) {
    return false;
  }

  measurement_out = *measurement;
  return true;
}

std::optional<double> PerformanceProfiler::GetMinUs(int32_t channel) const {
  ChannelMeasurement measurement;
  if (!GetMeasurementStats(channel, measurement)) {
    return std::nullopt;
  }
  return measurement.min_us;
}

std::optional<double> PerformanceProfiler::GetMaxUs(int32_t channel) const {
  ChannelMeasurement measurement;
  if (!GetMeasurementStats(channel, measurement)) {
    return std::nullopt;
  }
  return measurement.max_us;
}

std::optional<double> PerformanceProfiler::GetAvgUs(int32_t channel) const {
  ChannelMeasurement measurement;
  if (!GetMeasurementStats(channel, measurement)) {
    return std::nullopt;
  }
  return measurement.avg_us;
}

std::optional<double> PerformanceProfiler::GetLastUs(int32_t channel) const {
  ChannelMeasurement measurement;
  if (!GetMeasurementStats(channel, measurement)) {
    return std::nullopt;
  }
  return measurement.last_us;
}

std::optional<uint64_t> PerformanceProfiler::GetCount(int32_t channel) const {
  ChannelMeasurement measurement;
  if (!GetMeasurementStats(channel, measurement)) {
    return std::nullopt;
  }
  return measurement.count;
}

void PerformanceProfiler::Reset() {
  for (auto& measurement : measurements_) {
    measurement.count = 0;
    measurement.last_us = 0.0;
    measurement.min_us = 0.0;
    measurement.max_us = 0.0;
    measurement.avg_us = 0.0;
    measurement.is_running = false;
  }
}

void PerformanceProfiler::Reset(int32_t channel) {
  auto* measurement = findMeasurement(channel);
  if (measurement == nullptr) {
    std::cerr << "PerformanceProfiler unknown channel in Reset: " << channel
              << std::endl;
    return;
  }

  measurement->count = 0;
  measurement->last_us = 0.0;
  measurement->min_us = 0.0;
  measurement->max_us = 0.0;
  measurement->avg_us = 0.0;
  measurement->is_running = false;
}

void PerformanceProfiler::PrintMeasurement(int32_t channel) const {
  ChannelMeasurement measurement;
  if (!GetMeasurementStats(channel, measurement)) {
    std::cerr << "PerformanceProfiler unknown channel in PrintMeasurement: "
              << channel << std::endl;
    return;
  }

  printStats(measurement);
}

void PerformanceProfiler::PrintMeasurements() const {
  std::cout << std::left << std::setw(10) << "channel" << std::setw(24)
            << "name" << std::right << std::setw(12) << "count"
            << std::setw(14) << "last(us)" << std::setw(14) << "min(us)"
            << std::setw(14) << "max(us)" << std::setw(14) << "avg(us)"
            << std::endl;

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

void PerformanceProfiler::printStats(const ChannelMeasurement& measurement) {
  std::cout << std::left << std::setw(10) << measurement.channel << std::setw(24)
            << measurement.name << std::right << std::setw(12) << measurement.count
            << std::setw(14) << std::fixed << std::setprecision(3)
            << measurement.last_us << std::setw(14) << measurement.min_us
            << std::setw(14) << measurement.max_us << std::setw(14)
            << measurement.avg_us << std::endl;
}

}  // namespace rocos
