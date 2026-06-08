#include <rocos_app/UnitIntervalMotionProfile.h>

#include <ruckig/ruckig.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

struct UnitIntervalMotionProfile::Impl {
  explicit Impl(double dt) : otg(dt) {}

  static double ClampToUnitInterval(double value) {
    return std::clamp(value, 0.0, 1.0);
  }

  static bool IsValidLimit(double value) {
    return std::isfinite(value) && value > 0.0;
  }

  static bool AreValidLimits(double max_velocity,
                             double max_acceleration,
                             double max_jerk) {
    return IsValidLimit(max_velocity) &&
           IsValidLimit(max_acceleration) &&
           IsValidLimit(max_jerk);
  }

  bool HasValidStoredLimits() const {
    return AreValidLimits(max_velocity, max_acceleration, max_jerk);
  }

  bool ResolvePauseLimits(double requested_max_acceleration,
                          double requested_max_jerk,
                          double& resolved_max_acceleration,
                          double& resolved_max_jerk) const {
    resolved_max_acceleration = max_acceleration;
    resolved_max_jerk = max_jerk;

    if (requested_max_acceleration != -1.0) {
      if (!IsValidLimit(requested_max_acceleration)) {
        return false;
      }
      resolved_max_acceleration = requested_max_acceleration;
    }

    if (requested_max_jerk != -1.0) {
      if (!IsValidLimit(requested_max_jerk)) {
        return false;
      }
      resolved_max_jerk = requested_max_jerk;
    }

    return IsValidLimit(resolved_max_acceleration) &&
           IsValidLimit(resolved_max_jerk);
  }

  void SyncCurrentStateToInput() {
    input.current_position = {position};
    input.current_velocity = {velocity};
    input.current_acceleration = {acceleration};
  }

  void SyncLimitsToInput(double max_velocity_value,
                         double max_acceleration_value,
                         double max_jerk_value) {
    input.max_velocity = {max_velocity_value};
    input.max_acceleration = {max_acceleration_value};
    input.max_jerk = {max_jerk_value};
  }

  void MarkError() {
    active = false;
    error = true;
    stop_pending = false;
    stop_completed = false;
  }

  ruckig::Ruckig<1> otg;
  ruckig::InputParameter<1> input;
  ruckig::OutputParameter<1> output;

  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;

  double max_velocity = 0.0;
  double max_acceleration = 0.0;
  double max_jerk = 0.0;

  bool active = false;
  bool error = false;
  bool stop_pending = false;
  bool stop_completed = false;
};

UnitIntervalMotionProfile::UnitIntervalMotionProfile(double dt)
    : impl_(std::make_unique<Impl>(dt)) {
  Reset();
}

UnitIntervalMotionProfile::~UnitIntervalMotionProfile() = default;

UnitIntervalMotionProfile::UnitIntervalMotionProfile(
    UnitIntervalMotionProfile&&) noexcept = default;

UnitIntervalMotionProfile& UnitIntervalMotionProfile::operator=(
    UnitIntervalMotionProfile&&) noexcept = default;

void UnitIntervalMotionProfile::Reset(double position,
                                      double velocity,
                                      double acceleration) {
  impl_->input = ruckig::InputParameter<1>();
  impl_->output = ruckig::OutputParameter<1>();

  impl_->position = Impl::ClampToUnitInterval(position);
  impl_->velocity = velocity;
  impl_->acceleration = acceleration;

  impl_->max_velocity = 0.0;
  impl_->max_acceleration = 0.0;
  impl_->max_jerk = 0.0;

  impl_->active = false;
  impl_->error = false;
  impl_->stop_pending = false;
  impl_->stop_completed = false;

  impl_->SyncCurrentStateToInput();
}

void UnitIntervalMotionProfile::Start(double max_velocity,
                                      double max_acceleration,
                                      double max_jerk) {
  if (!Impl::AreValidLimits(max_velocity, max_acceleration, max_jerk)) {
    impl_->MarkError();
    return;
  }

  impl_->max_velocity = max_velocity;
  impl_->max_acceleration = max_acceleration;
  impl_->max_jerk = max_jerk;

  impl_->input = ruckig::InputParameter<1>();
  impl_->output = ruckig::OutputParameter<1>();

  impl_->position = 0.0;
  impl_->velocity = 0.0;
  impl_->acceleration = 0.0;

  impl_->input.control_interface = ruckig::ControlInterface::Position;

  impl_->SyncCurrentStateToInput();

  impl_->input.target_position = {1.0};
  impl_->input.target_velocity = {0.0};
  impl_->input.target_acceleration = {0.0};

  impl_->SyncLimitsToInput(impl_->max_velocity,
                           impl_->max_acceleration,
                           impl_->max_jerk);

  impl_->active = true;
  impl_->error = false;
  impl_->stop_pending = false;
  impl_->stop_completed = false;
}

void UnitIntervalMotionProfile::Resume() {
  if (!impl_->HasValidStoredLimits()) {
    impl_->MarkError();
    return;
  }

  impl_->input.control_interface = ruckig::ControlInterface::Position;

  impl_->SyncCurrentStateToInput();

  impl_->input.target_position = {1.0};
  impl_->input.target_velocity = {0.0};
  impl_->input.target_acceleration = {0.0};

  impl_->SyncLimitsToInput(impl_->max_velocity,
                           impl_->max_acceleration,
                           impl_->max_jerk);

  impl_->active = true;
  impl_->error = false;
  impl_->stop_pending = false;
  impl_->stop_completed = false;
}

void UnitIntervalMotionProfile::Pause(double max_acceleration,
                                      double max_jerk) {
  double used_max_acceleration = 0.0;
  double used_max_jerk = 0.0;

  if (IsStopped()) {
    impl_->active = false;
    impl_->stop_pending = false;
    impl_->stop_completed = false;
    return;
  }

  if (!impl_->HasValidStoredLimits() ||
      !impl_->ResolvePauseLimits(max_acceleration,
                                 max_jerk,
                                 used_max_acceleration,
                                 used_max_jerk)) {
    impl_->MarkError();
    return;
  }

  impl_->input.control_interface = ruckig::ControlInterface::Velocity;

  impl_->SyncCurrentStateToInput();

  impl_->input.target_velocity = {0.0};
  impl_->input.target_acceleration = {0.0};

  impl_->SyncLimitsToInput(impl_->max_velocity,
                           used_max_acceleration,
                           used_max_jerk);

  impl_->active = true;
  impl_->error = false;
  impl_->stop_pending = false;
  impl_->stop_completed = false;
}

void UnitIntervalMotionProfile::Stop(double max_acceleration,
                                     double max_jerk) {
  impl_->stop_completed = false;

  if (IsStopped()) {
    impl_->active = false;
    impl_->stop_pending = false;
    impl_->stop_completed = true;
    impl_->velocity = 0.0;
    impl_->acceleration = 0.0;
    impl_->input = ruckig::InputParameter<1>();
    impl_->output = ruckig::OutputParameter<1>();
    impl_->SyncCurrentStateToInput();
    return;
  }

  double used_max_acceleration = 0.0;
  double used_max_jerk = 0.0;
  if (!impl_->HasValidStoredLimits() ||
      !impl_->ResolvePauseLimits(max_acceleration,
                                 max_jerk,
                                 used_max_acceleration,
                                 used_max_jerk)) {
    impl_->MarkError();
    return;
  }

  impl_->input.control_interface = ruckig::ControlInterface::Velocity;

  impl_->SyncCurrentStateToInput();

  impl_->input.target_velocity = {0.0};
  impl_->input.target_acceleration = {0.0};

  impl_->SyncLimitsToInput(impl_->max_velocity,
                           used_max_acceleration,
                           used_max_jerk);

  impl_->active = true;
  impl_->error = false;
  impl_->stop_pending = true;
}

void UnitIntervalMotionProfile::Update() {
  if (!impl_->active || impl_->error) {
    return;
  }

  const auto result = impl_->otg.update(impl_->input, impl_->output);

  if (result != ruckig::Result::Working &&
      result != ruckig::Result::Finished) {
    impl_->MarkError();

    std::cerr << "[UnitIntervalMotionProfile] Ruckig update failed."
              << std::endl;
    return;
  }

  impl_->position = Impl::ClampToUnitInterval(impl_->output.new_position[0]);
  impl_->velocity = impl_->output.new_velocity[0];
  impl_->acceleration = impl_->output.new_acceleration[0];

  impl_->output.pass_to_input(impl_->input);

  if (result != ruckig::Result::Finished) {
    return;
  }

  impl_->active = false;

  if (impl_->input.control_interface == ruckig::ControlInterface::Position) {
    impl_->position = Impl::ClampToUnitInterval(
        impl_->input.target_position[0]);
    impl_->velocity = 0.0;
    impl_->acceleration = 0.0;
  } else if (impl_->input.control_interface ==
             ruckig::ControlInterface::Velocity) {
    impl_->velocity = 0.0;
    impl_->acceleration = 0.0;
  }

  impl_->SyncCurrentStateToInput();

  if (impl_->stop_pending) {
    impl_->active = false;
    impl_->stop_pending = false;
    impl_->stop_completed = true;
    impl_->velocity = 0.0;
    impl_->acceleration = 0.0;
    impl_->input = ruckig::InputParameter<1>();
    impl_->output = ruckig::OutputParameter<1>();
    impl_->SyncCurrentStateToInput();
  }
}

double UnitIntervalMotionProfile::position() const {
  return impl_->position;
}

double UnitIntervalMotionProfile::velocity() const {
  return impl_->velocity;
}

double UnitIntervalMotionProfile::acceleration() const {
  return impl_->acceleration;
}

bool UnitIntervalMotionProfile::IsActive() const {
  return impl_->active;
}

bool UnitIntervalMotionProfile::IsSegmentFinished() const {
  return !impl_->active && !impl_->error;
}

bool UnitIntervalMotionProfile::HasError() const {
  return impl_->error;
}

bool UnitIntervalMotionProfile::IsStopCompleted() const {
  return impl_->stop_completed;
}

bool UnitIntervalMotionProfile::HasReachedTarget(double epsilon) const {
  return std::abs(impl_->position - 1.0) < epsilon &&
         std::abs(impl_->velocity) < epsilon &&
         std::abs(impl_->acceleration) < epsilon;
}

bool UnitIntervalMotionProfile::IsStopped(double epsilon) const {
  return std::abs(impl_->velocity) < epsilon &&
         std::abs(impl_->acceleration) < epsilon;
}
