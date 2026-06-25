#ifndef ROCOS_APP_UNIT_INTERVAL_MOTION_PROFILE_H
#define ROCOS_APP_UNIT_INTERVAL_MOTION_PROFILE_H

#include <memory>

class VelocityProfile {
 public:
  explicit VelocityProfile(double dt);
  ~VelocityProfile();

  VelocityProfile(const VelocityProfile&) = delete;
  VelocityProfile& operator=(const VelocityProfile&) =
      delete;

  VelocityProfile(VelocityProfile&&) noexcept;
  VelocityProfile& operator=(VelocityProfile&&) noexcept;

  void Reset(double position = 0.0,
             double velocity = 0.0,
             double acceleration = 0.0);

  void Start(double max_velocity,
             double max_acceleration,
             double max_jerk);

  void Resume();

  void Pause(double max_acceleration = -1.0,
             double max_jerk = -1.0);

  void Stop(double max_acceleration = -1.0,
            double max_jerk = -1.0);

  void Update();

  double position() const;
  double velocity() const;
  double acceleration() const;

  bool IsActive() const;
  bool IsSegmentFinished() const;
  bool HasError() const;
  bool IsStopCompleted() const;
  bool HasReachedTarget(double epsilon = 1e-6) const;
  bool IsStopped(double epsilon = 1e-6) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

#endif  // ROCOS_APP_UNIT_INTERVAL_MOTION_PROFILE_H
