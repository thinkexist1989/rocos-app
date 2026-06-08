#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>

#include <Eigen/Geometry>

#include <rocos_app/UnitIntervalMotionProfile.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {

constexpr double kEps = 1e-9;

void RunUntilIdle(UnitIntervalMotionProfile& profile,
                  int max_steps = 100000) {
  for (int i = 0;
       i < max_steps && profile.IsActive() && !profile.HasError();
       ++i) {
    profile.Update();
  }
}

struct NormalizedLimits {
  double max_velocity = std::numeric_limits<double>::infinity();
  double max_acceleration = std::numeric_limits<double>::infinity();
  double max_jerk = std::numeric_limits<double>::infinity();
};

struct MoveJCommand {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
};

NormalizedLimits ComputeMoveJLimits(
    const std::vector<double>& q_start,
    const std::vector<double>& q_goal,
    const std::vector<double>& q_dot_max,
    const std::vector<double>& q_ddot_max,
    const std::vector<double>& q_jerk_max) {
  NormalizedLimits limits;

  for (std::size_t i = 0; i < q_start.size(); ++i) {
    const double delta = std::abs(q_goal[i] - q_start[i]);
    if (delta < kEps) {
      continue;
    }

    limits.max_velocity = std::min(limits.max_velocity,
                                   q_dot_max[i] / delta);
    limits.max_acceleration = std::min(limits.max_acceleration,
                                       q_ddot_max[i] / delta);
    limits.max_jerk = std::min(limits.max_jerk,
                               q_jerk_max[i] / delta);
  }

  return limits;
}

MoveJCommand SampleMoveJ(const std::vector<double>& q_start,
                         const std::vector<double>& q_goal,
                         double s,
                         double s_dot,
                         double s_ddot) {
  MoveJCommand command;
  command.position.resize(q_start.size());
  command.velocity.resize(q_start.size());
  command.acceleration.resize(q_start.size());

  for (std::size_t i = 0; i < q_start.size(); ++i) {
    const double delta = q_goal[i] - q_start[i];
    command.position[i] = q_start[i] + s * delta;
    command.velocity[i] = s_dot * delta;
    command.acceleration[i] = s_ddot * delta;
  }

  return command;
}

struct MoveLPath {
  Eigen::Vector3d p_start;
  Eigen::Vector3d dp;
  Eigen::Quaterniond q_start;
  Eigen::Quaterniond q_goal;
  double length = 0.0;
  double theta = 0.0;
};

MoveLPath MakeMoveLPath(const Eigen::Vector3d& p_start,
                        const Eigen::Vector3d& p_goal,
                        const Eigen::Quaterniond& q_start,
                        const Eigen::Quaterniond& q_goal) {
  MoveLPath path;
  path.p_start = p_start;
  path.dp = p_goal - p_start;
  path.length = path.dp.norm();
  path.q_start = q_start.normalized();
  path.q_goal = q_goal.normalized();

  if (path.q_start.dot(path.q_goal) < 0.0) {
    path.q_goal.coeffs() *= -1.0;
  }

  const double cos_half_theta = std::clamp(
      std::abs(path.q_start.dot(path.q_goal)), 0.0, 1.0);
  path.theta = 2.0 * std::acos(cos_half_theta);

  return path;
}

NormalizedLimits ComputeMoveLLimits(double length,
                                    double theta,
                                    double max_linear_velocity,
                                    double max_linear_acceleration,
                                    double max_linear_jerk,
                                    double max_angular_velocity,
                                    double max_angular_acceleration,
                                    double max_angular_jerk) {
  NormalizedLimits limits;

  if (length > kEps) {
    limits.max_velocity = std::min(limits.max_velocity,
                                   max_linear_velocity / length);
    limits.max_acceleration = std::min(limits.max_acceleration,
                                       max_linear_acceleration / length);
    limits.max_jerk = std::min(limits.max_jerk,
                               max_linear_jerk / length);
  }

  if (theta > kEps) {
    limits.max_velocity = std::min(limits.max_velocity,
                                   max_angular_velocity / theta);
    limits.max_acceleration = std::min(limits.max_acceleration,
                                       max_angular_acceleration / theta);
    limits.max_jerk = std::min(limits.max_jerk,
                               max_angular_jerk / theta);
  }

  return limits;
}

}  // namespace

TEST_CASE("UnitIntervalMotionProfile completes start update finish") {
  UnitIntervalMotionProfile profile(0.001);

  profile.Start(0.5, 1.0, 4.0);
  RunUntilIdle(profile);

  CHECK(profile.position() == doctest::Approx(1.0));
  CHECK(profile.HasReachedTarget());
  CHECK_FALSE(profile.HasError());
}

TEST_CASE("UnitIntervalMotionProfile pause stops before target") {
  UnitIntervalMotionProfile profile(0.001);

  profile.Start(0.5, 1.0, 4.0);
  for (int i = 0; i < 100; ++i) {
    profile.Update();
  }
  profile.Pause();
  RunUntilIdle(profile);

  CHECK(profile.position() > 0.0);
  CHECK(profile.position() < 1.0);
  CHECK(profile.IsStopped());
  CHECK_FALSE(profile.IsStopCompleted());
  CHECK_FALSE(profile.HasError());
}

TEST_CASE("UnitIntervalMotionProfile resumes from pause and reaches target") {
  UnitIntervalMotionProfile profile(0.001);

  profile.Start(0.5, 1.0, 4.0);
  for (int i = 0; i < 100; ++i) {
    profile.Update();
  }
  profile.Pause();
  RunUntilIdle(profile);
  profile.Resume();
  RunUntilIdle(profile);

  CHECK(profile.position() == doctest::Approx(1.0));
  CHECK(profile.HasReachedTarget());
  CHECK_FALSE(profile.HasError());
}

TEST_CASE("UnitIntervalMotionProfile stop completes after deceleration") {
  UnitIntervalMotionProfile profile(0.001);

  profile.Start(0.5, 1.0, 4.0);
  for (int i = 0; i < 100; ++i) {
    profile.Update();
  }
  profile.Stop();
  RunUntilIdle(profile);

  CHECK(profile.position() > 0.0);
  CHECK(profile.position() < 1.0);
  CHECK(profile.IsStopCompleted());
  CHECK_FALSE(profile.HasError());
}

TEST_CASE("UnitIntervalMotionProfile stop completes immediately when paused") {
  UnitIntervalMotionProfile profile(0.001);

  profile.Start(0.5, 1.0, 4.0);
  for (int i = 0; i < 100; ++i) {
    profile.Update();
  }
  profile.Pause();
  RunUntilIdle(profile);
  profile.Stop();

  CHECK(profile.IsStopped());
  CHECK(profile.IsStopCompleted());
  CHECK_FALSE(profile.HasError());
}

TEST_CASE("UnitIntervalMotionProfile reset returns progress to zero") {
  UnitIntervalMotionProfile profile(0.001);

  profile.Start(0.5, 1.0, 4.0);
  RunUntilIdle(profile);
  profile.Reset();

  CHECK(profile.position() == doctest::Approx(0.0));
  CHECK_FALSE(profile.HasError());
}

TEST_CASE("UnitIntervalMotionProfile tolerates update and stop before start") {
  UnitIntervalMotionProfile profile(0.001);

  profile.Update();
  profile.Stop();

  CHECK(profile.position() == doctest::Approx(0.0));
  CHECK(profile.IsStopped());
  CHECK(profile.IsStopCompleted());
  CHECK_FALSE(profile.HasError());
}

TEST_CASE("UnitIntervalMotionProfile rejects invalid limits before update") {
  const double nan = std::numeric_limits<double>::quiet_NaN();

  for (const auto invalid_limits : {
           std::array<double, 3>{0.0, 1.0, 4.0},
           std::array<double, 3>{0.5, 0.0, 4.0},
           std::array<double, 3>{0.5, 1.0, 0.0},
           std::array<double, 3>{-0.5, 1.0, 4.0},
           std::array<double, 3>{0.5, -1.0, 4.0},
           std::array<double, 3>{0.5, 1.0, -4.0},
           std::array<double, 3>{nan, 1.0, 4.0},
       }) {
    UnitIntervalMotionProfile profile(0.001);

    profile.Start(invalid_limits[0], invalid_limits[1], invalid_limits[2]);

    CHECK(profile.HasError());
    CHECK_FALSE(profile.IsActive());
    CHECK(profile.position() == doctest::Approx(0.0));
  }
}

TEST_CASE("MoveJ maps unit progress to joint position velocity acceleration") {
  const std::vector<double> q_start{1.0, -2.0, 0.5};
  const std::vector<double> q_goal{1.5, -3.0, 0.5};
  const std::vector<double> q_dot_max{1.0, 1.5, 100.0};
  const std::vector<double> q_ddot_max{2.0, 3.0, 100.0};
  const std::vector<double> q_jerk_max{10.0, 8.0, 100.0};

  const auto limits = ComputeMoveJLimits(q_start,
                                         q_goal,
                                         q_dot_max,
                                         q_ddot_max,
                                         q_jerk_max);

  CHECK(limits.max_velocity == doctest::Approx(1.5));
  CHECK(limits.max_acceleration == doctest::Approx(3.0));
  CHECK(limits.max_jerk == doctest::Approx(8.0));

  UnitIntervalMotionProfile profile(0.001);
  profile.Start(limits.max_velocity,
                limits.max_acceleration,
                limits.max_jerk);

  while (profile.IsActive() && !profile.HasError()) {
    profile.Update();
    const auto command = SampleMoveJ(q_start,
                                    q_goal,
                                    profile.position(),
                                    profile.velocity(),
                                    profile.acceleration());

    for (std::size_t i = 0; i < q_start.size(); ++i) {
      CHECK(std::abs(command.velocity[i]) <= q_dot_max[i] + 1e-9);
      CHECK(std::abs(command.acceleration[i]) <= q_ddot_max[i] + 1e-9);
    }
  }

  const auto final_command = SampleMoveJ(q_start,
                                        q_goal,
                                        profile.position(),
                                        profile.velocity(),
                                        profile.acceleration());

  CHECK_FALSE(profile.HasError());
  for (std::size_t i = 0; i < q_goal.size(); ++i) {
    CHECK(final_command.position[i] == doctest::Approx(q_goal[i]));
    CHECK(final_command.velocity[i] == doctest::Approx(0.0));
    CHECK(final_command.acceleration[i] == doctest::Approx(0.0));
  }
}

TEST_CASE("MoveL maps unit progress to linear position and shortest slerp") {
  const Eigen::Vector3d p_start(0.1, -0.2, 0.3);
  const Eigen::Vector3d p_goal(0.6, -0.2, 0.3);
  const Eigen::Quaterniond q_start = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond q_goal_same_rotation(
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  q_goal_same_rotation.coeffs() *= -1.0;

  const auto path = MakeMoveLPath(p_start,
                                  p_goal,
                                  q_start,
                                  q_goal_same_rotation);
  const auto limits = ComputeMoveLLimits(path.length,
                                         path.theta,
                                         1.0,
                                         2.0,
                                         10.0,
                                         M_PI / 4.0,
                                         M_PI,
                                         4.0 * M_PI);

  CHECK(path.length == doctest::Approx(0.5));
  CHECK(path.theta == doctest::Approx(M_PI / 2.0));
  CHECK(path.q_start.dot(path.q_goal) >= 0.0);
  CHECK(limits.max_velocity == doctest::Approx(0.5));
  CHECK(limits.max_acceleration == doctest::Approx(2.0));
  CHECK(limits.max_jerk == doctest::Approx(8.0));

  UnitIntervalMotionProfile profile(0.001);
  profile.Start(limits.max_velocity,
                limits.max_acceleration,
                limits.max_jerk);

  while (profile.IsActive() && !profile.HasError()) {
    profile.Update();

    const double s = profile.position();
    const double s_dot = profile.velocity();
    const double s_ddot = profile.acceleration();
    const Eigen::Vector3d p_cmd = path.p_start + s * path.dp;
    const Eigen::Quaterniond q_cmd = path.q_start.slerp(s, path.q_goal);

    CHECK(p_cmd.y() == doctest::Approx(p_start.y()));
    CHECK(p_cmd.z() == doctest::Approx(p_start.z()));
    CHECK(q_cmd.norm() == doctest::Approx(1.0));
    CHECK(path.length * std::abs(s_dot) <= 1.0 + 1e-9);
    CHECK(path.length * std::abs(s_ddot) <= 2.0 + 1e-9);
    CHECK(path.theta * std::abs(s_dot) <= M_PI / 4.0 + 1e-9);
    CHECK(path.theta * std::abs(s_ddot) <= M_PI + 1e-9);
  }

  const Eigen::Vector3d p_final =
      path.p_start + profile.position() * path.dp;
  const Eigen::Quaterniond q_final =
      path.q_start.slerp(profile.position(), path.q_goal);

  CHECK_FALSE(profile.HasError());
  CHECK((p_final - p_goal).norm() == doctest::Approx(0.0));
  CHECK(std::abs(q_final.dot(path.q_goal)) == doctest::Approx(1.0));
}
