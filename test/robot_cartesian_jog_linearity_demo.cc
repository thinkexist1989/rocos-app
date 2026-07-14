// Cartesian jog linearity demo.
//
// Run with the MuJoCo/EtherCAT simulator already started:
//   ./build/bin/robot_cartesian_jog_linearity_demo
//
// The demo moves to [0, 60, 0, 90, 0, -60, 0] deg, jogs BASE X/Y/Z for
// 2 seconds each, records flange FK points, and writes CSV/plots under
// build/jog_linearity.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "src/robot.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kJogSpeed = 0.03;       // m/s
constexpr double kJogDuration = 2.0;     // s
constexpr double kJogTimeout = 0.20;     // s, must be larger than feed period
constexpr double kSamplePeriod = 0.02;   // s

struct Point3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct Sample {
    std::string axis;
    double t{0.0};
    Point3 p;
    std::vector<double> q;
};

struct LineStats {
    std::string axis;
    int count{0};
    double travel{0.0};
    double max_error{0.0};
    double rms_error{0.0};
    double angle_deg{0.0};
    double dx{0.0};
    double dy{0.0};
    double dz{0.0};
};

bool isOk(rocos::Result result) {
    return result == rocos::Result::NoError;
}

double norm(const Point3& p) {
    return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

Point3 sub(const Point3& a, const Point3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

double dot(const Point3& a, const Point3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Point3 scale(const Point3& p, double s) {
    return {p.x * s, p.y * s, p.z * s};
}

Point3 flangePosition(rocos::Robot& robot) {
    const rocos::Frame flange = robot.getFlange();
    return {flange.p.x(), flange.p.y(), flange.p.z()};
}

std::vector<double> jointPositions(rocos::Robot& robot) {
    std::vector<double> q;
    const int n = robot.getJointNum();
    q.reserve(static_cast<size_t>(std::max(0, n)));
    for (int i = 0; i < n; ++i) {
        q.push_back(robot.getJointPosition(i));
    }
    return q;
}

bool waitControlDone(rocos::Robot& robot, double timeout_s) {
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeout_s));

    while (std::chrono::steady_clock::now() < deadline) {
        if (!robot.IsControlActive()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return !robot.IsControlActive();
}

rocos::JntArray makeHomeJoints() {
    rocos::JntArray q(7);
    const double deg[7] = {0.0, 60.0, 0.0, 90.0, 0.0, -60.0, 0.0};
    for (int i = 0; i < 7; ++i) {
        q(i) = deg[i] * kDegToRad;
    }
    return q;
}

bool moveHome(rocos::Robot& robot) {
    std::cout << "MoveJ to home: [0, 60, 0, 90, 0, -60, 0] deg" << std::endl;
    const rocos::Result rc = robot.MoveJ(makeHomeJoints(), 0.5, 1.0, 5.0);
    if (!isOk(rc)) {
        std::cerr << "MoveJ failed, result=" << static_cast<int>(rc) << std::endl;
        return false;
    }
    if (!waitControlDone(robot, 30.0)) {
        std::cerr << "MoveJ timeout, current state=" << robot.GetStateString() << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    return true;
}

rocos::Twist directionForAxis(const std::string& axis) {
    rocos::Twist twist;
    if (axis == "X") {
        twist.vel.x(1.0);
    } else if (axis == "Y") {
        twist.vel.y(1.0);
    } else {
        twist.vel.z(1.0);
    }
    return twist;
}

Point3 expectedForAxis(const std::string& axis) {
    if (axis == "X") return {1.0, 0.0, 0.0};
    if (axis == "Y") return {0.0, 1.0, 0.0};
    return {0.0, 0.0, 1.0};
}

bool jogAxis(rocos::Robot& robot, const std::string& axis, std::vector<Sample>& samples) {
    std::cout << "Jog BASE_" << axis << " for " << kJogDuration << " s" << std::endl;
    const rocos::Twist direction = directionForAxis(axis);
    const auto start = std::chrono::steady_clock::now();
    auto next_sample = start;

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start).count();
        if (elapsed >= kJogDuration) {
            break;
        }

        const rocos::Result rc = robot.MoveJogging(direction, kJogSpeed, kJogTimeout);
        if (!isOk(rc)) {
            std::cerr << "MoveJogging BASE_" << axis
                      << " failed, result=" << static_cast<int>(rc) << std::endl;
            return false;
        }

        if (now >= next_sample) {
            samples.push_back({axis, elapsed, flangePosition(robot), jointPositions(robot)});
            next_sample += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(kSamplePeriod));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    const rocos::Result stop_rc = robot.StopMotion();
    if (!isOk(stop_rc)) {
        std::cerr << "StopMotion after BASE_" << axis
                  << " returned " << static_cast<int>(stop_rc) << std::endl;
    }

    if (!waitControlDone(robot, 10.0)) {
        std::cerr << "Jog stop timeout after BASE_" << axis
                  << ", current state=" << robot.GetStateString() << std::endl;
        return false;
    }

    samples.push_back({axis, kJogDuration, flangePosition(robot), jointPositions(robot)});
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    return true;
}

LineStats computeLineStats(const std::string& axis, const std::vector<Sample>& samples) {
    std::vector<Point3> points;
    for (const auto& sample : samples) {
        if (sample.axis == axis) {
            points.push_back(sample.p);
        }
    }

    LineStats stats;
    stats.axis = axis;
    stats.count = static_cast<int>(points.size());
    if (points.size() < 2) {
        stats.angle_deg = std::numeric_limits<double>::quiet_NaN();
        return stats;
    }

    const Point3 delta = sub(points.back(), points.front());
    stats.dx = delta.x;
    stats.dy = delta.y;
    stats.dz = delta.z;
    stats.travel = norm(delta);
    if (stats.travel < 1e-9) {
        stats.angle_deg = std::numeric_limits<double>::quiet_NaN();
        return stats;
    }

    const Point3 line_dir = scale(delta, 1.0 / stats.travel);
    double sum_sq = 0.0;
    for (const auto& p : points) {
        const Point3 rel = sub(p, points.front());
        const Point3 projection = scale(line_dir, dot(rel, line_dir));
        const double err = norm(sub(rel, projection));
        stats.max_error = std::max(stats.max_error, err);
        sum_sq += err * err;
    }
    stats.rms_error = std::sqrt(sum_sq / static_cast<double>(points.size()));

    const Point3 expected = expectedForAxis(axis);
    const double cos_angle = std::clamp(dot(line_dir, expected), -1.0, 1.0);
    stats.angle_deg = std::acos(cos_angle) * 180.0 / kPi;
    return stats;
}

void writeTrajectoryCsv(const std::filesystem::path& path, const std::vector<Sample>& samples) {
    std::ofstream out(path);
    out << std::fixed << std::setprecision(9);
    out << "axis,t,x,y,z";
    const int max_joint_count = 7;
    for (int i = 0; i < max_joint_count; ++i) {
        out << ",q" << i;
    }
    out << '\n';

    for (const auto& sample : samples) {
        out << sample.axis << ',' << sample.t << ','
            << sample.p.x << ',' << sample.p.y << ',' << sample.p.z;
        for (int i = 0; i < max_joint_count; ++i) {
            if (i < static_cast<int>(sample.q.size())) {
                out << ',' << sample.q[static_cast<size_t>(i)];
            } else {
                out << ',';
            }
        }
        out << '\n';
    }
}

void writeSummaryCsv(const std::filesystem::path& path, const std::vector<LineStats>& stats) {
    std::ofstream out(path);
    out << std::fixed << std::setprecision(9);
    out << "axis,count,travel,max_error,rms_error,angle_deg,dx,dy,dz\n";
    for (const auto& item : stats) {
        out << item.axis << ',' << item.count << ','
            << item.travel << ',' << item.max_error << ','
            << item.rms_error << ',' << item.angle_deg << ','
            << item.dx << ',' << item.dy << ',' << item.dz << '\n';
    }
}

}  // namespace

int main() {
    const std::filesystem::path output_dir = "build/jog_linearity";
    std::filesystem::create_directories(output_dir);

    rocos::Robot robot;
    std::cout << "Initial state: " << robot.GetStateString() << std::endl;

    if (!robot.IsEnabled()) {
        const rocos::Result rc = robot.SetEnabled();
        if (!isOk(rc)) {
            std::cerr << "SetEnabled failed, result=" << static_cast<int>(rc)
                      << ", state=" << robot.GetStateString() << std::endl;
            return 1;
        }
    }

    if (robot.getJointNum() != 7) {
        std::cerr << "Expected 7 joints, got " << robot.getJointNum() << std::endl;
        return 1;
    }

    std::vector<Sample> samples;
    const std::vector<std::string> axes = {"X", "Y", "Z"};
    for (const auto& axis : axes) {
        if (!moveHome(robot)) {
            return 1;
        }
        samples.push_back({axis, -0.001, flangePosition(robot), jointPositions(robot)});
        if (!jogAxis(robot, axis, samples)) {
            return 1;
        }
    }

    std::vector<LineStats> stats;
    for (const auto& axis : axes) {
        stats.push_back(computeLineStats(axis, samples));
    }

    const auto trajectory_path = output_dir / "trajectory.csv";
    const auto summary_path = output_dir / "summary.csv";
    writeTrajectoryCsv(trajectory_path, samples);
    writeSummaryCsv(summary_path, stats);

    std::cout << "\nLinearity summary" << std::endl;
    std::cout << "axis  travel(m)  max_err(m)  rms_err(m)  angle_to_+axis(deg)  delta(x,y,z)" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    for (const auto& item : stats) {
        std::cout << item.axis << "     "
                  << item.travel << "    "
                  << item.max_error << "    "
                  << item.rms_error << "    "
                  << item.angle_deg << "              ("
                  << item.dx << ", " << item.dy << ", " << item.dz << ")"
                  << std::endl;
    }

    std::cout << "\nWrote: " << trajectory_path << std::endl;
    std::cout << "Wrote: " << summary_path << std::endl;

    const std::string plot_cmd =
        "python3 scripts/plot_jog_linearity.py "
        + trajectory_path.string() + " "
        + output_dir.string();
    const int plot_rc = std::system(plot_cmd.c_str());
    if (plot_rc == 0) {
        std::cout << "Plots written under: " << output_dir << std::endl;
    } else {
        std::cout << "Plot command failed. You can still inspect the CSV files directly." << std::endl;
    }

    robot.SetDisabled();
    return 0;
}
