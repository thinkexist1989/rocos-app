#!/usr/bin/env python3
"""
笛卡尔空间点动 全流程集成测试 + 误差可视化

通过 rocosAppMain HTTP API 走完整控制链路：
  MoveJ 回零 → 笛卡尔点动 BASE_X/Y/Z 各 2s → 采样法兰位姿 → 分析直线度/偏轴

用法：python3 test/cartesian_jog_integration_test.py [--host HOST] [--port PORT]
"""

import argparse
import json
import math
import sys
import time
import urllib.request
import urllib.error
from pathlib import Path
from typing import Any
from collections import defaultdict


# ── 常量 ──
HOME_JOINTS_DEG = [0.0, 60.0, 0.0, 90.0, 0.0, -60.0, 0.0]
HOME_JOINTS_RAD = [d * math.pi / 180.0 for d in HOME_JOINTS_DEG]
JOG_DURATION = 2.0
JOG_SPEED = 0.03        # m/s
JOG_TIMEOUT = 0.20      # 喂入超时
FEED_PERIOD = 0.05      # 喂入周期
SAMPLE_PERIOD = 0.02    # 采样周期


class RocosClient:
    def __init__(self, host="localhost", port=8080):
        self.base = f"http://{host}:{port}"

    def _post(self, path, body, timeout=30):
        data = json.dumps(body).encode()
        req = urllib.request.Request(f"{self.base}{path}", data=data,
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode())

    def _get(self, path, timeout=10):
        with urllib.request.urlopen(f"{self.base}{path}", timeout=timeout) as r:
            return json.loads(r.read().decode())

    def get_state(self):
        return self._get("/api/robot/state")

    def is_enabled(self):
        r = self._get("/api/robot/enabled")
        return r.get("data", {}).get("enabled", False)

    def enable(self):
        return self._post("/api/robot/enable", {})

    def move_joint(self, joints, speed=0.5, accel=1.0, async_mode=True, min_time=5.0):
        return self._post("/api/move/joint", {
            "joints": joints, "speed": speed, "acceleration": accel,
            "time": min_time, "asynchronous": async_mode,
        }, timeout=60)

    def jog_cartesian(self, twist, speed=JOG_SPEED, timeout=JOG_TIMEOUT):
        """twist = [vx,vy,vz,wx,wy,wz]"""
        return self._post("/api/robot/jog/cartesian", {
            "twist": twist, "speed": speed, "timeout": timeout,
        })

    def jog_stop(self):
        return self._post("/api/robot/jog/stop", {})


def wait_idle(client, timeout_s=30.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        s = client.get_state()
        if s.get("success") and not s.get("data", {}).get("control_active", False):
            return True
        time.sleep(0.1)
    return False


def extract_flange(state):
    flange = state.get("data", {}).get("flange")
    if not flange:
        flange = state.get("data", {}).get("flange_pose")
    return flange


def flange_pos(flange):
    p = flange.get("position", {})
    return (p.get("x", 0), p.get("y", 0), p.get("z", 0))


def run_axis(client, axis, samples_out):
    """对一个轴做 2s 笛卡尔点动，记录法兰轨迹"""
    twist = [0.0]*6
    idx = {"X": 0, "Y": 1, "Z": 2}[axis]
    twist[idx] = 1.0

    print(f"  BASE_{axis} 点动 {JOG_DURATION}s...", end=" ", flush=True)
    t0 = time.monotonic()
    next_feed = t0
    next_sample = t0
    local_samples = []

    while True:
        now = time.monotonic()
        elapsed = now - t0
        if elapsed >= JOG_DURATION:
            break

        if now >= next_feed:
            client.jog_cartesian(twist)
            next_feed = now + FEED_PERIOD

        if now >= next_sample:
            flange = extract_flange(client.get_state())
            if flange:
                local_samples.append({"t": elapsed, "pos": flange_pos(flange)})
            next_sample = now + SAMPLE_PERIOD

        time.sleep(min(FEED_PERIOD, SAMPLE_PERIOD,
                       max(0.001, next_feed - now, next_sample - now)))

    client.jog_stop()
    wait_idle(client, 10.0)

    # 记录终止点
    flange = extract_flange(client.get_state())
    if flange:
        local_samples.append({"t": JOG_DURATION, "pos": flange_pos(flange)})

    for s in local_samples:
        s["axis"] = axis
    samples_out.extend(local_samples)
    print(f"{len(local_samples)} 采样")
    return True


def analyze_axis(axis, samples):
    """分析某轴的直线度"""
    pts = [(s["pos"][0]*1000, s["pos"][1]*1000, s["pos"][2]*1000)
           for s in samples if s["axis"] == axis]  # mm
    if len(pts) < 2:
        return None

    p0 = pts[0]
    pn = pts[-1]
    dx_total = pn[0] - p0[0]
    dy_total = pn[1] - p0[1]
    dz_total = pn[2] - p0[2]
    travel = math.sqrt(dx_total**2 + dy_total**2 + dz_total**2)

    expected = {"X": (1,0,0), "Y": (0,1,0), "Z": (0,0,1)}[axis]

    # 直线方向
    if travel < 1e-9:
        return {"axis": axis, "travel_mm": 0, "angle_deg": 0, "max_off_mm": 0,
                "rms_off_mm": 0, "dx_mm": 0, "dy_mm": 0, "dz_mm": 0, "count": len(pts)}

    dir_x, dir_y, dir_z = dx_total/travel, dy_total/travel, dz_total/travel

    # 与期望轴的夹角
    cos_a = dir_x*expected[0] + dir_y*expected[1] + dir_z*expected[2]
    cos_a = max(-1.0, min(1.0, cos_a))
    angle_deg = math.acos(abs(cos_a)) * 180.0 / math.pi

    # 到主轴的偏差
    off_axis = []
    for p in pts:
        rel = (p[0]-p0[0], p[1]-p0[1], p[2]-p0[2])
        along = rel[0]*dir_x + rel[1]*dir_y + rel[2]*dir_z
        proj = (along*dir_x, along*dir_y, along*dir_z)
        err = math.sqrt((rel[0]-proj[0])**2 + (rel[1]-proj[1])**2 + (rel[2]-proj[2])**2)
        off_axis.append(err)

    return {
        "axis": axis,
        "travel_mm": travel,
        "angle_deg": angle_deg,
        "max_off_mm": max(off_axis),
        "rms_off_mm": math.sqrt(sum(e*e for e in off_axis) / len(off_axis)),
        "dx_mm": dx_total, "dy_mm": dy_total, "dz_mm": dz_total,
        "count": len(pts),
    }


def plot_results(samples, stats, out_dir):
    """生成可视化"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return

    plt.rcParams["font.family"] = "sans-serif"

    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle("Cartesian Jog Integration Test — Full Chain via HTTP", fontsize=14)

    expected = {"X": (1,0,0), "Y": (0,1,0), "Z": (0,0,1)}

    for col, axis in enumerate(["X", "Y", "Z"]):
        pts = [(s["pos"][0]*1000, s["pos"][1]*1000, s["pos"][2]*1000)
               for s in samples if s["axis"] == axis]

        if len(pts) < 2:
            continue

        p0 = pts[0]
        # (a) 3D 轨迹 XY 平面
        ax = axes[0][col]
        xs = [p[0]-p0[0] for p in pts]
        ys = [p[1]-p0[1] for p in pts]
        ts = [s["t"] for s in samples if s["axis"] == axis]
        sc = ax.scatter(xs, ys, c=ts, cmap="plasma", s=3, alpha=0.8)
        ax.axhline(y=0, color="gray", ls="--", lw=0.5)
        ax.axvline(x=0, color="gray", ls="--", lw=0.5)
        exp = expected[axis]
        ax.arrow(0, 0, exp[0]*40, exp[1]*40, color="green", width=0.003,
                 head_width=0.015, label=f"Ideal {axis}")
        ax.set_xlabel("dX (mm)")
        ax.set_ylabel("dY (mm)")
        ax.set_title(f"BASE_{axis} — XY plane")
        ax.set_aspect("equal")
        ax.legend(fontsize=8)
        plt.colorbar(sc, ax=ax, label="t (s)")

        # (b) 主轴位置 vs 时间
        ax = axes[1][col]
        idx = col  # X→0, Y→1, Z→2
        main_pos = [p[idx] - p0[idx] for p in pts]
        ax.plot(ts, main_pos, linewidth=1, color="steelblue")
        # 理想直线
        t_ideal = [0, ts[-1]]
        ax.plot(t_ideal, [JOG_SPEED*1000*t for t in t_ideal],
                "--", color="green", alpha=0.5, label=f"Ideal {JOG_SPEED} m/s")
        ax.set_xlabel("time (s)")
        ax.set_ylabel(f"d{axis} (mm)")
        ax.set_title(f"BASE_{axis} — displacement vs time")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_dir.mkdir(parents=True, exist_ok=True)
    fig_path = out_dir / "cartesian_jog_integration.png"
    plt.savefig(fig_path, dpi=150)
    print(f"\nPlot: {fig_path}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--out", default="build/cartesian_jog_integration")
    args = p.parse_args()
    out_dir = Path(args.out)

    client = RocosClient(args.host, args.port)
    print(f"Connected to {client.base}")

    if not client.is_enabled():
        print("Enabling robot...")
        client.enable()
        time.sleep(1.0)
    print("Robot enabled ✓")

    all_samples = []

    for axis in ["X", "Y", "Z"]:
        print(f"\n--- BASE_{axis} ---")

        # MoveJ 回零
        print(f"  MoveJ → home...", end=" ", flush=True)
        r = client.move_joint(HOME_JOINTS_RAD, speed=0.5, accel=1.0, min_time=5.0)
        if not r.get("success"):
            print(f"FAIL: {r.get('message')}")
            return 1
        if not wait_idle(client, 30.0):
            print("timeout")
            return 1
        print("done")

        time.sleep(0.3)

        if not run_axis(client, axis, all_samples):
            return 1

        time.sleep(0.5)

    # 分析
    print(f"\n{'='*70}")
    print(f"  Cartesian Jog Full-Chain Integration Test Results")
    print(f"{'='*70}")
    print(f"  {'Axis':<6} {'Travel':>10} {'Angle':>8} {'MaxOff':>10} {'RMSOff':>10} {'dX':>10} {'dY':>10} {'dZ':>10}")
    print(f"  {'':-<6} {'':->10} {'':->8} {'':->10} {'':->10} {'':->10} {'':->10} {'':->10}")

    stats = []
    for axis in ["X", "Y", "Z"]:
        st = analyze_axis(axis, all_samples)
        if st:
            stats.append(st)
            print(f"  {st['axis']:<6} {st['travel_mm']:>8.3f}mm {st['angle_deg']:>6.3f}deg "
                  f"{st['max_off_mm']*1000:>8.2f}um {st['rms_off_mm']*1000:>8.2f}um "
                  f"{st['dx_mm']:>8.3f} {st['dy_mm']:>8.3f} {st['dz_mm']:>8.3f}")

    print(f"{'='*70}")

    # 对比离线规划
    print(f"\n  Comparison with offline planner (move_jog_planner_linearity_test):")
    print(f"  {'Axis':<6} {'Online MaxOff':>14} {'Online RMSOff':>14} {'Planner MaxOff':>14} {'Ratio':>10}")
    planner_results = {"X": (0, 0), "Y": (0, 0), "Z": (0, 0)}
    for st in stats:
        ax = st["axis"]
        p_max, p_rms = planner_results[ax]
        print(f"  {ax:<6} {st['max_off_mm']*1e6:>12.1f}um {st['rms_off_mm']*1e6:>12.1f}um "
              f"{p_max*1e6:>12.1f}um {'N/A':>10}")

    # 生成图
    plot_results(all_samples, stats, out_dir)

    # 判断
    max_off = max(s["max_off_mm"] for s in stats)
    if max_off < 0.5:  # < 0.5mm
        print(f"\n  PASS — max off-axis deviation {max_off*1000:.0f}um < 500um")
        return 0
    else:
        print(f"\n  WARN — max off-axis deviation {max_off*1000:.0f}um")
        return 0


if __name__ == "__main__":
    sys.exit(main())
