#!/usr/bin/env python3
"""
笛卡尔往复点动 — 漂移累积测试

+3s → -3s 交替点动 N 个循环，每 20ms 采样法兰位姿，
验证末端是否越来越偏离初始点。

用法：python3 test/cartesian_reciprocating_test.py [--cycles N] [--host HOST] [--port PORT]
"""

import argparse
import json
import math
import sys
import time
import urllib.request
import urllib.error
from pathlib import Path
from collections import defaultdict


HOME_JOINTS_DEG = [0.0, 60.0, 0.0, 90.0, 0.0, -60.0, 0.0]
HOME_JOINTS_RAD = [d * math.pi / 180.0 for d in HOME_JOINTS_DEG]
PHASE_DURATION = 3.0    # 单程 3s
JOG_SPEED = 0.03        # m/s
JOG_TIMEOUT = 0.20
FEED_PERIOD = 0.05
SAMPLE_PERIOD = 0.02
CYCLES = 5              # 默认 5 个循环


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

    def move_joint(self, joints, speed=0.5, accel=1.0, min_time=5.0):
        return self._post("/api/move/joint", {
            "joints": joints, "speed": speed, "acceleration": accel,
            "time": min_time, "asynchronous": True,
        }, timeout=60)

    def jog_cartesian(self, twist, speed=JOG_SPEED, timeout=JOG_TIMEOUT):
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


def flange_pos(flange):
    p = flange.get("position", {})
    return (p.get("x", 0), p.get("y", 0), p.get("z", 0))


def jog_phase(client, twist, duration, samples_out, label):
    """执行一段点动，持续 duration 秒，记录采样。
    结束后等待完全减速停止，保证下一段从静止开始。"""
    t0 = time.monotonic()
    next_feed = t0
    next_sample = t0
    started = False  # 第一帧 FeedJog 可能因上一段未停被拒，循环直到成功

    while True:
        now = time.monotonic()
        elapsed = now - t0
        if elapsed >= duration:
            break

        if now >= next_feed:
            resp = client.jog_cartesian(twist)
            if resp.get("success"):
                started = True
            elif not started:
                # 上一段还在减速 → 重试，不推进计时
                next_feed = now + FEED_PERIOD
                next_sample = now + SAMPLE_PERIOD  # 也推迟采样，等真正开始
                t0 = now  # 重置计时起点，保证本段满 3s
                time.sleep(0.01)
                continue
            next_feed = now + FEED_PERIOD

        if now >= next_sample and started:
            state = client.get_state()
            flange = state.get("data", {}).get("flange",
                     state.get("data", {}).get("flange_pose"))
            if flange:
                p = flange_pos(flange)
                samples_out.append({"t_total": samples_out[-1]["t_total"] + elapsed
                                    if samples_out else elapsed,
                                    "pos": p, "label": label, "phase_elapsed": elapsed})
            next_sample = now + SAMPLE_PERIOD

        time.sleep(min(FEED_PERIOD, SAMPLE_PERIOD,
                       max(0.001, next_feed - now, next_sample - now)))

    # 停止 + 等减速到 0，保证下一段从静止开始
    client.jog_stop()
    wait_idle(client, 5.0)
    time.sleep(0.1)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--cycles", type=int, default=CYCLES)
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--out", default="build/cartesian_reciprocating")
    args = p.parse_args()

    client = RocosClient(args.host, args.port)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"笛卡尔往复点动测试 — {args.cycles} 循环, ±{PHASE_DURATION}s")
    print(f"连接到 {client.base}")

    if not client.is_enabled():
        client.enable()
        time.sleep(1.0)

    # MoveJ 回零
    print("MoveJ → home...", end=" ", flush=True)
    r = client.move_joint(HOME_JOINTS_RAD, speed=0.5, accel=1.0, min_time=5.0)
    if not r.get("success"):
        print(f"FAIL: {r.get('message')}")
        return 1
    wait_idle(client, 30.0)
    time.sleep(0.3)
    print("done")

    # 记录初始点
    state = client.get_state()
    flange = state.get("data", {}).get("flange",
             state.get("data", {}).get("flange_pose"))
    p0 = flange_pos(flange)
    print(f"初始位置: ({p0[0]:.6f}, {p0[1]:.6f}, {p0[2]:.6f})")

    all_samples = [{"t_total": 0.0, "pos": p0, "label": "init", "phase_elapsed": 0}]
    twist_pos_x = [1, 0, 0, 0, 0, 0]
    twist_neg_x = [-1, 0, 0, 0, 0, 0]

    for cycle in range(args.cycles):
        print(f"\nCycle {cycle+1}/{args.cycles}:", end=" ", flush=True)

        # +X 3s
        print("+X...", end=" ", flush=True)
        jog_phase(client, twist_pos_x, PHASE_DURATION, all_samples,
                  f"c{cycle+1}_+X")

        # -X 3s
        print("-X...", end=" ", flush=True)
        jog_phase(client, twist_neg_x, PHASE_DURATION, all_samples,
                  f"c{cycle+1}_-X")

        print("done")

    # 记录最终点
    state = client.get_state()
    flange = state.get("data", {}).get("flange",
             state.get("data", {}).get("flange_pose"))
    pn = flange_pos(flange)
    all_samples.append({"t_total": all_samples[-1]["t_total"], "pos": pn,
                        "label": "final", "phase_elapsed": 0})

    # ── 分析 ──
    # 提取每个 phase 的开始和结束位置
    phases = defaultdict(list)
    for s in all_samples:
        phases[s["label"]].append(s)

    # 每段的首尾位置
    phase_endpoints = []
    labels_in_order = ["init"]
    for c in range(args.cycles):
        labels_in_order.append(f"c{c+1}_+X")
        labels_in_order.append(f"c{c+1}_-X")
    labels_in_order.append("final")

    for label in labels_in_order:
        pts = phases.get(label, [])
        if not pts:
            continue
        first = pts[0]["pos"]
        last = pts[-1]["pos"]
        phase_endpoints.append({"label": label, "start": first, "end": last})

    print(f"\n{'='*80}")
    print(f"  笛卡尔往复点动 — 漂移分析 ({args.cycles} 循环)")
    print(f"{'='*80}")
    print(f"  {'Phase':<12} {'Start':>28} {'End':>28} {'dPos(mm)':>12}")
    print(f"  {'':-<12} {'':->28} {'':->28} {'':->12}")

    for ep in phase_endpoints:
        s, e = ep["start"], ep["end"]
        dp = math.sqrt((e[0]-s[0])**2 + (e[1]-s[1])**2 + (e[2]-s[2])**2) * 1000
        print(f"  {ep['label']:<12} ({s[0]:8.5f},{s[1]:8.5f},{s[2]:8.5f}) "
              f"({e[0]:8.5f},{e[1]:8.5f},{e[2]:8.5f}) {dp:>10.4f}")

    # 每个循环结束时的位置 vs 初始
    print(f"\n  {'Cycle':<8} {'End Pos (mm from start)':>40} {'Drift(mm)':>12} {'Accum(mm)':>12}")
    print(f"  {'':-<8} {'':->40} {'':->12} {'':->12}")
    accum_drift = 0.0
    for c in range(args.cycles):
        label = f"c{c+1}_-X"
        pts = phases.get(label, [])
        if pts:
            pend = pts[-1]["pos"]
            dx = (pend[0] - p0[0]) * 1000
            dy = (pend[1] - p0[1]) * 1000
            dz = (pend[2] - p0[2]) * 1000
            drift = math.sqrt(dx**2 + dy**2 + dz**2)
            print(f"  {c+1:<8} ({dx:8.5f},{dy:8.5f},{dz:8.5f}) {drift:>10.4f} mm")

    # 最终漂移
    dx_final = (pn[0] - p0[0]) * 1000
    dy_final = (pn[1] - p0[1]) * 1000
    dz_final = (pn[2] - p0[2]) * 1000
    drift_final = math.sqrt(dx_final**2 + dy_final**2 + dz_final**2)
    print(f"\n  总漂移: {drift_final:.4f} mm  (dX={dx_final:.4f}, dY={dy_final:.4f}, dZ={dz_final:.4f}) mm")

    # ── 画图 ──
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib 未安装，跳过绘图")
        return 0

    fig, axes = plt.subplots(2, 1, figsize=(16, 9), sharex=True)
    fig.suptitle(f"Cartesian Reciprocating Jog — {args.cycles} cycles, ±{PHASE_DURATION}s X-axis", fontsize=14)

    ts = [s["t_total"] for s in all_samples]
    xs = [(s["pos"][0] - p0[0]) * 1000 for s in all_samples]
    ys = [(s["pos"][1] - p0[1]) * 1000 for s in all_samples]
    zs = [(s["pos"][2] - p0[2]) * 1000 for s in all_samples]

    # 上：X 位置（主轴）vs 时间
    ax = axes[0]
    ax.plot(ts, xs, linewidth=0.5, color="steelblue", label="dX (main axis)")
    ax.axhline(y=0, color="gray", ls="--", lw=0.5)
    # 标记每段
    colors_phase = ["green", "red"] * args.cycles
    phase_start_t = 0
    for i, ep in enumerate(phase_endpoints):
        if ep["label"] == "init" or ep["label"] == "final":
            continue
        idxs = [j for j, s in enumerate(all_samples) if s["label"] == ep["label"]]
        if idxs:
            t_phase = [all_samples[j]["t_total"] for j in idxs]
            ax.axvspan(t_phase[0], t_phase[-1], alpha=0.08,
                       color=colors_phase[i-1] if i > 0 else "green")
    ax.set_ylabel("dX (mm)")
    ax.set_title("Main axis displacement X vs time")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 下：偏轴 Y,Z 偏差 vs 时间
    ax = axes[1]
    ax.plot(ts, ys, linewidth=0.5, alpha=0.8, label="dY (off-axis)")
    ax.plot(ts, zs, linewidth=0.5, alpha=0.8, label="dZ (off-axis)")
    ax.axhline(y=0, color="gray", ls="--", lw=0.5)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("deviation (mm)")
    ax.set_title("Off-axis deviation vs time")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    fig_path = out_dir / "cartesian_reciprocating.png"
    plt.savefig(fig_path, dpi=150)
    print(f"\nPlot: {fig_path}")

    # ── 判断 ──
    if drift_final < 0.5:
        print(f"\n✅ PASS — 最终漂移 {drift_final:.4f}mm < 0.5mm")
    else:
        print(f"\n⚠️  漂移 {drift_final:.4f}mm，存在累积趋势")

    return 0


if __name__ == "__main__":
    sys.exit(main())
