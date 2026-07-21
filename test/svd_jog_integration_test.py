#!/usr/bin/env python3
"""
SVD 零空间点动 全流程集成测试 (HTTP API)

通过 rocosAppMain 的 HTTP API 走完整控制链路：
  MoveJ 回零 → SVD 点动 2s (周期喂入) → 周期采样法兰位姿 → 对比位姿变化

前提：rocosAppMain 已启动（仿真或真实硬件均可）
用法：python3 test/svd_jog_integration_test.py [--host HOST] [--port PORT]
"""

import argparse
import json
import math
import sys
import time
import urllib.request
import urllib.error
from typing import Any

# ── 常量 ──
HOME_JOINTS_DEG = [0.0, 60.0, 0.0, 90.0, 0.0, -60.0, 0.0]
HOME_JOINTS_RAD = [d * math.pi / 180.0 for d in HOME_JOINTS_DEG]
DURATION = 2.0          # 点动时长 (s)
SVD_SPEED = 0.03        # 零空间维度速度 (rad/s)
FEED_PERIOD = 0.05      # 喂入周期 (s)，需 < timeout
TIMEOUT = 0.20           # 超时 (s)
SAMPLE_PERIOD = 0.02    # 采样周期 (s)
POS_TOLERANCE = 0.001   # 位置容许误差 (m) — 全链路比纯规划放宽
ROT_TOLERANCE = 0.001   # 旋转容许误差


class RocosClient:
    """ROCOS HTTP API 客户端"""

    def __init__(self, host: str = "localhost", port: int = 8080):
        self.base = f"http://{host}:{port}"

    def _post(self, path: str, body: dict) -> dict:
        url = f"{self.base}{path}"
        data = json.dumps(body).encode("utf-8")
        req = urllib.request.Request(url, data=data,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=30) as resp:
                return json.loads(resp.read().decode())
        except urllib.error.URLError as e:
            raise RuntimeError(f"HTTP error on {path}: {e}")

    def _get(self, path: str) -> dict:
        url = f"{self.base}{path}"
        try:
            with urllib.request.urlopen(url, timeout=10) as resp:
                return json.loads(resp.read().decode())
        except urllib.error.URLError as e:
            raise RuntimeError(f"HTTP error on {path}: {e}")

    def get_state(self) -> dict:
        """获取机器人状态"""
        return self._get("/api/robot/state")

    def is_enabled(self) -> bool:
        r = self._get("/api/robot/enabled")
        return r.get("data", {}).get("enabled", False)

    def enable(self) -> dict:
        return self._post("/api/robot/enable", {})

    def disable(self) -> dict:
        return self._post("/api/robot/disable", {})

    def move_joint(self, joints_rad: list, speed: float = 0.5,
                   accel: float = 1.0, asynchronous: bool = False,
                   time_ms: float = 0.0) -> dict:
        body = {
            "joints": joints_rad,
            "speed": speed,
            "acceleration": accel,
            "time": time_ms,
            "asynchronous": asynchronous,
        }
        return self._post("/api/move/joint", body)

    def get_move_status(self, task_id: str) -> dict:
        return self._get(f"/api/move/status?task_id={task_id}")

    def jog_svd(self, dim_speeds: list, timeout: float = 0.1,
                dir_threshold: float = 0.99) -> dict:
        body = {
            "dim_speeds": dim_speeds,
            "timeout": timeout,
            "dir_threshold": dir_threshold,
        }
        return self._post("/api/robot/jog/svd", body)

    def jog_stop(self) -> dict:
        return self._post("/api/robot/jog/stop", {})


def wait_move_done(client: RocosClient, timeout_s: float = 30.0):
    """等待运动完成（轮询 robot/state 的控制状态）"""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = client.get_state()
        if not state.get("success"):
            time.sleep(0.1)
            continue
        active = state.get("data", {}).get("control_active", False)
        if not active:
            return True
        time.sleep(0.1)
    return False


def extract_flange(state_resp: dict) -> dict | None:
    """从 /api/robot/state 响应中提取法兰位姿"""
    data = state_resp.get("data")
    if not data:
        return None
    flange = data.get("flange")
    if not flange:
        # 兼容旧字段名
        flange = data.get("flange_pose")
    return flange


def pose_to_tuple(flange: dict) -> tuple:
    """法兰位姿 → (x, y, z, ox, oy, oz, ow)"""
    pos = flange.get("position", {})
    ori = flange.get("orientation", {})
    return (pos.get("x", 0), pos.get("y", 0), pos.get("z", 0),
            ori.get("x", 0), ori.get("y", 0), ori.get("z", 0), ori.get("w", 0))


def position_delta(a: tuple, b: tuple) -> float:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def orientation_delta(a: tuple, b: tuple) -> float:
    """四元数角偏差 (rad)"""
    dot = abs(a[3] * b[3] + a[4] * b[4] + a[5] * b[5] + a[6] * b[6])
    dot = min(dot, 1.0)
    return 2.0 * math.acos(dot)


def run_test(host: str, port: int) -> int:
    client = RocosClient(host, port)

    print(f"连接到 rocosAppMain @ {client.base}")

    # ── 0. 确保使能 ──
    if not client.is_enabled():
        print("机器人未使能，正在使能...")
        r = client.enable()
        if not r.get("success"):
            print(f"使能失败: {r.get('message')}")
            return 1
        time.sleep(1.0)
        if not client.is_enabled():
            print("使能确认失败")
            return 1
    print("机器人已使能 ✓")

    # ── 1. MoveJ 回零 ──
    print(f"\n[1/4] MoveJ → 零位 {HOME_JOINTS_DEG}")
    r = client.move_joint(HOME_JOINTS_RAD, speed=0.5, accel=1.0,
                          asynchronous=True, time_ms=5.0)
    if not r.get("success"):
        print(f"MoveJ 失败: {r.get('message')}")
        return 1

    task_id = r.get("data", {}).get("task_id", "")
    print(f"  task_id: {task_id}")

    if not wait_move_done(client, timeout_s=30.0):
        print("MoveJ 超时")
        return 1
    print("  到位 ✓")

    # ── 2. 记录初始位姿 ──
    state = client.get_state()
    init_flange = extract_flange(state)
    if not init_flange:
        print("无法获取法兰位姿")
        return 1
    init_pose = pose_to_tuple(init_flange)
    print(f"\n[2/4] 初始法兰位姿: pos=({init_pose[0]:.6f}, {init_pose[1]:.6f}, {init_pose[2]:.6f}) "
          f"ori=({init_pose[3]:.4f}, {init_pose[4]:.4f}, {init_pose[5]:.4f}, {init_pose[6]:.4f})")

    # ── 3. SVD 零空间点动 2s ──
    print(f"\n[3/4] SVD 零空间点动 {DURATION}s, speed={SVD_SPEED} rad/s")
    samples: list[dict] = []
    t_start = time.monotonic()
    next_feed = t_start
    next_sample = t_start

    while True:
        now = time.monotonic()
        elapsed = now - t_start
        if elapsed >= DURATION:
            break

        # 周期喂入 SVD 指令
        if now >= next_feed:
            try:
                client.jog_svd([SVD_SPEED], timeout=TIMEOUT)
            except RuntimeError as e:
                print(f"  Feed 异常: {e}")
                return 1
            next_feed = now + FEED_PERIOD

        # 周期采样法兰位姿
        if now >= next_sample:
            state = client.get_state()
            flange = extract_flange(state)
            if flange:
                samples.append({"t": elapsed, "flange": flange})
            next_sample = now + SAMPLE_PERIOD

        # 小步 sleep 避免空转
        time.sleep(min(FEED_PERIOD, SAMPLE_PERIOD, next_feed - now, next_sample - now))

    # 停止点动
    client.jog_stop()
    print(f"  已停止，采样 {len(samples)} 次")

    # ── 4. 分析 ──
    print(f"\n[4/4] 位姿稳定性分析")
    final_flange = samples[-1]["flange"] if samples else init_flange
    final_pose = pose_to_tuple(final_flange)

    pos_delta = position_delta(init_pose, final_pose)
    ori_delta = orientation_delta(init_pose, final_pose)

    # 统计所有采样的波动
    pos_deltas = [position_delta(init_pose, pose_to_tuple(s["flange"])) for s in samples]
    ori_deltas = [orientation_delta(init_pose, pose_to_tuple(s["flange"])) for s in samples]
    max_pos = max(pos_deltas)
    max_ori = max(ori_deltas)
    rms_pos = math.sqrt(sum(d * d for d in pos_deltas) / len(pos_deltas))
    rms_ori = math.sqrt(sum(d * d for d in ori_deltas) / len(ori_deltas))

    pos_pass = max_pos < POS_TOLERANCE
    ori_pass = max_ori < ROT_TOLERANCE
    test_pass = pos_pass and ori_pass

    print(f"\n{'='*60}")
    print(f"  SVD 零空间点动 全流程集成测试结果")
    print(f"{'='*60}")
    print(f"  末端位姿初始: pos=({init_pose[0]:.6f}, {init_pose[1]:.6f}, {init_pose[2]:.6f})")
    print(f"                ori=({init_pose[3]:.4f}, {init_pose[4]:.4f}, {init_pose[5]:.4f}, {init_pose[6]:.4f})")
    print(f"  末端位姿终值: pos=({final_pose[0]:.6f}, {final_pose[1]:.6f}, {final_pose[2]:.6f})")
    print(f"                ori=({final_pose[3]:.4f}, {final_pose[4]:.4f}, {final_pose[5]:.4f}, {final_pose[6]:.4f})")
    print()
    print(f"  位置偏差 max={max_pos:.6f}m  rms={rms_pos:.6f}m  限值={POS_TOLERANCE}m  "
          f"[{'✓ PASS' if pos_pass else '✗ FAIL'}]")
    print(f"  姿态偏差 max={max_ori:.6f}rad rms={rms_ori:.6f}rad 限值={ROT_TOLERANCE}rad "
          f"[{'✓ PASS' if ori_pass else '✗ FAIL'}]")
    print(f"{'='*60}")

    if test_pass:
        print(f"\n  ✅ 全流程测试通过 — 零空间点动未改变末端位姿")
    else:
        print(f"\n  ❌ 全流程测试失败 — 末端位姿超出容许范围")
        print(f"\n  对比离线规划结果（偏差≈0），可能原因：")
        print(f"    1. 控制层关节跟踪误差")
        print(f"    2. 模型参数差异（URDF vs 真实 DH）")
        print(f"    3. 通信延迟导致喂入超时")

    return 0 if test_pass else 1


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="SVD 零空间点动 全流程集成测试")
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=8080)
    args = p.parse_args()
    sys.exit(run_test(args.host, args.port))
