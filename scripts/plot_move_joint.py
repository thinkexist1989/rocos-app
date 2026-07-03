#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoveJoint 关节轨迹可视化 + 轨迹一致性验证

验证方案：
  核心原理：q[i] = q_start[i] + s * delta[i]，s ∈ [0,1]
  只要 s 值相同，关节角就相同。暂停只是暂停了 s 的推进，不影响空间路径。

验证手段：
  1. 时域叠加图 — Normal 与 Pause&Resume 的 q(t) 叠加，确认暂停前后路径一致
  2. 相空间验证 — q vs s 散点图，两场景应完全重合于同一条直线
  3. 定量残差 — 对齐 s 后计算 |q_pr(s) - q_norm(s)|，应 ≈ 0
"""

import os
import sys
import matplotlib.pyplot as plt
import numpy as np
import csv
from scipy.interpolate import interp1d

CSV_DIR = "/tmp/move_joint_csv/"
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output", "move_joint")
os.makedirs(OUTPUT_DIR, exist_ok=True)

SCENARIOS = {
    "normal":        ("Normal",        "blue"),
    "pause_resume":  ("Pause&Resume",  "red"),
    "pause_stop":    ("Pause&Stop",    "green"),
}

JOINT_KEYS  = ['q0', 'q1', 'q2']
JOINT_NAMES = ['Joint 0', 'Joint 1', 'Joint 2']
JOINT_COLORS = ['#2196F3', '#FF5722', '#4CAF50']


def load_csv(filename: str) -> dict:
    filepath = os.path.join(CSV_DIR, filename)
    if not os.path.exists(filepath):
        print(f"[ERROR] File not found: {filepath}")
        print(f"  Run: ./build/bin/move_joint_test")
        sys.exit(1)

    steps, times, q0, q1, q2 = [], [], [], [], []
    with open(filepath, 'r') as f:
        for row in csv.DictReader(f):
            steps.append(int(row['step']))
            times.append(float(row['time']))
            q0.append(float(row['q0']))
            q1.append(float(row['q1']))
            q2.append(float(row['q2']))
    return {
        'step':  np.array(steps),
        'time':  np.array(times),
        'q0':    np.array(q0),
        'q1':    np.array(q1),
        'q2':    np.array(q2),
    }


def reconstruct_s(data: dict, q_start: list, delta_max_idx: int) -> np.ndarray:
    """从最大位移关节反推归一化进度 s ∈ [0,1]"""
    key = JOINT_KEYS[delta_max_idx]
    delta = data[key][-1] - data[key][0]  # total displacement (signed)
    if abs(delta) < 1e-12:
        return np.zeros_like(data['time'])
    return (data[key] - q_start[delta_max_idx]) / delta


# ======================================================================
# 验证图 1：时域叠加 — q(t) 曲线对比
# ======================================================================

def plot_time_overlay(direction: str, label: str, q_start: list, q_goal: list):
    """Normal + Pause&Resume + Pause&Stop 三条 q(t) 曲线叠加"""
    normal       = load_csv(f"{direction}_normal.csv")
    pause_resume = load_csv(f"{direction}_pause_resume.csv")
    pause_stop   = load_csv(f"{direction}_pause_stop.csv")

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))
    fig.suptitle(f'Verification 1: Time-Domain Overlay — {label}',
                 fontsize=14, fontweight='bold')

    for j, (key, name) in enumerate(zip(JOINT_KEYS, JOINT_NAMES)):
        ax = axes[j]

        # Normal — 基准实线
        ax.plot(normal['time'], normal[key], '-',
                color='blue', linewidth=2.0, alpha=0.9, label='Normal (ground truth)')

        # Pause&Resume — 虚线叠加
        ax.plot(pause_resume['time'], pause_resume[key], '--',
                color='red', linewidth=1.5, alpha=0.8, label='Pause & Resume')

        # Pause&Stop — 点线
        ax.plot(pause_stop['time'], pause_stop[key], ':',
                color='green', linewidth=1.5, alpha=0.8, label='Pause & Stop')

        ax.axhline(y=q_goal[j], color='gray', linestyle='--', alpha=0.3)
        ax.set_title(name)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Position')
        ax.legend(fontsize=7, loc='lower right')
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, f"verify1_time_overlay_{direction}.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out}")


# ======================================================================
# 验证图 2：相空间 q vs s — 空间路径是否重合
# ======================================================================

def plot_phase_space(direction: str, label: str, q_start: list, q_goal: list):
    """q vs s 散点图：Normal 实线, Pause&Resume 虚线，应完全重合"""
    normal       = load_csv(f"{direction}_normal.csv")
    pause_resume = load_csv(f"{direction}_pause_resume.csv")

    # 用位移最大的 joint 2 反推 s
    s_norm = reconstruct_s(normal, q_start, delta_max_idx=2)
    s_pr   = reconstruct_s(pause_resume, q_start, delta_max_idx=2)

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))
    fig.suptitle(f'Verification 2: Phase Space q vs s — {label}',
                 fontsize=14, fontweight='bold')

    for j, (key, name) in enumerate(zip(JOINT_KEYS, JOINT_NAMES)):
        ax = axes[j]

        # Normal — 粗实线（连续推进）
        ax.plot(s_norm, normal[key], '-', color='blue', linewidth=2.5,
                alpha=0.7, label='Normal')

        # Pause&Resume — 虚线叠加（在同一空间路径上反复）
        ax.plot(s_pr, pause_resume[key], '--', color='red', linewidth=1.2,
                alpha=0.9, label='Pause & Resume')

        # 理论直线: q = q_start + s * (q_goal - q_start)
        s_line = np.linspace(0, 1, 100)
        q_line = q_start[j] + s_line * (q_goal[j] - q_start[j])
        ax.plot(s_line, q_line, ':', color='gray', linewidth=0.8, alpha=0.5,
                label='theory')

        ax.set_title(f'{name}\nq = {q_start[j]:.0f} + s*({q_goal[j]-q_start[j]:.0f})')
        ax.set_xlabel('s (normalized progress)')
        ax.set_ylabel('Position')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, f"verify2_phase_space_{direction}.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out}")


# ======================================================================
# 验证图 3：定量残差 — |q_pr(s) - q_norm(s)|
# ======================================================================

def plot_residual(direction: str, label: str, q_start: list, q_goal: list):
    """对齐 s 后计算 Pause&Resume 与 Normal 的关节角偏差"""
    normal       = load_csv(f"{direction}_normal.csv")
    pause_resume = load_csv(f"{direction}_pause_resume.csv")

    s_norm = reconstruct_s(normal, q_start, delta_max_idx=2)
    s_pr   = reconstruct_s(pause_resume, q_start, delta_max_idx=2)

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    fig.suptitle(f'Verification 3: Residual |q_pr - q_norm| vs s — {label}',
                 fontsize=14, fontweight='bold')

    for j, (key, name) in enumerate(zip(JOINT_KEYS, JOINT_NAMES)):
        ax = axes[j]

        # 以 Normal 为基准插值：f_norm(s) → q_norm
        # 去除 s 重复值（暂停期间 s 不变）
        mask = np.diff(s_norm, prepend=-999) != 0
        s_uniq  = s_norm[mask]
        q_uniq  = normal[key][mask]

        if len(s_uniq) < 2:
            ax.text(0.5, 0.5, 'insufficient data', transform=ax.transAxes, ha='center')
            continue

        f_interp = interp1d(s_uniq, q_uniq, kind='linear',
                            bounds_error=False, fill_value='extrapolate')

        # 对每个 Pause&Resume 点，计算偏差
        residual = np.abs(pause_resume[key] - f_interp(s_pr))

        max_err = np.max(residual)
        ax.plot(s_pr, residual, '-', color='red', linewidth=0.8, alpha=0.8)
        ax.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
        ax.set_title(f'{name}  (max err = {max_err:.2e})')
        ax.set_xlabel('s')
        ax.set_ylabel('|q_diff|')
        ax.grid(True, alpha=0.3)
        if max_err > 1e-15:
            ax.set_yscale('log')

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, f"verify3_residual_{direction}.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out}")


# ======================================================================
# 验证图 4：Pause&Stop 轨迹截断验证
# ======================================================================

def plot_stop_overlay(direction: str, label: str, q_start: list, q_goal: list):
    """验证 Stop 后的轨迹是 Normal 的精确截断"""
    normal     = load_csv(f"{direction}_normal.csv")
    pause_stop = load_csv(f"{direction}_pause_stop.csv")

    s_norm = reconstruct_s(normal, q_start, delta_max_idx=2)
    s_stop = reconstruct_s(pause_stop, q_start, delta_max_idx=2)
    s_stop_final = s_stop[-1]

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))
    fig.suptitle(f'Verification 4: Stop Truncation — {label}\n'
                 f'Stop final s = {s_stop_final:.4f}',
                 fontsize=14, fontweight='bold')

    for j, (key, name) in enumerate(zip(JOINT_KEYS, JOINT_NAMES)):
        ax = axes[j]

        # Normal 全轨迹（浅色）
        ax.plot(normal['time'], normal[key], '-',
                color='blue', linewidth=1.0, alpha=0.35, label='Normal (full)')

        # Normal 截断到相同 s
        cutoff = np.searchsorted(s_norm, s_stop_final)
        ax.plot(normal['time'][:cutoff], normal[key][:cutoff], '-',
                color='blue', linewidth=2.0, alpha=0.9, label='Normal (truncated)')

        # Pause&Stop
        ax.plot(pause_stop['time'], pause_stop[key], '--',
                color='green', linewidth=1.5, alpha=0.9, label='Pause & Stop')

        # 标记 Stop 位置
        ax.axvline(x=pause_stop['time'][-1], color='red', linestyle=':',
                   alpha=0.6, label=f'stop @ t={pause_stop["time"][-1]:.3f}s')

        ax.set_title(name)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Position')
        ax.legend(fontsize=7, loc='lower right')
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, f"verify4_stop_truncation_{direction}.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out}")


# ======================================================================
# 原有：单场景细节图
# ======================================================================

def plot_single_scenario(data: dict, title: str, filename: str, targets: list):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5), sharex=True)
    fig.suptitle(title, fontsize=14, fontweight='bold')

    t = data['time']
    for i, (key, name) in enumerate(zip(JOINT_KEYS, JOINT_NAMES)):
        axes[i].plot(t, data[key], color=JOINT_COLORS[i], linewidth=1.2)
        axes[i].axhline(y=targets[i], color='gray', linestyle='--', alpha=0.4)
        axes[i].set_title(name)
        axes[i].set_xlabel('Time [s]')
        axes[i].set_ylabel('Position')
        axes[i].grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, filename)
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out}")


def plot_direction_overview(direction: str, label: str, target: list):
    fig, axes = plt.subplots(3, 3, figsize=(18, 12), sharex=True)
    fig.suptitle(f'MoveJoint — {label} Scenario Comparison', fontsize=16, fontweight='bold')

    for col, (sc_key, (sc_label, sc_color)) in enumerate(SCENARIOS.items()):
        data = load_csv(f"{direction}_{sc_key}.csv")
        t = data['time']
        for row in range(3):
            ax = axes[row][col]
            ax.plot(t, data[JOINT_KEYS[row]], color=sc_color, linewidth=1.2)
            ax.axhline(y=target[row], color='gray', linestyle='--', alpha=0.4)
            ax.set_ylabel(JOINT_NAMES[row])
            ax.grid(True, alpha=0.3)
            if row == 0:
                ax.set_title(sc_label, fontsize=12)
            if row == 2:
                ax.set_xlabel('Time [s]')

    plt.tight_layout()
    out = os.path.join(OUTPUT_DIR, f"{direction}_overview.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out}")


# ======================================================================
# main
# ======================================================================

def main():
    print("=" * 65)
    print("  MoveJoint — Trajectory Verification & Visualization")
    print("=" * 65)
    print(f"  CSV dir: {CSV_DIR}")
    print(f"  Output : {OUTPUT_DIR}")
    print()

    # ─── 正向 [0,1,2] → [1,2,4] ───
    fwd_start, fwd_goal = [0, 1, 2], [1, 2, 4]
    print("[..] Forward: [0,1,2] -> [1,2,4]")

    for sc_key, (sc_label, _) in SCENARIOS.items():
        data = load_csv(f"forward_{sc_key}.csv")
        print(f"      {sc_label:15s}: {len(data['step']):5d} pts, "
              f"final=[{data['q0'][-1]:.4f}, {data['q1'][-1]:.4f}, {data['q2'][-1]:.4f}]")
        plot_single_scenario(data,
            f"Forward — {sc_label}  [0,1,2] -> [1,2,4]",
            f"forward_{sc_key}.png", fwd_goal)

    plot_direction_overview("forward", "Forward  [0,1,2]->[1,2,4]", fwd_goal)

    print()
    print("[验证] Forward verification plots...")
    plot_time_overlay("forward",   "Forward  [0,1,2]→[1,2,4]", fwd_start, fwd_goal)
    plot_phase_space("forward",    "Forward  [0,1,2]→[1,2,4]", fwd_start, fwd_goal)
    plot_residual("forward",       "Forward  [0,1,2]→[1,2,4]", fwd_start, fwd_goal)
    plot_stop_overlay("forward",   "Forward  [0,1,2]→[1,2,4]", fwd_start, fwd_goal)

    # ─── 反向 [1,2,4] → [0,1,2] ───
    rev_start, rev_goal = [1, 2, 4], [0, 1, 2]
    print()
    print("[..] Reverse: [1,2,4] -> [0,1,2]")

    for sc_key, (sc_label, _) in SCENARIOS.items():
        data = load_csv(f"reverse_{sc_key}.csv")
        print(f"      {sc_label:15s}: {len(data['step']):5d} pts, "
              f"final=[{data['q0'][-1]:.4f}, {data['q1'][-1]:.4f}, {data['q2'][-1]:.4f}]")
        plot_single_scenario(data,
            f"Reverse — {sc_label}  [1,2,4] -> [0,1,2]",
            f"reverse_{sc_key}.png", rev_goal)

    plot_direction_overview("reverse", "Reverse  [1,2,4]->[0,1,2]", rev_goal)

    print()
    print("[验证] Reverse verification plots...")
    plot_time_overlay("reverse",   "Reverse  [1,2,4]→[0,1,2]", rev_start, rev_goal)
    plot_phase_space("reverse",    "Reverse  [1,2,4]→[0,1,2]", rev_start, rev_goal)
    plot_residual("reverse",       "Reverse  [1,2,4]→[0,1,2]", rev_start, rev_goal)
    plot_stop_overlay("reverse",   "Reverse  [1,2,4]→[0,1,2]", rev_start, rev_goal)

    print()
    print("=" * 65)
    print("  Done! See:", OUTPUT_DIR)
    print("=" * 65)


if __name__ == '__main__':
    main()
