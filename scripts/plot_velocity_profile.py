#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UnitVelocityProfile 曲线可视化脚本

从单元测试生成的 CSV 文件中读取数据，绘制：
  - 场景 1：正常执行    (normal_profile.csv)
  - 场景 2：暂停与继续  (pause_resume_profile.csv)
  - 场景 3：直接停止    (stop_profile.csv)

每个场景包含三个子图：位置、速度、加速度。
另外生成一张总对比图，将三种场景的速度曲线叠在一起。
"""

import os
import sys
import matplotlib.pyplot as plt
import numpy as np
import csv

# CSV data directory (must match kCsvDir in test code)
CSV_DIR = "/tmp/velocity_profile_csv/"

# 输出图片目录
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output", "velocity_profile")
os.makedirs(OUTPUT_DIR, exist_ok=True)


def load_csv(filename: str) -> dict:
    """读取 CSV 文件，返回各列数据字典。"""
    filepath = os.path.join(CSV_DIR, filename)
    if not os.path.exists(filepath):
        print(f"[ERROR] 文件不存在: {filepath}")
        print(f"  请先运行单元测试: ./build/bin/unit_velocity_profile_test")
        sys.exit(1)

    steps, times, positions, velocities, accelerations = [], [], [], [], []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            steps.append(int(row['step']))
            times.append(float(row['time']))
            positions.append(float(row['position']))
            velocities.append(float(row['velocity']))
            accelerations.append(float(row['acceleration']))

    return {
        'step': np.array(steps),
        'time': np.array(times),
        'position': np.array(positions),
        'velocity': np.array(velocities),
        'acceleration': np.array(accelerations),
    }


def plot_single_scenario(data: dict, title: str, filename: str):
    """绘制单个场景的三子图（位置 / 速度 / 加速度）。"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(title, fontsize=16, fontweight='bold')

    t = data['time']

    # 位置
    axes[0].plot(t, data['position'], 'b-', linewidth=1.2)
    axes[0].set_ylabel('Position')
    axes[0].set_ylim(-0.05, 1.15)
    axes[0].axhline(y=1.0, color='gray', linestyle='--', alpha=0.5, label='Target (s=1.0)')
    axes[0].legend(loc='lower right')
    axes[0].grid(True, alpha=0.3)

    # 速度
    axes[1].plot(t, data['velocity'], 'r-', linewidth=1.2)
    axes[1].set_ylabel('Velocity')
    axes[1].axhline(y=0.0, color='gray', linestyle='--', alpha=0.5)
    axes[1].grid(True, alpha=0.3)

    # 加速度
    axes[2].plot(t, data['acceleration'], 'g-', linewidth=1.2)
    axes[2].set_ylabel('Acceleration')
    axes[2].set_xlabel('Time [s]')
    axes[2].axhline(y=0.0, color='gray', linestyle='--', alpha=0.5)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = os.path.join(OUTPUT_DIR, filename)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out_path}")


def plot_comparison(normal: dict, pause_resume: dict, stop: dict):
    """生成对比图：三条速度曲线叠加，三条位置曲线叠加。"""
    fig, axes = plt.subplots(2, 1, figsize=(14, 9))
    fig.suptitle('UnitVelocityProfile — Scenario Comparison', fontsize=16, fontweight='bold')

    # 位置对比
    axes[0].plot(normal['time'], normal['position'], 'b-', linewidth=1.5, alpha=0.8, label='Normal')
    axes[0].plot(pause_resume['time'], pause_resume['position'], 'r-', linewidth=1.5, alpha=0.8, label='Pause & Resume')
    axes[0].plot(stop['time'], stop['position'], 'g-', linewidth=1.5, alpha=0.8, label='Stop')
    axes[0].axhline(y=1.0, color='gray', linestyle='--', alpha=0.4)
    axes[0].set_ylabel('Position')
    axes[0].set_ylim(-0.05, 1.15)
    axes[0].legend(loc='lower right')
    axes[0].grid(True, alpha=0.3)

    # 速度对比
    axes[1].plot(normal['time'], normal['velocity'], 'b-', linewidth=1.5, alpha=0.8, label='Normal')
    axes[1].plot(pause_resume['time'], pause_resume['velocity'], 'r-', linewidth=1.5, alpha=0.8, label='Pause & Resume')
    axes[1].plot(stop['time'], stop['velocity'], 'g-', linewidth=1.5, alpha=0.8, label='Stop')
    axes[1].axhline(y=0.0, color='gray', linestyle='--', alpha=0.4)
    axes[1].set_ylabel('Velocity')
    axes[1].set_xlabel('Time [s]')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = os.path.join(OUTPUT_DIR, "comparison_overview.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[OK]  Saved: {out_path}")


def main():
    print("=" * 60)
    print("  UnitVelocityProfile — Curve Visualization")
    print("=" * 60)
    print(f"  CSV  dir: {CSV_DIR}")
    print(f"  Output  : {OUTPUT_DIR}")
    print()

    # 加载三个 CSV
    print("[..] Loading CSV data...")
    normal       = load_csv("normal_profile.csv")
    pause_resume = load_csv("pause_resume_profile.csv")
    stop         = load_csv("stop_profile.csv")
    print(f"      Normal:       {len(normal['step']):5d} data points, duration {normal['time'][-1]:.3f}s")
    print(f"      Pause&Resume:  {len(pause_resume['step']):5d} data points, duration {pause_resume['time'][-1]:.3f}s")
    print(f"      Stop:          {len(stop['step']):5d} data points, final pos={stop['position'][-1]:.4f}")
    print()

    # 绘制单场景图
    print("[..] Plotting single scenarios...")
    plot_single_scenario(normal,       "Scenario 1: Normal Execution",          "01_normal_profile.png")
    plot_single_scenario(pause_resume, "Scenario 2: Pause & Resume",           "02_pause_resume_profile.png")
    plot_single_scenario(stop,         "Scenario 3: Direct Stop",              "03_stop_profile.png")

    # 绘制总对比图
    print("[..] Plotting comparison overview...")
    plot_comparison(normal, pause_resume, stop)

    print()
    print("=" * 60)
    print("  Done! See:", OUTPUT_DIR)
    print("=" * 60)


if __name__ == '__main__':
    main()
