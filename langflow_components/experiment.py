#!/usr/bin/env python3
"""ROCOS 自主试验积累 — 系统化探索机器人能力，积累经验"""
import json, urllib.request, time, os, sys

BASE = "http://localhost:8080"
MEMORY_DIR = "/tmp/rocos_memory"

def get(path, params=None):
    url = f"{BASE}{path}"
    if params:
        url += "?" + "&".join(f"{k}={v}" for k, v in params.items() if v)
    req = urllib.request.Request(url, headers={"Accept": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read())

def post(path, body):
    data = json.dumps(body).encode()
    req = urllib.request.Request(f"{BASE}{path}", data=data,
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read()) if r.status == 200 else {}

def read_state():
    d = get("/api/robot/state")["data"]
    return {
        "state": d["robot_state"], "enabled": d["is_enabled"],
        "x": round(d["flange"]["position"]["x"], 4),
        "y": round(d["flange"]["position"]["y"], 4),
        "z": round(d["flange"]["position"]["z"], 4),
        "ox": round(d["flange"]["orientation"]["x"], 4),
        "oy": round(d["flange"]["orientation"]["y"], 4),
        "oz": round(d["flange"]["orientation"]["z"], 4),
        "ow": round(d["flange"]["orientation"]["w"], 4),
        "joints": [round(j["position"], 4) for j in d["joint_states"]],
    }

def record(action, params, summary, success):
    """写入记忆"""
    import hashlib
    mem = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "action": action, "params": params, "summary": summary,
        "success": success,
    }
    fpath = os.path.join(MEMORY_DIR, "rocos_memory.json")
    os.makedirs(MEMORY_DIR, exist_ok=True)
    if os.path.exists(fpath):
        with open(fpath) as f:
            data = json.load(f)
    else:
        data = {"executions": [], "patterns": [], "stats": {"total_motions": 0, "errors": 0, "recoveries": 0}}
    data["executions"].append(mem)
    data["executions"] = data["executions"][-200:]
    data["stats"]["total_motions"] = data["stats"].get("total_motions", 0) + 1
    if not success:
        data["stats"]["errors"] = data["stats"].get("errors", 0) + 1
    with open(fpath, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

def recover():
    """错误恢复"""
    s = read_state()
    if s["state"] != "ERROR_STATE":
        return True
    print("  ⚡ 恢复中...")
    post("/api/robot/disable", {})
    time.sleep(0.5)
    post("/api/robot/enable", {})
    time.sleep(0.5)
    s2 = read_state()
    return s2["state"] != "ERROR_STATE"

def safe_movel(dx, dy, dz, vel=0.15):
    """安全 MoveL: 读状态 → 执行 → 记录"""
    s = read_state()
    if s["state"] != "STOPPED" or not s["enabled"]:
        if s["state"] == "ERROR_STATE":
            recover()
            s = read_state()
        if s["state"] != "STOPPED":
            return False

    target = {"x": s["x"] + dx, "y": s["y"] + dy, "z": s["z"] + dz}
    ori = {"x": s["ox"], "y": s["oy"], "z": s["oz"], "w": s["ow"]}
    total = (dx**2 + dy**2 + dz**2)**0.5

    r = post("/api/robot/movel", {
        "pose": {"position": target, "orientation": ori},
        "velocity": vel,
    })
    ok = r.get("success", False)
    summary = f"Δ({dx:+.3f},{dy:+.3f},{dz:+.3f})m vel={vel} → {'OK' if ok else 'FAIL'}"
    record("MoveL", {"dx": dx, "dy": dy, "dz": dz, "vel": vel, "from": (s["x"], s["y"], s["z"])}, summary, ok)
    return ok

def safe_movej(joints, vel=0.3, acc=1.0):
    """安全 MoveJ"""
    s = read_state()
    if s["state"] != "STOPPED" or not s["enabled"]:
        if s["state"] == "ERROR_STATE":
            recover()
        if s["state"] != "STOPPED":
            return False
    r = post("/api/robot/movej", {"joints": joints, "velocity": vel, "acceleration": acc})
    ok = r.get("success", False)
    summary = f"MoveJ {[round(j,3) for j in joints[:3]]}... vel={vel} → {'OK' if ok else 'FAIL'}"
    record("MoveJ", {"joints": joints, "vel": vel, "acc": acc}, summary, ok)
    return ok

def wait_stop(timeout=5):
    """等待运动完成"""
    for _ in range(timeout * 2):
        s = read_state()
        if s["state"] == "STOPPED":
            return True
        if s["state"] == "ERROR_STATE":
            recover()
        time.sleep(0.5)
    return False

def nullspace_jog(direction_vec, speed=0.3, timeout_s=1.5):
    """零空间点动"""
    s = read_state()
    if s["state"] != "STOPPED" or not s["enabled"]:
        return False
    r = post("/api/robot/jog/nullspace", {
        "joints": direction_vec, "direction": "POSITIVE",
        "speed": speed, "timeout": timeout_s,
    })
    ok = r.get("success", False)
    record("Nullspace", {"vec": direction_vec, "speed": speed, "timeout": timeout_s},
           f"Nullspace {direction_vec[:3]}... timeout={timeout_s}s → {'OK' if ok else 'FAIL'}", ok)
    return ok

# ====== 实验主循环 ======
experiments = [
    # 阶段1: MoveL 六方向探索 (小幅度)
    ("MoveL 上", lambda: (safe_movel(0, 0, 0.03), wait_stop(3))),
    ("MoveL 下", lambda: (safe_movel(0, 0, -0.03), wait_stop(3))),
    ("MoveL 左", lambda: (safe_movel(0, -0.03, 0), wait_stop(3))),
    ("MoveL 右", lambda: (safe_movel(0, 0.03, 0), wait_stop(3))),
    ("MoveL 前", lambda: (safe_movel(0.03, 0, 0), wait_stop(3))),
    ("MoveL 后", lambda: (safe_movel(-0.03, 0, 0), wait_stop(3))),

    # 阶段2: MoveL 速度探索 (同方向不同速度)
    ("MoveL 慢速0.08", lambda: (safe_movel(0, 0.02, 0, vel=0.08), wait_stop(3))),
    ("MoveL 中速0.20", lambda: (safe_movel(0, -0.02, 0, vel=0.20), wait_stop(3))),
    ("MoveL 快速0.35", lambda: (safe_movel(0, 0.02, 0, vel=0.35), wait_stop(3))),

    # 阶段3: MoveJ 关节空间探索
    ("MoveJ 小关节运动", lambda: (safe_movej([0.0, 0.9, 0.0, 1.5, 0.0, -1.0, 0.0], vel=0.3), wait_stop(3))),
    ("MoveJ 回原位", lambda: (safe_movej([0.0, 1.047, 0.0, 1.571, 0.0, -1.047, 0.0], vel=0.4), wait_stop(3))),

    # 阶段4: 零空间探索
    ("Nullspace J1+J3", lambda: (nullspace_jog([1, 0, -1, 0, 0, 0, 0], speed=0.3, timeout_s=1.5), wait_stop(3))),
    ("Nullspace J4+J6", lambda: (nullspace_jog([0, 0, 0, 1, 0, -1, 0], speed=0.25, timeout_s=1.5), wait_stop(3))),
    ("Nullspace 全轴微动", lambda: (nullspace_jog([0.5, 0.3, -0.5, 0.3, -0.3, 0.3, -0.5], speed=0.2, timeout_s=1.0), wait_stop(3))),

    # 阶段5: 连续微调模式
    ("MoveL 微调上", lambda: (safe_movel(0, 0, 0.02, vel=0.1), wait_stop(2))),
    ("MoveL 微调右", lambda: (safe_movel(0, 0.02, 0, vel=0.1), wait_stop(2))),
    ("MoveL 微调前", lambda: (safe_movel(0.02, 0, 0, vel=0.1), wait_stop(2))),
    ("MoveL 微调下", lambda: (safe_movel(0, 0, -0.02, vel=0.1), wait_stop(2))),
]

print(f"{'='*60}")
print(f"ROCOS 自主试验 — 共 {len(experiments)} 步")
print(f"{'='*60}")

passed = 0
failed = 0

for i, (name, fn) in enumerate(experiments):
    s = read_state()
    print(f"\n[{i+1}/{len(experiments)}] {name}")
    print(f"  状态: {s['state']} 法兰: ({s['x']:.3f},{s['y']:.3f},{s['z']:.3f})")
    try:
        ok = fn()
        if ok:
            passed += 1
            print(f"  ✅ 完成")
        else:
            failed += 1
            print(f"  ❌ 失败")
    except Exception as e:
        failed += 1
        print(f"  💥 异常: {e}")
        record("Error", {"exp": name}, str(e)[:100], False)
        recover()

print(f"\n{'='*60}")
print(f"试验完成: {passed} 通过, {failed} 失败")
print(f"记忆存储: {MEMORY_DIR}/rocos_memory.json")

# 自省
import os
if os.path.exists(os.path.join(MEMORY_DIR, "rocos_memory.json")):
    with open(os.path.join(MEMORY_DIR, "rocos_memory.json")) as f:
        data = json.load(f)
    n = len(data["executions"])
    errs = sum(1 for e in data["executions"] if not e.get("success", True))
    print(f"累计记忆: {n} 条, {errs} 次错误")
print(f"{'='*60}")
