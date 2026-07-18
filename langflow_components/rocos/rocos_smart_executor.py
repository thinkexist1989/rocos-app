"""RocosSmartExecutor Langflow component.

Semantic cache router + safe executor in one Tool.
"""

import json
import os
import re
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Optional

from langchain_core.tools import Tool
from langflow.custom import Component
from langflow.io import Output, StrInput
from langflow.schema import Data


DEFAULT_MEMORY_DIR = "/tmp/rocos_memory"
DEFAULT_MEMORY_FILE = "rocos_memory.json"
ACTION_TYPE_ALIASES = {
    "movej": "joint",
    "joint": "joint",
    "joint_move": "joint",
    "movel": "cartesian_delta",
    "cartesian": "cartesian_delta",
    "cartesian_delta": "cartesian_delta",
    "null": "nullspace",
    "nullspace": "nullspace",
    "svd": "svd",
    "svd_jog": "svd",
}


def _error(code: int, message: str) -> dict:
    return {"success": False, "code": code, "message": message, "data": None}


def _url(base_url: str, path: str, params: Optional[dict] = None) -> str:
    url = f"{base_url.rstrip('/')}{path}"
    if params:
        query = urllib.parse.urlencode({k: v for k, v in params.items() if v is not None})
        if query:
            url = f"{url}?{query}"
    return url


def _decode_json(body: str) -> dict:
    if not body:
        return {}
    return json.loads(body)


def _parse_http_error(e: urllib.error.HTTPError) -> dict:
    try:
        body = e.read().decode("utf-8")
        parsed = _decode_json(body)
        if isinstance(parsed, dict):
            parsed.setdefault("success", False)
            parsed.setdefault("code", e.code)
            parsed.setdefault("message", e.reason)
            return parsed
    except Exception:
        pass
    return _error(e.code, f"HTTP {e.code}: {e.reason}")


def _rocos_request(
    base_url: str,
    path: str,
    method: str = "GET",
    body: Optional[dict] = None,
    params: Optional[dict] = None,
    expect_json: bool = True,
    timeout: float = 30.0,
):
    url = _url(base_url, path, params)
    data = json.dumps(body).encode("utf-8") if body is not None else None
    headers = {"Accept": "application/json" if expect_json else "*/*"}
    if data is not None:
        headers["Content-Type"] = "application/json"
    try:
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            text = resp.read().decode("utf-8")
            return _decode_json(text) if expect_json else text
    except urllib.error.HTTPError as e:
        return _parse_http_error(e)
    except urllib.error.URLError as e:
        return _error(-1, f"网络错误: {e}")
    except json.JSONDecodeError as e:
        return _error(-2, f"JSON 解析错误: {e}")


def _rocos_get(base_url: str, path: str, params: Optional[dict] = None) -> dict:
    return _rocos_request(base_url, path, method="GET", params=params, timeout=10.0)


def _rocos_post(base_url: str, path: str, body: Optional[dict] = None) -> dict:
    return _rocos_request(base_url, path, method="POST", body=body or {})


def _wait_move(base_url: str) -> dict:
    return _rocos_post(base_url, "/api/robot/wait_move", {})


def _normalize_intent(text: str) -> str:
    t = re.sub(r"\s+", "", text.strip().lower())
    synonyms = {
        "向上": "往上", "向上边": "往上", "up": "往上",
        "向下": "往下", "向下边": "往下", "down": "往下",
        "向左": "往左", "left": "往左",
        "向右": "往右", "right": "往右",
        "向前": "往前", "forward": "往前",
        "向后": "往后", "back": "往后",
    }
    for old, new in synonyms.items():
        t = t.replace(old, new)
    return t


def _load_memory(memory_dir: str) -> dict:
    fpath = Path(memory_dir) / DEFAULT_MEMORY_FILE
    fpath.parent.mkdir(parents=True, exist_ok=True)
    if not fpath.exists():
        return {
            "executions": [],
            "patterns": [],
            "semantic_cache": {},
            "stats": {"total_motions": 0, "errors": 0, "recoveries": 0},
        }
    try:
        return json.loads(fpath.read_text())
    except json.JSONDecodeError:
        return {
            "executions": [],
            "patterns": [],
            "semantic_cache": {},
            "stats": {"total_motions": 0, "errors": 0, "recoveries": 0},
        }


def _save_memory(memory_dir: str, data: dict):
    fpath = Path(memory_dir) / DEFAULT_MEMORY_FILE
    fpath.parent.mkdir(parents=True, exist_ok=True)
    fpath.write_text(json.dumps(data, indent=2, ensure_ascii=False))


def _cache_query(memory_dir: str, intent: str) -> Optional[dict]:
    data = _load_memory(memory_dir)
    entry = data.get("semantic_cache", {}).get(_normalize_intent(intent))
    if not isinstance(entry, dict):
        return None
    freq = entry.get("frequency", 0)
    if freq < 3:
        return None
    return {
        "action_name": entry.get("action_name", ""),
        "action": entry.get("action", {}),
        "frequency": freq,
        "success_count": entry.get("success_count", 0),
        "failure_count": entry.get("failure_count", 0),
        "confidence": min(0.99, 0.5 + freq * 0.1),
        "cached": True,
    }


def _record_to_memory(memory_dir: str, entry: dict, intent: str = "", action_template: Optional[dict] = None):
    mem = _load_memory(memory_dir)
    entry.setdefault("timestamp", time.strftime("%Y-%m-%dT%H:%M:%S"))
    mem.setdefault("executions", []).append(entry)
    if len(mem["executions"]) > 200:
        mem["executions"] = mem["executions"][-200:]

    stats = mem.setdefault("stats", {})
    stats["total_motions"] = stats.get("total_motions", 0) + 1
    if not entry.get("success", True):
        stats["errors"] = stats.get("errors", 0) + 1

    if intent and action_template:
        key = _normalize_intent(intent)
        cache = mem.setdefault("semantic_cache", {})
        now = entry["timestamp"]
        success = entry.get("success", True)
        if key not in cache or not isinstance(cache[key], dict):
            cache[key] = {
                "action_name": "safe_executor",
                "action": action_template,
                "frequency": 0,
                "success_count": 0,
                "failure_count": 0,
                "first_seen": now,
            }
        cached = cache[key]
        cached["action"] = action_template
        cached["action_name"] = "safe_executor"
        cached["frequency"] = cached.get("frequency", 0) + 1
        cached["last_used"] = now
        if success:
            cached["success_count"] = cached.get("success_count", 0) + 1
        else:
            cached["failure_count"] = cached.get("failure_count", 0) + 1

    _save_memory(memory_dir, mem)


def _parse_tool_input(raw: str, default_action_json: str, default_intent: str):
    raw = (raw or "").strip()
    action_json = default_action_json
    intent = default_intent
    if not raw:
        return intent, action_json

    try:
        parsed = json.loads(raw)
        if isinstance(parsed, dict):
            if "__arg1" in parsed:
                return _parse_tool_input(str(parsed["__arg1"]), default_action_json, default_intent)
            intent = parsed.get("intent", intent)
            if "action_json" in parsed:
                action_json = parsed["action_json"]
            elif "action" in parsed:
                action_json = parsed["action"]
            else:
                action_json = parsed
            return intent, action_json
    except json.JSONDecodeError:
        pass

    return raw, action_json


def _normalize_action(action: dict) -> dict:
    normalized = dict(action)
    raw_type = str(normalized.get("type", "")).strip().lower()
    normalized["type"] = ACTION_TYPE_ALIASES.get(raw_type, raw_type)
    return normalized


def _pos_distance(a: dict, b: dict) -> float:
    return (
        (a.get("x", 0.0) - b.get("x", 0.0)) ** 2
        + (a.get("y", 0.0) - b.get("y", 0.0)) ** 2
        + (a.get("z", 0.0) - b.get("z", 0.0)) ** 2
    ) ** 0.5


def _max_joint_error(a: list, b: list) -> float:
    if not a or not b:
        return float("inf")
    return max(abs(a[i] - b[i]) for i in range(min(len(a), len(b))))


def _max_joint_delta(a: list, b: list) -> float:
    if not a or not b:
        return 0.0
    return max(abs(a[i] - b[i]) for i in range(min(len(a), len(b))))


def _extract_joints(state_data: dict) -> list:
    return [j["position"] for j in state_data.get("joint_states", [])]


def _verify_action(atype: str, action: dict, before_joints: list, after_joints: list,
                   after_pos: dict, target_pos: Optional[dict] = None,
                   joints_target: Optional[list] = None):
    if atype == "cartesian_delta":
        tolerance = action.get("verify_cartesian_tolerance", 0.005)
        error = _pos_distance(after_pos, target_pos or {})
        return error <= tolerance, f"cartesian_error={error:.6f}, tolerance={tolerance:.6f}"
    if atype == "joint":
        tolerance = action.get("verify_joint_tolerance", 0.005)
        error = _max_joint_error(after_joints, joints_target or [])
        return error <= tolerance, f"joint_error={error:.6f}, tolerance={tolerance:.6f}"
    if atype in ("nullspace", "svd"):
        min_delta = action.get("verify_min_joint_delta", 1e-4)
        delta = _max_joint_delta(before_joints, after_joints)
        return delta >= min_delta, f"max_joint_delta={delta:.6f}, min_delta={min_delta:.6f}"
    return True, ""


def _execute_action(base_url: str, memory_dir: str, action: dict, intent: str = "") -> Data:
    action = _normalize_action(action)
    atype = action.get("type", "")
    if atype not in ("cartesian_delta", "joint", "nullspace", "svd"):
        return Data(
            text=(
                "EXECUTED=false\n"
                "reason=UNSUPPORTED_ACTION_TYPE\n"
                f"❌ 不支持的动作类型: {atype}，支持 cartesian_delta/joint/nullspace/svd"
            ),
            data={"success": False},
        )

    speed = action.get("speed", 0.15)
    if speed > 0.5:
        return Data(
            text=f"EXECUTED=false\nreason=SPEED_LIMIT\n⛔ 速度 {speed} > 0.5，拒绝执行",
            data={"success": False, "rejected": True},
        )

    state = _rocos_get(base_url, "/api/robot/state")
    sdata = state.get("data", {})
    robot_state = sdata.get("robot_state", "UNKNOWN")
    is_enabled = sdata.get("is_enabled", False)
    if robot_state != "STOPPED" or not is_enabled:
        text = f"❌ 机器人未就绪: state={robot_state}, enabled={is_enabled}"
        if robot_state == "ERROR_STATE":
            text += "\n💡 请先调用 ResetRobotFault 复位"
        return Data(text=f"EXECUTED=false\nreason=ROBOT_NOT_READY\n{text}", data={"success": False, "state": robot_state})

    flange = sdata.get("flange", {})
    before_pos = flange.get("position", {"x": 0, "y": 0, "z": 0})
    before_ori = flange.get("orientation", {"x": 0, "y": 0, "z": 0, "w": 1})
    joints_before = [j["position"] for j in sdata.get("joint_states", [])]

    if atype == "cartesian_delta":
        delta = action.get("delta", [0, 0, 0, 0, 0, 0])
        if len(delta) < 3:
            return Data(text="EXECUTED=false\nreason=INVALID_ACTION\n❌ cartesian_delta 需要至少3个元素 [dx,dy,dz]", data={"success": False})
        total = (delta[0] ** 2 + delta[1] ** 2 + delta[2] ** 2) ** 0.5
        if total > 0.05:
            return Data(text=f"EXECUTED=false\nreason=CARTESIAN_STEP_LIMIT\n⛔ 笛卡尔位移 {total*100:.1f}cm > 5cm，拒绝", data={"success": False, "rejected": True})
        if total == 0:
            return Data(text="EXECUTED=false\nreason=ZERO_MOTION\n⛔ 零位移，拒绝", data={"success": False, "rejected": True})
        target_pos = {
            "x": before_pos["x"] + delta[0],
            "y": before_pos["y"] + delta[1],
            "z": before_pos["z"] + delta[2],
        }
        api_path = "/api/robot/movel"
        result = _rocos_post(base_url, api_path, {
            "pose": {"position": target_pos, "orientation": before_ori},
            "velocity": speed,
        })
    elif atype == "joint":
        joints_target = action.get("joints", [])
        if not joints_target or len(joints_target) < 3:
            return Data(text="EXECUTED=false\nreason=INVALID_ACTION\n❌ joint 类型需要 joints 数组", data={"success": False})
        max_step = max(abs(joints_target[i] - joints_before[i]) for i in range(min(len(joints_target), len(joints_before))))
        if max_step > 0.3:
            return Data(text=f"EXECUTED=false\nreason=JOINT_STEP_LIMIT\n⛔ 关节单步 {max_step:.2f}rad > 0.3rad，拒绝", data={"success": False, "rejected": True})
        api_path = "/api/robot/movej"
        result = _rocos_post(base_url, api_path, {
            "joints": joints_target,
            "velocity": speed,
            "acceleration": action.get("acceleration", 1.0),
        })
    elif atype == "nullspace":
        api_path = "/api/robot/jog/nullspace"
        result = _rocos_post(base_url, api_path, {
            "joints": action.get("joints_vec", action.get("joints", [])),
            "direction": action.get("direction", "POSITIVE"),
            "speed": speed,
            "timeout": action.get("timeout", 1.5),
        })
    else:
        api_path = "/api/robot/jog/svd"
        result = _rocos_post(base_url, api_path, {
            "dim_speeds": action.get("dim_speeds", action.get("speeds", [])),
            "timeout": action.get("timeout", 1.5),
            "dir_threshold": action.get("dir_threshold", 0.99),
        })

    api_ok = result.get("success", False)
    api_code = result.get("code", 0)
    final_state = "UNKNOWN"
    wait_result = {"success": False, "code": 0, "message": "not called"}
    if api_ok:
        wait_result = _wait_move(base_url)
        final_state = wait_result.get("data", {}).get("robot_state", "UNKNOWN")

    after = {}
    adata = {}
    after_pos = {}
    joints_after = []
    verify_ok = False
    verify_reason = "not checked"
    verify_timeout = action.get("verify_timeout", 2.0)
    verify_interval = action.get("verify_interval", 0.05)
    verify_started = time.monotonic()
    while True:
        after = _rocos_get(base_url, "/api/robot/state")
        adata = after.get("data", {})
        final_state = adata.get("robot_state", final_state)
        after_pos = adata.get("flange", {}).get("position", {})
        joints_after = _extract_joints(adata)
        verify_ok, verify_reason = _verify_action(
            atype, action, joints_before, joints_after, after_pos,
            target_pos if atype == "cartesian_delta" else None,
            joints_target if atype == "joint" else None,
        )
        if verify_ok or final_state == "ERROR_STATE" or (time.monotonic() - verify_started) >= verify_timeout:
            break
        time.sleep(verify_interval)

    wait_ok = (not api_ok) or wait_result.get("success", False)
    success = api_ok and wait_ok and final_state == "STOPPED" and verify_ok

    entry = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "action": atype,
        "params": action,
        "before": {
            "state": robot_state,
            "x": round(before_pos["x"], 4),
            "y": round(before_pos["y"], 4),
            "z": round(before_pos["z"], 4),
        },
        "after": {
            "state": final_state,
            "x": round(after_pos.get("x", 0), 4),
            "y": round(after_pos.get("y", 0), 4),
            "z": round(after_pos.get("z", 0), 4),
        },
        "success": success,
        "error_code": None if success else api_code,
        "api_path": api_path,
        "wait_code": wait_result.get("code", 0),
        "verify_ok": verify_ok,
        "verify_reason": verify_reason,
    }
    _record_to_memory(memory_dir, entry, intent if (intent and success) else "", action if (intent and success) else None)

    if success:
        dx = round(after_pos.get("x", 0) - before_pos["x"], 4)
        dy = round(after_pos.get("y", 0) - before_pos["y"], 4)
        dz = round(after_pos.get("z", 0) - before_pos["z"], 4)
        text = (
            "EXECUTED=true\n"
            f"api_path={api_path}\n"
            f"api_code={api_code}\n"
            f"wait_code={wait_result.get('code', 0)}\n"
            f"final_state={final_state}\n"
            f"{verify_reason}\n"
            f"✅ {atype} 执行成功\n"
            f"API: {api_path}\n"
            f"位移: Δx={dx:.4f} Δy={dy:.4f} Δz={dz:.4f}\n"
            f"状态: {robot_state} → {final_state}\n"
            f"已自动记录记忆" + (f" + 缓存学习 '{intent}'" if intent else "")
        )
    elif final_state == "ERROR_STATE":
        text = (
            "EXECUTED=false\n"
            f"api_path={api_path}\n"
            f"api_code={api_code}\n"
            f"wait_code={wait_result.get('code', 0)}\n"
            f"final_state={final_state}\n"
            f"❌ 执行失败 → ERROR_STATE (code={api_code})\n💡 请调用 ResetRobotFault 复位"
        )
    elif not wait_ok:
        text = (
            "EXECUTED=false\n"
            "reason=WAIT_MOVE_FAILED\n"
            f"api_path={api_path}\n"
            f"api_code={api_code}\n"
            f"wait_code={wait_result.get('code', 0)}\n"
            f"final_state={final_state}\n"
            f"❌ WaitMove 失败: {wait_result.get('message', '')}"
        )
    elif not verify_ok:
        text = (
            "EXECUTED=false\n"
            "reason=VERIFY_MOTION_FAILED\n"
            f"api_path={api_path}\n"
            f"api_code={api_code}\n"
            f"wait_code={wait_result.get('code', 0)}\n"
            f"final_state={final_state}\n"
            f"{verify_reason}\n"
            "❌ API 返回成功，但真实状态没有达到目标或没有检测到运动"
        )
    else:
        text = (
            "EXECUTED=false\n"
            f"api_path={api_path}\n"
            f"api_code={api_code}\n"
            f"wait_code={wait_result.get('code', 0)}\n"
            f"final_state={final_state}\n"
            f"❌ 执行失败 (code={api_code}): {result.get('message', '')}\n状态: {final_state}"
        )

    return Data(text=text, data={
        "success": success,
        "state": final_state,
        "api_code": api_code,
        "api_path": api_path,
        "wait_code": wait_result.get("code", 0),
        "before_pos": before_pos,
        "after_pos": after_pos,
        "before_joints": joints_before,
        "after_joints": joints_after,
        "verify_ok": verify_ok,
        "verify_reason": verify_reason,
        "memory_recorded": True,
    })


class RocosSmartExecutor(Component):
    """语义缓存路由 + 安全执行器。

    Agent 可以先把自然语言原样传进来；缓存命中则直接执行。
    未命中时，Agent 再传 {"intent": "...", "action_json": {...}} 执行并学习。
    """

    display_name: str = "ROCOS 智能执行器"
    description: str = (
        "先查语义缓存，命中则跳过推理直接执行；未命中时接收结构化 action_json 执行，"
        "并自动记录 Memory / 学习 SemanticCache。"
    )
    icon: str = "Zap"
    name: str = "rocos_smart_executor"

    inputs = [
        StrInput(name="base_url", display_name="API 基础地址", value="http://localhost:8080"),
        StrInput(name="intent", display_name="语义指令", value=""),
        StrInput(
            name="action_json",
            display_name="动作 (JSON, 未命中后使用)",
            value="{}",
            info='{"type":"cartesian_delta","delta":[0,0.02,0,0,0,0],"speed":0.1}',
        ),
        StrInput(name="memory_dir", display_name="记忆目录", value=DEFAULT_MEMORY_DIR, advanced=True),
    ]

    outputs = [
        Output(display_name="Tool", name="component_as_tool", method="to_tool", types=["Tool"], tool_mode=True),
    ]

    def to_tool(self) -> Tool:
        base_url = self.base_url
        memory_dir = self.memory_dir or DEFAULT_MEMORY_DIR
        default_intent = self.intent
        default_action_json = self.action_json

        def _run(input_str: str = "") -> str:
            intent, action_json = _parse_tool_input(input_str, default_action_json, default_intent)
            hit = _cache_query(memory_dir, intent) if intent else None
            if hit:
                result = _execute_action(base_url, memory_dir, hit.get("action", {}), intent)
                return (
                    f"CACHE HIT: '{intent}' "
                    f"(conf={hit.get('confidence', 0):.0%}, freq={hit.get('frequency', 0)})\n"
                    f"{result.text}"
                )

            if isinstance(action_json, str):
                try:
                    action = json.loads(action_json)
                except json.JSONDecodeError:
                    return (
                        "EXECUTED=false\n"
                        "reason=CACHE_MISS_NEEDS_ACTION_JSON\n"
                        f"CACHE MISS: '{intent}'\n"
                        "需要 Agent 生成 action_json 后再次调用本工具。"
                    )
            elif isinstance(action_json, dict):
                action = action_json
            else:
                return f"EXECUTED=false\nreason=INVALID_ACTION_JSON\nCACHE MISS: '{intent}'\n动作参数类型错误，需要 JSON 对象。"

            if not action or not action.get("type"):
                return (
                    "EXECUTED=false\n"
                    "reason=CACHE_MISS_NEEDS_ACTION_JSON\n"
                    f"CACHE MISS: '{intent}'\n"
                    "请根据用户意图生成安全动作 JSON 后再次调用，例如 "
                    '{"intent":"往上一点","action_json":{"type":"cartesian_delta","delta":[0,0,0.02,0,0,0],"speed":0.1}}'
                )

            result = _execute_action(base_url, memory_dir, action, intent)
            return f"CACHE MISS -> SAFE EXECUTE\n{result.text}"

        return Tool(
            name="RocosSmartExecutor",
            description=(
                "ROCOS 智能执行入口。输入自然语言会先查缓存，命中直接执行；"
                "未命中后请再次传 JSON: "
                '{"intent":"往上一点","action_json":{"type":"cartesian_delta","delta":[0,0,0.02,0,0,0],"speed":0.1}}。'
                "支持 type=cartesian_delta/joint/nullspace/svd。"
            ),
            func=_run,
        )

    def execute(self) -> Data:
        intent, action_json = _parse_tool_input("", self.action_json, self.intent)
        hit = _cache_query(self.memory_dir, intent) if intent else None
        if hit:
            return _execute_action(self.base_url, self.memory_dir, hit.get("action", {}), intent)
        try:
            action = json.loads(action_json) if isinstance(action_json, str) else action_json
        except json.JSONDecodeError:
            return Data(text=f"CACHE MISS: '{intent}'", data={"cached": False, "intent": intent})
        return _execute_action(self.base_url, self.memory_dir, action, intent)
