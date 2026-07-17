"""
ROCOS 自进化记忆系统 — 越用越聪明

三层闭环:
  1. RocosMemory      — 记录每次动作 (执行记忆)
  2. RocosReflect     — 分析历史提取模式 (自省)
  3. RocosKnowledge   — 累积可查询的经验规则 (知识库)

存储后端: 本地 JSON 文件 (零外部依赖)
路径: {memory_dir}/rocos_memory.json  (默认 /tmp/rocos_memory/rocos_memory.json)
"""

import json
import os
import time
import urllib.request
import urllib.error
from pathlib import Path
from typing import Optional
from langflow.custom import Component
from langflow.io import Output, StrInput, FloatInput
from langflow.schema import Data

DEFAULT_MEMORY_DIR = "/tmp/rocos_memory"
DEFAULT_MEMORY_FILE = "rocos_memory.json"


# ---- 内存存储引擎 ----

class MemoryStore:
    """本地 JSON 文件存储，零依赖"""

    def __init__(self, filepath: str):
        self.filepath = filepath
        Path(os.path.dirname(filepath)).mkdir(parents=True, exist_ok=True)
        if not os.path.exists(filepath):
            self._save({"executions": [], "patterns": [], "stats": {
                "total_motions": 0, "errors": 0, "recoveries": 0,
                "total_distance": 0.0, "favored_positions": [],
            }})

    def _load(self) -> dict:
        try:
            with open(self.filepath) as f:
                return json.load(f)
        except (json.JSONDecodeError, FileNotFoundError):
            return {"executions": [], "patterns": [], "stats": {}}

    def _save(self, data: dict):
        with open(self.filepath, 'w') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    def record(self, entry: dict):
        data = self._load()
        data["executions"].append(entry)
        # 只保留最近 200 条
        if len(data["executions"]) > 200:
            data["executions"] = data["executions"][-200:]
        # 更新统计
        stats = data.setdefault("stats", {})
        stats["total_motions"] = stats.get("total_motions", 0) + 1
        if not entry.get("success", True):
            stats["errors"] = stats.get("errors", 0) + 1
        self._save(data)

    def record_recovery(self):
        data = self._load()
        stats = data.setdefault("stats", {})
        stats["recoveries"] = stats.get("recoveries", 0) + 1
        self._save(data)

    def get_recent(self, n: int = 10) -> list:
        data = self._load()
        return data["executions"][-n:]

    def get_stats(self) -> dict:
        return self._load().get("stats", {})

    def add_pattern(self, pattern: dict):
        data = self._load()
        # 去重: 如果已存在相似 pattern 则更新置信度
        for p in data["patterns"]:
            if p.get("name") == pattern.get("name"):
                p["confidence"] = min(1.0, p.get("confidence", 0.5) + 0.15)
                p["count"] = p.get("count", 1) + 1
                self._save(data)
                return
        pattern["count"] = 1
        data["patterns"].append(pattern)
        self._save(data)

    def get_patterns(self) -> list:
        return self._load().get("patterns", [])

    def get_knowledge_card(self) -> str:
        """生成一张经验卡片 — 给 Agent 做上下文"""
        data = self._load()
        stats = data.get("stats", {})
        patterns = data.get("patterns", [])
        recent = data["executions"][-5:]

        card = "## ROCOS 经验累积\n\n"
        card += f"总动作: {stats.get('total_motions', 0)}  |  "
        card += f"错误: {stats.get('errors', 0)}  |  "
        card += f"恢复: {stats.get('recoveries', 0)}\n\n"

        if patterns:
            card += "### 学到的模式\n"
            for p in sorted(patterns, key=lambda x: x.get("confidence", 0), reverse=True)[:5]:
                card += f"- [{p.get('confidence', 0):.0%}] {p.get('name', '')}: {p.get('advice', '')}\n"

        if recent:
            card += "\n### 最近动作\n"
            for e in reversed(recent[-3:]):
                ts = e.get("timestamp", "")[:19]
                card += f"- [{ts}] {e.get('action', '?')}: {e.get('summary', '')}\n"

        return card


# ---- HTTP helpers ----

def _rocos_get(base_url: str, path: str, params: Optional[dict] = None) -> dict:
    url = f"{base_url.rstrip('/')}{path}"
    if params:
        query = "&".join(f"{k}={v}" for k, v in params.items() if v is not None)
        url = f"{url}?{query}"
    try:
        req = urllib.request.Request(url, headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=10) as resp:
            body = resp.read().decode("utf-8")
            return json.loads(body) if body else {}
    except Exception as e:
        return {"success": False, "code": -1, "message": str(e), "data": None}


def _rocos_post(base_url: str, path: str, body: dict) -> dict:
    url = f"{base_url.rstrip('/')}{path}"
    try:
        data = json.dumps(body).encode("utf-8")
        req = urllib.request.Request(
            url, data=data,
            headers={"Content-Type": "application/json", "Accept": "application/json"},
            method="POST"
        )
        with urllib.request.urlopen(req, timeout=30) as resp:
            resp_body = resp.read().decode("utf-8")
            return json.loads(resp_body) if resp_body else {}
    except Exception as e:
        return {"success": False, "code": -1, "message": str(e), "data": None}


# ---- 1. 执行记忆 ----

class RocosMemory(Component):
    """记录每次机器人动作到持久化存储"""

    display_name: str = "ROCOS 执行记忆"
    description: str = (
        "记录机器人每次动作（运动类型、参数、前后状态、结果）到本地 JSON 文件。"
        "自动累积统计信息（总次数、错误率等）。"
        "Use this tool after every robot action to build an experience database."
    )
    icon: str = "Database"
    name: str = "rocos_memory"

    inputs = [
        StrInput(name="base_url", display_name="API 基础地址", value="http://localhost:8080"),
        StrInput(name="action", display_name="动作类型", info="如 MoveJ, MoveL, JogStart 等", value="MoveL"),
        StrInput(name="params_json", display_name="动作参数 (JSON)", info="动作的参数字典 JSON", value="{}"),
        StrInput(name="summary", display_name="摘要", info="一句话描述做了什么", value=""),
        StrInput(name="memory_dir", display_name="存储目录", value=DEFAULT_MEMORY_DIR, advanced=True),
    ]

    outputs = [
        Output(display_name="Memory Entry", name="entry", method="record_action"),
    ]

    def record_action(self) -> Data:
        # 读当前状态
        state = _rocos_get(self.base_url, "/api/robot/state")
        sdata = state.get("data", {})
        robot_state = sdata.get("robot_state", "UNKNOWN")
        flange = sdata.get("flange", {}).get("position", {})

        # 解析参数
        try:
            params = json.loads(self.params_json) if self.params_json else {}
        except json.JSONDecodeError:
            params = {"raw": self.params_json}

        entry = {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "action": self.action,
            "params": params,
            "state_snapshot": {
                "robot_state": robot_state,
                "flange_x": round(flange.get("x", 0), 4),
                "flange_y": round(flange.get("y", 0), 4),
                "flange_z": round(flange.get("z", 0), 4),
            },
            "summary": self.summary or f"{self.action} @ {robot_state}",
            "success": robot_state != "ERROR_STATE",
        }

        store = MemoryStore(os.path.join(self.memory_dir, DEFAULT_MEMORY_FILE))
        store.record(entry)

        stats = store.get_stats()
        text = (
            f"📝 已记录: {entry['summary']}\n"
            f"累计: {stats.get('total_motions', 0)} 次动作, "
            f"{stats.get('errors', 0)} 次错误, "
            f"{stats.get('recoveries', 0)} 次恢复"
        )
        self.status = text[:100]
        return Data(text=text, data=entry)


# ---- 2. 自省分析 ----

class RocosReflect(Component):
    """分析历史记忆，提取模式和改进建议"""

    display_name: str = "ROCOS 经验自省"
    description: str = (
        "分析历史执行记录，自动发现模式: 哪些参数导致失败？哪些目标位置安全？"
        "Use this periodically to let the system learn from past actions."
    )
    icon: str = "BrainCircuit"
    name: str = "rocos_reflect"

    inputs = [
        StrInput(name="memory_dir", display_name="存储目录", value=DEFAULT_MEMORY_DIR, advanced=True),
    ]

    outputs = [
        Output(display_name="Insights", name="insights", method="analyze"),
    ]

    def analyze(self) -> Data:
        store = MemoryStore(os.path.join(self.memory_dir, DEFAULT_MEMORY_FILE))
        recent = store.get_recent(100)
        stats = store.get_stats()

        if not recent:
            return Data(text="暂无历史数据可供分析", data={"patterns": []})

        insights = []

        # 分析 1: 错误模式
        errors = [e for e in recent if not e.get("success", True)]
        if errors:
            # 统计错误前的动作类型
            from collections import Counter
            error_actions = Counter(e.get("action", "?") for e in errors)
            top_error = error_actions.most_common(1)[0]
            insights.append({
                "name": "高频错误动作",
                "advice": f"{top_error[0]} 出现了 {top_error[1]} 次错误，下次执行前先确认状态并降低速度",
                "confidence": min(0.9, top_error[1] / 5),
            })

        # 分析 2: 成功模式 — 最常用位置区域
        successes = [e for e in recent if e.get("success", True)]
        if successes:
            z_values = [e.get("state_snapshot", {}).get("flange_z", 0) for e in successes if "state_snapshot" in e]
            if z_values:
                avg_z = sum(z_values) / len(z_values)
                min_z, max_z = min(z_values), max(z_values)
                insights.append({
                    "name": "安全工作区间",
                    "advice": f"Z 轴安全区间: [{min_z:.3f}, {max_z:.3f}]，均值 {avg_z:.3f}。超出此范围时谨慎",
                    "confidence": 0.7,
                })

        # 分析 3: 恢复率
        if stats.get("errors", 0) > 0:
            recovery_rate = stats.get("recoveries", 0) / stats.get("errors", 0)
            if recovery_rate < 0.5:
                insights.append({
                    "name": "低恢复率",
                    "advice": "错误恢复率低，建议在 Agent 提示词中加入错误恢复流程指引",
                    "confidence": 0.8,
                })

        # 分析 4: 速度趋势
        velocities = []
        for e in recent:
            v = e.get("params", {}).get("velocity", 0)
            if v:
                velocities.append(v)
        if velocities:
            avg_vel = sum(velocities) / len(velocities)
            if avg_vel > 0.5:
                insights.append({
                    "name": "速度偏高",
                    "advice": f"平均速度 {avg_vel:.2f}，建议降低至 0.3 以下提高安全性",
                    "confidence": 0.6,
                })

        # 存储 pattern
        for ins in insights:
            store.add_pattern(ins)

        # 生成报告
        text = "## 自省分析报告\n\n"
        for ins in sorted(insights, key=lambda x: x["confidence"], reverse=True):
            text += f"- [{ins['confidence']:.0%}] {ins['name']}: {ins['advice']}\n"

        if not insights:
            text += "✅ 未发现明显模式，数据量可能不足。继续积累中...\n"

        text += f"\n分析样本: {len(recent)} 条记录"

        self.status = f"发现 {len(insights)} 个模式，覆盖 {len(recent)} 条记录"
        return Data(text=text, data={"patterns": insights, "stats": stats})


# ---- 3. 知识库 ----

class RocosKnowledge(Component):
    """查询累积的经验知识"""

    display_name: str = "ROCOS 经验知识"
    description: str = (
        "查询从历史执行中积累的经验知识: 统计信息、学到的模式、最近动作。"
        "可连接到 Agent 的 context 或 system_prompt，让每次对话都带着历史经验。"
        "Use this tool at the start of a session to load accumulated experience."
    )
    icon: str = "BookOpen"
    name: str = "rocos_knowledge"

    inputs = [
        StrInput(name="memory_dir", display_name="存储目录", value=DEFAULT_MEMORY_DIR, advanced=True),
    ]

    outputs = [
        Output(display_name="Knowledge Card", name="knowledge", method="get_knowledge"),
    ]

    def get_knowledge(self) -> Data:
        store = MemoryStore(os.path.join(self.memory_dir, DEFAULT_MEMORY_FILE))
        card = store.get_knowledge_card()
        stats = store.get_stats()
        self.status = f"经验: {stats.get('total_motions', 0)} 次动作, {len(store.get_patterns())} 个模式"
        return Data(text=card, data={"stats": stats, "patterns": store.get_patterns()})
