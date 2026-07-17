"""
ROCOS 语义层组件 — 将实战中迭代出的控制模式编码为可复用组件。

包含:
  - RocosAgentPrompt: 生成最优的 Agent 系统提示词
  - RocosErrorRecovery: 自动错误恢复 (ERROR_STATE → disable → enable → STOPPED)
  - RocosSafeMoveL: 先读状态再执行 MoveL 的安全模式
"""

from langflow.custom import Component
from langflow.io import Output, StrInput, FloatInput, DropdownInput
from langflow.schema import Data
import json
import urllib.request
import urllib.error
from typing import Optional


# ---- HTTP Helpers ----

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
    except urllib.error.URLError as e:
        return {"success": False, "code": -1, "message": f"Network error: {e}", "data": None}
    except json.JSONDecodeError as e:
        return {"success": False, "code": -2, "message": f"JSON parse error: {e}", "data": None}


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
    except urllib.error.URLError as e:
        return {"success": False, "code": -1, "message": f"Network error: {e}", "data": None}
    except json.JSONDecodeError as e:
        return {"success": False, "code": -2, "message": f"JSON parse error: {e}", "data": None}


# ---- 1. Agent 系统提示词 ----

class RocosAgentPrompt(Component):
    """生成 ROCOS 机器人控制 Agent 的最佳系统提示词"""

    display_name: str = "ROCOS Agent 提示词"
    description: str = (
        "生成经过实战迭代优化的 ROCOS 机器人控制 Agent 系统提示词。"
        "将此输出连接到 Agent 的 system_prompt 字段。"
        "包含: 先读状态再运动的模式、错误恢复流程、安全默认值、零空间语义。"
    )
    icon: str = "Brain"
    name: str = "rocos_agent_prompt"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
        StrInput(
            name="robot_description",
            display_name="机器人描述",
            info="可选的机器人定制描述（型号、应用场景等）",
            value="7-DOF 机械臂，仿真模式",
        ),
    ]

    outputs = [
        Output(display_name="System Prompt", name="prompt", method="build_prompt"),
    ]

    def build_prompt(self) -> Data:
        prompt = f"""你是 ROCOS 机器人控制助手。控制对象: {self.robot_description}。

## 核心规则 — 每次运动前必须遵守:

1. **先读状态再决策**: 调用 GetRobotState 获取真实的当前位置/姿态/状态
2. **基于真实坐标计算**: 不要假设机器人在上一次指令的目标位置，用 GetRobotState 返回的实际值
3. **小幅度执行**: 默认每次位移 ≤5cm，速度 ≤0.3
4. **错误恢复**: 如果 robot_state 是 ERROR_STATE，先调用 DisableRobot 再 EnableRobot

## 工作流程:
```
GetRobotState → 检查 STOPPED+enabled → 计算小幅度目标 → MoveJ/MoveL → 验证结果
```

## 状态码参考:
- STOPPED: 就绪，可以运动
- RUNNING: 运动中，等待完成
- ERROR_STATE: 需要恢复 (Disable → Enable)
- code=-2307: Fatal 错误，需要 Disable+Enable 恢复

## API 参考:
- MoveJ: /api/robot/movej, 字段 joints(数组), velocity, acceleration, jerk
- MoveL: /api/robot/movel, 字段 pose(position+orientation), velocity
- 零空间: /api/robot/jog/nullspace, 字段 joints(方向向量), speed, timeout(超时自动停)
- 坐标系: /api/robot/tool_frame, /api/robot/object_frame

## 安全守则:
- 不确定时用 MoveJ 比 MoveL 更安全（关节空间运动不会 IK 失败）
- 零空间运动前先读当前关节角度
- 接近目标点时缩小速度和步长
- Always respond in Chinese (简体中文)."""
        self.status = f"Prompt generated ({len(prompt)} chars)"
        return Data(text=prompt, data={"system_prompt": prompt})


# ---- 2. 错误恢复 ----

class RocosErrorRecovery(Component):
    """自动从 ERROR_STATE 恢复机器人"""

    display_name: str = "ROCOS 错误恢复"
    description: str = (
        "自动将机器人从 ERROR_STATE 恢复到 STOPPED 就绪状态。"
        "执行 Disable → Enable 循环。"
        "Use this tool when robot_state is ERROR_STATE or motion commands return code=-2307."
    )
    icon: str = "RefreshCw"
    name: str = "rocos_error_recovery"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Recovery Result", name="result", method="recover"),
    ]

    def recover(self) -> Data:
        base = self.base_url.rstrip("/")

        # Step 1: Read state
        state = _rocos_get(base, "/api/robot/state")
        robot_state = state.get("data", {}).get("robot_state", "UNKNOWN")

        if robot_state != "ERROR_STATE":
            text = f"机器人当前状态为 {robot_state}，无需恢复"
            self.status = text
            return Data(text=text, data={"state": robot_state, "recovered": False})

        # Step 2: Disable
        d = _rocos_post(base, "/api/robot/disable", {})
        if not d.get("success"):
            text = f"Disable 失败: {d.get('message')}"
            self.status = text
            return Data(text=text, data=d)

        # Step 3: Enable
        e = _rocos_post(base, "/api/robot/enable", {})
        if not e.get("success"):
            text = f"Enable 失败: {e.get('message')}"
            self.status = text
            return Data(text=text, data=e)

        # Step 4: Verify
        import time
        time.sleep(0.5)
        final = _rocos_get(base, "/api/robot/state")
        final_state = final.get("data", {}).get("robot_state", "UNKNOWN")

        text = (
            f"恢复完成: ERROR_STATE → Disable → Enable → {final_state}\n"
            f"{'✅ 成功，机器人就绪' if final_state == 'STOPPED' else '⚠️ 请检查'}"
        )
        self.status = text[:100]
        return Data(text=text, data={"recovered": final_state == "STOPPED", "state": final_state})


# ---- 3. 安全 MoveL（先读再动） ----

class RocosSafeMoveL(Component):
    """先读取真实状态，再执行笛卡尔直线运动"""

    display_name: str = "安全 MoveL（先读后动）"
    description: str = (
        "安全的笛卡尔直线运动: 先自动读取机器人当前真实位姿，然后执行小幅度的 MoveL。"
        "delta_x/y/z 相对于当前位置的增量（米），推荐 ≤0.05m。"
        "Use this for safe, small incremental Cartesian movements. Automatically reads state first."
    )
    icon: str = "Shield"
    name: str = "rocos_safe_movel"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            value="http://localhost:8080",
        ),
        FloatInput(
            name="delta_x",
            display_name="ΔX (m)",
            info="X 方向增量，正=前，负=后，推荐 ±0.03",
            value=0.0,
        ),
        FloatInput(
            name="delta_y",
            display_name="ΔY (m)",
            info="Y 方向增量，正=右，负=左，推荐 ±0.03",
            value=0.0,
        ),
        FloatInput(
            name="delta_z",
            display_name="ΔZ (m)",
            info="Z 方向增量，正=上，负=下，推荐 ±0.05",
            value=0.0,
        ),
        FloatInput(
            name="velocity",
            display_name="速度",
            info="运动速度，默认 0.15（慢速安全）",
            value=0.15,
        ),
    ]

    outputs = [
        Output(display_name="Move Result", name="result", method="safe_move"),
    ]

    def safe_move(self) -> Data:
        base = self.base_url.rstrip("/")

        # Step 1: 读取真实状态
        state = _rocos_get(base, "/api/robot/state")
        sdata = state.get("data", {})

        robot_state = sdata.get("robot_state", "UNKNOWN")
        is_enabled = sdata.get("is_enabled", False)

        if robot_state != "STOPPED" or not is_enabled:
            text = f"❌ 机器人未就绪: state={robot_state}, enabled={is_enabled}"
            self.status = text
            return Data(text=text, data={"ready": False, "state": robot_state})

        flange = sdata.get("flange", {})
        current_pos = flange.get("position", {"x": 0, "y": 0, "z": 0})
        current_ori = flange.get("orientation", {"x": 0, "y": 0, "z": 0, "w": 1})

        # Step 2: 计算目标位姿（基于真实坐标 + 增量）
        target_pos = {
            "x": current_pos["x"] + self.delta_x,
            "y": current_pos["y"] + self.delta_y,
            "z": current_pos["z"] + self.delta_z,
        }

        # 安全检查: 位移不超过 5cm
        total_delta = (self.delta_x**2 + self.delta_y**2 + self.delta_z**2) ** 0.5
        if total_delta > 0.05:
            self.status = f"Warning: delta={total_delta:.3f}m > 5cm"

        # Step 3: 执行 MoveL
        result = _rocos_post(base, "/api/robot/movel", {
            "pose": {"position": target_pos, "orientation": current_ori},
            "velocity": self.velocity,
        })

        if result.get("success"):
            text = (
                f"✅ 安全 MoveL\n"
                f"当前 (真实): x={current_pos['x']:.4f} y={current_pos['y']:.4f} z={current_pos['z']:.4f}\n"
                f"目标:         x={target_pos['x']:.4f} y={target_pos['y']:.4f} z={target_pos['z']:.4f}\n"
                f"增量: Δx={self.delta_x:.4f} Δy={self.delta_y:.4f} Δz={self.delta_z:.4f}"
            )
        else:
            code = result.get("code", 0)
            text = f"❌ MoveL 失败 (code={code}): {result.get('message')}"
            if code == -2307:
                text += "\n💡 机器人进入 ERROR_STATE，请使用 'ROCOS 错误恢复' 工具"

        self.status = text[:100]
        return Data(text=text, data=result)
