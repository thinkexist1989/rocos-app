"""
ROCOS 机器人状态查询组件组。

提供获取机器人完整状态、关节信息、使能状态、URDF 模型的功能。
每个组件封装一个 GET 请求到 ROCOS HTTP API。
"""

import json
import urllib.request
import urllib.error
from typing import Optional
from langflow.custom import Component
from langflow.io import Output, StrInput
from langflow.schema import Data


# ---- 内部辅助函数 ----

def _rocos_get(base_url: str, path: str, params: Optional[dict] = None) -> dict:
    """向 ROCOS API 发送 GET 请求，返回解析后的 JSON 响应"""
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
        return {"success": False, "code": -1, "message": f"网络错误: {e}", "data": None}
    except json.JSONDecodeError as e:
        return {"success": False, "code": -2, "message": f"JSON 解析错误: {e}", "data": None}


def _rocos_post(base_url: str, path: str, body: dict) -> dict:
    """向 ROCOS API 发送 POST 请求，返回解析后的 JSON 响应"""
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
        return {"success": False, "code": -1, "message": f"网络错误: {e}", "data": None}
    except json.JSONDecodeError as e:
        return {"success": False, "code": -2, "message": f"JSON 解析错误: {e}", "data": None}


# ---- 状态查询组件 ----

class GetRobotState(Component):
    """获取机器人完整状态快照"""

    display_name: str = "获取机器人状态"
    description: str = (
        "获取 ROCOS 机器人的完整状态快照，包括 FSM 状态、各关节位置/速度/力矩、"
        "法兰位姿 (flange)、当前工具坐标系和工件坐标系位姿、以及硬件总线状态。"
        "Use this tool when you need to know the robot's current state, joint positions, or end-effector pose."
    )
    icon: str = "Activity"
    name: str = "get_robot_state"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="State Data", name="state_data", method="fetch_state"),
    ]

    def fetch_state(self) -> Data:
        result = _rocos_get(self.base_url, "/api/robot/state")
        if result.get("success"):
            data = result.get("data", {})
            # 格式化摘要信息
            state = data.get("robot_state", "UNKNOWN")
            joints = data.get("joint_states", [])
            joint_summary = ", ".join(
                f"{j['name']}={j['position']:.3f}" for j in joints[:3]
            )
            flange = data.get("flange", {})
            pos = flange.get("position", {})
            text = (
                f"机器人状态: {state}\n"
                f"使能: {data.get('is_enabled', False)}, 运动: {data.get('motion_busy', False)}\n"
                f"关节角度 (前3轴): {joint_summary}\n"
                f"法兰位置: x={pos.get('x',0):.3f}, y={pos.get('y',0):.3f}, z={pos.get('z',0):.3f}"
            )
        else:
            text = f"❌ {result.get('message', '查询失败')}"
            data = None
        self.status = text[:100]
        return Data(text=text, data=result)


class GetRobotInfo(Component):
    """获取机器人关节参数信息"""

    display_name: str = "获取机器人信息"
    description: str = (
        "获取 ROCOS 机器人的关节参数信息，包括各关节的传动比 (ratio)、"
        "每单位编码器计数 (cnt_per_unit)、单位名称 (unit_name) 等配置参数。"
        "Use this when you need to know joint configuration details like ratios and encoder counts."
    )
    icon: str = "Info"
    name: str = "get_robot_info"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Info Data", name="info_data", method="fetch_info"),
    ]

    def fetch_info(self) -> Data:
        result = _rocos_get(self.base_url, "/api/robot/info")
        if result.get("success"):
            infos = result.get("data", {}).get("joint_infos", [])
            lines = [f"共 {len(infos)} 个关节:"]
            for j in infos:
                lines.append(
                    f"  [{j['id']}] {j['name']}: ratio={j['ratio']}, "
                    f"cnt/unit={j['cnt_per_unit']}, unit={j['unit_name']}"
                )
            text = "\n".join(lines)
        else:
            text = f"❌ {result.get('message', '查询失败')}"
        self.status = f"获取了 {len(result.get('data', {}).get('joint_infos', []))} 个关节信息"
        return Data(text=text, data=result)


class GetRobotEnabled(Component):
    """查询机器人使能状态"""

    display_name: str = "查询使能状态"
    description: str = (
        "查询 ROCOS 机器人当前是否已使能 (enabled)。"
        "返回 enebled 标志、disabled 标志和 robot_state (FSM 状态)。"
        "Use this before issuing motion commands to verify the robot is ready."
    )
    icon: str = "Power"
    name: str = "get_robot_enabled"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Enabled Status", name="enabled_status", method="check_enabled"),
    ]

    def check_enabled(self) -> Data:
        result = _rocos_get(self.base_url, "/api/robot/enabled")
        if result.get("success"):
            d = result.get("data", {})
            text = (
                f"使能: {'✅ 是' if d.get('enabled') else '❌ 否'}, "
                f"状态: {d.get('robot_state', 'UNKNOWN')}"
            )
        else:
            text = f"❌ {result.get('message', '查询失败')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class GetRobotURDF(Component):
    """获取机器人 URDF 模型 XML"""

    display_name: str = "获取 URDF 模型"
    description: str = (
        "获取 ROCOS 机器人的 URDF 模型 XML 文本。"
        "包含运动学链、关节信息、连杆 mesh 路径等完整模型描述。"
        "Use this when you need the robot's kinematic tree or link/joint structure."
    )
    icon: str = "FileText"
    name: str = "get_robot_urdf"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="URDF Content", name="urdf_content", method="fetch_urdf"),
    ]

    def fetch_urdf(self) -> Data:
        result = _rocos_get(self.base_url, "/api/robot/urdf")
        # URDF 返回 XML 文本，可能不在标准 JSON 响应结构内
        if isinstance(result, dict):
            text = result.get("message", "") or str(result)[:500]
        else:
            text = str(result)[:2000] + "..." if len(str(result)) > 2000 else str(result)
        self.status = "URDF 获取完成"
        return Data(text=text[:2000], data={"urdf_preview": text[:2000]})
