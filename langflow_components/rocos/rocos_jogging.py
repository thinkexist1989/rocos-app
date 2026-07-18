"""
ROCOS jogging components.

The UI keeps the compact flag style (J0, BASE_X, TOOL_YAW, NULLSPACE), while
the implementation calls the current vector-based /api/robot/jog/* endpoints.
"""

from langflow.custom import Component
from langflow.io import Output, StrInput, DropdownInput, FloatInput, IntInput
from langflow.schema import Data
import json as _json
import urllib.error as _urlerror
import urllib.parse as _urlparse
import urllib.request as _urlreq
from typing import Optional as _Optional


def _rocos_get(_base, _path, _params: _Optional[dict] = None):
    _url = f"{_base.rstrip('/')}{_path}"
    if _params:
        _q = "&".join(f"{k}={v}" for k, v in _params.items() if v is not None)
        _url = f"{_url}?{_q}"
    try:
        _req = _urlreq.Request(_url, headers={"Accept": "application/json"})
        with _urlreq.urlopen(_req, timeout=10) as _r:
            return _json.loads(_r.read().decode("utf-8"))
    except _urlerror.URLError as _e:
        return {"success": False, "code": -1, "message": f"网络错误: {_e}", "data": None}
    except _json.JSONDecodeError as _e:
        return {"success": False, "code": -2, "message": f"JSON解析错误: {_e}", "data": None}


def _rocos_post(_base, _path, _body: dict):
    _url = f"{_base.rstrip('/')}{_path}"
    try:
        _data = _json.dumps(_body).encode("utf-8")
        _req = _urlreq.Request(_url, data=_data, headers={"Content-Type": "application/json", "Accept": "application/json"}, method="POST")
        with _urlreq.urlopen(_req, timeout=30) as _r:
            return _json.loads(_r.read().decode("utf-8"))
    except _urlerror.URLError as _e:
        return {"success": False, "code": -1, "message": f"网络错误: {_e}", "data": None}
    except _json.JSONDecodeError as _e:
        return {"success": False, "code": -2, "message": f"JSON解析错误: {_e}", "data": None}


def _rocos_delete(_base, _path, _params: _Optional[dict] = None):
    _url = f"{_base.rstrip('/')}{_path}"
    if _params:
        _q = "&".join(f"{k}={v}" for k, v in _params.items() if v is not None)
        _url = f"{_url}?{_q}"
    try:
        _req = _urlreq.Request(_url, headers={"Accept": "application/json"}, method="DELETE")
        with _urlreq.urlopen(_req, timeout=10) as _r:
            return _json.loads(_r.read().decode("utf-8"))
    except _urlerror.URLError as _e:
        return {"success": False, "code": -1, "message": f"网络错误: {_e}", "data": None}
    except _json.JSONDecodeError as _e:
        return {"success": False, "code": -2, "message": f"JSON解析错误: {_e}", "data": None}


def _rocos_text(_base, _path, _params: _Optional[dict] = None):
    _url = f"{_base.rstrip('/')}{_path}"
    if _params:
        _q = "&".join(f"{k}={v}" for k, v in _params.items() if v is not None)
        _url = f"{_url}?{_q}"
    _req = _urlreq.Request(_url, headers={"Accept": "*/*"})
    with _urlreq.urlopen(_req, timeout=10) as _r:
        return _r.read().decode("utf-8")


def parse_float_list(_text: str, _expected_len: _Optional[int] = None):
    _vals = [float(x.strip()) for x in _text.split(",") if x.strip()]
    if _expected_len is not None and len(_vals) != _expected_len:
        raise ValueError(f"expected {_expected_len} values, got {len(_vals)}")
    return _vals


JOG_FLAGS = [
    "J0", "J1", "J2", "J3", "J4", "J5", "J6",
    "TOOL_X", "TOOL_Y", "TOOL_Z", "TOOL_ROLL", "TOOL_PITCH", "TOOL_YAW",
    "FLANGE_X", "FLANGE_Y", "FLANGE_Z", "FLANGE_ROLL", "FLANGE_PITCH", "FLANGE_YAW",
    "OBJECT_X", "OBJECT_Y", "OBJECT_Z", "OBJECT_ROLL", "OBJECT_PITCH", "OBJECT_YAW",
    "BASE_X", "BASE_Y", "BASE_Z", "BASE_ROLL", "BASE_PITCH", "BASE_YAW",
    "NULLSPACE",
]

DIRECTIONS = ["POSITIVE", "NEGATIVE", "NONE"]
AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2, "ROLL": 3, "PITCH": 4, "YAW": 5}


def _sign(direction: str) -> int:
    if direction == "POSITIVE":
        return 1
    if direction == "NEGATIVE":
        return -1
    return 0


class JogStart(Component):
    """启动点动"""

    display_name: str = "启动点动"
    description: str = (
        "按方向向量启动 ROCOS 点动。支持关节点动 J0~J6、笛卡尔点动 BASE/FLANGE/TOOL/OBJECT "
        "坐标系的 X/Y/Z/ROLL/PITCH/YAW，以及 NULLSPACE 零空间点动。"
        "Use this tool to jog the robot in a specific direction or axis."
    )
    icon: str = "MousePointerClick"
    name: str = "jog_start"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
        DropdownInput(
            name="flag",
            display_name="点动方向/轴",
            info="选择点动的目标轴或方向",
            options=JOG_FLAGS,
            value="J0",
        ),
        DropdownInput(
            name="direction",
            display_name="运动方向",
            info="POSITIVE=正方向, NEGATIVE=负方向, NONE=停止点动",
            options=DIRECTIONS,
            value="POSITIVE",
        ),
        IntInput(
            name="joint_count",
            display_name="关节数量",
            info="模型关节数量，用于生成关节/零空间点动向量",
            value=7,
        ),
        StrInput(
            name="nullspace_vector",
            display_name="零空间方向向量",
            info="NULLSPACE 时使用，长度需等于关节数量，如: 1,0,0,0,0,0,0",
            value="1,0,0,0,0,0,0",
        ),
        FloatInput(
            name="speed",
            display_name="点动速度",
            info="点动速度，关节单位 rad/s，笛卡尔单位按后端 MoveJog 解释",
            value=0.005,
        ),
        FloatInput(
            name="timeout",
            display_name="命令有效期(s)",
            info="单次点动命令有效期，建议大于上位机重复下发周期",
            value=0.3,
        ),
        FloatInput(
            name="dir_threshold",
            display_name="方向一致性阈值",
            info="连续点动方向余弦相似度阈值",
            value=0.99,
        ),
    ]

    outputs = [
        Output(display_name="Jog Result", name="jog_result", method="start_jog"),
    ]

    def start_jog(self) -> Data:
        sign = _sign(self.direction)
        if sign == 0:
            data = JogStop.stop_jog_for_url(self.base_url)
            self.status = data.text[:100]
            return data

        try:
            joint_count = int(self.joint_count)
            if joint_count <= 0:
                raise ValueError("joint_count must be positive")
        except ValueError:
            return Data(text="❌ 参数错误: joint_count 必须是正整数", data=None)

        if self.flag.startswith("J") and self.flag[1:].isdigit():
            joint_index = int(self.flag[1:])
            if joint_index >= joint_count:
                return Data(text=f"❌ 关节索引越界: J{joint_index}, joint_count={joint_count}", data=None)
            joints = [0.0] * joint_count
            joints[joint_index] = float(sign)
            result = _rocos_post(self.base_url, "/api/robot/jog/joint", {
                "joints": joints,
                "speed": self.speed,
                "timeout": self.timeout,
                "dir_threshold": self.dir_threshold,
            })
        elif self.flag == "NULLSPACE":
            try:
                joints = [sign * value for value in parse_float_list(self.nullspace_vector, joint_count)]
            except ValueError as exc:
                return Data(text=f"❌ 零空间方向向量错误: {exc}", data=None)
            result = _rocos_post(self.base_url, "/api/robot/jog/nullspace", {
                "joints": joints,
                "speed": self.speed,
                "timeout": self.timeout,
                "dir_threshold": self.dir_threshold,
            })
        else:
            try:
                frame, axis = self.flag.split("_", 1)
                axis_index = AXIS_INDEX[axis]
            except (ValueError, KeyError):
                return Data(text=f"❌ 不支持的点动 flag: {self.flag}", data=None)
            twist = [0.0] * 6
            twist[axis_index] = float(sign)
            result = _rocos_post(self.base_url, "/api/robot/jog/cartesian", {
                "frame": frame,
                "twist": twist,
                "speed": self.speed,
                "timeout": self.timeout,
                "dir_threshold": self.dir_threshold,
            })

        if result.get("success"):
            text = f"✅ 点动已启动: {self.flag} {self.direction}"
        else:
            text = f"❌ 点动失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class JogStop(Component):
    """停止点动"""

    display_name: str = "停止点动"
    description: str = (
        "停止 ROCOS 机器人所有正在进行的点动运动。"
        "Use this to stop jogging immediately."
    )
    icon: str = "MousePointer"
    name: str = "jog_stop"

    inputs = [
        StrInput(
            name="base_url",
            display_name="API 基础地址",
            info="ROCOS API 的基础 URL",
            value="http://localhost:8080",
        ),
    ]

    outputs = [
        Output(display_name="Result", name="result", method="stop_jog"),
    ]

    @staticmethod
    def stop_jog_for_url(base_url: str) -> Data:
        result = _rocos_post(base_url, "/api/robot/jog/stop", {})
        if result.get("success"):
            text = "✅ 点动已停止"
        else:
            text = f"⚠️ {result.get('message', '停止点动失败')}"
        return Data(text=text, data=result)

    def stop_jog(self) -> Data:
        data = self.stop_jog_for_url(self.base_url)
        self.status = data.text[:100]
        return data
