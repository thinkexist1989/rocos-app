"""
ROCOS named frame management components.
"""

from langflow.custom import Component
from langflow.io import Output, StrInput
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


def _frame_from_inputs(component) -> dict:
    return {
        "position": {
            "x": float(component.pos_x),
            "y": float(component.pos_y),
            "z": float(component.pos_z),
        },
        "orientation": {
            "x": float(component.ori_x),
            "y": float(component.ori_y),
            "z": float(component.ori_z),
            "w": float(component.ori_w),
        },
    }


def _frame_inputs(default_name: str, default_z: str = "0.0") -> list:
    return [
        StrInput(name="base_url", display_name="API 基础地址", value="http://localhost:8080"),
        StrInput(name="name", display_name="坐标系名称", value=default_name),
        StrInput(name="pos_x", display_name="位置 X (m)", value="0.0"),
        StrInput(name="pos_y", display_name="位置 Y (m)", value="0.0"),
        StrInput(name="pos_z", display_name="位置 Z (m)", value=default_z),
        StrInput(name="ori_x", display_name="Orientation X", value="0.0"),
        StrInput(name="ori_y", display_name="Orientation Y", value="0.0"),
        StrInput(name="ori_z", display_name="Orientation Z", value="0.0"),
        StrInput(name="ori_w", display_name="Orientation W", value="1.0"),
    ]


class GetToolFrames(Component):
    """获取所有工具坐标系名称列表"""

    display_name: str = "获取工具坐标系列表"
    description: str = (
        "获取所有已定义的工具坐标系名称，以及当前激活的工具坐标系名称。"
        "Use this to list available tool frames before activating one."
    )
    icon: str = "List"
    name: str = "get_tool_frames"

    inputs = [StrInput(name="base_url", display_name="API 基础地址", value="http://localhost:8080")]
    outputs = [Output(display_name="Frames List", name="frames", method="fetch_tool_frames")]

    def fetch_tool_frames(self) -> Data:
        result = _rocos_get(self.base_url, "/api/robot/tool_frames")
        if result.get("success"):
            data = result.get("data", {})
            text = f"工具坐标系: {data.get('names', [])}\n当前激活: {data.get('active') or '(无)'}"
        else:
            text = f"❌ {result.get('message', '查询失败')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class GetToolFrame(Component):
    """按名称获取工具坐标系"""

    display_name: str = "获取工具坐标系"
    description: str = (
        "按名称查询工具坐标系完整位姿。"
        "Use this to inspect a named TCP frame before activating or editing it."
    )
    icon: str = "Search"
    name: str = "get_tool_frame"

    inputs = [
        StrInput(name="base_url", display_name="API 基础地址", value="http://localhost:8080"),
        StrInput(name="name", display_name="坐标系名称", value="tcp_default"),
    ]
    outputs = [Output(display_name="Frame", name="frame", method="fetch_frame")]

    def fetch_frame(self) -> Data:
        result = _rocos_get(self.base_url, "/api/robot/tool_frame", {"name": self.name})
        if result.get("success"):
            text = f"工具坐标系 '{self.name}': {result.get('data', {}).get('frame', {})}"
        else:
            text = f"❌ 查询失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class SetToolFrame(Component):
    """设置或新增工具坐标系"""

    display_name: str = "设置工具坐标系"
    description: str = (
        "设置或新增命名工具坐标系 (Tool Frame / TCP)，位姿表示工具相对于法兰的偏移。"
        "Use this to define a TCP offset from the flange."
    )
    icon: str = "Crosshair"
    name: str = "set_tool_frame"

    inputs = _frame_inputs("tcp_default", "0.2")
    outputs = [Output(display_name="Frame Result", name="frame_result", method="set_frame")]

    def set_frame(self) -> Data:
        try:
            frame = _frame_from_inputs(self)
        except ValueError:
            return Data(text="❌ 参数错误: 位姿数值格式不正确", data=None)
        result = _rocos_post(self.base_url, "/api/robot/tool_frame", {
            "name": self.name,
            "frame": frame,
        })
        if result.get("success"):
            text = f"✅ 工具坐标系 '{self.name}' 已设置"
        else:
            text = f"❌ 设置失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class ActivateToolFrame(Component):
    """激活指定工具坐标系"""

    display_name: str = "激活工具坐标系"
    description: str = (
        "激活指定工具坐标系。"
        "Use this to switch the active TCP for subsequent motion commands."
    )
    icon: str = "CheckSquare"
    name: str = "activate_tool_frame"

    inputs = [
        StrInput(name="base_url", display_name="API 基础地址", value="http://localhost:8080"),
        StrInput(name="name", display_name="坐标系名称", value="tcp_default"),
    ]
    outputs = [Output(display_name="Result", name="result", method="activate")]

    def activate(self) -> Data:
        result = _rocos_post(self.base_url, "/api/robot/active_tool_frame", {"name": self.name})
        if result.get("success"):
            text = f"✅ 已激活工具坐标系: {self.name}"
        else:
            text = f"❌ 激活失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class RemoveToolFrame(Component):
    """删除工具坐标系"""

    display_name: str = "删除工具坐标系"
    description: str = (
        "按名称删除工具坐标系。"
        "Use this to remove an obsolete TCP frame."
    )
    icon: str = "Trash"
    name: str = "remove_tool_frame"

    inputs = [
        StrInput(name="base_url", display_name="API 基础地址", value="http://localhost:8080"),
        StrInput(name="name", display_name="坐标系名称", value="tcp_default"),
    ]
    outputs = [Output(display_name="Result", name="result", method="remove")]

    def remove(self) -> Data:
        result = _rocos_delete(self.base_url, "/api/robot/tool_frame", {"name": self.name})
        if result.get("success"):
            text = f"✅ 工具坐标系 '{self.name}' 已删除"
        else:
            text = f"❌ 删除失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class GetObjectFrames(Component):
    """获取所有工件坐标系名称列表"""

    display_name: str = "获取工件坐标系列表"
    description: str = (
        "获取所有已定义的工件坐标系名称，以及当前激活的工件坐标系名称。"
        "Use this to list available object/workpiece frames."
    )
    icon: str = "List"
    name: str = "get_object_frames"

    inputs = [StrInput(name="base_url", display_name="API 基础地址", value="http://localhost:8080")]
    outputs = [Output(display_name="Frames List", name="frames", method="fetch_object_frames")]

    def fetch_object_frames(self) -> Data:
        result = _rocos_get(self.base_url, "/api/robot/object_frames")
        if result.get("success"):
            data = result.get("data", {})
            text = f"工件坐标系: {data.get('names', [])}\n当前激活: {data.get('active') or '(无)'}"
        else:
            text = f"❌ {result.get('message', '查询失败')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class GetObjectFrame(Component):
    """按名称获取工件坐标系"""

    display_name: str = "获取工件坐标系"
    description: str = (
        "按名称查询工件坐标系完整位姿。"
        "Use this to inspect a named workpiece frame before activating or editing it."
    )
    icon: str = "Search"
    name: str = "get_object_frame"

    inputs = [
        StrInput(name="base_url", display_name="API 基础地址", value="http://localhost:8080"),
        StrInput(name="name", display_name="坐标系名称", value="table_1"),
    ]
    outputs = [Output(display_name="Frame", name="frame", method="fetch_frame")]

    def fetch_frame(self) -> Data:
        result = _rocos_get(self.base_url, "/api/robot/object_frame", {"name": self.name})
        if result.get("success"):
            text = f"工件坐标系 '{self.name}': {result.get('data', {}).get('frame', {})}"
        else:
            text = f"❌ 查询失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class SetObjectFrame(Component):
    """设置或新增工件坐标系"""

    display_name: str = "设置工件坐标系"
    description: str = (
        "设置或新增命名工件坐标系 (Object Frame / Workpiece Frame)，位姿表示工件相对于基座。"
        "Use this to define a workpiece/base coordinate system."
    )
    icon: str = "Box"
    name: str = "set_object_frame"

    inputs = _frame_inputs("table_1", "0.0")
    outputs = [Output(display_name="Frame Result", name="frame_result", method="set_frame")]

    def set_frame(self) -> Data:
        try:
            frame = _frame_from_inputs(self)
        except ValueError:
            return Data(text="❌ 参数错误: 位姿数值格式不正确", data=None)
        result = _rocos_post(self.base_url, "/api/robot/object_frame", {
            "name": self.name,
            "frame": frame,
        })
        if result.get("success"):
            text = f"✅ 工件坐标系 '{self.name}' 已设置"
        else:
            text = f"❌ 设置失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class ActivateObjectFrame(Component):
    """激活指定工件坐标系"""

    display_name: str = "激活工件坐标系"
    description: str = (
        "激活指定工件坐标系。"
        "Use this to switch the active workpiece coordinate system."
    )
    icon: str = "CheckSquare"
    name: str = "activate_object_frame"

    inputs = [
        StrInput(name="base_url", display_name="API 基础地址", value="http://localhost:8080"),
        StrInput(name="name", display_name="坐标系名称", value="table_1"),
    ]
    outputs = [Output(display_name="Result", name="result", method="activate")]

    def activate(self) -> Data:
        result = _rocos_post(self.base_url, "/api/robot/active_object_frame", {"name": self.name})
        if result.get("success"):
            text = f"✅ 已激活工件坐标系: {self.name}"
        else:
            text = f"❌ 激活失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class RemoveObjectFrame(Component):
    """删除工件坐标系"""

    display_name: str = "删除工件坐标系"
    description: str = (
        "按名称删除工件坐标系。"
        "Use this to remove an obsolete workpiece frame."
    )
    icon: str = "Trash"
    name: str = "remove_object_frame"

    inputs = [
        StrInput(name="base_url", display_name="API 基础地址", value="http://localhost:8080"),
        StrInput(name="name", display_name="坐标系名称", value="table_1"),
    ]
    outputs = [Output(display_name="Result", name="result", method="remove")]

    def remove(self) -> Data:
        result = _rocos_delete(self.base_url, "/api/robot/object_frame", {"name": self.name})
        if result.get("success"):
            text = f"✅ 工件坐标系 '{self.name}' 已删除"
        else:
            text = f"❌ 删除失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class LoadFrames(Component):
    """从 YAML 加载坐标系"""

    display_name: str = "加载坐标系 YAML"
    description: str = (
        "从 YAML 文件加载工具坐标系、工件坐标系及 active frame 名称。"
        "Use this to restore saved frame definitions from disk."
    )
    icon: str = "FolderOpen"
    name: str = "load_frames"

    inputs = [
        StrInput(name="base_url", display_name="API 基础地址", value="http://localhost:8080"),
        StrInput(name="path", display_name="YAML 路径", value="config/frames.yaml"),
    ]
    outputs = [Output(display_name="Result", name="result", method="load")]

    def load(self) -> Data:
        result = _rocos_post(self.base_url, "/api/robot/frames/load", {"path": self.path})
        if result.get("success"):
            text = f"✅ 坐标系已加载: {self.path}"
        else:
            text = f"❌ 加载失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)


class SaveFrames(Component):
    """保存坐标系到 YAML"""

    display_name: str = "保存坐标系 YAML"
    description: str = (
        "将当前工具坐标系、工件坐标系及 active frame 名称保存到 YAML 文件。"
        "Use this to persist frame definitions to disk."
    )
    icon: str = "Save"
    name: str = "save_frames"

    inputs = [
        StrInput(name="base_url", display_name="API 基础地址", value="http://localhost:8080"),
        StrInput(name="path", display_name="YAML 路径", value="config/frames.yaml"),
    ]
    outputs = [Output(display_name="Result", name="result", method="save")]

    def save(self) -> Data:
        result = _rocos_post(self.base_url, "/api/robot/frames/save", {"path": self.path})
        if result.get("success"):
            text = f"✅ 坐标系已保存: {self.path}"
        else:
            text = f"❌ 保存失败 (code={result.get('code')}): {result.get('message', '')}"
        self.status = text[:100]
        return Data(text=text, data=result)
