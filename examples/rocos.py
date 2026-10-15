"""Small ROCOS JSON/HTTP client. Python 3 standard library; no pip dependencies.

    with Robot("192.168.1.117", port=8080) as robot:
        robot.enable()
        robot.movej(robot.get_joints())

Angles are radians. Constructing Robot performs no network requests. Entering
the context acquires control and starts heartbeat; exit releases control.
"""

import json as jsonlib
import logging
import math
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from dataclasses import dataclass, replace

TOKEN_HEADER = "X-Rocos-Control-Token"
CLIENT_HEADER = "X-Rocos-Client-Id"
_log = logging.getLogger(__name__)


@dataclass
class Pose:
    """Cartesian position in metres and quaternion orientation (qx, qy, qz, qw)."""

    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    def copy(self):
        """Independent pose with the same position and orientation."""
        return replace(self)

    def translated(self, dx=0.0, dy=0.0, dz=0.0):
        """New pose translated along the reference-frame axes, in metres."""
        result = replace(self, x=self.x + dx, y=self.y + dy, z=self.z + dz)
        result.to_dict()
        return result

    @classmethod
    def from_dict(cls, value):
        position, orientation = value["position"], value["orientation"]
        return cls(position["x"], position["y"], position["z"],
                   orientation["x"], orientation["y"], orientation["z"], orientation["w"])

    def to_dict(self):
        values = (self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Pose 各分量必须是有限数值")
        if sum(value * value for value in values[3:]) <= 0:
            raise ValueError("Pose 姿态四元数不能全为零")
        return {"position": {"x": self.x, "y": self.y, "z": self.z},
                "orientation": {"x": self.qx, "y": self.qy, "z": self.qz, "w": self.qw}}


class ApiError(RuntimeError):
    """Controller business failure, including failures returned with HTTP 200."""

    def __init__(self, path, response, http_status=200):
        self.path = path
        self.response = response
        self.http_status = http_status
        self.code = response.get("code")
        self.error_name = response.get("error_name", "")
        self.message = response.get("message", "")
        self.data = response.get("data")
        detail = self.data.get("error") if isinstance(self.data, dict) else self.data
        super().__init__("{}: code={}, error_name={}, message={}, detail={}".format(
            path, self.code, self.error_name, self.message, detail))


class TransportError(RuntimeError):
    """Connection, HTTP, or response-format failure. Requests are never replayed."""


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, file, code, message, headers, new_url):
        # Never forward credentials or replay a control request to another URL.
        return None


class Robot:
    """Robot object: construct once, then call motion and query methods directly."""

    def __init__(self, host="127.0.0.1", port=8080, *, timeout=5,
                 client_name="python-robot", client_id=None, auto_acquire=True):
        address = str(host).strip().rstrip("/")
        if "://" not in address:
            if address.count(":") > 1 and not address.startswith("["):
                address = "[{}]".format(address)
            parsed = urllib.parse.urlsplit("http://" + address)
            address = "http://" + address
            if parsed.port is None:
                address += ":{}".format(port)
        parsed = urllib.parse.urlsplit(address)
        if (parsed.scheme not in ("http", "https") or not parsed.hostname or
                parsed.username is not None or parsed.password is not None or
                parsed.path or parsed.query or parsed.fragment):
            raise ValueError("host 必须是主机地址或完整的 http(s)://主机:端口，不包含路径或账号")
        if parsed.port is not None and not 1 <= parsed.port <= 65535:
            raise ValueError("端口必须在 1 到 65535 之间")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout 必须是正的有限秒数")
        self.base_url = address
        self.timeout = timeout
        self.client_name = client_name
        self.auto_acquire = auto_acquire
        self.client_id = client_id or "python-" + str(uuid.uuid4())
        self.token = None
        self.heartbeat_error = None
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread = None
        self.http = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())

    @staticmethod
    def _path(path):
        value = str(path)
        parsed = urllib.parse.urlsplit(value)
        if parsed.scheme or parsed.netloc or parsed.fragment:
            raise ValueError("接口只接受当前控制器的路径，不接受完整 URL 或 fragment")
        if any(part in (".", "..") for part in parsed.path.split("/")):
            raise ValueError("接口路径不能包含 . 或 ..")
        value = parsed.path.lstrip("/")
        if value.startswith("api/"):
            value = "/" + value
        elif value.startswith(("robot/", "control/", "calibration/", "script/")):
            value = "/api/" + value
        else:
            value = "/api/robot/" + value
        return value + ("?" + parsed.query if parsed.query else "")

    def request(self, path, body=None, headers=None, timeout=None, checked=True,
                *, method=None, params=None):
        """Low-level compatibility API: returns the complete response envelope.

        Explicit headers replace automatic auth headers (useful for auth tests).
        Normally use get/post instead. There are no automatic request retries.
        """
        path = self._path(path)
        method = (method or ("GET" if body is None else "POST")).upper()
        if params:
            def value(item):
                return str(item).lower() if isinstance(item, bool) else item
            query = {key: [value(item) for item in val] if isinstance(val, (list, tuple))
                     else value(val) for key, val in params.items() if val is not None}
            path += ("&" if "?" in path else "?") + urllib.parse.urlencode(query, doseq=True)
        if (headers is None and method != "GET" and self.heartbeat_error is not None and
                path.split("?", 1)[0] not in ("/api/control/acquire", "/api/control/takeover", "/api/control/release")):
            raise self.heartbeat_error
        request_headers = {"Accept": "application/json"}
        if headers is None:
            if self.token:
                request_headers[TOKEN_HEADER] = self.token
                request_headers[CLIENT_HEADER] = self.client_id
        else:
            request_headers.update(headers)
        data = None
        if body is not None:
            request_headers["Content-Type"] = "application/json"
            data = jsonlib.dumps(body, allow_nan=False).encode("utf-8")
        request = urllib.request.Request(self.base_url + path, data=data,
                                         headers=request_headers, method=method)
        request_timeout = self.timeout if timeout is None else timeout
        if not math.isfinite(request_timeout) or request_timeout <= 0:
            raise ValueError("timeout 必须是正的有限秒数")
        try:
            try:
                response = self.http.open(request, timeout=request_timeout)
            except urllib.error.HTTPError as error:
                response = error  # Preserve JSON business errors on HTTP 4xx/5xx.
            with response:
                status = response.status
                payload = response.read()
        except (urllib.error.URLError, TimeoutError, OSError) as error:
            raise TransportError("{} {}: 网络请求失败：{}；不会自动重发，请核对控制器状态。".format(
                method, path, error)) from error
        try:
            result = jsonlib.loads(payload)
        except (ValueError, UnicodeError) as error:
            raise TransportError("{}: HTTP {}，响应不是 JSON（模型/网格下载不使用此 JSON 客户端）".format(
                path, status)) from error
        if not isinstance(result, dict) or "success" not in result or "code" not in result:
            raise TransportError("{}: HTTP {}，响应不是 ROCOS 标准 JSON".format(path, status))
        if checked and (result["success"] is not True or result["code"] != 0):
            raise ApiError(path, result, status)
        if not 200 <= status < 300 and result["success"] is True:
            raise TransportError("{}: HTTP {} 与成功响应不一致".format(path, status))
        return result

    def get(self, path="state", params=None, *, raw=False, checked=True, timeout=None):
        """GET JSON. Defaults to data; raw=True/checked=False returns full envelope."""
        result = self.request(path, params=params, method="GET", checked=checked, timeout=timeout)
        return result if raw or not checked else result.get("data")

    def post(self, path, json=None, *, raw=False, checked=True, timeout=None):
        """POST a dict as JSON, with automatic control headers."""
        self._ensure_control(path)
        result = self.request(path, {} if json is None else json, method="POST",
                              checked=checked, timeout=timeout)
        return result if raw or not checked else result.get("data")

    def state(self):
        return self.get("state")

    get_state = state

    def get_joints(self):
        return [joint["position"] for joint in self.state()["joint_states"]]

    getjoints = get_joints
    get_joint_positions = get_joints

    @staticmethod
    def _select_joint(values, joint):
        if joint is None:
            return values
        if isinstance(joint, bool) or not isinstance(joint, int):
            raise TypeError("joint 必须是从 0 开始的整数索引")
        if not 0 <= joint < len(values):
            raise IndexError("joint 索引应在 0 到 {} 之间".format(len(values) - 1))
        return values[joint]

    def get_joint_position(self, joint=None):
        """All joint positions, or one zero-based joint, in radians."""
        return self._select_joint(self.get_joints(), joint)

    getjointposition = get_joint_position

    def get_joint_velocities(self):
        return [joint["velocity"] for joint in self.state()["joint_states"]]

    def get_joint_velocity(self, joint=None):
        return self._select_joint(self.get_joint_velocities(), joint)

    getjointvelocity = get_joint_velocity

    def get_joint_torques(self):
        return [joint["torque"] for joint in self.state()["joint_states"]]

    def get_joint_torque(self, joint=None):
        return self._select_joint(self.get_joint_torques(), joint)

    getjointtorque = get_joint_torque

    def get_flange(self):
        """Flange pose from the controller's kinematic base, without tool offset."""
        return Pose.from_dict(self.state()["flange"])

    getflange = get_flange

    def get_pose(self):
        """Current TCP pose, including the controller's configured tool offset."""
        return Pose.from_dict(self.state()["tool"])

    getpose = get_pose
    get_tool = get_pose
    gettool = get_pose

    def get_info(self):
        return self.get("info")

    def is_moving(self):
        return self.state()["motion_busy"]

    def is_enabled(self):
        return self.state()["is_enabled"]

    def _ensure_control(self, path):
        if self._path(path).startswith("/api/control/"):
            return
        if self.heartbeat_error:
            raise self.heartbeat_error
        if self.auto_acquire and self.token is None:
            self.connect()

    def _claim(self, action, keep_alive):
        result = self.request("control/" + action, {
            "client_name": self.client_name, "client_id": self.client_id}, headers={})
        self.token = result["data"]["token"]
        self.client_id = result["data"]["client_id"]
        self.heartbeat_error = None
        if keep_alive:
            self.start_heartbeat(result["data"])
        return result["data"]

    def acquire(self, keep_alive=True):
        return self._claim("acquire", keep_alive)

    def takeover(self, keep_alive=True):
        """Explicit takeover only; ordinary acquisition never steals control."""
        self.stop_heartbeat()
        return self._claim("takeover", keep_alive)

    def connect(self, *, takeover=False, keep_alive=True):
        if takeover:
            return self.takeover(keep_alive)
        if self.token:
            try:
                owner = self.post("control/heartbeat")
            except ApiError as error:
                if error.code not in (3006, 3007) or error.error_name == "FunctionNotSupported":
                    raise
                # Explicit reconnect is a new user action; never retry a motion.
                self.stop_heartbeat()
                self.token = None
                self.heartbeat_error = None
            else:
                if keep_alive and not (self._heartbeat_thread and self._heartbeat_thread.is_alive()):
                    self.start_heartbeat(owner)
                elif not keep_alive:
                    self.stop_heartbeat()
                return owner
        return self.acquire(keep_alive)

    def start_heartbeat(self, owner):
        self.stop_heartbeat()
        interval = owner.get("heartbeat_interval_seconds", 5)
        if not math.isfinite(interval) or interval <= 0:
            raise ValueError("无效的服务端心跳间隔")
        stop_event = threading.Event()
        self._heartbeat_stop = stop_event
        self.heartbeat_error = None
        headers = {TOKEN_HEADER: self.token, CLIENT_HEADER: self.client_id}

        def renew():
            while not stop_event.wait(interval):
                try:
                    self.request("control/heartbeat", {}, headers=headers)
                except ApiError as error:
                    if error.code in (3006, 3007):
                        self.heartbeat_error = error
                        _log.warning("控制权已失效，请重新申请：%s", error)
                        return
                    _log.warning("心跳失败：%s", error)
                except (RuntimeError, ValueError) as error:
                    _log.warning("心跳请求失败，将重试：%s", error)

        self._heartbeat_thread = threading.Thread(target=renew, daemon=True, name="rocos-heartbeat")
        self._heartbeat_thread.start()

    def stop_heartbeat(self):
        self._heartbeat_stop.set()
        if self._heartbeat_thread:
            self._heartbeat_thread.join(timeout=self.timeout + 1)
            if self._heartbeat_thread.is_alive():
                raise TransportError("心跳请求未结束，暂不释放或替换凭据")
            self._heartbeat_thread = None

    def release(self):
        self.stop_heartbeat()
        if not self.token:
            return None
        try:
            result = self.post("control/release")
        except ApiError as error:
            if error.code in (3006, 3007) and error.error_name != "FunctionNotSupported":
                self.token = None
            raise
        self.token = None
        self.heartbeat_error = None
        return result

    def close(self):
        try:
            self.release()
        except ApiError as error:
            if error.code not in (3006, 3007) or error.error_name == "FunctionNotSupported":
                raise

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if exc_type is None:
            self.close()
        else:
            try:
                self.close()
            except Exception:
                _log.exception("异常退出时释放控制权失败")
        return False

    def enable(self):
        return self.post("enable")

    def disable(self):
        return self.post("disable")

    def set_work_mode(self, mode):
        return self.post("workmode", {"mode": mode})

    def wait_move(self, timeout=20):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout 必须是正的有限秒数")
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TransportError("等待运动超时，未确认运动完成；不会自动重发运动")
            interval = min(2.0, remaining)
            result = self.request("wait_move", {"timeout": interval},
                                  timeout=interval + self.timeout, checked=False)
            if result["success"] is False and result["code"] == 5002:
                continue
            if result["success"] is not True or result["code"] != 0:
                raise ApiError("/api/robot/wait_move", result)
            return result["data"]

    def _move(self, name, body, velocity, acceleration, timeout, wait, parameters):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout 必须是正的有限秒数")
        payload = dict(body)
        payload.setdefault("velocity", velocity)
        payload.setdefault("acceleration", acceleration)
        payload.update(parameters)
        # Reject non-JSON/NaN inputs before acquiring control or changing mode.
        jsonlib.dumps(payload, allow_nan=False)
        self._ensure_control(name)
        state = self.state()
        if not state["is_enabled"]:
            raise RuntimeError("机器人未使能，请先调用 robot.enable()")
        if state["work_mode"] != "position":
            self.set_work_mode("position")
        # Always submit asynchronously, so Python controls waiting and heartbeat.
        payload["asynchronous"] = True
        accepted = self.post(name, payload)
        if not wait:
            return accepted
        result = self.wait_move(timeout)
        if result.get("motion_outcome") == "stopped":
            raise RuntimeError("运动被停止，未确认到达目标")
        return result

    def movej(self, joints, velocity=0.1, acceleration=0.3, timeout=20, *, wait=True, **parameters):
        """Joint vector or complete MoveJ dict; blocking by default; angles in rad."""
        body = dict(joints) if isinstance(joints, dict) else {"joints": list(joints)}
        result = self._move("movej", body, velocity, acceleration, timeout, wait, parameters)
        if wait:
            actual = [joint["position"] for joint in self.state()["joint_states"]]
            if len(actual) != len(body["joints"]) or any(
                    abs(a - b) > 1e-4 for a, b in zip(actual, body["joints"])):
                raise RuntimeError("运动结束但关节位置与目标不符：{}".format(actual))
        return result

    movrj = movej  # Convenience alias for the spelling used in the original request.

    def movel(self, pose, velocity=0.05, acceleration=0.1, timeout=20, *, wait=True, **parameters):
        """Straight-line flange target for the current legacy Robot controller."""
        value = pose.to_dict() if isinstance(pose, Pose) else pose
        return self._move("movel", {"pose": value}, velocity, acceleration, timeout, wait, parameters)

    def movej_ik(self, pose, velocity=0.1, acceleration=0.3, timeout=20, *, wait=True, **parameters):
        """Joint move to a flange target via the controller's inverse kinematics."""
        value = pose.to_dict() if isinstance(pose, Pose) else pose
        return self._move("movej_ik", {"pose": value}, velocity, acceleration, timeout, wait, parameters)

    def stop(self, *, wait=True, timeout=20):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout 必须是正的有限秒数")
        result = self.post("stop")
        if wait:
            deadline = time.monotonic() + timeout
            while self.state()["motion_busy"]:
                if time.monotonic() >= deadline:
                    raise TransportError("停止后仍在运动，未确认停止")
                time.sleep(0.02)
        return result


class RocosClient(Robot):
    """Backward-compatible name and constructor for the existing CLI examples."""

    def __init__(self, base_url, timeout=5):
        super().__init__(base_url, timeout=timeout, client_name="python-http-example", auto_acquire=False)
