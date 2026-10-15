#!/usr/bin/env python3
"""ROCOS REST client example; Python 3 standard library only.

python3 http_control.py --base-url http://192.168.1.117:8080 state
python3 http_control.py --base-url http://192.168.1.117:8080 auth-test
python3 http_control.py --base-url http://192.168.1.117:8080 idle-test
python3 http_control.py --base-url http://192.168.1.117:8080 move-demo
"""

import argparse
import json
import math
import sys
import threading
import time
import urllib.error
import urllib.request
import uuid


TOKEN_HEADER = "X-Rocos-Control-Token"
CLIENT_HEADER = "X-Rocos-Client-Id"


class ApiError(RuntimeError):
    def __init__(self, path, response):
        self.response = response
        super().__init__("{}: code={}, error_name={}, message={}, detail={}".format(
            path, response.get("code"), response.get("error_name", ""),
            response.get("message"), (response.get("data") or {}).get("error")))


class RocosClient:
    def __init__(self, base_url, timeout=5):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.client_id = "python-" + str(uuid.uuid4())
        self.token = None
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread = None
        self.heartbeat_error = None
        # LAN requests must go directly to the controller, not through HTTP_PROXY.
        self.http = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def request(self, path, body=None, headers=None, timeout=None, checked=True):
        """Explicit headers replace auth headers, allowing negative auth tests."""
        request_headers = {"Content-Type": "application/json"}
        if headers is None:
            if self.token:
                request_headers[TOKEN_HEADER] = self.token
                request_headers[CLIENT_HEADER] = self.client_id
        else:
            request_headers.update(headers)
        data = None if body is None else json.dumps(body, allow_nan=False).encode("utf-8")
        request = urllib.request.Request(
            self.base_url + path, data=data, headers=request_headers,
            method="GET" if body is None else "POST")
        try:
            with self.http.open(request, timeout=timeout or self.timeout) as response:
                result = json.load(response)
        except urllib.error.HTTPError as error:
            raise RuntimeError("{}: HTTP {} {}".format(path, error.code, error.reason)) from error
        except (urllib.error.URLError, TimeoutError, OSError) as error:
            raise RuntimeError("{}: 网络请求失败：{}。不会自动重发运动指令。".format(
                path, error)) from error
        if not isinstance(result, dict) or "success" not in result or "code" not in result:
            raise RuntimeError("{}: 响应不是 ROCOS 标准 JSON：{!r}".format(path, result))
        # Business failures can arrive with HTTP 200, including positive error codes.
        if checked and (result["success"] is not True or result["code"] != 0):
            raise ApiError(path, result)
        return result

    def state(self):
        return self.request("/api/robot/state")["data"]

    def acquire(self, keep_alive=True):
        result = self.request("/api/control/acquire", {
            "client_name": "python-http-example", "client_id": self.client_id}, headers={})
        self.token = result["data"]["token"]
        # Always use the returned ID, including when the server generated it.
        self.client_id = result["data"]["client_id"]
        print("已获得控制权，client_id={}，TTL={} 秒".format(
            self.client_id, result["data"]["ttl_seconds"]))
        if keep_alive:
            self.start_heartbeat(result["data"])
        return result["data"]

    def start_heartbeat(self, owner):
        self.stop_heartbeat()
        self._heartbeat_stop.clear()
        self.heartbeat_error = None
        headers = {TOKEN_HEADER: self.token, CLIENT_HEADER: self.client_id}
        interval = owner.get("heartbeat_interval_seconds", 5)

        def renew():
            while not self._heartbeat_stop.wait(interval):
                try:
                    self.request("/api/control/heartbeat", {}, headers=headers)
                except ApiError as error:
                    if error.response["code"] in (3006, 3007):
                        self.heartbeat_error = error
                        print("控制权已失效，请重新申请：{}".format(error), file=sys.stderr)
                        return
                    print("心跳失败：{}".format(error), file=sys.stderr)
                except (RuntimeError, ValueError) as error:
                    print("心跳网络失败，将重试：{}".format(error), file=sys.stderr)

        self._heartbeat_thread = threading.Thread(target=renew, daemon=True)
        self._heartbeat_thread.start()

    def stop_heartbeat(self):
        self._heartbeat_stop.set()
        if self._heartbeat_thread:
            self._heartbeat_thread.join(timeout=self.timeout + 1)
            if self._heartbeat_thread.is_alive():
                raise RuntimeError("心跳请求未结束，暂不释放或替换凭据")
            self._heartbeat_thread = None

    def release(self):
        self.stop_heartbeat()
        if self.token:
            self.request("/api/control/release", {})
            self.token = None
            print("已释放控制权")

    def wait_move(self, timeout=20):
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError("等待运动超时，未确认运动完成")
            # Each protected request renews the token. Public GETs do not renew it.
            interval = min(2.0, remaining)
            result = self.request("/api/robot/wait_move", {"timeout": interval},
                                  timeout=interval + self.timeout, checked=False)
            if result["success"] is False and result["code"] == 5002:
                continue
            if result["success"] is not True or result["code"] != 0:
                raise ApiError("/api/robot/wait_move", result)
            return result["data"]

    def movej(self, joints, velocity=0.1, acceleration=0.3, timeout=20):
        # Deliberately omit jerk/max_vel: do not overwrite shared motion limits.
        self.request("/api/robot/movej", {
            "joints": joints, "velocity": velocity, "acceleration": acceleration,
            "asynchronous": True})
        result = self.wait_move(timeout)
        if result.get("motion_outcome") == "stopped":
            raise RuntimeError("运动被停止，未到达目标")
        actual = [joint["position"] for joint in self.state()["joint_states"]]
        if len(actual) != len(joints) or any(abs(a - b) > 1e-4 for a, b in zip(actual, joints)):
            raise RuntimeError("运动结束但关节位置与目标不符：{}".format(actual))


def auth_test(client):
    """An empty MoveJ body never reaches motion planning, even with valid auth."""
    def expect(label, headers, code, success=False, path="/api/robot/movej", body=None):
        result = client.request(path, {} if body is None else body,
                                headers=headers, checked=False)
        if result["code"] != code or result["success"] is not success:
            raise RuntimeError("鉴权测试失败 {}：{}".format(label, result))
        print("PASS {:<24} code={} {}".format(label, code, result["message"]))

    expect("缺少 token", {}, 3006)
    expect("无效 token", {TOKEN_HEADER: "invalid-example-token",
                           CLIENT_HEADER: client.client_id}, 3007)
    client.acquire()
    token = client.token
    client_id = client.client_id
    try:
        expect("有效 token，缺少 client_id", {TOKEN_HEADER: token}, 3007)
        expect("有效 token，错误 client_id", {TOKEN_HEADER: token,
                                           CLIENT_HEADER: "another-client"}, 3007)
        # 1001 indicates auth succeeded, then the empty motion body was rejected.
        expect("两请求头正确，空运动参数", {TOKEN_HEADER: token,
                                         CLIENT_HEADER: client_id}, 1001)
        expect("第二客户端申请控制权", {}, 3008, path="/api/control/acquire",
               body={"client_name": "python-competing-client", "client_id": str(uuid.uuid4())})
    finally:
        client.release()
    expect("释放后的旧 token", {TOKEN_HEADER: token, CLIENT_HEADER: client_id}, 3007)
    print("鉴权测试通过；未执行运动，未切换模式或修改运动限制。")


def idle_test(client):
    """Keep the acquired credentials without sending requests for TTL + 10 s."""
    owner = client.acquire(keep_alive=False)
    try:
        before = client.request("/api/robot/movej", {}, checked=False)
        if before["success"] is not False or before["code"] != 1001:
            raise RuntimeError("静默前鉴权未通过：{}".format(before))
        print("PASS 静默前：鉴权通过，空 MoveJ 被参数校验拒绝，code=1001", flush=True)
        duration = owner["ttl_seconds"] + 10
        started = time.monotonic()
        print("开始静默 {} 秒；期间不发送任何 HTTP 请求。".format(duration), flush=True)
        while time.monotonic() - started < duration:
            time.sleep(max(0, min(30, duration - (time.monotonic() - started))))
            print("静默已持续 {:.1f} 秒".format(time.monotonic() - started), flush=True)
        after = client.request("/api/robot/movej", {}, checked=False)
        if after["success"] is not False or after["code"] != 3007:
            raise RuntimeError("静默超时后的令牌状态与预期不符：{}".format(after))
        print("PASS 静默后旧 token：code={}，message={}".format(
            after["code"], after["message"]), flush=True)
        client.token = None  # The old credentials no longer own the control lock.
        client.acquire()
        renewed = client.request("/api/robot/movej", {}, checked=False)
        if renewed["success"] is not False or renewed["code"] != 1001:
            raise RuntimeError("重新申请后鉴权未恢复：{}".format(renewed))
        print("PASS 重新申请：新 token 鉴权通过，code=1001", flush=True)
    finally:
        try:
            client.release()
        except ApiError as error:
            if error.response["code"] != 3007:
                raise
            client.token = None  # May have expired if interrupted during the wait.
    print("静默超时测试通过；未执行运动、切换模式或修改运动限制。")


def hold_test(client):
    """No robot commands; heartbeats alone maintain the lease past its original TTL."""
    owner = client.acquire()
    duration = owner["ttl_seconds"] + 10
    try:
        print("保持会话 {} 秒，只发送心跳，不发送运动指令。".format(duration), flush=True)
        started = time.monotonic()
        while time.monotonic() - started < duration:
            time.sleep(max(0, min(5, duration - (time.monotonic() - started))))
            if client.heartbeat_error:
                raise client.heartbeat_error
        result = client.request("/api/control/heartbeat", {})
        print("PASS 超过原始 TTL 后仍持有控制权，code={}".format(result["code"]))
    finally:
        client.release()


def move_demo(client, args):
    initial = client.state()
    if initial.get("hw_state", {}).get("hw_type") != 1:
        raise RuntimeError("move-demo 仅用于 HardwareSim 仿真；当前不是仿真硬件")
    if initial["motion_busy"]:
        raise RuntimeError("机器人正在运动，请结束当前运动后再运行示例")
    original = [joint["position"] for joint in initial["joint_states"]]
    if not 0 <= args.joint < len(original):
        raise RuntimeError("joint 索引应在 0 到 {} 之间".format(len(original) - 1))
    if any(joint["mode"] != "CSP" for joint in initial["joint_states"]):
        raise RuntimeError("示例要求关节已处于 CSP；示例不切换 CSP/CSV/CST")
    target = original.copy()
    target[args.joint] += args.delta
    client.acquire()
    cleanup_errors = []
    try:
        # Recheck after acquisition; gRPC clients are outside the HTTP control lock.
        if client.state()["motion_busy"]:
            raise RuntimeError("申请控制权期间出现新运动，请停止其他客户端操作")
        if initial["work_mode"] != "position":
            client.request("/api/robot/workmode", {"mode": "position"})
        if not initial["is_enabled"]:
            client.request("/api/robot/enable", {})
        print("初始关节位置 [rad]：", original)
        print("关节 {} 相对运动 {} rad，velocity={} rad/s".format(
            args.joint, args.delta, args.velocity))
        client.movej(target, args.velocity, args.acceleration, args.move_timeout)
        print("已到达目标：", [j["position"] for j in client.state()["joint_states"]])
        client.movej(original, args.velocity, args.acceleration, args.move_timeout)
        print("已回到初始位置")
    finally:
        # A timed-out POST may already have started motion; try stopping before release.
        # On interruption/failure, stop in place rather than issue another MoveJ.
        def cleanup(label, action):
            try:
                action()
            except Exception as error:
                cleanup_errors.append("{}：{}".format(label, error))

        def stop_and_wait():
            client.request("/api/robot/stop", {})
            end = time.monotonic() + args.move_timeout
            while client.state()["motion_busy"]:
                if time.monotonic() >= end:
                    raise RuntimeError("停止后仍在运动，未确认停止")
                time.sleep(0.05)

        cleanup("停止", stop_and_wait)
        if not cleanup_errors:
            if not initial["is_enabled"]:
                cleanup("恢复未使能状态", lambda: client.request("/api/robot/disable", {}))
            if initial["work_mode"] != "position":
                cleanup("恢复工作模式", lambda: client.request(
                    "/api/robot/workmode", {"mode": initial["work_mode"]}))
                if not initial.get("control_active", False):
                    cleanup("恢复模式的停止状态", stop_and_wait)
        cleanup("释放控制权", client.release)
        if cleanup_errors:
            raise RuntimeError("清理未全部完成；请检查控制器状态：" + "；".join(cleanup_errors))


def main():
    parser = argparse.ArgumentParser(description="ROCOS HTTP 跨电脑控制与鉴权示例（无需 pip 安装）")
    parser.add_argument("--base-url", default="http://127.0.0.1:8080",
                        help="控制电脑地址，如 http://192.168.1.117:8080")
    parser.add_argument("action", choices=("state", "auth-test", "idle-test", "hold-test", "move-demo"))
    parser.add_argument("--joint", type=int, default=0, help="从 0 开始的关节索引")
    parser.add_argument("--delta", type=float, default=0.03, help="相对角度，rad；默认约 1.72°")
    parser.add_argument("--velocity", type=float, default=0.1, help="MoveJ 速度，rad/s")
    parser.add_argument("--acceleration", type=float, default=0.3, help="MoveJ 加速度，rad/s²")
    parser.add_argument("--move-timeout", type=float, default=20, help="单次运动超时，秒")
    args = parser.parse_args()
    if not args.base_url.startswith(("http://", "https://")):
        parser.error("base-url 必须以 http:// 或 https:// 开头")
    if any(not math.isfinite(v) for v in (
            args.delta, args.velocity, args.acceleration, args.move_timeout)):
        parser.error("运动参数必须是有限数值")
    if min(args.velocity, args.acceleration, args.move_timeout) <= 0:
        parser.error("速度、加速度和超时必须大于 0")
    try:
        client = RocosClient(args.base_url)
        if args.action == "state":
            print(json.dumps(client.state(), ensure_ascii=False, indent=2))
        elif args.action == "auth-test":
            auth_test(client)
        elif args.action == "idle-test":
            idle_test(client)
        elif args.action == "hold-test":
            hold_test(client)
        else:
            move_demo(client, args)
        return 0
    except KeyboardInterrupt:
        print("操作已中断；如在运动阶段，示例已尝试停止并释放控制权。", file=sys.stderr)
        return 130
    except (RuntimeError, ValueError, KeyError) as error:
        print("失败：{}".format(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
