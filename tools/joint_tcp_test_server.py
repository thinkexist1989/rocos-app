#!/usr/bin/env python3
"""本地 TCP 上位机：10 Hz 发送 joint_4 正弦目标，接收客户端 100 Hz 状态。仅用标准库。"""
import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import socket
import threading
import time
from urllib.parse import urlsplit


MODES = {"position", "impedance", "impedance_friction"}


class TestServer:
    def __init__(self):
        self.lock = threading.Lock()
        self.shutdown = threading.Event()
        self.connection = None
        self.peer = ""
        self.feedback = None
        self.command = None
        self.target = None
        self.mode = "position"
        self.running = False
        self.period = 10.0
        self.phase = 0.0
        self.started = 0.0
        self.connected_at = 0.0
        self.last_received = 0.0
        self.last_history = 0.0
        self.sent = deque(maxlen=1000)
        self.received = deque(maxlen=1000)
        self.history = deque(maxlen=400)
        self.error = ""

    def action(self, data):
        with self.lock:
            if self.connection is None:
                raise ValueError("请先启动并连接 C++ 客户端")
            action = data.get("action")
            if action == "disconnect":
                # 主动断线会触发客户端故障下使能；不会自动重新启动客户端。
                self.connection.shutdown(socket.SHUT_RDWR)
                return
            if self.feedback is None or time.monotonic() - self.last_received > 0.5:
                raise ValueError("尚无有效状态，或状态超过 0.5 秒未更新")
            if action == "start":
                mode, period = data.get("mode"), data.get("period", 10)
                if mode not in MODES:
                    raise ValueError("正弦模式只能选择 position / impedance / impedance_friction")
                if type(period) not in (int, float) or not math.isfinite(period) or not 4 <= period <= 120:
                    raise ValueError("正弦周期需在 4～120 秒之间")
                # 保留六轴目标；首次启动取实际姿态。反正弦确定起始相位，避免位置跳变。
                target = list(self.target if self.target is not None else self.feedback["q"])
                if not -1 <= target[3] <= 1:
                    raise ValueError("joint_4 起始角度需在 [-1, 1] rad 内，请先在上层移动到该范围")
                self.target = target
                self.phase = math.asin(target[3])
                self.started = time.monotonic()
                self.period, self.mode, self.running = float(period), mode, True
            elif action == "hold":
                if self.target is None:
                    raise ValueError("尚未开始发送指令")
                self.running = False
            elif action == "stop":
                # stop 的 q 也必须存在：切 CSP，用 MoveJ 停靠到点击时的实际姿态。
                self.target = list(self.feedback["q"])
                self.mode, self.running = "stop", False
            else:
                raise ValueError("未知操作")

    def next_command(self, now):
        """调用方持锁；暂停后仍持续发送同一目标，满足客户端指令看门狗。"""
        if self.target is None:
            return None
        if self.running:
            self.target[3] = math.sin(self.phase + 2 * math.pi * (now - self.started) / self.period)
        return dict(frame_header="0xAA56", timestamp_us=time.time_ns() // 1000,
                    mode=self.mode, q=list(self.target))

    def record_feedback(self, frame, now):
        if not isinstance(frame, dict) or frame.get("frame_header") != "0xAA55":
            raise ValueError("状态帧头应为 0xAA55")
        if type(frame.get("timestamp_us")) is not int or frame["timestamp_us"] < 0:
            raise ValueError("状态时间戳必须为非负整数")
        for key in ("q", "dq", "tau_actual", "tau_calculated", "tau_friction"):
            value = frame.get(key)
            if key in ("tau_calculated", "tau_friction") and key in frame and value is None:
                continue
            if not isinstance(value, list) or len(value) != 7 or any(
                    type(x) not in (int, float) or not math.isfinite(x) for x in value):
                raise ValueError("状态字段无效：" + key)
        with self.lock:
            self.feedback, self.last_received = frame, now
            self.received.append(now)
            if now - self.last_history >= 0.05:
                reference = self.command["q"][3] if self.command else None
                self.history.append([now - self.connected_at, reference, frame["q"][3]])
                self.last_history = now

    def state(self):
        with self.lock:
            now = time.monotonic()
            for times in (self.sent, self.received):
                while times and times[0] < now - 1:
                    times.popleft()
            return dict(connected=self.connection is not None, peer=self.peer,
                        running=self.running, sending=self.target is not None,
                        mode=self.mode, period=self.period, tx_hz=len(self.sent), rx_hz=len(self.received),
                        feedback_age_ms=(now - self.last_received) * 1000 if self.feedback else None,
                        command=self.command, feedback=self.feedback, history=list(self.history), error=self.error)

    def serve_client(self, connection):
        connection.settimeout(0.005)
        buffer = b""
        next_send = time.monotonic()
        while not self.shutdown.is_set():
            now = time.monotonic()
            if now >= next_send:
                with self.lock:
                    command = self.next_command(now)
                if command is not None:
                    payload = (json.dumps(command, allow_nan=False, separators=(",", ":")) + "\n").encode()
                    # 慢客户端不能无限阻塞后台发送线程。
                    connection.settimeout(0.2)
                    connection.sendall(payload)
                    connection.settimeout(0.005)
                    with self.lock:
                        self.command = command
                        self.sent.append(time.monotonic())
                next_send += 0.1
                if next_send < now:
                    next_send = now + 0.1  # 调度延误后不突发补发旧轨迹点。
            try:
                chunk = connection.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                return
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                if len(line) > 16384:
                    raise ValueError("状态帧超过 16 KiB")
                self.record_feedback(json.loads(line), time.monotonic())
            if len(buffer) > 16384:
                raise ValueError("状态帧超过 16 KiB")

    def listen(self, listener):
        listener.settimeout(0.2)
        while not self.shutdown.is_set():
            try:
                connection, address = listener.accept()
            except socket.timeout:
                continue
            with connection:
                with self.lock:
                    self.connection, self.peer = connection, f"{address[0]}:{address[1]}"
                    self.feedback = self.command = self.target = None
                    self.mode, self.running, self.error = "position", False, ""
                    self.sent.clear()
                    self.received.clear()
                    self.history.clear()
                    self.last_received = self.last_history = 0
                    self.connected_at = time.monotonic()
                print("客户端已连接：", self.peer, flush=True)
                try:
                    self.serve_client(connection)
                except (OSError, ValueError) as error:
                    with self.lock:
                        self.error = str(error)
                finally:
                    with self.lock:
                        self.connection = None
                        self.running = False
                        self.target = None
                    print("客户端已断开，重新连接后需手动开始", flush=True)


def http_handler(server):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, body, content_type="application/json; charset=utf-8"):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/":
                self.reply(200, Path(__file__).with_name("joint_tcp_test.html").read_bytes(), "text/html; charset=utf-8")
            elif path == "/api/state":
                self.reply(200, json.dumps(server.state(), allow_nan=False).encode())
            else:
                self.reply(404, b'{"error":"Not found"}')

        def do_POST(self):
            if self.path != "/api/action":
                return self.reply(404, b'{"error":"Not found"}')
            try:
                # 本工具没有远程登录；只接受同源页面 / 本地脚本请求。
                origin = self.headers.get("Origin")
                if origin and origin != "http://" + self.headers.get("Host", ""):
                    return self.reply(403, b'{"error":"Origin rejected"}')
                length = int(self.headers.get("Content-Length", 0))
                if not 0 < length <= 1024:
                    raise ValueError("操作内容长度无效")
                data = json.loads(self.rfile.read(length))
                if not isinstance(data, dict):
                    raise ValueError("操作内容必须是 JSON 对象")
                server.action(data)
                self.reply(200, b'{"ok":true}')
            except (ValueError, OSError) as error:
                self.reply(400, json.dumps(dict(error=str(error))).encode())

        def log_message(self, *_):
            pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tcp-host", default="127.0.0.1")
    parser.add_argument("--tcp-port", type=int, default=9000)
    parser.add_argument("--web-host", default="127.0.0.1")
    parser.add_argument("--web-port", type=int, default=8000)
    args = parser.parse_args()
    server = TestServer()
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((args.tcp_host, args.tcp_port))
        listener.listen(1)
        with ThreadingHTTPServer((args.web_host, args.web_port), http_handler(server)) as web:
            worker = threading.Thread(target=server.listen, args=(listener,), daemon=True)
            worker.start()
            print(f"TCP: {args.tcp_host}:{args.tcp_port}  |  页面: http://{args.web_host}:{args.web_port}", flush=True)
            print("等待客户端和页面操作；默认不发送运动指令。Ctrl+C 断开连接。", flush=True)
            try:
                web.serve_forever(poll_interval=0.2)
            except KeyboardInterrupt:
                pass
            finally:
                server.shutdown.set()
                worker.join(timeout=2)


if __name__ == "__main__":
    main()
