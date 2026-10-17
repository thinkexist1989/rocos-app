"""Python 页面服务端与真实 C++ 可执行文件的 HardwareSim 联调；不连接真机。"""
import argparse
import json
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    tcp_port, web_port = free_port(), free_port()
    while web_port == tcp_port:
        web_port = free_port()
    base = f"http://127.0.0.1:{web_port}"
    directory = Path(tempfile.mkdtemp(prefix="rocos-tcp-page-test-"))
    print("联调日志：", directory, flush=True)
    processes, logs = [], []

    def launch(name, command):
        log = open(directory / (name + ".log"), "w")
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    def state():
        with urlopen(base + "/api/state", timeout=2) as response:
            return json.load(response)

    def action(name, **kwargs):
        request = Request(base + "/api/action", json.dumps(dict(action=name, **kwargs)).encode(),
                          {"Content-Type": "application/json"})
        with urlopen(request, timeout=2) as response:
            assert json.load(response)["ok"]

    def wait_for(predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            current = state()
            if predicate(current):
                return current
            time.sleep(.05)
        raise AssertionError("状态未达到预期：" + json.dumps(state(), ensure_ascii=False))

    try:
        launch("server", [sys.executable, str(root / "tools/joint_tcp_test_server.py"),
                          "--tcp-port", str(tcp_port), "--web-port", str(web_port)])
        deadline = time.monotonic() + 5
        while True:
            try:
                assert not state()["connected"]
                break
            except URLError:
                assert time.monotonic() < deadline, "Python 服务端未启动"
                time.sleep(.05)
        with urlopen(base) as response:
            page = response.read().decode()
            assert "关节 TCP 测试台" in page and 'id="chart"' in page
        client = launch("client", [str(args.binary.resolve()), "--sim", "--config=" + str(args.config.resolve()),
                                   "--host=127.0.0.1", "--port=" + str(tcp_port)])
        initial = wait_for(lambda s: s["feedback"] is not None)
        assert not initial["sending"] and initial["command"] is None, "连接后不应自动发运动指令"
        try:
            action("start", mode="position", period=0)
            raise AssertionError("非法周期未拒绝")
        except HTTPError as error:
            assert error.code == 400
        baseline = initial["feedback"]["q"]
        action("start", mode="position", period=10)
        deadline = time.monotonic() + 10.8
        commanded, measured = [], []
        while time.monotonic() < deadline:
            current = state()
            assert client.poll() is None, "C++ 客户端提前退出"
            if current["command"]:
                q = current["command"]["q"]
                assert -1 <= q[3] <= 1
                assert all(q[j] == baseline[j] for j in range(7) if j != 3)
                commanded.append(q[3])
                measured.append(current["feedback"]["q"][3])
            time.sleep(.05)
        assert min(commanded) < -.99 and max(commanded) > .99, "正弦未覆盖 -1～1"
        assert min(measured) < -.9 and max(measured) > .9, "CSP 未跟随正弦"
        assert 8 <= current["tx_hz"] <= 12 and 70 <= current["rx_hz"] <= 130, "收发频率异常"
        assert current["feedback"]["tau_calculated"] is None
        action("hold")
        current = wait_for(lambda s: not s["running"])
        target = current["command"]["q"]
        time.sleep(1.3)  # 超过默认 1 秒看门狗，验证暂停仍发心跳指令。
        assert client.poll() is None and state()["command"]["q"] == target
        for mode in ("impedance", "impedance_friction"):
            action("start", mode=mode, period=10)
            current = wait_for(lambda s: s["command"]["mode"] == mode and
                               s["feedback"]["tau_calculated"] is not None)
            if mode == "impedance":
                assert current["feedback"]["tau_friction"] == [0] * 7
            else:
                assert len(current["feedback"]["tau_friction"]) == 7
            action("hold")
        action("stop")
        current = wait_for(lambda s: s["command"]["mode"] == "stop" and
                           s["feedback"]["tau_calculated"] is None, timeout=12)
        assert not current["running"]
        action("disconnect")
        wait_for(lambda s: not s["connected"])
        assert client.wait(timeout=5) == 1, "断线应触发故障退出"
        # 同一个服务端接受新客户端，但不沿用旧目标，也不会自动开始。
        client = launch("reconnected", [str(args.binary.resolve()), "--sim", "--config=" + str(args.config.resolve()),
                                        "--port=" + str(tcp_port)])
        current = wait_for(lambda s: s["connected"] and s["feedback"] is not None)
        assert not current["sending"] and current["command"] is None
        action("disconnect")
        assert client.wait(timeout=5) == 1
        print("PASS：正弦幅值/其余六轴保持/四模式/10Hz+100Hz/暂停心跳/断线/重新连接", flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        for log in logs:
            log.close()


if __name__ == "__main__":
    main()
