"""Loopback TCP server + HardwareSim integration. Never connects to real hardware."""
import argparse
import json
import math
from pathlib import Path
import signal
import socket
import statistics
import subprocess
import tempfile
import time


class Session:
    def __init__(self, binary, config, directory, name, timeout=1.0):
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.listener.settimeout(8)
        self.out = open(directory / (name + ".stdout.log"), "w+")
        self.err = open(directory / (name + ".stderr.log"), "w+")
        self.process = subprocess.Popen([
            str(binary), "--sim", "--config=" + str(config), "--host=127.0.0.1",
            "--port=" + str(self.listener.getsockname()[1]), "--command_timeout=" + str(timeout),
        ], stdout=self.out, stderr=self.err)
        self.connection, _ = self.listener.accept()
        self.connection.settimeout(.02)
        self.buffer = b""
        self.frames = []
        # Wait for feedback while disabled, before sending any target.
        deadline = time.monotonic() + 5
        while len(self.frames) < 3:
            assert time.monotonic() < deadline, "No initial feedback: " + self.logs()
            self.receive()

    def logs(self):
        self.out.seek(0)
        self.err.seek(0)
        return self.out.read() + self.err.read()

    def receive(self):
        try:
            data = self.connection.recv(65536)
        except socket.timeout:
            return
        if not data:
            return
        self.buffer += data
        while b"\n" in self.buffer:
            raw, self.buffer = self.buffer.split(b"\n", 1)
            frame = json.loads(raw)
            assert frame["frame_header"] == "0xAA55"
            assert isinstance(frame["timestamp_us"], int) and frame["timestamp_us"] > 0
            for field in ["q", "dq", "tau_actual", "tau_calculated", "tau_friction"]:
                value = frame[field]
                if value is None:
                    assert field in ("tau_calculated", "tau_friction")
                else:
                    assert len(value) == 7 and all(math.isfinite(x) for x in value)
            self.frames.append((time.monotonic(), frame))

    @staticmethod
    def command(mode, q, tau=None):
        value = dict(frame_header="0xAA56", timestamp_us=time.time_ns() // 1000, mode=mode, q=q)
        if tau is not None:
            value["tau"] = tau
        return (json.dumps(value) + "\n").encode()

    def collect(self, duration, mode, q, tau=None, fragment=False, extra=b""):
        start = len(self.frames)
        deadline = time.monotonic() + duration
        next_send = time.monotonic()
        first = True
        while time.monotonic() < deadline:
            assert self.process.poll() is None, "Client exited early: " + self.logs()
            now = time.monotonic()
            if now >= next_send:
                payload = self.command(mode, q, tau)
                if fragment and first:
                    self.connection.sendall(payload[:9])
                    time.sleep(.003)
                    self.connection.sendall(payload[9:31])
                    time.sleep(.003)
                    self.connection.sendall(payload[31:] + extra)
                else:
                    self.connection.sendall(payload + (extra if first else b""))
                first = False
                next_send += .1
            self.receive()
        return self.frames[start:]

    def close(self):
        self.connection.close()
        self.listener.close()
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.out.close()
        self.err.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    binary, config = args.binary.resolve(), args.config.resolve()
    directory = Path(tempfile.mkdtemp(prefix="rocos-joint-tcp-test-"))
    print("TCP test logs:", directory, flush=True)
    session = Session(binary, config, directory, "modes")
    try:
        zero = [0.0] * 7
        target = [.05, 0, 0, 0, 0, 0, 0]
        frames = session.collect(1.0, "position", target, fragment=True,
                                 extra=b'{"mode":"bad"}\n')
        assert abs(frames[-1][1]["q"][0] - .05) < .001
        assert all(f["tau_calculated"] is None and f["tau_friction"] is None for _, f in frames)
        assert any(.001 < f["q"][0] < .049 for _, f in frames), "CSP jumped to target without interpolation"
        # Request CST while the CSP reference is still moving: exercise the braking handover.
        session.collect(.1, "position", [.3, 0, 0, 0, 0, 0, 0])
        frames = session.collect(1.2, "impedance", target, [1000000] * 7)
        baseline = frames[-1][1]["tau_calculated"]
        assert baseline is not None
        # 巨大 tau、反向 tau、缺省 tau 应得到同一力矩，不影响预置或运行。
        for tau in ([-1000000] * 7, None):
            frames = session.collect(.4, "impedance", target, tau)
            for _, frame in frames[-10:]:
                assert frame["tau_friction"] == zero
                assert frame["tau_calculated"] is not None
                assert all(abs(a - b) < .03 for a, b in zip(frame["tau_calculated"], baseline)), "Command tau affected torque"
        frames = session.collect(.7, "impedance_friction", target, [1000000] * 7)
        assert all(f["tau_friction"] is not None for _, f in frames[-10:])
        baseline = frames[-1][1]["tau_calculated"]
        frames = session.collect(.4, "impedance_friction", target, [-1000000] * 7)
        assert all(all(abs(a - b) < .03 for a, b in zip(f["tau_calculated"], baseline))
                   for _, f in frames[-10:]), "Command tau affected friction impedance"
        # Repeated stop targets must not restart MoveJ every 100 ms.
        stop = [-.03, 0, 0, 0, 0, 0, 0]
        frames = session.collect(2.7, "stop", stop)
        assert abs(frames[-1][1]["q"][0] - stop[0]) < .001, "stop MoveJ never reached target"
        assert all(f["tau_calculated"] is None and f["tau_friction"] is None for _, f in frames[-100:])
        # Two commands in one TCP write; the second target is the newest.
        first = session.command("position", zero)
        second = session.command("position", target)
        session.connection.sendall(first + second)
        frames = session.collect(.8, "position", target)
        assert abs(frames[-1][1]["q"][0] - .05) < .001
        intervals = [b[0] - a[0] for a, b in zip(frames, frames[1:])]
        assert len(frames) >= 60 and .007 < statistics.median(intervals) < .014, "Feedback is not approximately 100 Hz"
        # A new mode received during MoveJ is applied after its completion.
        session.collect(.2, "stop", [-.05, 0, 0, 0, 0, 0, 0])
        frames = session.collect(2.0, "position", [.02, 0, 0, 0, 0, 0, 0])
        assert min(f["q"][0] for _, f in frames) < -.048, "MoveJ was overwritten by a queued online target"
        assert abs(frames[-1][1]["q"][0] - .02) < .001
        session.connection.shutdown(socket.SHUT_RDWR)
        assert session.process.wait(timeout=5) == 1, "Disconnect must stop control"
        assert "robot disabled" in session.logs()
    finally:
        session.close()

    session = Session(binary, config, directory, "timeout", timeout=.35)
    try:
        session.connection.sendall(session.command("position", [0.0] * 7))
        deadline = time.monotonic() + 5
        while session.process.poll() is None and time.monotonic() < deadline:
            session.receive()
        assert session.process.wait(timeout=1) == 1
        assert "TCP command timeout" in session.logs()
    finally:
        session.close()
    print("PASS: four modes, 10 Hz commands, Ruckig CSP motion, ignored command tau, stop MoveJ, "
          "nullable 100 Hz feedback, split/coalesced messages, disconnect and command timeout")


if __name__ == "__main__":
    main()
