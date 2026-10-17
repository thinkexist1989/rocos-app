"""HardwareSim regression: drive velocity feedback, request/reply and nonfatal command timeout."""
import argparse
import json
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="rocos-request-reply-timeout-") as directory:
        log_path = Path(directory) / "client.log"
        with socket.socket() as listener, log_path.open("w") as output:
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            listener.settimeout(8)
            process = subprocess.Popen([
                str(args.binary.resolve()), "--sim", "--config=" + str(args.config.resolve()),
                "--host=127.0.0.1", "--port=" + str(listener.getsockname()[1]),
                "--command_timeout=.15", "--notcp_log_data",
            ], stdout=output, stderr=output)
            try:
                connection, _ = listener.accept()
                with connection:
                    buffer = b""

                    def quiet(duration):
                        assert not buffer, "Unexpected buffered response"
                        connection.settimeout(duration)
                        try:
                            data = connection.recv(65536)
                        except socket.timeout:
                            return
                        raise AssertionError("Unsolicited feedback or disconnection: " + repr(data))

                    def request(mode, q=None):
                        nonlocal buffer
                        command = dict(frame_header="0xAA56", timestamp_us=time.time_ns() // 1000, mode=mode)
                        if q is not None:
                            command["q"] = [q, 0, 0, 0, 0, 0, 0]
                        connection.sendall((json.dumps(command) + "\n").encode())
                        connection.settimeout(5)
                        while b"\n" not in buffer:
                            data = connection.recv(65536)
                            assert data, "Disconnected before reply"
                            buffer += data
                        line, buffer = buffer.split(b"\n", 1)
                        frame = json.loads(line)
                        assert frame["frame_header"] == "0xAA55"
                        assert frame["timestamp_us"] > 0
                        return frame

                    quiet(.2)
                    request("free")  # First command can enter free directly, without any position target.
                    quiet(.2)
                    initial_free = request("free")
                    assert len(initial_free["tau_calculated"]) == 7 and initial_free["tau_friction"] == [0] * 7
                    # HardwareSim echoes CSP positions but its independent velocity channel remains zero.
                    # Changing positions must not produce position-difference velocities in dq.
                    motion = []
                    for _ in range(12):
                        motion.append(request("position", .04))
                        time.sleep(.01)
                    assert any(abs(b["q"][0] - a["q"][0]) > 1e-5 for a, b in zip(motion, motion[1:])), \
                        "No simulated position motion to exercise velocity source"
                    assert all(frame["dq"] == [0] * 7 for frame in motion), \
                        "dq must use the zero drive velocity channel despite changing positions"
                    for mode, target, idle in [
                        ("position", .04, 1.0),
                        ("impedance", .04, .5),
                        ("impedance_friction", .04, .5),
                        ("free", None, .35),
                        ("impedance", .04, .35),
                        ("free", .3, .35),  # Supplied position must be ignored in free.
                        ("position", .02, .8),
                        ("free", None, .35),
                        ("stop", 0, 1.3),
                        ("free", None, .35),
                        ("position", .01, .6),
                    ]:
                        first = request(mode, target)
                        quiet(idle)  # All intervals exceed command_timeout, without sending keepalives.
                        assert process.poll() is None, "Client exited during command timeout"
                        state = request(mode, target)
                        assert state["timestamp_us"] > first["timestamp_us"], "Sampling stopped during timeout"
                        expected_q = first["q"][0] if mode == "free" else target
                        assert abs(state["q"][0] - expected_q) < .002, "Last target did not finish, remain held or get ignored in free"
                        if mode == "free":
                            # The immediate entry reply may still contain the previous mode's snapshot.
                            # Compare two settled free snapshots while changing the ignored position target.
                            other = request("free", .4)
                            assert other["tau_calculated"] == state["tau_calculated"], "free acquired position stiffness"
                            assert other["q"] == state["q"], "free followed an ignored position target"
                        torque_mode = mode in ("impedance", "impedance_friction", "free")
                        for field in ("tau_calculated", "tau_friction"):
                            assert isinstance(state[field], list) and len(state[field]) == 7
                            if not torque_mode:
                                assert state[field] == [0] * 7, (mode, field, state[field])
                    quiet(.2)
                    process.send_signal(signal.SIGINT)
                    assert process.wait(timeout=5) == 0, "Clean shutdown failed"
                logs = log_path.read_text()
                assert "TCP 指令超时" in logs and "[TCP/RESUMED]" in logs
                assert "[TCP/ERROR]" not in logs, logs
                print("PASS: drive velocity independent of position changes, five modes across command timeouts, "
                      "free ignores optional q, initial/CSP/CST/stop free transitions, "
                      "sampling continues, new requests resume, no unsolicited feedback, clean shutdown")
            except Exception:
                print(log_path.read_text())
                raise
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()


if __name__ == "__main__":
    main()
