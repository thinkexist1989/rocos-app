"""SDK transport errors and execution against an isolated HardwareSim controller."""
import http.server
import json
import pathlib
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest

root = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "examples"))
from rocos import ApiError, Pose, Robot, TransportError

binary = pathlib.Path(sys.argv[1]).resolve()
urdf = pathlib.Path(sys.argv[2]).resolve()
sys.argv = [sys.argv[0]]


class TransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.calls = []

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_POST(self):
                cls.calls.append(self.path)
                self.rfile.read(int(self.headers.get("Content-Length", 0)))
                if self.path.endswith("drop"):
                    self.connection.shutdown(socket.SHUT_RDWR)
                    self.connection.close()
                    return
                if self.path.endswith("redirect"):
                    self.send_response(302)
                    self.send_header("Location", "/redirect-target")
                    self.end_headers()
                    return
                if self.path.endswith("text"):
                    self.send_response(200)
                    self.end_headers()
                    self.wfile.write(b"not JSON")
                    return
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"success": False, "code": 1001,
                    "error_name": "BadInput", "message": "bad parameter", "data": ["detail"]}).encode())

            def log_message(self, *args):
                pass

        cls.server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.robot = Robot("127.0.0.1", cls.server.server_port, auto_acquire=False)

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()

    def test_http_business_error_preserves_details(self):
        with self.assertRaises(ApiError) as caught:
            self.robot.post("error", {})
        error = caught.exception
        self.assertEqual((error.code, error.http_status, error.data), (1001, 400, ["detail"]))
        result = self.robot.post("error", {}, checked=False)
        self.assertFalse(result["success"])

    def test_network_failure_never_replays_a_command(self):
        before = len(self.calls)
        with self.assertRaises(TransportError):
            self.robot.post("drop", {})
        self.assertEqual(len(self.calls), before + 1)

    def test_redirect_is_not_followed(self):
        before = len(self.calls)
        with self.assertRaises(TransportError):
            self.robot.post("redirect", {})
        self.assertEqual(self.calls[before:], ["/api/robot/redirect"])

    def test_non_json_and_external_paths_are_rejected(self):
        with self.assertRaises(TransportError):
            self.robot.post("text", {})
        with self.assertRaises(ValueError):
            self.robot.post("http://another-host/steal", {})
        with self.assertRaises(ValueError):
            self.robot.get("../state")


class RobotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="rocos-python-sdk-")
        cls.log = open(pathlib.Path(cls.directory.name) / "controller.log", "w")
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        cls.url = "http://127.0.0.1:" + str(port)
        cls.process = subprocess.Popen([str(binary), "--sim=true", "--grpc=false",
            "--http_host=127.0.0.1", "--http_port=" + str(port),
            "--http_control_ttl_seconds=2", "--urdf=" + str(urdf)],
            cwd=cls.directory.name, stdout=cls.log, stderr=subprocess.STDOUT)
        try:
            cls.reader = Robot(cls.url)
            deadline = time.monotonic() + 10
            while True:
                try:
                    state = cls.reader.get()
                    break
                except TransportError:
                    if time.monotonic() >= deadline or cls.process.poll() is not None:
                        raise RuntimeError("Test controller failed to start")
                    time.sleep(0.05)
            assert state["hw_state"]["hw_type"] == 1
        except BaseException:
            cls.tearDownClass()
            raise

    @classmethod
    def tearDownClass(cls):
        if cls.process.poll() is None:
            cls.process.terminate()
            try:
                cls.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                cls.process.kill()
                cls.process.wait()
        cls.log.close()
        cls.directory.cleanup()

    def assert_no_owner(self):
        self.assertFalse(self.reader.get("control/status")["has_owner"])

    def test_01_read_only_and_path_query_formats(self):
        self.assertEqual(len(self.reader.get_joints()), 7)
        self.assertIsInstance(self.reader.get_pose(), Pose)
        self.assertTrue(self.reader.is_enabled())
        for path in ("state", "/robot/state", "robot/state", "/api/robot/state"):
            self.assertFalse(self.reader.get(path, params={"raw_data": False})["raw_data"])
        self.assertTrue(self.reader.get("state", params={"raw_data": True})["raw_data"])
        self.assertEqual(self.reader.get("info", raw=True)["code"], 0)
        self.assert_no_owner()

    def test_02_lazy_control_and_heartbeat(self):
        robot = Robot(self.url)
        try:
            self.assertIsNone(robot.token)
            robot.enable()  # No explicit acquire or token plumbing.
            self.assertIsNotNone(robot.token)
            time.sleep(3.1)  # Longer than the real 2-second lease.
            self.assertEqual(self.reader.get("control/status")["owner_client_id"], robot.client_id)
            competitor = Robot(self.url)
            with self.assertRaises(ApiError) as caught:
                competitor.connect()
            self.assertEqual(caught.exception.code, 3008)
        finally:
            robot.close()
        self.assert_no_owner()

    def test_03_blocking_async_json_and_alias_moves(self):
        with Robot(self.url) as robot:
            robot.enable()
            original = robot.get_joints()
            target = original.copy()
            target[0] += 0.02
            result = robot.movej(target)
            self.assertFalse(result["motion_busy"])
            self.assertAlmostEqual(robot.get_joints()[0], target[0], delta=1e-4)
            robot.movej(original, wait=False)
            robot.wait_move()
            self.assertAlmostEqual(robot.get_joints()[0], original[0], delta=1e-4)
            payload = {"joints": target, "velocity": 0.1, "acceleration": 0.3}
            robot.movej(payload)
            self.assertNotIn("asynchronous", payload)
            robot.movrj(original)
            accepted = robot.post("movej", {"joints": target, "velocity": 0.1,
                "acceleration": 0.3, "asynchronous": True})
            self.assertIn("motion_busy", accepted)
            robot.wait_move()
            robot.movej(original)
            robot.stop()
        self.assert_no_owner()

    def test_04_api_errors_and_pose_mapping(self):
        with Robot(self.url) as robot:
            with self.assertRaises(ApiError) as caught:
                robot.post("movej", {})
            self.assertEqual(caught.exception.code, 1001)
            self.assertEqual(robot.post("movej", {}, checked=False)["code"], 1001)
            with self.assertRaises(ApiError) as unsupported:
                robot.post("pause", {})
            self.assertEqual(unsupported.exception.error_name, "FunctionNotSupported")
            self.assertIsNone(robot.heartbeat_error)
            with self.assertRaises(ApiError) as ik:
                robot.movej_ik(Pose(100, 100, 100))
            self.assertEqual(ik.exception.code, -2349)
            self.assertEqual(Pose.from_dict(robot.get_pose().to_dict()), robot.get_pose())
        self.assert_no_owner()

    def test_05_context_cleanup_preserves_original_exception(self):
        with self.assertRaisesRegex(ValueError, "application failed"):
            with Robot(self.url):
                raise ValueError("application failed")
        self.assert_no_owner()

    def test_06_takeover_stops_old_session_without_automatic_reclaim(self):
        old = Robot(self.url)
        new = Robot(self.url)
        old.connect()
        try:
            new.takeover()
            deadline = time.monotonic() + 3
            while old.heartbeat_error is None and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertIsNotNone(old.heartbeat_error)
            self.assertEqual(len(old.get_joints()), 7)  # Reads still work.
            with self.assertRaises(ApiError) as caught:
                old.enable()
            self.assertEqual(caught.exception.code, 3007)
            old.close()  # Closing an already invalid lease is harmless.
            self.assertEqual(self.reader.get("control/status")["owner_client_id"], new.client_id)
            with self.assertRaises(ApiError) as occupied:
                old.connect()  # Explicit reconnect still cannot steal a live lease.
            self.assertEqual(occupied.exception.code, 3008)
        finally:
            old.close()
            new.close()
        self.assert_no_owner()
        old.connect()  # Once released, explicit reconnect can start a fresh session.
        old.close()
        self.assert_no_owner()

    def test_07_invalid_local_parameters_do_not_start_motion(self):
        robot = Robot(self.url)
        with self.assertRaises(ValueError):
            robot.movej([0] * 7, timeout=-1)
        with self.assertRaises(ValueError):
            robot.movej([float("nan")] * 7)
        self.assertFalse(self.reader.get()["motion_busy"])
        self.assert_no_owner()

    def test_08_motion_prepares_position_mode_and_stop_waits(self):
        with Robot(self.url) as robot:
            original = robot.get_joints()
            robot.set_work_mode("cart_imp")
            robot.movej(original)
            self.assertEqual(robot.get()["work_mode"], "position")
            target = original.copy()
            target[0] += 0.2
            robot.movej(target, wait=False)
            time.sleep(0.1)
            robot.stop()
            self.assertFalse(robot.get()["motion_busy"])
            self.assertGreater(abs(robot.get_joints()[0] - target[0]), 0.01)
            robot.movej(original)
        self.assert_no_owner()


unittest.main(verbosity=2)
