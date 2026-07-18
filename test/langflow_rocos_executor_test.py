import importlib.util
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]


class _Component:
    pass


class _Output:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class _StrInput:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class _FloatInput(_StrInput):
    pass


class _DropdownInput(_StrInput):
    pass


class _Data:
    def __init__(self, text="", data=None):
        self.text = text
        self.data = data or {}


class _Message:
    def __init__(self, text=""):
        self.text = text


class _Tool:
    def __init__(self, name, description, func):
        self.name = name
        self.description = description
        self.func = func


def install_langflow_stubs():
    langflow = types.ModuleType("langflow")
    langflow_custom = types.ModuleType("langflow.custom")
    langflow_io = types.ModuleType("langflow.io")
    langflow_schema = types.ModuleType("langflow.schema")
    langchain_core = types.ModuleType("langchain_core")
    langchain_tools = types.ModuleType("langchain_core.tools")

    langflow_custom.Component = _Component
    langflow_io.Output = _Output
    langflow_io.StrInput = _StrInput
    langflow_io.FloatInput = _FloatInput
    langflow_io.DropdownInput = _DropdownInput
    langflow_schema.Data = _Data
    langflow_schema.Message = _Message
    langchain_tools.Tool = _Tool

    sys.modules.update({
        "langflow": langflow,
        "langflow.custom": langflow_custom,
        "langflow.io": langflow_io,
        "langflow.schema": langflow_schema,
        "langchain_core": langchain_core,
        "langchain_core.tools": langchain_tools,
    })


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def ready_state(x=0.0, y=0.0, z=0.0, joints=None):
    if joints is None:
        joints = [0.0 for _ in range(7)]
    return {
        "success": True,
        "data": {
            "robot_state": "STOPPED",
            "is_enabled": True,
            "active_tool_frame_name": "flange",
            "active_object_frame_name": "base",
            "flange": {
                "position": {"x": x, "y": y, "z": z},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "joint_states": [{"position": value} for value in joints],
        },
    }


class RocosExecutorComponentTest(unittest.TestCase):
    def setUp(self):
        install_langflow_stubs()

    def test_safe_executor_exposes_tool_output(self):
        mod = load_module("rocos_safe_executor_test_mod", "langflow_components/rocos/rocos_safe_executor.py")

        out = mod.RocosSafeExecutor.outputs[0]
        self.assertEqual(out.display_name, "Tool")
        self.assertEqual(out.name, "component_as_tool")
        self.assertEqual(out.method, "to_tool")
        self.assertIn("Tool", out.types)
        self.assertTrue(hasattr(mod.RocosSafeExecutor, "to_tool"))

    def test_safe_executor_tool_executes_action_json(self):
        mod = load_module("rocos_safe_executor_exec_test_mod", "langflow_components/rocos/rocos_safe_executor.py")
        calls = []

        def fake_post(base_url, path, body=None):
            calls.append((path, body))
            return {"success": True, "code": 0, "message": "ok"}

        comp = mod.RocosSafeExecutor()
        comp.base_url = "http://unit.test"
        comp.memory_dir = tempfile.mkdtemp()
        comp.action_json = "{}"
        comp.intent = "往上"

        action = {
            "type": "cartesian_delta",
            "delta": [0, 0, 0.01, 0, 0, 0],
            "speed": 0.1,
        }
        with patch.object(mod, "_rocos_get", side_effect=[ready_state(), ready_state(0, 0, 0.01), ready_state(0, 0, 0.01)]), \
             patch.object(mod, "_rocos_post", side_effect=fake_post), \
             patch("time.sleep", return_value=None):
            result = comp.to_tool().func(json.dumps(action))

        self.assertIn("执行成功", result)
        self.assertIn("EXECUTED=true", result)
        self.assertIn("api_path=/api/robot/movel", result)
        self.assertEqual(calls[0][0], "/api/robot/movel")

    def test_safe_executor_accepts_movej_alias(self):
        mod = load_module("rocos_safe_executor_movej_test_mod", "langflow_components/rocos/rocos_safe_executor.py")
        calls = []

        def fake_post(base_url, path, body=None):
            calls.append((path, body))
            return {"success": True, "code": 0, "message": "ok"}

        comp = mod.RocosSafeExecutor()
        comp.base_url = "http://unit.test"
        comp.memory_dir = tempfile.mkdtemp()
        comp.action_json = "{}"
        comp.intent = "向左摆头"

        action = {
            "type": "MoveJ",
            "joints": [0.1, 0, -0.1, 0, 0, 0, 0.1],
            "speed": 0.2,
        }
        with patch.object(mod, "_rocos_get", side_effect=[ready_state(), ready_state(), ready_state(joints=action["joints"])]), \
             patch.object(mod, "_rocos_post", side_effect=fake_post), \
             patch("time.sleep", return_value=None):
            result = comp.to_tool().func(json.dumps(action))

        self.assertIn("执行成功", result)
        self.assertIn("EXECUTED=true", result)
        self.assertIn("api_path=/api/robot/movej", result)
        self.assertEqual(calls[0][0], "/api/robot/movej")
        self.assertEqual(calls[0][1]["joints"], action["joints"])

    def test_safe_executor_rejects_http_ok_without_joint_motion(self):
        mod = load_module("rocos_safe_executor_no_motion_test_mod", "langflow_components/rocos/rocos_safe_executor.py")
        calls = []

        def fake_post(base_url, path, body=None):
            calls.append((path, body))
            return {"success": True, "code": 0, "message": "ok"}

        comp = mod.RocosSafeExecutor()
        comp.base_url = "http://unit.test"
        comp.memory_dir = tempfile.mkdtemp()
        comp.action_json = "{}"
        comp.intent = "向左摆头"

        action = {
            "type": "MoveJ",
            "joints": [0.1, 0, -0.1, 0, 0, 0, 0.1],
            "speed": 0.2,
            "verify_timeout": 0,
        }
        with patch.object(mod, "_rocos_get", side_effect=[ready_state(), ready_state(), ready_state()]), \
             patch.object(mod, "_rocos_post", side_effect=fake_post), \
             patch("time.sleep", return_value=None):
            result = comp.to_tool().func(json.dumps(action))

        self.assertIn("EXECUTED=false", result)
        self.assertIn("VERIFY_MOTION_FAILED", result)
        self.assertEqual(calls[0][0], "/api/robot/movej")

    def test_smart_executor_cache_hit_executes_cached_action(self):
        mod = load_module("rocos_smart_executor_test_mod", "langflow_components/rocos/rocos_smart_executor.py")

        with tempfile.TemporaryDirectory() as td:
            memory = Path(td) / "rocos_memory.json"
            memory.write_text(json.dumps({
                "executions": [],
                "patterns": [],
                "stats": {"total_motions": 0, "errors": 0, "recoveries": 0},
                "semantic_cache": {
                    "往上": {
                        "action_name": "safe_executor",
                        "action": {
                            "type": "cartesian_delta",
                            "delta": [0, 0, 0.01, 0, 0, 0],
                            "speed": 0.1,
                        },
                        "frequency": 3,
                        "success_count": 3,
                        "failure_count": 0,
                    }
                },
            }))

            calls = []

            def fake_post(base_url, path, body=None):
                calls.append((path, body))
                return {"success": True, "code": 0, "message": "ok"}

            comp = mod.RocosSmartExecutor()
            comp.base_url = "http://unit.test"
            comp.memory_dir = td
            comp.action_json = "{}"
            comp.intent = ""

            with patch.object(mod, "_rocos_get", side_effect=[ready_state(), ready_state(0, 0, 0.01), ready_state(0, 0, 0.01)]), \
                 patch.object(mod, "_rocos_post", side_effect=fake_post), \
                 patch.object(mod.time, "sleep", return_value=None):
                tool = comp.to_tool()
                result = tool.func("往上")

        self.assertIn("CACHE HIT", result)
        self.assertIn("执行成功", result)
        self.assertIn("EXECUTED=true", result)
        self.assertIn("api_path=/api/robot/movel", result)
        self.assertEqual(calls[0][0], "/api/robot/movel")
        self.assertEqual(calls[0][1]["pose"]["position"]["z"], 0.01)

    def test_smart_executor_accepts_movej_alias(self):
        mod = load_module("rocos_smart_executor_movej_test_mod", "langflow_components/rocos/rocos_smart_executor.py")
        calls = []

        def fake_post(base_url, path, body=None):
            calls.append((path, body))
            return {"success": True, "code": 0, "message": "ok"}

        comp = mod.RocosSmartExecutor()
        comp.base_url = "http://unit.test"
        comp.memory_dir = tempfile.mkdtemp()
        comp.action_json = "{}"
        comp.intent = ""

        payload = {
            "intent": "向左摆头",
            "action_json": {
                "type": "movej",
                "joints": [0.1, 0, -0.1, 0, 0, 0, 0.1],
                "speed": 0.2,
            },
        }
        target_joints = payload["action_json"]["joints"]
        with patch.object(mod, "_rocos_get", side_effect=[ready_state(), ready_state(), ready_state(joints=target_joints)]), \
             patch.object(mod, "_rocos_post", side_effect=fake_post), \
             patch.object(mod.time, "sleep", return_value=None):
            result = comp.to_tool().func(json.dumps(payload))

        self.assertIn("SAFE EXECUTE", result)
        self.assertIn("执行成功", result)
        self.assertIn("EXECUTED=true", result)
        self.assertIn("api_path=/api/robot/movej", result)
        self.assertEqual(calls[0][0], "/api/robot/movej")

    def test_smart_executor_cache_miss_without_action_is_not_executed(self):
        mod = load_module("rocos_smart_executor_miss_test_mod", "langflow_components/rocos/rocos_smart_executor.py")

        with tempfile.TemporaryDirectory() as td:
            comp = mod.RocosSmartExecutor()
            comp.base_url = "http://unit.test"
            comp.memory_dir = td
            comp.action_json = "{}"
            comp.intent = ""

            with patch.object(mod, "_rocos_post") as post:
                result = comp.to_tool().func("向左摆头")

        self.assertIn("EXECUTED=false", result)
        self.assertIn("CACHE_MISS_NEEDS_ACTION_JSON", result)
        post.assert_not_called()


if __name__ == "__main__":
    unittest.main()
