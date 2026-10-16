"""Offline adapter checks against Studio's real evaluate/preprocess functions.

No real hardware is opened: execution is --sim=true, other checks --check_only.
"""
import argparse
import ast
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--studio", type=Path, required=True)
    args = parser.parse_args()
    binary, studio = args.binary.resolve(), args.studio.resolve()
    sys.path.insert(0, str(studio / "src"))
    from dynamics_id.config import DataConfig
    from dynamics_id.preprocessing import preprocess

    # Only extract the actual pure evaluate function, avoiding Pinocchio/model
    # initialization. All arithmetic under test comes from the user's Studio.
    tree = ast.parse((studio / "src/dynamics_id/trajectory.py").read_text())
    evaluate_node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "evaluate")
    namespace = {"np": np}
    exec(compile(ast.Module(body=[evaluate_node], type_ignores=[]), "studio-evaluate", "exec"), namespace)
    evaluate = namespace["evaluate"]
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="rocos-studio-test-") as directory:
        work = Path(directory)
        # Reuse the real Robot's SRS geometry, adding Studio's bounded joint tags.
        urdf_tree = ET.parse(root / "config/robot.urdf")
        for element in urdf_tree.findall("joint"):
            if element.get("type") == "continuous":
                element.set("type", "revolute")
                limit = element.find("hardware/limit")
                ET.SubElement(element, "limit", lower=limit.get("lower"), upper=limit.get("upper"),
                              velocity=limit.get("vel"), effort=limit.get("effort"))
        urdf = work / "robot.urdf"
        urdf_tree.write(urdf)
        # Deliberately permute Studio order relative to Robot/URDF/hardware IDs.
        names = ["joint_3", "joint_1", "joint_7", "joint_2", "joint_6", "joint_4", "joint_5"]
        duration = 2.0
        coefficients = np.zeros((7, 2, 2))
        coefficients[:, 0, 0] = np.arange(1, 8) * .0004
        coefficients[:, 1, 1] = np.arange(1, 8) * .0002
        meta = dict(joint_names=names.copy(),
                    config=dict(duration=duration, bounds=[dict(lower=-.1, upper=.1, velocity=.3, acceleration=1, center=0)] * 7),
                    coefficients=coefficients.tolist(), limits_verified=True)
        metadata = work / "trajectory.json"
        metadata.write_text(json.dumps(meta))
        common = [str(binary), f"--urdf={urdf}", f"--trajectory={metadata}"]

        def run(extra, expected=0):
            result = subprocess.run(common + extra, cwd=work, text=True, capture_output=True, timeout=60)
            assert result.returncode == expected, result.stdout + result.stderr
            return result

        run(["--check_only"])
        run(["--check_only", "--sim=false"])  # Proves check_only does not connect hardware.
        run(["--check_only", f"--trajectory={work / 'trajectory.csv'}"])
        for scale in ("0", "-0.1", "1.01", "nan", "inf"):
            assert "time_scale must be" in run(["--check_only", f"--time_scale={scale}"], expected=1).stderr
        assert "too many trajectory points" in run(["--check_only", "--time_scale=1e-300"], expected=1).stderr
        output = work / "measurements.sim.csv"
        run(["--sim=true", "--loops=2", f"--output={output}"])
        data = np.genfromtxt(output, names=True, delimiter=",")
        assert len(data) == 4002 and np.all(np.diff(data["time"]) > 0)
        assert set(data["loop"]) == {1, 2}
        report = yaml.safe_load(Path(str(output) + ".metadata.yaml").read_text())
        assert report["simulation"] is True and report["completed"] is True
        assert report["time_scale"] == 1
        assert report["servo_period_seconds"] == .001
        assert report["torque_source"] == "Robot::getJointLoadTorque"
        assert report["torque_conversion"] == "none; save the Robot getter return value directly"
        assert len(data.dtype.names) == 2 + 3 * len(names)
        assert [j["name"] for j in report["joints"]] == names
        assert [j["robot_index"] for j in report["joints"]] == [2, 0, 6, 1, 5, 3, 4]
        servo_q, _, _ = evaluate(coefficients, np.linspace(0, duration, 2001), duration, np.zeros(7))
        for j in range(1, 8):
            # Save Robot::getJointLoadTorque() directly. HardwareSim returns zero.
            assert np.all(data[f"tau_{j}"] == 0)
            assert f"load_raw_{j}" not in data.dtype.names
            assert np.all(data[f"dq_{j}"] == 0)  # Do not fabricate measured velocity.
            assert np.max(np.abs(data[f"q_{j}"] - np.tile(servo_q[:, j - 1], 2))) < 2e-5
        imported = preprocess(output, 7, DataConfig(cutoff_hz=8, trim_seconds=.05))
        assert imported.q.shape[1] == 7 and len(imported.time) > 100
        assert imported.report["segments"] == 2
        assert imported.report["missing_temperature"] == list(range(1, 8))
        original = output.read_bytes()
        run(["--sim=true", f"--output={output}"], expected=1)
        assert output.read_bytes() == original

        # At 0.1 the same function advances by 0.1 ms per 1 ms control cycle:
        # ten times as many samples and ten times the duration, same spatial path.
        slowed_output = work / "slow.sim.csv"
        run(["--sim=true", "--time_scale=.1", f"--output={slowed_output}"])
        slowed = np.genfromtxt(slowed_output, names=True, delimiter=",")
        phase_step = .001 * .1
        steps = int(np.ceil(duration / phase_step))
        assert len(slowed) == steps + 1 and np.all(np.diff(slowed["time"]) > 0)
        assert slowed["time"][-1] >= duration / .1 * .9
        expected_q, _, _ = evaluate(coefficients, np.minimum(np.arange(steps + 1) * phase_step, duration), duration, np.zeros(7))
        for j in range(1, 8):
            assert np.max(np.abs(slowed[f"q_{j}"] - expected_q[:, j - 1])) < 2e-5
        slowed_report = yaml.safe_load(Path(str(slowed_output) + ".metadata.yaml").read_text())
        assert slowed_report["time_scale"] == .1 and slowed_report["servo_period_seconds"] == .001
        assert slowed_report["completed"] is True
        assert len(preprocess(slowed_output, 7, DataConfig()).time) > 100

        # Invalid input must fail before Robot construction or hardware access.
        meta["coefficients"][0][0][0] = 100
        metadata.write_text(json.dumps(meta))
        assert "exceeds limits" in run(["--check_only"], expected=1).stderr
        meta["coefficients"] = coefficients.tolist()
        meta["joint_names"][0] = "missing_joint"
        metadata.write_text(json.dumps(meta))
        assert "Invalid hardware mapping" in run(["--check_only"], expected=1).stderr
        meta["joint_names"] = names
        metadata.write_text(json.dumps(meta))

        partial = work / "partial.sim.csv"
        process = subprocess.Popen(common + ["--sim=true", "--loops=2", f"--output={partial}"], cwd=work,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        # Wait for actual recording, not a fixed sleep during Robot initialization.
        for line in process.stdout:
            if "Recording 2 excitation" in line:
                time.sleep(.1)
                process.send_signal(signal.SIGTERM)
                break
        stdout, stderr = process.communicate(timeout=10)
        assert process.returncode == 130, stdout + stderr
        partial_report = yaml.safe_load(Path(str(partial) + ".metadata.yaml").read_text())
        assert partial_report["completed"] is False and partial_report["samples"] > 0
        assert partial_report["failure"] == "Interrupted by signal"
    print("PASS: Studio trajectory, Robot replay, time scaling at 0.1, name mapping, direct LoadTorque feedback, CSV import, validation failures and interrupted recording")


if __name__ == "__main__":
    main()
