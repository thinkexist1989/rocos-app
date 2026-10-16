"""Compare with Studio's real generator/preprocessor; never connect real hardware."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np
import pandas as pd
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--studio", type=Path, required=True)
    args = parser.parse_args()
    binary, studio = args.binary.resolve(), args.studio.resolve()
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(studio / "src"))
    from dynamics_id.friction_config import FrictionTrajectoryConfig, FrictionDataConfig
    from dynamics_id.friction_data import prepare_friction_data

    with tempfile.TemporaryDirectory(prefix="rocos-friction-test-") as directory:
        work = Path(directory)
        tree = ET.parse(Path(__file__).resolve().parents[1] / "config/robot.urdf")
        limits = {}
        for joint in tree.findall("joint[@type='continuous']"):
            joint.set("type", "revolute")
            limit = joint.find("hardware/limit")
            limits[joint.get("name")] = {key: float(limit.get(key)) for key in ("lower", "upper", "vel")}
            ET.SubElement(joint, "limit", lower=limit.get("lower"), upper=limit.get("upper"),
                          velocity=limit.get("vel"), effort=limit.get("effort"))
        urdf = work / "robot.urdf"
        tree.write(urdf)

        # Model metadata only: no Pinocchio dependency is needed by these two
        # functions. Their arithmetic comes directly from the Studio source.
        class FixtureModel:
            names = ["joint_3", "joint_1", "joint_7", "joint_2", "joint_6", "joint_4", "joint_5"]
            n = 7
            hash = hashlib.sha256(urdf.read_bytes()).hexdigest()

            def summary(self):
                return dict(joints=[dict(type="revolute", velocity=limits[name]["vel"],
                                         lower=limits[name]["lower"], upper=limits[name]["upper"]) for name in self.names])

        model = FixtureModel()
        model.lower = np.array([limits[name]["lower"] for name in model.names])
        model.upper = np.array([limits[name]["upper"] for name in model.names])
        source = ast.parse((studio / "src/dynamics_id/friction_protocol.py").read_text())
        function = next(node for node in source.body if isinstance(node, ast.FunctionDef) and node.name == "generate_friction_trajectory")
        namespace = dict(np=np, pd=pd, RobotModel=lambda _: model, FrictionTrajectoryConfig=FrictionTrajectoryConfig,
                         save_json=lambda path, value: path.write_text(json.dumps(value)))
        exec(compile(ast.Module(body=[function], type_ignores=[]), "studio-generator", "exec"), namespace)
        generate = namespace["generate_friction_trajectory"]
        config = FrictionTrajectoryConfig(joint="joint_2", lower=-.02, upper=.02, pose=[0.]*7,
                                         speeds=[.04, .05], acceleration=1, repeats=3, dwell_seconds=0, sample_rate=100)
        generate(urdf, work, config)
        protocol_file = work / "friction_protocol.json"
        original_protocol = protocol_file.read_text()
        torque_config = root / "config/joint_rated_torque.yaml"
        common = [str(binary), f"--urdf={urdf}", f"--trajectory={protocol_file}", f"--torque_config={torque_config}"]

        def run(extra, expected=0):
            result = subprocess.run(common + extra, cwd=work, capture_output=True, text=True, timeout=60)
            assert result.returncode == expected, result.stdout + result.stderr
            return result

        run(["--check_only", "--sim=false"])
        bad_torque = work / "bad_torque.yaml"
        ratings = yaml.safe_load(torque_config.read_text())
        for value in (None, 0, -1, float("nan")):
            invalid = {"rated_torque": dict(ratings["rated_torque"])}
            if value is None:
                del invalid["rated_torque"]["joint_3"]
            else:
                invalid["rated_torque"]["joint_3"] = value
            bad_torque.write_text(yaml.safe_dump(invalid))
            result = run(["--check_only", "--sim=false", f"--torque_config={bad_torque}"], expected=1)
            assert "rated_torque" in result.stderr or "Rated torque" in result.stderr
        for scale in ("0", "-1", "1.01", "nan", "inf", "1e-300"):
            run(["--check_only", f"--time_scale={scale}"], expected=1)
        # Invalid speeds must fail before hardware access or ordered-set use.
        for speeds in ([float("nan"), .05], [float("inf"), .05], [0, .05], [-.04, .05], [.04, .04]):
            invalid = json.loads(original_protocol)
            invalid["config"]["speeds"] = speeds
            protocol_file.write_text(yaml.safe_dump(invalid))
            result = run(["--check_only", "--sim=false"], expected=1)
            assert "Speed must be finite and positive" in result.stderr or "Duplicate speed settings" in result.stderr
        protocol_file.write_text(original_protocol)

        original_urdf = urdf.read_bytes()
        for scale in ("0", "nan"):
            invalid_urdf = ET.fromstring(original_urdf)
            invalid_urdf.find("joint[@name='joint_2']/hardware/transform").set("cnt_per_unit", scale)
            ET.ElementTree(invalid_urdf).write(urdf)
            invalid = json.loads(original_protocol)
            invalid["urdf_hash"] = hashlib.sha256(urdf.read_bytes()).hexdigest()
            protocol_file.write_text(json.dumps(invalid))
            assert "position conversion" in run(["--check_only", "--sim=false"], expected=1).stderr
        urdf.write_bytes(original_urdf)
        protocol_file.write_text(original_protocol)
        output = work / "measured.sim.csv"
        run(["--sim=true", f"--output={output}"])
        measured = pd.read_csv(output)
        reference = pd.read_csv(work / "friction_trajectory.csv")
        assert np.all(np.diff(measured.time) > 0)
        assert np.all(np.diff(measured.reference_time) > 0)
        assert set(measured.repeat) == {0, 1, 2} and measured.pair.nunique() == 6 and measured.sweep.nunique() == 12
        assert set(measured.phase) == {"ramp", "steady", "dwell"}
        assert np.isclose(measured.reference_time.iloc[-1], reference.time.iloc[-1] + .01)
        for j in range(1, 8):
            expected_q = np.interp(measured.reference_time, reference.time, reference[f"q_{j}"])
            np.testing.assert_allclose(measured[f"q_{j}"], expected_q, atol=3e-5)
            # HardwareSim retains the last MoveJ velocity command; servoJ does
            # not update it. Record that getter value, never the reference dq.
            assert measured[f"dq_{j}"].nunique() == 1 and measured[f"dq_{j}"].abs().max() < .002
            assert np.all(measured[f"tau_{j}"] == 0)
            assert f"temp_{j}" not in measured and f"current_{j}" not in measured
        assert set(np.round(measured.target_velocity, 6)) == {-.05, -.04, 0., .04, .05}
        report = yaml.safe_load(Path(str(output) + ".metadata.yaml").read_text())
        assert report["completed"] and report["simulation"] and report["samples"] == len(measured)
        assert report["joint_names"] == model.names and report["robot_indices"] == [2, 0, 6, 1, 5, 3, 4]
        assert report["torque_source"] == "Robot::getJointTorque" and report["torque_unit"] == "Nm"
        assert report["torque_conversion"] == "getJointTorque * rated_torque_Nm / 1000"
        assert report["rated_torque_Nm"] == dict(zip([f"joint_{j}" for j in range(1, 8)], [48, 48, 40, 40, 7, 7, 7]))
        assert not report["temperature_recorded"]
        before = output.read_bytes()
        run([f"--output={output}"], expected=1)
        assert output.read_bytes() == before

        data_config = FrictionDataConfig(stationary_others_confirmed=True)
        points, _, quality = prepare_friction_data(output, model, 3, data_config, "a"*32, allow_empty=True)
        assert points.empty and quality["mapping"]["velocity"] == "dq_4"

        # Separate synthetic fixture checks the import/pairing contract. These
        # are explicitly artificial signals, never written by the collector.
        synthetic = measured.copy()
        synthetic["time"] = synthetic.reference_time
        synthetic["dq_4"] = np.interp(synthetic.reference_time, reference.time, reference.dq_4)
        friction = .2*np.tanh(synthetic.dq_4/.01) + .04*synthetic.dq_4
        synthetic["tau_4"] = 1.3*np.sin(synthetic.q_4) + friction
        synthetic_path = work / "synthetic.csv"
        synthetic.to_csv(synthetic_path, index=False)
        points, _, quality = prepare_friction_data(synthetic_path, model, 3, data_config, "b"*32)
        assert len(quality["pairs"]) == 6 and all(pair["status"] == "accepted" for pair in quality["pairs"])
        np.testing.assert_allclose(points.friction, .2*np.tanh(points.velocity/.01) + .04*points.velocity, atol=.001)

        slow = work / "slow"
        slow.mkdir()
        generate(urdf, slow, config.model_copy(update=dict(repeats=1, sample_rate=1000)))
        slow_output = work / "slow.sim.csv"
        run(["--sim=true", "--time_scale=.5", f"--trajectory={slow/'friction_trajectory.csv'}", f"--output={slow_output}"])
        slower = pd.read_csv(slow_output)
        assert set(np.round(slower.target_velocity, 6)) == {-.025, -.02, 0., .02, .025}
        assert slower.time.iloc[-1] > slower.reference_time.iloc[-1]*1.8

        for key, value in (("urdf_hash", "wrong"), ("joint_index", 0), ("initial_pose", [0]*7)):
            invalid = json.loads(original_protocol)
            invalid[key] = value
            protocol_file.write_text(json.dumps(invalid))
            run(["--check_only", "--sim=false"], expected=1)
        protocol_file.write_text(original_protocol)
        interrupted_output = work / "interrupted.csv"
        process = subprocess.Popen(common + ["--sim=true", f"--output={interrupted_output}"], cwd=work,
                                   text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            for line in process.stdout:
                if "Recording friction sweeps" in line:
                    time.sleep(.05)
                    process.send_signal(signal.SIGTERM)
                    break
            stdout, stderr = process.communicate(timeout=15)
            assert process.returncode == 130, stdout + stderr
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
        interrupted_report = yaml.safe_load(Path(str(interrupted_output) + ".metadata.yaml").read_text())
        assert not interrupted_report["completed"] and interrupted_report["samples"] > 0
        assert len(pd.read_csv(interrupted_output)) == interrupted_report["samples"]
    print("PASS: Studio sweeps, timing, name mapping, feedback, scaling, CSV pairing, validation and interruption")


if __name__ == "__main__":
    main()
