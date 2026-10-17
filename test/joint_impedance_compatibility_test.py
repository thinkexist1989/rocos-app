"""Offline comparison with Studio, plus HardwareSim smoke tests. No real hardware."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bin", type=Path, required=True)
    parser.add_argument("--studio", type=Path, required=True)
    args = parser.parse_args()
    binaries = args.bin.resolve()
    root = Path(__file__).resolve().parents[1]
    # Use Studio's actual pure friction function without importing its GUI/API.
    source = ast.parse((args.studio / "src/dynamics_id/friction.py").read_text())
    function = next(node for node in source.body if isinstance(node, ast.FunctionDef) and node.name == "friction_matrix")
    namespace = {"np": np}
    exec(compile(ast.Module(body=[function], type_ignores=[]), "studio-friction", "exec"), namespace)

    def run(command, expected=0):
        result = subprocess.run([str(x) for x in command], capture_output=True, text=True, timeout=30)
        assert result.returncode == expected, result.stdout + result.stderr
        return result

    with tempfile.TemporaryDirectory(prefix="rocos-impedance-") as directory:
        work = Path(directory)
        tree = ET.parse(root / "config/robot.urdf")
        ids = [7, 3, 5, 1, 6, 2, 4]  # Bus order differs from model/Robot order.
        for index, joint in enumerate(tree.findall("joint[@type='continuous']")):
            joint.set("type", "revolute")
            hardware = joint.find("hardware")
            hardware.set("id", str(ids[index]))
            hardware.find("transform").set("torque_per_unit", "100")
            limit = hardware.find("limit")
            ET.SubElement(joint, "limit", lower=limit.get("lower"), upper=limit.get("upper"),
                          velocity=limit.get("vel"), effort=limit.get("effort"))
        urdf = work / "robot.urdf"
        tree.write(urdf)
        model = pin.buildModelFromUrdf(str(urdf))
        names = list(model.names)[1:]
        nominal = np.concatenate([inertia.toDynamicParameters() for inertia in list(model.inertias)[1:]])
        rng = np.random.default_rng(42)
        P = np.linalg.qr(np.column_stack([nominal, rng.normal(size=(70, 34))]))[0]
        beta = P.T @ nominal + rng.normal(size=35) * .0001
        identification = dict(converged=True, joint_names=names, base_parameters=beta.tolist(),
                              base_parameter_count=len(beta), urdf_hash=hashlib.sha256(urdf.read_bytes()).hexdigest())
        (work / "identification.json").write_text(json.dumps(identification))
        np.savez_compressed(work / "identification.npz", P=P, base=beta)
        run([sys.executable, root / "tools/export_impedance_model.py", work, work / "rigid_model.json"])

        parameters = dict(coulomb=.2, load_coefficient=.03, stribeck_amplitude=.1, viscous_at_reference=.08,
                          viscous_temperature_slope=0, reference_temperature_c=25, stribeck_velocity=.15, smooth_velocity=.05)
        friction = dict(model="stribeck_load_temperature_v1", joint_names=names, urdf_hash=identification["urdf_hash"],
                        joint_parameters={name: dict(parameters) for name in reversed(names)})
        config = dict(urdf="robot.urdf", rigid_model="rigid_model.json", friction="friction_model.json",
                      kp=[20, 18, 16, 14, 12, 10, 8], kd=[4, 3.5, 3, 2.5, 2, 1.5, 1],
                      velocity_limit=[.4]*7, friction_scale=.7,
                      rated_torque=[48, 48, 40, 40, 7, 7, 7],
                      torque_limit=[29]*7, torque_rate_limit=[100]*7)
        config_file = work / "controller.yaml"
        config_file.write_text(yaml.safe_dump(config))
        common = [binaries / "jointImpedanceMain", f"--config={config_file}"]

        # URDF 版本只提供模型和控制参数，完全不提供辨识/摩擦文件路径。
        urdf_config = {key: value for key, value in config.items() if key not in ("rigid_model", "friction")}
        urdf_config_file = work / "urdf_controller.yaml"
        urdf_config_file.write_text(yaml.safe_dump(urdf_config))
        urdf_common = [binaries / "jointImpedanceUrdfMain", f"--config={urdf_config_file}"]
        run([binaries / "joint_impedance_urdf_test", urdf_config_file, "link_8"])
        run(urdf_common + ["--check_only", "--sim=false"])
        urdf_smoke = run(urdf_common + ["--sim=true", "--duration=.15", "--debug_interval=.1"])
        assert "[阻抗调试]" in urdf_smoke.stdout and "friction(Nm)" not in urdf_smoke.stdout

        for thermal in (False, True):
            for item in friction["joint_parameters"].values():
                item["viscous_temperature_slope"] = .002 if thermal else 0
            (work / "friction_model.json").write_text(json.dumps(friction))
            cases = []
            for _ in range(100):
                q, dq = rng.uniform(-.3, .3, (2, 7))
                target = q + rng.uniform(-.05, .05, 7)
                temperature = rng.uniform(15, 50, 7)
                rigid = pin.computeJointTorqueRegressor(model, model.createData(), q, dq, np.zeros(7)) @ P @ beta
                coefficients = np.tile([.2, .03, .1, .08, .002 if thermal else 0], (7, 1))
                matrix = namespace["friction_matrix"](dq[None], temperature[None], rigid[None],
                                                       np.full(7, .15), np.full(7, .05), np.full(7, 25), True, thermal)
                torque = (rigid + config["friction_scale"] * (matrix @ coefficients.flatten())
                          + config["kp"] * (target-q) - config["kd"] * dq)
                cases.append({key: value.tolist() for key, value in dict(q=q, dq=dq, target=target,
                                                                        temperature=temperature, torque=torque).items()})
            references = work / "reference.json"
            references.write_text(json.dumps(cases))
            run([binaries / "joint_impedance_test", config_file, references])
            run(common + ["--check_only", "--sim=false"])
            smoke = run(common + ["--sim=true", "--duration=.15", "--debug_interval=.1"], expected=1 if thermal else 0)
            if not thermal:
                assert "[阻抗调试]" in smoke.stdout and "requested(Nm) sent(Nm) load(raw)" in smoke.stdout, smoke.stdout + smoke.stderr
                quiet = run(common + ["--sim=true", "--duration=.03", "--debug_interval=0"])
                assert "[阻抗调试]" not in quiet.stdout

        run([binaries / "robot_torque_test", urdf])
        friction["urdf_hash"] = "wrong-model"
        (work / "friction_model.json").write_text(json.dumps(friction))
        assert "Friction model/URDF hash mismatch" in run(common + ["--check_only", "--sim=false"], expected=1).stderr
        del friction["urdf_hash"]  # Old exports without provenance remain supported.
        (work / "friction_model.json").write_text(json.dumps(friction))
        run(common + ["--check_only", "--sim=false"])
        for scale in (0, 1, -.1, 1.1, float("nan"), float("inf")):
            config["friction_scale"] = scale
            config_file.write_text(yaml.safe_dump(config))
            valid = 0 <= scale <= 1
            result = run(common + ["--check_only"], expected=0 if valid else 1)
            if not valid:
                assert "friction_scale must be finite and in [0, 1]" in result.stderr
        config["friction_scale"] = .7
        config["kp"][0] = -1
        config_file.write_text(yaml.safe_dump(config))
        assert "kp must be nonnegative" in run(common + ["--check_only"], expected=1).stderr
        config["kp"][0] = 20
        for bad_limit, error in ((None, "Missing velocity_limit"), ([.4]*6, "Wrong length: velocity_limit"),
                                 ([0]*7, "velocity_limit must be positive"), ([-.1]*7, "velocity_limit must be positive"),
                                 ([float("nan")]*7, "Nonfinite values: velocity_limit"),
                                 ([1000]*7, "velocity_limit exceeds URDF velocity limit")):
            if bad_limit is None:
                config.pop("velocity_limit")
            else:
                config["velocity_limit"] = bad_limit
            config_file.write_text(yaml.safe_dump(config))
            rejected = run(common + ["--check_only"], expected=1)
            assert error in rejected.stderr, (bad_limit, rejected.stderr)
        config["velocity_limit"] = [.4]*7
        config_file.write_text(yaml.safe_dump(config))
        assert "debug_interval must be" in run(common + ["--check_only", "--debug_interval=.01"], expected=1).stderr
        urdf.write_bytes(urdf.read_bytes() + b"\n")
        assert "hash mismatch" in run(common + ["--check_only"], expected=1).stderr
        np.savez_compressed(work / "identification.npz", P=P, base=beta+1)
        run([sys.executable, root / "tools/export_impedance_model.py", work, work / "bad.json"], expected=1)
    print("PASS: 100 independent KDL URDF references, 200 Studio references, debug output, bidirectional speed guards, YAML validation, HardwareSim and atomic torque validation")


if __name__ == "__main__":
    main()
