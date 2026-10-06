"""The controller side never reads simulator ground truth and never talks to other drones."""

import ast
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_PACKAGES = ["holo_fleet/ha", "holo_fleet/perception", "holo_fleet/control"]
FORBIDDEN_IMPORTS = ("holoocean", "holo_fleet.sim", "holo_fleet.referee", "holo_fleet.analysis", "holo_fleet.runner",
                     "holo_fleet.comms")
GROUND_TRUTH_KEYS = ("PoseSensor", "VelocitySensor", "LocationSensor", "RotationSensor", "DynamicsSensor",
                     "CollisionSensor", "GPSSensor")


def _py_files():
    for pkg in CONTROLLER_PACKAGES:
        yield from sorted((ROOT / pkg).rglob("*.py"))


def _imports(path: Path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                yield a.name
        elif isinstance(node, ast.ImportFrom) and node.module:
            yield node.module


@pytest.mark.parametrize("path", list(_py_files()), ids=lambda p: str(p.relative_to(ROOT)))
def test_controller_packages_do_not_import_simulator_or_referee(path):
    bad = [m for m in _imports(path) if any(m == f or m.startswith(f + ".") for f in FORBIDDEN_IMPORTS)]
    assert not bad, f"{path} imports {bad}"


@pytest.mark.parametrize("path", list(_py_files()), ids=lambda p: str(p.relative_to(ROOT)))
def test_controller_code_never_names_ground_truth_sensors(path):
    if path.name == "sensor_suite.py":      # holds the PRIVILEGED list used for stripping
        return
    text = path.read_text(encoding="utf-8")
    for key in GROUND_TRUTH_KEYS:
        assert key not in text, f"{path} mentions {key}"


def test_simulator_strips_privileged_sensors_before_the_controller():
    from holo_fleet.sim.holo_env import HoloFleetSim
    from holo_fleet.sim.scenarios import pair_crossing

    sim = HoloFleetSim(pair_crossing(seed=0))
    name = sim.names[0]
    sim.latest[name] = {"PoseSensor": np.eye(4), "VelocitySensor": np.zeros(3), "CollisionSensor": False,
                        "ChaseCamera": np.zeros((4, 4, 3)), "SideCamera": np.zeros((4, 4, 3)),
                        "DVLSensor": np.zeros(7), "Compass": np.array([1.0, 0, 0]), "DepthSensor": np.array([-4.5]),
                        "ProxSonar_e+00": -np.ones(72)}
    sim.stamp[name] = {k: 0.0 for k in sim.latest[name]}
    frame = sim.frames()[name]
    for key in ("PoseSensor", "VelocitySensor", "CollisionSensor", "ChaseCamera", "SideCamera"):
        assert key not in frame.data and key not in frame.stamp
    for key in ("DVLSensor", "Compass", "DepthSensor", "ProxSonar_e+00"):
        assert key in frame.data


def test_controller_uses_ground_truth_flag_is_false():
    from holo_fleet.control.controller import DroneController

    assert DroneController.uses_ground_truth is False
