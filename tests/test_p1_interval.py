"""The P1 interval is strict, and a missing return is not clearance."""

from holo_fleet.control.separation import swept_margin


def test_separation_accounts_for_both_vehicles_and_the_entire_interval():
    assert swept_margin(1.25, 0.5, 1.0, 1.0, 0.25) < 0.0
    assert swept_margin(1.25, 0.5, 0.5, 1.0, 0.25) == 0.0
    assert swept_margin(1.30, 0.5, 0.5, 1.0, 0.25) > 0.0

def test_empty_observation_does_not_supply_a_separation_bound():
    from types import SimpleNamespace
    from holo_fleet.config import DEFAULT
    from holo_fleet.control.separation import interval_record

    assert interval_record(SimpleNamespace(targets=[]), None, None, None, 0.1, DEFAULT) == {"status": "UNOBSERVED"}

def test_timestamps_and_nonfinite_values_cannot_supply_fresh_sensor_evidence():
    from holo_fleet.perception.frame import SensorFrame

    frame = SensorFrame(1.0, {"future": [1.0], "invalid": [float("nan")], "valid": [1.0]},
                        {"future": 1.01, "invalid": 0.9, "valid": 0.9})
    assert frame.sample("future", 0.2, 1) is None
    assert frame.sample("invalid", 0.2, 1) is None
    assert frame.sample("valid", 0.2, 1).tolist() == [1.0]


def test_invalid_dvl_cannot_support_a_conditional_interval():
    from types import SimpleNamespace

    import numpy as np

    from holo_fleet.config import DEFAULT
    from holo_fleet.control.separation import interval_record
    from holo_fleet.perception.frame import SensorFrame

    obs = SimpleNamespace(targets=[object()], unhealthy=(), sense_ok=True, d_min=3.0, v_world=np.zeros(3))
    data = {"DVLSensor": [float("nan"), 0.0, 0.0], "IMUSensor": np.zeros((2, 3))}
    frame = SensorFrame(1.0, data, {name: 1.0 for name in data})
    assert interval_record(obs, frame, np.zeros(3), np.zeros(8), 0.1, DEFAULT)["status"] == "MOTION_UNOBSERVED"
