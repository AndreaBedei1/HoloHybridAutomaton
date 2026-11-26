"""The P1 interval is strict, and a missing return is not clearance."""

from types import SimpleNamespace

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.control.controller import DroneController
from holo_fleet.control.separation import interval_record, swept_margin
from holo_fleet.ha.spec import Mode
from holo_fleet.perception.frame import SensorFrame
from holo_fleet.perception.sonar_geometry import SECTORS, sonar_sensor_name
from holo_fleet.perception.sonar_processing import DYNAMIC, UNCONFIRMED, Echo, SectorReading
from holo_fleet.sim.scenarios import SCENARIOS


def _controller():
    return DroneController(SCENARIOS["p1_head_on"](DEFAULT).plans[0])


def _frame(t, *, echoes=(), missing=(), stamps=None):
    data = {"DVLSensor": np.zeros(7), "Compass": [1.0, 0.0, 0.0], "DepthSensor": [-5.0],
            "IMUSensor": np.zeros((4, 3))}
    ranges = DEFAULT.perc.sonar.ranges()
    for sector in SECTORS:
        if sector not in missing:
            data[sonar_sensor_name(sector)] = np.zeros(ranges.size)
    for sector, lo, hi in echoes:
        data[sonar_sensor_name(sector)][(ranges >= lo) & (ranges <= hi)] = 0.9
    times = {key: t for key in data}
    for sector, stamp in (stamps or {}).items():
        times[sonar_sensor_name(sector)] = stamp
    return SensorFrame(t, data, times)


def test_reusing_a_sonar_frame_cannot_confirm_its_echo():
    ctrl = _controller()
    returns = (("FRONT", 2.5, 2.7),)
    ctrl.step(_frame(0.0, echoes=returns), 0.1)
    assert ctrl.last_obs.readings["FRONT"].echoes[0].cls == UNCONFIRMED
    ctrl.step(_frame(0.1, echoes=returns, stamps={"FRONT": 0.0}), 0.1)
    assert ctrl.last_obs.readings["FRONT"].echoes[0].cls == UNCONFIRMED
    ctrl.step(_frame(0.2, echoes=returns), 0.1)
    assert ctrl.last_obs.readings["FRONT"].echoes[0].cls == DYNAMIC


def test_nearest_bound_ages_through_sonar_loss_but_cannot_jump_over_a_gap():
    ctrl = _controller()
    returns = (("FRONT", 3.2, 3.45),)
    ctrl.step(_frame(0.0, echoes=returns), 0.1)
    ctrl.step(_frame(0.1, echoes=returns), 0.1)
    before = ctrl.last_record["p1_interval"]
    assert before["status"] == "INTERVAL_CONDITIONAL"
    ctrl.step(_frame(0.2, missing=SECTORS), 0.1)
    during = ctrl.last_record["p1_interval"]
    assert during["status"] == "INTERVAL_CONDITIONAL"
    assert during["source"] == "PROPAGATED"
    assert during["lower_m"] == pytest.approx(before["lower_m"] - 0.14)
    ctrl.step(_frame(0.4, missing=SECTORS), 0.1)
    assert ctrl.last_record["p1_interval"]["status"] == "UNOBSERVED"


def test_a_staggered_capture_cannot_start_a_nearest_bound():
    ctrl = _controller()
    returns = (("FRONT", 3.2, 3.45),)
    ctrl.step(_frame(0.1, echoes=returns, stamps={"LEFT": 0.0}), 0.1)
    ctrl.step(_frame(0.2, echoes=returns, stamps={"LEFT": 0.1}), 0.1)
    assert ctrl.last_obs.targets
    assert ctrl.last_record["p1_interval"]["status"] == "UNOBSERVED"


def test_a_near_unconfirmed_echo_cannot_be_masked_by_a_far_confirmed_echo():
    ctrl = _controller()
    far = (("FRONT", 3.2, 3.45),)
    ctrl.step(_frame(0.0, echoes=far), 0.1)
    ctrl.step(_frame(0.1, echoes=far + (("LEFT", 0.75, 0.9),)), 0.1)
    assert ctrl.last_obs.d_min > 2.0
    assert ctrl.last_obs.readings["LEFT"].echoes[0].cls == UNCONFIRMED
    assert ctrl.last_record["p1_interval"]["status"] == "NO_MARGIN"


def test_a_new_near_return_limits_the_bound_during_partial_sonar_loss():
    ctrl = _controller()
    ctrl.step(_frame(0.0, echoes=(("FRONT", 3.2, 3.45),)), 0.1)
    assert ctrl.last_record["p1_interval"]["status"] == "INTERVAL_CONDITIONAL"
    ctrl.step(_frame(0.1, echoes=(("LEFT", 0.75, 0.9),), missing=("FRONT",)), 0.1)
    assert ctrl.last_record["p1_interval"]["status"] == "NO_MARGIN"


def test_greedy_merge_of_equal_ranges_keeps_independent_threat_hypotheses():
    ctrl = _controller()
    returns = (("FRONT", 1.5, 1.7), ("LEFT", 1.5, 1.7))
    ctrl.step(_frame(0.0, echoes=returns), 0.1)
    ctrl.step(_frame(0.1, echoes=returns), 0.1)
    assert [target.pattern for target in ctrl.last_obs.targets] == [frozenset({"FRONT", "LEFT"})]
    patterns = {pattern for pattern, _ in ctrl.flows.threats(ctrl.last_obs, DEFAULT.sep.d_warning)}
    assert patterns == {frozenset({"FRONT", "LEFT"}), frozenset({"FRONT"}), frozenset({"LEFT"})}


def test_opposing_confirmed_threats_request_hold_when_no_direction_opens_both():
    ctrl = _controller()
    returns = (("FRONT", 1.5, 1.7), ("REAR", 1.5, 1.7))
    ctrl.step(_frame(0.0, echoes=returns), 0.1)
    ctrl.step(_frame(0.1, echoes=returns), 0.1)
    assert ctrl.last_record["mode"] == "COLLISION_AVOIDANCE"
    assert not ctrl.flows.last_choice.feasible
    assert ctrl.last_record["v_cmd"] == [0.0, 0.0, 0.0]


def test_nonadjacent_sonar_threat_preempts_formation_recovery():
    plan = SCENARIOS["p1_two_lines"](DEFAULT).plans[0]
    ctrl = DroneController(plan)
    ctrl.ha.mode = Mode.FORMATION_RECOVERY
    # The displaced middle slot returns from front. An outer slot has closed
    # from the side, outside its planned range. Echoes carry neither identity.
    returns = (("FRONT", 3.2, 3.45), ("LEFT", 1.5, 1.7))
    ctrl.step(_frame(0.0, echoes=returns), 0.1)
    ctrl.step(_frame(0.1, echoes=returns), 0.1)
    assert ctrl.last_record["mode"] == "COLLISION_AVOIDANCE"
    assert [pattern for pattern, _ in ctrl.flows.threats(ctrl.last_obs, DEFAULT.sep.d_ca_exit)] == [frozenset({"LEFT"})]


def test_known_blind_range_limits_far_echo_evidence():
    readings = {s: SectorReading(s, age=0.0, healthy=True) for s in SECTORS}
    readings["DOWN"].blind_from = 1.2
    readings["FRONT"].echoes = [Echo("FRONT", 3.2, 3.45, 0.9)]
    obs = SimpleNamespace(readings=readings, v_world=np.zeros(3))
    assert interval_record(obs, _frame(1.0), np.zeros(3), np.zeros(8), 0.1, DEFAULT)["status"] == "NO_MARGIN"


def test_an_occluded_far_echo_cannot_be_needed_for_a_nearest_bound():
    visible = (("LEFT", 2.9, 3.05),)
    unobstructed = _controller()
    unobstructed.step(_frame(0.0, echoes=visible + (("LEFT", 6.3, 6.5),)), 0.1)
    occluded = _controller()
    occluded.step(_frame(0.0, echoes=visible), 0.1)
    assert occluded.last_record["p1_interval"]["status"] == "INTERVAL_CONDITIONAL"
    assert occluded.last_record["p1_interval"]["lower_m"] == unobstructed.last_record["p1_interval"]["lower_m"]


def test_motion_bound_is_checked_after_the_final_velocity_filter(monkeypatch):
    ctrl = _controller()
    monkeypatch.setattr(ctrl.flows, "structure_filter", lambda _obs, _v: np.array([0.6, 0.0, 0.0]))
    ctrl.step(_frame(0.0, echoes=(("FRONT", 3.2, 3.45),)), 0.1)
    assert ctrl.last_record["v_cmd"] == [0.6, 0.0, 0.0]
    assert ctrl.last_record["p1_interval"]["status"] == "MOTION_OUTSIDE"


def test_separation_accounts_for_both_vehicles_and_the_entire_interval():
    assert swept_margin(1.25, 0.5, 1.0, 1.0, 0.25) < 0.0
    assert swept_margin(1.25, 0.5, 0.5, 1.0, 0.25) == 0.0
    assert swept_margin(1.30, 0.5, 0.5, 1.0, 0.25) > 0.0

def test_empty_observation_does_not_supply_a_separation_bound():
    assert interval_record(SimpleNamespace(readings={}), None, None, None, 0.1, DEFAULT) == {"status": "UNOBSERVED"}


def test_timestamps_and_nonfinite_values_cannot_supply_fresh_sensor_evidence():
    frame = SensorFrame(1.0, {"future": [1.0], "invalid": [float("nan")], "valid": [1.0]},
                        {"future": 1.01, "invalid": 0.9, "valid": 0.9})
    assert frame.sample("future", 0.2, 1) is None
    assert frame.sample("invalid", 0.2, 1) is None
    assert frame.sample("valid", 0.2, 1).tolist() == [1.0]


def test_invalid_dvl_cannot_support_a_conditional_interval():
    readings = {s: SectorReading(s, age=0.0, healthy=True) for s in SECTORS}
    readings["FRONT"].echoes = [Echo("FRONT", 3.0, 3.2, 0.9)]
    obs = SimpleNamespace(readings=readings, v_world=np.zeros(3))
    data = {"DVLSensor": [float("nan"), 0.0, 0.0], "IMUSensor": np.zeros((2, 3))}
    frame = SensorFrame(1.0, data, {name: 1.0 for name in data})
    assert interval_record(obs, frame, np.zeros(3), np.zeros(8), 0.1, DEFAULT)["status"] == "MOTION_UNOBSERVED"
