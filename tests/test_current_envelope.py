"""Control-feasible current envelope (DI-27): one definition for runtime, run evaluation and formal checks."""

import json
import sys
from pathlib import Path

import numpy as np
import pytest
import z3

from holo_fleet.config import DEFAULT
from holo_fleet.control.controller import EnvelopeMonitor
from holo_fleet.control.current_envelope import (AUTH_NOMINAL, axis_command, check_current, check_onboard,
                                                 head_limit, lateral_limit, single_axis_speed)
from holo_fleet.control.lowlevel import AUTHORITY

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "formal"))
SURVEY = (DEFAULT.form.v_nominal, 0.0, 0.0)          # 0.30 m/s along +x


def test_lateral_current_admitted():
    c = check_current((0.0, 0.60, 0.0), SURVEY)
    assert c.ok and c.feasible and c.in_range and c.lateral == pytest.approx(0.60) and abs(c.head) < 1e-12


def test_head_current_admitted_up_to_the_authority():
    for h in (0.30, 0.35, 0.40):
        c = check_current((-h, 0.0, 0.0), SURVEY)
        assert c.ok and c.head == pytest.approx(h), (h, c.required_cmd)


def test_head_current_too_strong():
    c = check_current((-0.45, 0.0, 0.0), SURVEY)
    assert not c.feasible and not c.ok and c.in_range            # inside the old 0.6 m/s bound, yet infeasible
    assert c.required_cmd > AUTH_NOMINAL and "head" in c.reason
    assert not check_current((-0.60, 0.0, 0.0), SURVEY).ok          # formation_recovery_head_current


def test_vertical_current():
    assert check_current((0.0, 0.0, 0.20), SURVEY).ok
    c = check_current((0.0, 0.0, -0.30), SURVEY)
    assert c.feasible and not c.in_range and not c.ok and "vertical" in c.reason


def test_diagonal_currents_depend_on_the_direction():
    following = check_current((0.42, 0.42, 0.0), SURVEY)          # |w| = 0.59, with the motion and across
    against = check_current((-0.42, 0.42, 0.0), SURVEY)           # |w| = 0.59, against the motion and across
    assert following.ok and not against.ok
    assert following.magnitude == pytest.approx(against.magnitude)  # same |w|: the scalar bound could not tell
    assert not check_current((0.50, 0.40, 0.0), SURVEY).ok         # |w| = 0.64: beyond the exercised range


def test_mission_speed_changes_the_margin():
    assert head_limit(0.30) == pytest.approx(0.41, abs=0.01)
    assert head_limit(0.50) == pytest.approx(0.21, abs=0.01)
    assert head_limit(0.50) < head_limit(0.40) < head_limit(0.30)
    assert lateral_limit(0.50) < lateral_limit(0.30) <= DEFAULT.env.current_validated_max
    w = (-0.30, 0.0, 0.0)
    assert check_current(w, (0.30, 0.0, 0.0)).ok and not check_current(w, (0.50, 0.0, 0.0)).ok
    # hover / queue hold: the whole current is lateral and only the range limits it
    assert check_current((0.0, -0.60, 0.0), (0.0, 0.0, 0.0)).ok


def test_limits_agree_with_the_calibration():
    cal = json.loads((ROOT / "results" / "calibration" / "head_current_authority.json").read_text())
    held = [c["drift_m_s"] for c in cal["cases"].values() if c["kind"] == "head" and c["monitor_violation_at_s"] is None]
    lost = [c["drift_m_s"] for c in cal["cases"].values() if c["kind"] == "head" and c["monitor_violation_at_s"] is not None]
    assert max(held) <= head_limit(cal["survey_speed_m_s"]) < min(lost)
    sat = [c["through_water_speed_when_saturated_m_s"] for c in cal["cases"].values()
           if c["kind"] == "head" and c["saturated_fraction"] > 0.5]
    tol = DEFAULT.plant.cmd_model_tolerance
    assert all(abs(single_axis_speed() - v) <= tol * v for v in sat)
    lat = cal["cases"]["lateral_0.60"]
    assert lat["monitor_violation_at_s"] is None and check_current((0.0, 0.60, 0.0), SURVEY).ok


def test_onboard_check_is_the_steady_command_of_the_loop():
    # the estimate is in the controller's linear model: steady command = |v_d - w_est| / surge_speed_per_cmd
    c = check_onboard((-0.55, 0.0, 0.0), SURVEY, DEFAULT, AUTHORITY["nominal"])
    assert c.required_cmd == pytest.approx((0.30 + 0.55) / DEFAULT.plant.surge_speed_per_cmd)
    assert c.authority == AUTHORITY["nominal"] == AUTH_NOMINAL and c.ok
    assert not check_onboard((-0.70, 0.0, 0.0), SURVEY, DEFAULT, AUTHORITY["nominal"]).ok


def test_monitor_declares_and_logs_an_infeasible_request():
    mon = EnvelopeMonitor()
    bad = check_onboard((-0.80, 0.0, 0.0), SURVEY, DEFAULT, AUTH_NOMINAL)
    states = [mon.update(False, bad, 0.1) for _ in range(45)]
    assert states[38] and not states[-1]                          # declared after 4 s, not before
    rec = mon.record()
    assert rec["status"] == "ENVELOPE_VIOLATION" and "not control-feasible" in rec["reason"]
    for key in ("current_est", "v_desired", "head", "lateral", "vertical", "through_water", "required_cmd", "authority"):
        assert key in rec
    good = check_onboard((0.0, 0.10, 0.0), SURVEY, DEFAULT, AUTH_NOMINAL)
    for _ in range(41):
        mon.update(False, good, 0.1)
    assert mon.ok and mon.record()["status"] == "ENVELOPE_OK"
    assert mon.update(True, None, 0.1)                            # avoidance manoeuvres are not judged


def test_formal_model_uses_the_same_parameters():
    from check_formation import envelope_terms

    T = envelope_terms(DEFAULT)
    for r in (0.1, 0.3, 0.6, 0.713):
        val = z3.simplify(T["g"](z3.RealVal(str(r))))
        assert float(val.as_fraction()) == pytest.approx(axis_command(r, DEFAULT), rel=1e-9)
    assert float(z3.simplify(T["A"]).as_fraction()) == pytest.approx(AUTHORITY["nominal"])
    assert float(z3.simplify(T["range"]).as_fraction()) == pytest.approx(DEFAULT.env.current_validated_max)


def test_scalar_bound_is_gone_from_the_configuration():
    assert not hasattr(DEFAULT.env, "current_drift_max")
    assert np.isclose(DEFAULT.env.current_validated_max, 0.6) and np.isclose(DEFAULT.env.current_vertical_max, 0.25)
