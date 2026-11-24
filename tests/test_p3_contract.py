"""The P3 contract (referee/p3.py) and the outcome classes of robustness_metrics (criteria v8 and v9)."""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.referee import p3
from holo_fleet.referee.criteria import CRITERIA_VERSION, E_TOL_M, t_hold_s
from holo_fleet.referee.robustness_metrics import p3_late_parts, refine_v8, refine_v9


def test_tolerance_hold_and_closing_speed_are_the_declared_ones():
    assert E_TOL_M == 1.0 and abs(t_hold_s(DEFAULT) - 8.9) < 0.05 and CRITERIA_VERSION == "v9"
    assert abs(p3.V_CLOSE(DEFAULT) - (DEFAULT.form.v_slot_max - DEFAULT.form.v_nominal)) < 1e-12 and p3.V_CLOSE(DEFAULT) > 0.0


def test_recovery_needs_the_whole_hold_inside_the_band_and_starting_before_the_deadline():
    t = np.arange(0.0, 40.0, 0.1)
    ok = (t >= 10.0) & (t < 15.0) | (t >= 20.0)
    assert p3._held(t, ok, 0, 30.0, 8.9) == 20.0
    assert p3._held(t, ok, 0, 19.0, 8.9) is None
    assert abs(p3._longest(t, ~ok, 0, len(t) - 1) - 9.9) < 0.11


def _out(cls_failed=(), a=None, b=None, c=None, gate_nav=0, self_induced=False, pv=0):
    return {"failed_requirements": list(cls_failed), "p3": {"P3a": {"d": {"verdict": a}} if a else {},
                                                            "P3b": {"verdict": b} if b else {}, "P3c": {"verdict": c} if c else {}},
            "gate_nav": {"committed_beyond": gate_nav, "self_induced": self_induced}, "premise_v": {"live_declared_dead": pv}}


def test_the_classes_of_criteria_v8():
    assert refine_v8("PASS", _out(a="MET", b="MET")) == "PASS"
    assert refine_v8("PASS", _out(b="NOT_MET")) == "P3_FAILURE"
    assert refine_v8("PASS", _out(a="PREMISE_VIOLATED", b="MET")) == "PREMISE_VIOLATED"
    assert refine_v8("INCOMPLETE", _out(("not waiting",), b="MET")) == "P3_FAILURE"
    assert refine_v8("INCOMPLETE", _out(("gate", "progress"))) == "INCOMPLETE"
    assert refine_v8("INCOMPLETE", _out(("gate",), gate_nav=5)) == "PREMISE_VIOLATED"
    assert refine_v8("INCOMPLETE", _out(("gate",), gate_nav=5, self_induced=True)) == "INCOMPLETE"
    assert refine_v8("SAFETY_FAILURE", _out(b="MET")) == "SAFETY_FAILURE"


def test_a_late_recovery_is_not_a_pass_nor_a_failure():
    assert refine_v8("PASS", _out(b="LATE")) == "P3_LATE"
    assert refine_v8("PASS", _out(a="LATE", b="NOT_MET")) == "P3_FAILURE"
    assert refine_v8("INCOMPLETE", _out(("gate",), b="LATE")) == "INCOMPLETE"


def test_criteria_v9_p3_is_eventual_recovery():
    """Within or after the old deadline, a completed run whose formation recovered is PASS; the late parts stay recorded."""
    assert refine_v9("PASS", _out(a="MET", b="MET")) == "PASS"
    late = _out(a="LATE", b="LATE")
    assert refine_v8("PASS", late) == "P3_LATE" and refine_v9("PASS", late) == "PASS"
    assert p3_late_parts(late) == ["P3a:d", "P3b"]
    assert p3_late_parts(_out(a="MET", b="MET")) == []


def test_criteria_v9_never_promotes_a_real_failure():
    assert refine_v9("PASS", _out(b="NOT_MET")) == "P3_FAILURE"
    assert refine_v9("PASS", _out(a="LATE", b="NOT_MET")) == "P3_FAILURE"
    assert refine_v9("INCOMPLETE", _out(("progress",), b="LATE")) == "INCOMPLETE"
    assert refine_v9("INCOMPLETE", _out(("gate",), b="LATE")) == "INCOMPLETE"
    assert refine_v9("INCOMPLETE", _out(("no false vacancy",), b="MET")) == "P3_FAILURE"
    assert refine_v9("PASS", _out(a="LATE", b="PREMISE_VIOLATED")) == "PREMISE_VIOLATED"
    assert refine_v9("SAFETY_FAILURE", _out(b="LATE")) == "SAFETY_FAILURE"
    assert refine_v9("INVALID", _out(b="LATE")) == "INVALID"


def test_criteria_v9_degraded_formation_after_a_permanent_failure():
    """P3c (the degraded formation of the healthy drones, re-formed and vacant) recovered late is PASS; never re-formed
    is P3_FAILURE; an inadmissible failure stays PREMISE_VIOLATED."""
    assert refine_v9("PASS", _out(c="MET")) == "PASS"
    assert refine_v9("PASS", _out(c="LATE")) == "PASS"
    assert refine_v9("PASS", _out(c="NOT_MET")) == "P3_FAILURE"
    assert refine_v9("PASS", _out(c="NOT_ADMISSIBLE")) == "PREMISE_VIOLATED"


def test_the_b2_medium_minus_one_metre_bias_run_is_a_late_pass():
    """gate_hold_nav_bias_m10, B2 MEDIUM, seed 8000 (negative navigation bias run of the legacy results): online SUCCESS, no failed requirement, P3a of two
    drones LATE (the ~217 s wait at the queue), P3b MET at 348.5 s: v8 P3_LATE, v9 PASS with the late parts recorded."""
    out = {"failed_requirements": [], "gate_nav": {}, "premise_v": {},
           "p3": {"P3a": {"drone_0": {"verdict": "LATE"}, "drone_1": {"verdict": "MET"}, "drone_2": {"verdict": "LATE"}},
                  "P3b": {"verdict": "MET", "recovered": 348.5}}}
    assert refine_v8("PASS", out) == "P3_LATE"
    assert refine_v9("PASS", out) == "PASS"
    assert p3_late_parts(out) == ["P3a:drone_0", "P3a:drone_2"]
