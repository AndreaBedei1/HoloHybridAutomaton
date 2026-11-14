"""The P3 contract (referee/p3.py) and the classes of criteria v8 (robustness_metrics.refine_v8)."""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.referee import p3
from holo_fleet.referee.criteria import CRITERIA_VERSION, E_TOL_M, t_hold_s
from holo_fleet.referee.robustness_metrics import refine_v8


def test_tolerance_hold_and_closing_speed_are_the_declared_ones():
    assert E_TOL_M == 1.0 and abs(t_hold_s(DEFAULT) - 8.9) < 0.05 and CRITERIA_VERSION == "v8"
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
