"""Premise Q of the occupancy latch (formal/check_mutex.py M3q / M3hq / M4dq): the referee monitor ``queue_cls``.

A queued drone (where the latch model M3q applies) classifies at its navigation estimate; the monitor flags a queued sample
whose error exceeds QUEUE_CLS_ERR_M on a horizontal axis of the gate frame, or whose classification pose comes from a beacon
fix or a structure fit.  Other steps before the commitment within sonar range (approach, retreat after a give-up) are
outside that model and only counted by source.  Committed samples, samples past the CR entry and samples beyond sonar range
are not considered.  Premise Q2: the FRONT sonar of a queued drone off the axis (the body plus FRONT_LEVER_M at the heading
tolerance) is at most QUEUE_OUTWARD_M outward of its queue point, on either side of the axis.
"""

import math
from types import SimpleNamespace

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.referee.robustness_metrics import FRONT_LEVER_M, QUEUE_CLS_ERR_M, QUEUE_OUTWARD_M, premise_monitors
from holo_fleet.sim.scenarios import SCENARIOS

GATE = SCENARIOS["line_parallel_mutex"](DEFAULT).plans[0].gates[0]


class _Still:
    def describe(self):
        return []


def _monitors(rows_per_drone, truth, queue_l=None, plans=True):
    """queue_l: the planned queue lateral of each drone (default: the true lateral of its first sample)."""
    names = [f"drone_{k}" for k in range(len(rows_per_drone))]
    tr = np.asarray(truth, float)
    if queue_l is None:
        queue_l = [float(GATE.to_gate_frame(tr[0, k])[1]) for k in range(len(rows_per_drone))]
    sc = SimpleNamespace(judged_gates=[GATE], sim=SimpleNamespace(current=_Still()),
                         plans=[SimpleNamespace(queue_lateral=ql) for ql in queue_l] if plans else [])
    t = np.arange(len(rows_per_drone[0]), dtype=float) * 0.1
    return premise_monitors(sc, DEFAULT, names, [], t, np.asarray(truth, float), rows_per_drone, len(t))


def _at(s, lat):
    return np.asarray(GATE.from_gate_frame([s, lat, 0.0]), float)


def _row(p, s, committed=False, src="", off=None, queued=True):
    return {"mode": "MUTEX_APPROACH", "nav_p": list(p), "committed": committed, "cls_source": src, "cls_offset": off,
            "gate": {"id": GATE.gate_id, "s": s, "at_queue": "True" if queued else "False"}}


def test_the_bound_is_the_navigation_premise_per_axis():
    assert QUEUE_CLS_ERR_M == DEFAULT.gate.nav_err_m == DEFAULT.acoustic.nav_err_gate_m


def test_an_error_within_the_bound_on_both_axes_holds():
    truth = _at(-5.4, 1.75)
    est = _at(-5.4 + QUEUE_CLS_ERR_M - 0.05, 1.75 - QUEUE_CLS_ERR_M + 0.05)  # diagonal: above the bound in norm
    q = _monitors([[_row(est, -5.4)]], [[truth]])["queue_cls"]
    assert q["holds"] and q["samples"] == 1 and q["sources"] == {"nav": 1}


def test_an_error_beyond_the_bound_on_one_axis_is_flagged():
    truth = _at(-5.4, 0.0)
    q = _monitors([[_row(_at(-5.4 + QUEUE_CLS_ERR_M + 0.05, 0.0), -5.4)]], [[truth]])["queue_cls"]
    assert q["beyond"] == 1 and not q["holds"] and q["max_axis_m"] > QUEUE_CLS_ERR_M


def test_a_fit_pose_before_the_commitment_is_outside_the_model():
    truth = _at(-5.4, 0.0)
    q = _monitors([[_row(truth, -5.4, src="fit", off=[0.0, 0.0])]], [[truth]])["queue_cls"]
    assert q["beyond"] == 1 and q["sources"] == {"fit": 1}


def test_committed_past_the_entry_or_out_of_range_samples_are_not_judged():
    far = _at(-5.4, 0.0) + np.array([3.0, 0.0, 0.0])
    rows = [[_row(far, -5.4, committed=True), _row(far, -0.5), _row(far, -DEFAULT.perc.sonar.range_max - 1.0)]]
    truth = [[_at(-5.4, 0.0)]] * 3
    q = _monitors(rows, truth)["queue_cls"]
    assert q["samples"] == 0 and q["holds"]


def test_a_beacon_pose_while_retreating_is_counted_outside_the_model_not_judged():
    truth = _at(-4.0, 0.0)
    out = _monitors([[_row(truth, -4.0, src="beacon", off=[0.0, 0.0], queued=False)]], [[truth]])["queue_cls"]
    assert out["holds"] and out["samples"] == 0 and out["outside_queue_sources"] == {"beacon": 1}


def _gate_vec(ds, dl):
    """World-frame horizontal vector of a gate-frame displacement (ds along the axis, dl lateral)."""
    return (_at(-5.4 + ds, dl) - _at(-5.4, 0.0))[:2].tolist()


def test_the_classification_offset_of_a_queued_beacon_pose_is_measured_not_the_raw_estimate():
    truth = _at(-5.4, 0.0)
    away = _monitors([[_row(truth, -5.4, src="beacon", off=_gate_vec(1.2, 0.0))]], [[truth]])["queue_cls"]
    assert away["beyond"] == 1 and abs(away["max_axis_m"] - 1.2) < 2e-3 and away["sources"] == {"beacon": 1}
    # a navigation error cancelled by the offset: the classification pose is the truth (still a beacon pose, so flagged)
    back = _monitors([[_row(_at(-5.4 - 1.2, 0.0), -5.4, src="beacon", off=_gate_vec(1.2, 0.0))]], [[truth]])["queue_cls"]
    assert back["beyond"] == 1 and back["max_axis_m"] < 2e-3


def test_an_offset_on_a_navigation_sample_is_ignored():
    truth = _at(-5.4, 0.0)
    q = _monitors([[_row(truth, -5.4, src="", off=_gate_vec(1.5, 0.0))]], [[truth]])["queue_cls"]
    assert q["holds"] and q["max_axis_m"] < 2e-3 and q["sources"] == {"nav": 1}


def test_a_lateral_error_is_judged_on_its_own_axis():
    truth = _at(-5.4, 1.75)
    inside = _monitors([[_row(_at(-5.4, 1.75 + QUEUE_CLS_ERR_M - 1e-6), -5.4)]], [[truth]])["queue_cls"]
    assert inside["holds"]
    out = _monitors([[_row(_at(-5.4, 1.75 - QUEUE_CLS_ERR_M - 0.2), -5.4)]], [[truth]])["queue_cls"]
    assert out["beyond"] == 1 and abs(out["max_axis_m"] - (QUEUE_CLS_ERR_M + 0.2)) < 2e-3


def test_two_drones_over_several_steps_are_indexed_per_drone_and_step():
    t0, t1 = _at(-5.4, -1.75), _at(-5.4, 1.75)
    clean = [_row(t0, -5.4)] * 3
    late = [_row(t1, -5.4), _row(t1, -5.4), _row(_at(-5.4, 1.75 + 1.3), -5.4)]
    truth = [[t0, t1]] * 3
    q = _monitors([clean, late], truth)["queue_cls"]
    assert q["samples"] == 6 and q["beyond"] == 1 and abs(q["max_axis_m"] - 1.3) < 2e-3 and q["sources"] == {"nav": 6}


def test_rows_without_an_estimate_or_a_judged_gate_are_skipped_and_a_missing_source_is_navigation():
    truth = _at(-5.4, 0.0)
    no_nav = dict(_row(truth, -5.4), nav_p=None)
    other_gate = dict(_row(truth, -5.4), gate={"id": "not_judged", "s": -5.4, "at_queue": "True"})
    none_src = dict(_row(truth, -5.4), cls_source=None)
    assert _monitors([[no_nav]], [[truth]])["queue_cls"]["samples"] == 0
    assert _monitors([[other_gate]], [[truth]])["queue_cls"]["samples"] == 0
    assert _monitors([[none_src]], [[truth]])["queue_cls"]["sources"] == {"nav": 1}


def test_the_judged_interval_is_from_sonar_range_to_the_cr_entry_excluded():
    far, edge = -DEFAULT.perc.sonar.range_max, DEFAULT.gate.cr_s_min
    truth = _at(far, 0.0)
    assert _monitors([[_row(truth, far)]], [[truth]])["queue_cls"]["samples"] == 1
    assert _monitors([[_row(truth, edge)]], [[truth]])["queue_cls"]["samples"] == 0


def test_a_violation_raises_the_cause_tag():
    from holo_fleet.referee.robustness_metrics import cause_tags

    def tags(beyond):
        out = {"queue_cls": {"beyond": beyond}, "P1": {"holds": True, "collisions": []}, "P2": {"holds": True}}
        return cause_tags(out, {}, [], {}, [[]], [], 0, 0.1, {})

    assert "QUEUE_CLS_PREMISE_VIOLATED" in tags(1) and "QUEUE_CLS_PREMISE_VIOLATED" not in tags(0)


def test_a_true_position_too_far_outward_of_the_queue_point_is_flagged():
    est = _at(-5.75, 3.5)  # the estimate at the outer queue point of 3
    far = _monitors([[_row(est, -5.75)]], [[_at(-5.75, 3.5 + QUEUE_OUTWARD_M + 0.05)]], queue_l=[3.5])["queue_cls"]
    assert not far["holds"] and far["outward_beyond"] == 1 and far["max_outward_m"] > QUEUE_OUTWARD_M
    near = _monitors([[_row(est, -5.75)]], [[_at(-5.75, 3.5 + QUEUE_OUTWARD_M - 0.05)]], queue_l=[3.5])["queue_cls"]
    assert near["holds"] and near["outward_beyond"] == 0


def test_an_inward_offset_and_the_centre_point_are_not_bounded_by_the_outward_premise():
    inward = _monitors([[_row(_at(-5.75, 3.5), -5.75)]], [[_at(-5.75, 3.5 - 0.95)]], queue_l=[3.5])["queue_cls"]
    assert inward["holds"] and inward["outward_beyond"] == 0
    centre = _monitors([[_row(_at(-5.75, 0.0), -5.75)]], [[_at(-5.75, 0.95)]], queue_l=[0.0])["queue_cls"]
    assert centre["holds"]


def test_the_outward_offset_is_judged_at_the_front_sonar():
    assert abs(FRONT_LEVER_M - 0.24 * math.sin(math.radians(DEFAULT.gate.heading_tol_deg))) < 1e-9 and FRONT_LEVER_M > 0.02
    est = _at(-5.75, 3.5)
    body_in = _monitors([[_row(est, -5.75)]], [[_at(-5.75, 3.5 + QUEUE_OUTWARD_M - FRONT_LEVER_M / 2)]], queue_l=[3.5])
    assert body_in["queue_cls"]["outward_beyond"] == 1 and not body_in["queue_cls"]["holds"]  # the sonar is beyond
    ok = _monitors([[_row(est, -5.75)]], [[_at(-5.75, 3.5 + QUEUE_OUTWARD_M - FRONT_LEVER_M - 0.005)]], queue_l=[3.5])
    assert ok["queue_cls"]["holds"] and abs(ok["queue_cls"]["max_outward_m"] - (QUEUE_OUTWARD_M - 0.005)) < 2e-3


def test_outward_is_away_from_the_axis_on_both_sides_and_at_every_off_axis_point():
    for ql in (-3.5, -1.75, 1.75, 3.5):
        side = math.copysign(1.0, ql)
        est = _at(-5.75, ql)
        far = _monitors([[_row(est, -5.75)]], [[_at(-5.75, ql + side * (QUEUE_OUTWARD_M + 0.05))]], queue_l=[ql])
        assert far["queue_cls"]["outward_beyond"] == 1, ql
        inward = _monitors([[_row(est, -5.75)]], [[_at(-5.75, ql - side * 0.95)]], queue_l=[ql])
        assert inward["queue_cls"]["holds"] and inward["queue_cls"]["outward_beyond"] == 0, ql


def test_a_queued_sample_without_a_plan_fails_closed():
    truth = _at(-5.4, 1.75)
    q = _monitors([[_row(truth, -5.4)]], [[truth]], plans=False)["queue_cls"]
    assert q["unjudged"] == 1 and not q["holds"]
