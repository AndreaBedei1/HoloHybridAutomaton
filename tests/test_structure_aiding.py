"""Map-aided correction of the dead-reckoning position at the gate (perception/structure_aiding.py).

Synthetic sonar profiles are generated from the TRUE pose (leading edge of each sector at the true nearest-bar range);
the drone's estimate carries a lateral error, as the compass bias of the realistic MEDIUM level produces at the gate.
"""

import math

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.perception.sonar_processing import DYNAMIC, EchoClassifier
from holo_fleet.perception.structure_aiding import MAX_CORR_M, StructureAiding
from holo_fleet.sim.scenarios import SCENARIOS

RANGES = DEFAULT.perc.sonar.ranges()
SC = SCENARIOS["lost_drone_mutex"](DEFAULT)
GATE = SC.plans[0].structures[0]
BARS = list(GATE.bars)


def _world(s, l, dz=0.0):
    left = np.array([-GATE.axis[1], GATE.axis[0], 0.0])
    return GATE.center + s * np.asarray(GATE.axis, float) + l * left + np.array([0.0, 0.0, dz])


def _R():
    yaw = math.atan2(GATE.axis[1], GATE.axis[0])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _profiles(clf, p_true, R, extra=None):
    out = {}
    for s in sg.SECTORS:
        prof = np.zeros_like(RANGES)
        a, _ = clf._sector_frame(s, R)
        w = clf.structure_window(p_true + R @ sg.MOUNTS[s], a)
        if w is not None:
            prof[(RANGES >= w[0]) & (RANGES <= w[0] + 0.15)] = 0.9
        if extra and s in extra:
            r = extra[s]
            prof[(RANGES >= r) & (RANGES <= r + 0.15)] = 0.9
        out[s] = prof
    return out


def _run(aid, clf, p_true, p_est, R, n=12, extra=None):
    p = p_est.copy()
    for _ in range(n):
        corr = aid.update(p, R, _profiles(clf, p_true, R, extra), {s: 0.0 for s in sg.SECTORS}, 0.25)
        if corr is not None:
            p = p + corr
    return p


def test_lateral_drift_at_the_gate_is_corrected():
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    aid = StructureAiding(clf, DEFAULT)
    R = _R()
    p_true = _world(-0.9, -0.5)
    p_est = _world(-0.9, 0.0)                         # believes it is centred: 0.5 m lateral error
    p = _run(aid, clf, p_true, p_est, R)
    assert np.linalg.norm(p[:2] - p_true[:2]) < 0.15, (p - p_true)
    assert aid.applied_m <= MAX_CORR_M


def test_no_structure_in_range_means_no_correction():
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    aid = StructureAiding(clf, DEFAULT)
    R = _R()
    p_est = _world(-20.0, 0.0)
    assert np.allclose(_run(aid, clf, _world(-20.0, -0.5), p_est, R), p_est)


def test_a_single_unexplained_echo_never_moves_the_estimate():
    """Only the FRONT sector returns an echo (a vehicle ahead, no bar in the other cones): no correction."""
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    aid = StructureAiding(clf, DEFAULT)
    R = _R()
    p_est = _world(-4.0, 0.0)
    for _ in range(10):
        profs = {s: np.zeros_like(RANGES) for s in sg.SECTORS}
        profs["FRONT"][(RANGES >= 2.0) & (RANGES <= 2.15)] = 0.9
        assert aid.update(p_est, R, profs, {s: 0.0 for s in sg.SECTORS}, 0.25) is None


def test_after_the_correction_a_vehicle_in_front_of_a_bar_stays_an_obstacle():
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    aid = StructureAiding(clf, DEFAULT)
    R = _R()
    p_true = _world(-2.0, -0.4)
    p = _run(aid, clf, p_true, _world(-2.0, 0.0), R)
    profs = _profiles(clf, p_true, R, extra={"FRONT": 0.6})       # a hull 0.6 m ahead, well before the bars
    clf2 = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    for _ in range(2):
        out = clf2.classify(profs, {s: 0.0 for s in sg.SECTORS}, p, R, None)
    assert out["FRONT"].echoes[0].cls == DYNAMIC


def test_an_offset_beyond_the_bound_is_ignored_without_error():
    """Echoes that only an offset beyond MAX_CORR_M would explain: no correction, no exception."""
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    aid = StructureAiding(clf, DEFAULT)
    R = _R()
    p_true = _world(-1.0, -0.5)
    for est in (_world(-1.0, 1.9), _world(1.5, 1.5), _world(-3.5, 1.6)):
        p = _run(aid, clf, p_true, est, R, n=4)                                   # must not raise
        assert np.linalg.norm(p[:2] - est[:2]) <= MAX_CORR_M + 1e-9               # and never moves beyond the bound


def test_one_sector_is_not_enough_to_move_the_estimate():
    """Only FRONT returns the bars (the other sonars occluded): no correction."""
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    aid = StructureAiding(clf, DEFAULT)
    R = _R()
    p_true, p_est = _world(-0.9, -0.5), _world(-0.9, 0.0)
    for _ in range(6):
        profs = _profiles(clf, p_true, R)
        for s in ("REAR", "LEFT", "RIGHT", "UP", "DOWN"):
            profs[s] = np.zeros_like(RANGES)
        assert aid.update(p_est, R, profs, {s: 0.0 for s in sg.SECTORS}, 0.25) is None


def test_the_queue_point_is_out_of_the_aiding_range():
    """At the queue point (5.4 m before the gate, neighbours around) the estimate is never corrected."""
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    aid = StructureAiding(clf, DEFAULT)
    R = _R()
    p_est = _world(-5.4, -3.5)
    assert np.allclose(_run(aid, clf, _world(-5.4, -3.0), p_est, R, n=6), p_est)


def test_a_missing_bar_echo_never_moves_a_correct_estimate():
    """Estimate = truth, one sector's bar echo missing (a fade or an occlusion): no correction (CLEAN behaviour)."""
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    R = _R()
    for s_gate in (-1.6, -1.2, -0.9, -0.5):
        p = _world(s_gate, -0.1)
        for missing in sg.SECTORS:
            aid = StructureAiding(clf, DEFAULT)
            for _ in range(6):
                profs = _profiles(clf, p, R)
                profs[missing] = np.zeros_like(RANGES)
                assert aid.update(p, R, profs, {s: 0.0 for s in sg.SECTORS}, 0.25) is None, (s_gate, missing, aid.last)
