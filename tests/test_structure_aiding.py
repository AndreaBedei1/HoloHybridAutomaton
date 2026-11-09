"""Map-aided correction of the dead-reckoning position at the gate (perception/structure_aiding.py).

Synthetic sonar profiles are generated from the TRUE pose (leading edge of each sector at the true nearest-bar range
within the nominal cone, the model the CLEAN runs support: structure_aiding._pred); the drone's estimate carries a
lateral error, as the compass bias of the realistic MEDIUM level produces at the gate.
"""

import math

import numpy as np
import pytest

from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates
from holo_fleet.config import DEFAULT
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.perception.sonar_processing import DYNAMIC, EchoClassifier
from holo_fleet.perception.structure_aiding import HULL_LAT_M, MAX_CORR_M, StructureAiding
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


def _first_bar_echo(clf, pos, a):
    d = np.asarray(clf.struct_pts) - pos
    r = np.linalg.norm(d, axis=1)
    ang = np.arccos(np.clip((d @ a) / np.maximum(r, 1e-9), -1.0, 1.0))
    m = clf.model
    sel = (ang <= np.radians(m.half_angle_deg)) & (r >= m.range_min) & (r <= m.range_max)
    return float(r[sel].min()) if sel.any() else None


def _profiles(clf, p_true, R, extra=None):
    out = {}
    for s in sg.SECTORS:
        prof = np.zeros_like(RANGES)
        a, _ = clf._sector_frame(s, R)
        w = _first_bar_echo(clf, p_true + R @ sg.MOUNTS[s], a)
        if w is not None:
            prof[(RANGES >= w) & (RANGES <= w + 0.15)] = 0.9
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


def test_the_lateral_drift_is_removed_during_the_approach_before_the_bars():
    """Closed loop: the drone keeps its estimate on the centre line while it advances through the passage; its true
    lateral offset is the navigation error.  It is corrected before the drone reaches the gate plane, and no
    accepted correction ever increases the error."""
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    R = _R()
    for e0 in (-0.55, -0.35, 0.45):
        aid = StructureAiding(clf, DEFAULT)
        e = e0                                              # true lateral position - estimated (estimate held at 0)
        for s_gate in np.arange(-2.2, -0.3, 0.03):         # 0.3 m/s at 10 Hz
            p_est = _world(s_gate, 0.0)
            corr = aid.update(p_est, R, _profiles(clf, _world(s_gate, e), R), {s: 0.0 for s in sg.SECTORS}, 0.25)
            if corr is not None:
                c_l = float(corr @ np.array([-R[1, 0], R[0, 0], 0.0]))
                assert abs(e - c_l) <= abs(e) + 0.05, (e0, s_gate, e, c_l)
                e -= c_l                                    # the drone re-centres its corrected estimate
        assert abs(e) < 0.15, (e0, e, aid.last)


def test_a_silent_sector_does_not_block_a_well_matched_correction():
    """A vertical sonar that does not return the bar (a weak grazing return, a fade): the other sectors match the
    corrected pose closely, the silent one costs every candidate the same and does not inflate the fit RMS.  The drone
    is 0.25 m inside the side bar's inner edge, the estimate 0.55 m off towards the axis."""
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    R = _R()
    l_true = -(GATE.inner_width / 2.0 - 0.25)
    p_true = _world(-1.25, l_true)
    for mute in ("DOWN", "UP"):
        aid = StructureAiding(clf, DEFAULT)
        p = _world(-1.25, l_true + 0.55)
        for _ in range(12):
            profs = _profiles(clf, p_true, R)
            profs[mute] = np.zeros_like(RANGES)
            corr = aid.update(p, R, profs, {s: 0.0 for s in sg.SECTORS}, 0.25)
            if corr is not None:
                p = p + corr
        assert np.linalg.norm(p[:2] - p_true[:2]) < 0.15, (mute, p - p_true, aid.last)


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


GATES = list(SC.plans[0].structures)


def test_a_fit_that_puts_the_hull_outside_the_opening_is_rejected():
    """Robust clearance: the drone's hull is really 0.1 m beyond the free opening.  Whatever the fit says, no
    correction is applied and the gate localization is never valid: the drone must not enter."""
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    aid = StructureAiding(clf, DEFAULT, gates=GATES)
    R = _R()
    l_true = -(GATE.inner_width / 2.0 - HULL_LAT_M + 0.1)
    p_true, p_est = _world(-1.1, l_true), _world(-1.1, 0.0)
    for k in range(12):
        corr = aid.update(p_est, R, _profiles(clf, p_true, R), {s: 0.0 for s in sg.SECTORS}, 0.25, t=0.1 * k)
        assert corr is None
        assert not aid.valid(0.1 * k)


def test_a_centred_drone_has_a_valid_gate_localization():
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    aid = StructureAiding(clf, DEFAULT, gates=GATES)
    R = _R()
    p = _world(-1.18, 0.05)
    for k in range(6):
        aid.update(p, R, _profiles(clf, p, R), {s: 0.0 for s in sg.SECTORS}, 0.25, t=0.1 * k)
    assert aid.valid(0.5), aid.last
    assert not aid.valid(0.5 + 1.2)                    # a confirmation expires


def test_an_equally_good_far_position_makes_the_fit_ambiguous():
    """Logged echoes (MEDIUM-ALL integrated_short seed 2, drone 1, t = 85.9 s): the estimate is on the axis, the drone
    really 0.56 m to the left.  The estimate explains three sectors and the true pose explains them as well (total cost
    0.818 vs 0.820): the echoes cannot separate them, so the localization must not be confirmed.  The echoes were
    recorded at the track's 1.5 m wide G06 (0.18 x 0.22 m bars), so that gate is the map here."""
    gates = load_arena_gates(HORSESHOE_TRACK, ["G06"], inner_size={}, bar_section={})
    g = gates[0]
    clf = EchoClassifier(gate_bars=[b for gg in gates for b in gg.bars], cfg=DEFAULT)
    aid = StructureAiding(clf, DEFAULT, gates=gates)
    p_est = g.from_gate_frame([-0.44, 0.0, 0.04])
    yaw = math.radians(5.6)
    R = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    edges = {"FRONT": [0.42, 6.57, 9.18], "RIGHT": [0.52, 1.17, 2.12], "UP": [0.72], "DOWN": [0.77]}
    profs = {}
    for s in sg.SECTORS:
        prof = np.zeros_like(RANGES)
        for r in edges.get(s, []):
            prof[(RANGES >= r) & (RANGES <= r + 0.15)] = 0.9
        profs[s] = prof
    for k in range(6):
        assert aid.update(p_est, R, profs, {s: 0.0 for s in sg.SECTORS}, 0.25, t=0.1 * k) is None
        assert not aid.valid(0.1 * k), aid.last


# ---------------------------------------------------------------------------------------------- acquisition region (C5)
def test_the_acquisition_region_is_derived_and_covers_the_warning_band_of_the_bars():
    """The fit can be accepted from tan(half angle) * (outer half-extent + MAX_CORR_M) before the gate plane (module
    docstring), which must include the band in which an unexplained bar echo holds the drone (d_warning_exit from the
    bar, plus the hull)."""
    from holo_fleet.perception.structure_aiding import acquisition_distance

    d = acquisition_distance(DEFAULT, GATE)
    outer = GATE.inner_width / 2.0 + 0.25
    assert d == pytest.approx(math.tan(math.radians(DEFAULT.perc.sonar.half_angle_deg)) * (outer + MAX_CORR_M), abs=1e-6)
    assert d >= DEFAULT.sep.d_warning_exit + float(sg.HULL_HALF[0]) + 0.5


def test_a_longitudinal_error_is_corrected():
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    R = _R()
    p_true, p_est = _world(-2.0, -0.5), _world(-2.4, -0.5)        # 0.4 m along the axis, plus a lateral offset
    aid = StructureAiding(clf, DEFAULT, gates=GATES)
    p = _run(aid, clf, p_true, p_est, R, n=12)
    assert np.linalg.norm(p[:2] - p_true[:2]) < 0.15, (p - p_true, aid.last)


def test_another_drone_ahead_does_not_move_a_correct_estimate():
    """The previous drone in the opening or just beyond it, seen in FRONT at the ranges of the bars or in between:
    the estimate (= truth) is never moved by more than the consistency band."""
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    R = _R()
    for s_gate in (-3.6, -2.6, -1.6):
        p = _world(s_gate, 0.2)
        for r_other in np.arange(abs(s_gate) - 0.6, abs(s_gate) + 1.2, 0.15):
            aid = StructureAiding(clf, DEFAULT, gates=GATES)
            q = _run(aid, clf, p, p.copy(), R, n=8, extra={"FRONT": float(r_other)})
            assert np.linalg.norm(q[:2] - p[:2]) <= 0.15, (s_gate, r_other, q - p, aid.last)


def _axial_run(aid, clf, p_true, p_est, R, others, n=20):
    p = p_est.copy()
    for _ in range(n):
        corr = aid.update(p, R, _profiles(clf, p_true, R), {s: 0.0 for s in sg.SECTORS}, 0.25, others=others)
        if corr is not None:
            p = p + corr
    return p


def _axial_setup():
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    return clf, StructureAiding(clf, DEFAULT, gates=[GATE], acq_min=6.0)


def test_with_every_other_drone_excluded_a_single_front_echo_corrects_the_along_axis_error_only():
    clf, aid = _axial_setup()
    p_true, p_est = _world(-2.7, 0.0), _world(-3.3, 0.0)
    p = _axial_run(aid, clf, p_true, p_est, _R(), others=[])
    assert abs(float(GATE.to_gate_frame(p)[0]) - (-2.7)) <= 0.15
    assert abs(float(GATE.to_gate_frame(p)[1])) <= 1e-6


def test_without_identity_a_single_front_echo_never_moves_the_estimate():
    clf, aid = _axial_setup()
    p_est = _world(-3.3, 0.0)
    p = _axial_run(aid, clf, _world(-2.7, 0.0), p_est, _R(), others=None)
    assert np.allclose(p, p_est)


def test_an_echo_another_drone_could_have_made_never_moves_the_estimate():
    clf, aid = _axial_setup()
    p_est = _world(-3.3, 0.0)
    near_bar = np.array([_world(-0.2, 0.3)])                      # a drone located in the sonar's cone at the echo range
    p = _axial_run(aid, clf, _world(-2.7, 0.0), p_est, _R(), others=[(near_bar, 0.5)])
    assert np.allclose(p, p_est)
