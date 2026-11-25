"""Geometry of the P2 critical region (CR): the approach immediately before the gate and the gate passage itself, not a
region after the gate (holo_fleet/config.py GateRule).  Every user of the CR (referee, perception, control, scope) must
agree with the referee's box."""

from types import SimpleNamespace as NS

import numpy as np
import pytest

from holo_fleet.arena_bridge import GATE_BAR_SECTION_M, HORSESHOE_TRACK, load_arena_gates
from holo_fleet.config import DEFAULT
from holo_fleet.control import gate_safety as gs
from holo_fleet.perception.acoustic import AcousticTracker
from holo_fleet.perception.acoustic_gate import AcousticExit
from holo_fleet.perception.exit_witness import ExitWitness
from holo_fleet.perception.passage import inside_keepout
from holo_fleet.perception.sonar_geometry import HULL_HALF, R_OUT
from holo_fleet.referee.referee import Referee
from holo_fleet.scope import obstruction_box

G = DEFAULT.gate
PREV_HALF_LEN = 0.8  # the former symmetric CR |s| <= 0.8


def test_the_exit_plane_covers_the_whole_gate_passage():
    """A hull touches the G06 frame (bars 0.30 m deep along the axis) only while its centre is within bar half depth +
    R_OUT of the gate plane, any heading: the CR must reach that far beyond the gate plane."""
    depth = float(GATE_BAR_SECTION_M["G06"][1])
    assert R_OUT == pytest.approx(float(np.linalg.norm(HULL_HALF)))
    assert G.cr_s_max >= depth / 2.0 + R_OUT
    assert G.cr_s_min <= -(depth / 2.0 + R_OUT)


def test_the_exit_plane_covers_the_bars_of_the_arena_gate():
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    s_bars = [float(g.to_gate_frame(b.center + b.axes @ (np.array([x, y, z]) * b.half))[0])
              for b in g.bars for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    assert max(abs(s) for s in s_bars) + R_OUT <= G.cr_s_max


def test_the_monitored_region_is_not_shrunk():
    assert G.cr_s_max - G.cr_s_min >= 2 * PREV_HALF_LEN - 1e-9
    assert G.cr_s_min <= -PREV_HALF_LEN and 0.0 <= G.cr_s_max
    assert G.cr_half_len == pytest.approx((G.cr_s_max - G.cr_s_min) / 2.0) == pytest.approx(PREV_HALF_LEN)
    assert G.cr_center_s == pytest.approx((G.cr_s_max + G.cr_s_min) / 2.0) == pytest.approx(-0.25)
    assert (G.cr_s_min, G.cr_s_max) == (-1.05, 0.55)


def test_the_box_helpers_describe_one_box():
    lo, hi, c, h = G.cr_lo(), G.cr_hi(), G.cr_center(), G.cr_half()
    assert np.allclose(lo, [G.cr_s_min, -G.cr_half_width, -G.cr_half_height])
    assert np.allclose(hi, [G.cr_s_max, G.cr_half_width, G.cr_half_height])
    assert np.allclose(c, (lo + hi) / 2.0) and np.allclose(h, (hi - lo) / 2.0)


def test_in_cr_is_the_box_and_drops_only_the_post_passage_part():
    for s in np.linspace(-PREV_HALF_LEN, 0.0, 17):
        assert G.in_cr(s, 0.0, 0.0)
    for s in np.linspace(-0.54, 0.54, 11):
        assert G.in_cr(s, 0.0, 0.0)
    assert G.in_cr(-1.0, 0.0, 0.0) and not G.in_cr(-1.06, 0.0, 0.0)
    assert not G.in_cr(0.56, 0.0, 0.0) and not G.in_cr(0.8, 0.0, 0.0)
    assert G.in_cr(0.6, 0.0, 0.0, margin=0.1) and not G.in_cr(0.0, G.cr_half_width + 0.01, 0.0)


def _samples(n=4000, seed=3):
    rng = np.random.default_rng(seed)
    q = rng.uniform([-3.0, -1.8, -1.8], [2.5, 1.8, 1.8], size=(n, 3))
    edges = np.array([[x, y, z] for x in (G.cr_s_min, G.cr_s_max) for y in (-G.cr_half_width, G.cr_half_width)
                      for z in (-G.cr_half_height, G.cr_half_height)])
    return np.concatenate([q, edges, edges * (1 + 1e-6), edges * (1 - 1e-6)])


def _box(q):
    lo, hi = G.cr_lo(), G.cr_hi()
    return bool(np.all(q >= lo) and np.all(q <= hi))


def test_the_referee_and_the_vectorised_predicate_agree_with_the_box():
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    ref = NS(cfg=DEFAULT)
    Q = _samples()
    vec = G.in_cr(Q[:, 0], Q[:, 1], Q[:, 2])
    for k, q in enumerate(Q):
        expect = _box(q)
        assert bool(vec[k]) == expect
        assert G.in_cr(*q) == expect
        p = g.from_gate_frame(q)
        assert Referee.cr_inside(ref, g, p) == _box(np.asarray(g.to_gate_frame(p), float))  # exact corners: round-off


def test_perception_and_control_use_the_same_box():
    """Exit witness, acoustic exit, keep-out box, P2 constraint and the scope's obstruction volume all hold the CR."""
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    w = ExitWitness(DEFAULT)
    ae = AcousticExit(DEFAULT, AcousticTracker(DEFAULT), [1])
    for o in (w, ae):
        assert np.allclose(o.lo, G.cr_lo()) and np.allclose(o.hi, G.cr_hi())
    assert w.cr_pts[:, 0].min() == pytest.approx(G.cr_s_min) and w.cr_pts[:, 0].max() == pytest.approx(G.cr_s_max)
    lo, hi = obstruction_box(g)
    assert np.all(lo <= G.cr_lo() - R_OUT + 1e-9) and np.all(hi >= G.cr_hi() + R_OUT - 1e-9)
    gc = gs.constraint_for(DEFAULT, g, g.from_gate_frame(G.cr_center()), np.zeros(3), authorized=False)
    assert np.allclose(gc.cr_lo, G.cr_lo()) and np.allclose(gc.cr_hi, G.cr_hi())
    for q in _samples(600, seed=5):
        if _box(q):
            assert inside_keepout(q[0], q[1], q[2], G)
            assert gs.constraint_for(DEFAULT, g, g.from_gate_frame(q), np.zeros(3), False).margins_world(np.zeros(3))[0] < 0.0


def test_the_hold_follows_the_entry_plane():
    assert gs.hold_s(DEFAULT) == pytest.approx(G.cr_s_min - (G.nav_err_m + gs.hold_reach(gs.P2Bounds.from_cfg(DEFAULT))
                                                             + gs.SLACK_M))
    assert G.stage2_hold_s == pytest.approx(-2.5525, abs=1e-6)
    assert G.merge_s == pytest.approx(G.cr_s_min - G.merge_before_cr)
