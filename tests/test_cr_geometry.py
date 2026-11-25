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


def test_physical_clearance_accounts_for_bars_and_the_hull():
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    s_bars = [float(g.to_gate_frame(b.center + b.axes @ (np.array([x, y, z]) * b.half))[0])
              for b in g.bars for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    assert max(s_bars) + R_OUT <= G.passage_clear_s
    assert G.passage_clear_s >= float(GATE_BAR_SECTION_M["G06"][1]) / 2 + R_OUT


def test_critical_region_ends_at_the_gate_plane():
    assert (G.cr_s_min, G.cr_s_max) == (-0.8, 0.0)
    assert G.cr_half_len == pytest.approx(0.4)
    assert G.cr_center_s == pytest.approx(-0.4)


def test_the_box_helpers_describe_one_box():
    lo, hi, c, h = G.cr_lo(), G.cr_hi(), G.cr_center(), G.cr_half()
    assert np.allclose(lo, [G.cr_s_min, -G.cr_half_width, -G.cr_half_height])
    assert np.allclose(hi, [G.cr_s_max, G.cr_half_width, G.cr_half_height])
    assert np.allclose(c, (lo + hi) / 2.0) and np.allclose(h, (hi - lo) / 2.0)


def test_cr_excludes_the_physical_post_plane_passage():
    for s in np.linspace(-0.8, 0.0, 17):
        assert G.in_cr(s, 0.0, 0.0)
    assert not G.in_cr(-0.81, 0.0, 0.0) and not G.in_cr(0.01, 0.0, 0.0)
    assert inside_keepout(0.54, 0.0, 0.0, G)


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


def test_perception_covers_physical_passage_and_control_uses_cr():
    """Admission sensing covers the passage while the P2 control filter protects the CR."""
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    w = ExitWitness(DEFAULT)
    ae = AcousticExit(DEFAULT, AcousticTracker(DEFAULT), [1])
    for o in (w, ae):
        assert np.allclose(o.lo, G.cr_lo()) and np.allclose(o.hi, G.passage_hi())
    assert w.cr_pts[:, 0].min() == pytest.approx(G.cr_s_min) and w.cr_pts[:, 0].max() == pytest.approx(G.passage_clear_s)
    lo, hi = obstruction_box(g)
    assert np.all(lo <= G.cr_lo() - R_OUT + 1e-9) and np.all(hi >= G.passage_hi() + R_OUT - 1e-9)
    gc = gs.constraint_for(DEFAULT, g, g.from_gate_frame(G.cr_center()), np.zeros(3), authorized=False)
    assert np.allclose(gc.cr_lo, G.cr_lo()) and np.allclose(gc.cr_hi, G.cr_hi())
    for q in _samples(600, seed=5):
        if _box(q):
            assert inside_keepout(q[0], q[1], q[2], G)
            assert gs.constraint_for(DEFAULT, g, g.from_gate_frame(q), np.zeros(3), False).margins_world(np.zeros(3))[0] < 0.0


def test_the_hold_follows_the_entry_plane():
    assert gs.hold_s(DEFAULT) == pytest.approx(G.cr_s_min - (G.nav_err_m + gs.hold_reach(gs.P2Bounds.from_cfg(DEFAULT))
                                                             + gs.SLACK_M))
    assert G.stage2_hold_s == pytest.approx(-2.3025, abs=1e-6)
    assert G.merge_s == pytest.approx(G.cr_s_min - G.merge_before_cr)
