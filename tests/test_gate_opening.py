"""The widened opening of gate G06 (holo_fleet/arena_bridge.py, GATE_INNER_SIZE_M)."""

import numpy as np

from holo_fleet.arena_bridge import GATE_INNER_SIZE_M, HORSESHOE_TRACK, load_arena_gates, visual_gates
from holo_fleet.config import DEFAULT
from holo_fleet.perception.structure_aiding import HULL_LAT_M


def _side_bar_inner_edges(g):
    edges = []
    for b in g.bars:
        s, l, dz = g.to_gate_frame(b.center)
        if abs(l) > 0.1:                                         # the two vertical side bars
            edges.append(abs(l) - b.half[1])
    return edges


def test_the_map_has_the_widened_opening():
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    assert (g.inner_width, g.inner_height) == GATE_INNER_SIZE_M["G06"] == (2.0, 2.0)
    assert np.allclose(_side_bar_inner_edges(g), g.inner_width / 2.0, atol=1e-6)


def test_the_spawned_bars_match_the_map():
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    vg = visual_gates(HORSESHOE_TRACK, ["G06"])[0]
    spawned = sorted(np.round(np.asarray(b.dimensions_m, float) / 2.0, 6).tolist() for b in vg.bars)
    mapped = sorted(np.round(np.asarray(b.half, float), 6).tolist() for b in g.bars)
    assert spawned == mapped
    centres = sorted(np.round(np.asarray(b.position, float), 6).tolist() for b in vg.bars)
    assert centres == sorted(np.round(np.asarray(b.center, float), 6).tolist() for b in g.bars)


def test_the_track_opening_is_still_available():
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"], inner_size={})[0]
    assert (g.inner_width, g.inner_height) == (1.5, 1.5)


def test_the_opening_ends_at_the_critical_region_and_widens_the_hull_margin():
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    assert g.inner_width / 2.0 <= DEFAULT.gate.cr_half_width + 1e-9    # a drone in the opening is in the CR
    assert g.inner_height / 2.0 <= DEFAULT.gate.cr_half_height + 1e-9
    assert g.inner_width / 2.0 - HULL_LAT_M > 0.58                     # MEDIUM lateral DR error at the gate
    assert 1.5 / 2.0 - HULL_LAT_M < 0.45
