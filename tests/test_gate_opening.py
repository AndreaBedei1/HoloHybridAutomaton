"""The opening and the bar section of gate G06 (holo_fleet/arena_bridge.py, GATE_INNER_SIZE_M, GATE_BAR_SECTION_M)."""

import numpy as np
import pytest

from holo_fleet.arena_bridge import (GATE_BAR_SECTION_M, GATE_INNER_SIZE_M, HORSESHOE_TRACK, load_arena_gates,
                                     visual_gates)
from holo_fleet.config import DEFAULT
from holo_fleet.mission import gate_frame_points
from holo_fleet.perception.structure_aiding import HULL_LAT_M


def _bars(g):
    """(part, s, l, dz, half) of the four bars; half in the bar's local axes (depth, across, along)."""
    out = []
    for b in g.bars:
        s, l, dz = g.to_gate_frame(b.center)
        out.append(("side" if abs(l) > abs(dz) else "top_bottom", s, l, dz, np.asarray(b.half, float)))
    return out


def _opening(g):
    """Free opening (width, height) between the inner faces of the bars."""
    w = [2.0 * (abs(l) - h[1]) for part, _s, l, _dz, h in _bars(g) if part == "side"]
    h = [2.0 * (abs(dz) - hh[2]) for part, _s, _l, dz, hh in _bars(g) if part == "top_bottom"]
    return w, h


def test_the_map_has_the_widened_opening_and_four_bars():
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    assert (g.inner_width, g.inner_height) == GATE_INNER_SIZE_M["G06"] == (2.0, 2.0)
    assert len(g.bars) == 4                                     # no lead-in posts or other landmarks
    w, h = _opening(g)
    assert np.allclose(w, 2.0, atol=1e-9) and np.allclose(h, 2.0, atol=1e-9)


@pytest.mark.parametrize("section", [(0.18, 0.22), (0.30, 0.40), (0.50, 0.60)])
def test_thicker_bars_grow_outwards_and_keep_the_opening(section):
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"], bar_section={"G06": section})[0]
    t, depth = section
    w, h = _opening(g)
    assert np.allclose(w, 2.0, atol=1e-9) and np.allclose(h, 2.0, atol=1e-9)
    for part, s, _l, _dz, half in _bars(g):
        assert abs(s) < 1e-9 and abs(2.0 * half[0] - depth) < 1e-9
        assert abs(2.0 * (half[1] if part == "side" else half[2]) - t) < 1e-9


def test_the_bar_section_is_the_configured_one():
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    t, depth = GATE_BAR_SECTION_M["G06"]
    for part, _s, _l, _dz, half in _bars(g):
        assert abs(2.0 * half[0] - depth) < 1e-9
        assert abs(2.0 * (half[1] if part == "side" else half[2]) - t) < 1e-9


def test_the_spawned_bars_match_the_map():
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    vg = visual_gates(HORSESHOE_TRACK, ["G06"])[0]
    spawned = sorted(np.round(np.asarray(b.dimensions_m, float) / 2.0, 6).tolist() for b in vg.bars)
    mapped = sorted(np.round(np.asarray(b.half, float), 6).tolist() for b in g.bars)
    assert spawned == mapped
    centres = sorted(np.round(np.asarray(b.position, float), 6).tolist() for b in vg.bars)
    assert centres == sorted(np.round(np.asarray(b.center, float), 6).tolist() for b in g.bars)


def test_queue_and_formal_geometry_use_the_mapped_bars():
    """mission.gate_frame_points (queue masking, formal M4f and the corridor/mask views) is the map itself."""
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    P = gate_frame_points()
    t, depth = GATE_BAR_SECTION_M["G06"]
    assert np.allclose(P.max(axis=0), [depth / 2.0, 1.0 + t, 1.0 + t], atol=1e-9)
    assert np.allclose(P.min(axis=0), [-depth / 2.0, -1.0 - t, -1.0 - t], atol=1e-9)
    assert np.allclose(P, gate_frame_points(g))
    inside = (np.abs(P[:, 1]) < 1.0 - 1e-9) & (np.abs(P[:, 2]) < 1.0 - 1e-9)
    assert not inside.any()                                     # no bar surface inside the free opening


def test_the_track_geometry_is_still_available():
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"], inner_size={}, bar_section={})[0]
    assert (g.inner_width, g.inner_height) == (1.5, 1.5)
    assert len(g.bars) == 4
    w, h = _opening(g)
    assert np.allclose(w, 1.5, atol=1e-9) and np.allclose(h, 1.5, atol=1e-9)


def test_the_opening_ends_at_the_critical_region_and_widens_the_hull_margin():
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    assert g.inner_width / 2.0 <= DEFAULT.gate.cr_half_width + 1e-9    # a drone in the opening is in the CR
    assert g.inner_height / 2.0 <= DEFAULT.gate.cr_half_height + 1e-9
    assert g.inner_width / 2.0 - HULL_LAT_M > 0.58                     # MEDIUM lateral DR error at the gate
    assert 1.5 / 2.0 - HULL_LAT_M < 0.45
