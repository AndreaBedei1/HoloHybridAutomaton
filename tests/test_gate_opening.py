"""The widened opening of gate G06 (holo_fleet/arena_bridge.py, GATE_INNER_SIZE_M)."""

import numpy as np

from holo_fleet.arena_bridge import GATE_INNER_SIZE_M, GATE_MARKERS_M, HORSESHOE_TRACK, load_arena_gates, visual_gates
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.perception.sonar_processing import EchoClassifier
from holo_fleet.config import DEFAULT
from holo_fleet.perception.structure_aiding import HULL_LAT_M


def _side_bar_inner_edges(g):
    edges = []
    for b in g.bars:
        s, l, dz = g.to_gate_frame(b.center)
        if abs(l) > 0.1 and abs(s) < 0.5:                        # the two vertical side bars (not the posts)
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
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"], inner_size={}, markers={})[0]
    assert (g.inner_width, g.inner_height) == (1.5, 1.5)
    assert len(g.bars) == 4


def test_the_opening_ends_at_the_critical_region_and_widens_the_hull_margin():
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    assert g.inner_width / 2.0 <= DEFAULT.gate.cr_half_width + 1e-9    # a drone in the opening is in the CR
    assert g.inner_height / 2.0 <= DEFAULT.gate.cr_half_height + 1e-9
    assert g.inner_width / 2.0 - HULL_LAT_M > 0.58                     # MEDIUM lateral DR error at the gate
    assert 1.5 / 2.0 - HULL_LAT_M < 0.45


def _posts(g):
    return [b for b in g.bars if abs(g.to_gate_frame(b.center)[0]) > 0.5]


def test_two_lead_in_posts_flank_the_hold_point_outside_the_critical_region():
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    posts = _posts(g)
    s_m, l_m = GATE_MARKERS_M["G06"]
    assert len(posts) == 2 and len(g.bars) == 6
    for b in posts:
        s, l, dz = g.to_gate_frame(b.center)
        assert abs(s - s_m) < 1e-6 and abs(abs(l) - l_m) < 1e-6 and abs(dz) < 1e-6
        assert abs(s) - b.half[0] > DEFAULT.gate.cr_half_len                   # outside the critical region
        assert abs(l) - b.half[1] - HULL_LAT_M > DEFAULT.env.struct_keepout    # a drone on the axis keeps clear
    s_hold = -(DEFAULT.gate.cr_half_len + float(sg.HULL_HALF[0]) + 0.15)
    assert abs(s_hold - s_m) < 0.1


def test_at_the_hold_point_the_side_sonars_see_the_posts_first():
    """LEFT/RIGHT, on the axis at the hold point: the nearest mapped structure in the nominal cone is the post."""
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    clf = EchoClassifier(gate_bars=list(g.bars), cfg=DEFAULT)
    yaw = np.arctan2(g.axis[1], g.axis[0])
    R = np.array([[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    p = g.from_gate_frame([GATE_MARKERS_M["G06"][0], 0.0, 0.0])
    for sec in ("LEFT", "RIGHT"):
        a, _ = clf._sector_frame(sec, R)
        m = p + R @ sg.MOUNTS[sec]
        d = np.asarray(clf.struct_pts) - m
        r = np.linalg.norm(d, axis=1)
        inside = np.degrees(np.arccos(np.clip((d @ a) / r, -1.0, 1.0))) <= clf.model.half_angle_deg
        k = np.argmin(np.where(inside, r, np.inf))
        assert abs(g.to_gate_frame(np.asarray(clf.struct_pts)[k])[0] - GATE_MARKERS_M["G06"][0]) < 0.2, sec
