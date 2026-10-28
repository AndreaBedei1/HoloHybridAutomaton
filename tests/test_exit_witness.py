"""Positive exit evidence of the CR occupancy latch (perception/exit_witness.py)."""

import math

import numpy as np
import pytest

from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates
from holo_fleet.config import DEFAULT
from holo_fleet.mission import MissionPlan, Path, Slot, abreast_slots, queue_line
from holo_fleet.perception.exit_witness import ExitWitness, speed_bound
from holo_fleet.perception.gate_perception import GatePerception
from holo_fleet.perception.sonar_geometry import AXES, HULL_HALF, MOUNTS, NEAR_FIELD_WIDEN_M, SECTORS
from holo_fleet.perception.targets import Target

G = DEFAULT.gate
ENV = DEFAULT.env
G06 = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
YAW = math.atan2(G06.axis[1], G06.axis[0])
QS = queue_line(abreast_slots(3), G)


def tg(pattern, r, cls="DYNAMIC", ranges=None):
    return Target(pattern=frozenset(pattern), r_min=r, d_lower=r, age=0.0, closing_rate=0.0, cls=cls,
                  ranges=ranges or {})


def perception(lateral=0.0):
    plan = MissionPlan(drone_id="d", slot_index=0, slots=[Slot(0, 0, 0)], path=Path(np.array([[-20.0, 0], [20.0, 0]]), 0.0),
                       gates=[G06], structures=[G06], launch_position=G06.from_gate_frame([QS, lateral, 0.0]),
                       launch_yaw_deg=math.degrees(YAW), static_rank=0, s_end=40.0, queue_lateral=lateral, queue_s=QS)
    return GatePerception(plan, DEFAULT)


def run(gp, seq, lateral=0.0, t0=0.0, yaw=YAW):
    p, o, t = G06.from_gate_frame([QS, lateral, 0.0]), None, t0
    for targets in seq:
        o = gp.update(t, p, yaw, targets, {})
        t += 0.1
    return o, t


def _rot(yaw):
    return np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]])


def _hull_surface(c, yaw, k=5):
    """Surface samples of a BlueROV2 hull box centred at c (world), heading yaw."""
    h, R = HULL_HALF, _rot(yaw)
    pts = []
    for ax in range(3):
        u, v = [i for i in range(3) if i != ax]
        for a in np.linspace(-h[u], h[u], k):
            for b in np.linspace(-h[v], h[v], k):
                for sg in (-1, 1):
                    q = np.zeros(3)
                    q[u], q[v], q[ax] = a, b, sg * h[ax]
                    pts.append(c + R @ q)
    return np.array(pts)


def _near_range(son, axis, pts):
    d = pts - son
    r = np.linalg.norm(d, axis=1)
    ang = np.arccos(np.clip((d @ axis) / r, -1.0, 1.0))
    inside = ang <= np.radians(DEFAULT.perc.sonar.half_angle_deg) + np.arctan2(NEAR_FIELD_WIDEN_M, r)
    return float(r[inside].min()) if inside.any() else None


def test_the_front_bound_is_the_v2_corridor_bound():
    w, gp = ExitWitness(), perception()
    for n in (2, 3, 4, 5, 6):
        for lat in (s.lateral for s in abreast_slots(n)):
            p = G06.from_gate_frame([queue_line(abreast_slots(n), G), lat, 0.0])
            assert abs(w.r_far(G06, p, YAW, "FRONT") - gp.cr_ranges(G06, p, YAW)[1]) < 1e-9


@pytest.mark.parametrize("n", [2, 3, 6])
def test_no_hull_inside_the_cr_passes_w1(n):
    """Brute force: every hull centred in the CR (grid incl. its faces, three headings), seen by any sector of a
    queued drone (headings within heading_tol), gives an echo (worst case: eps_range_far beyond its near surface)
    that W1 rejects."""
    w = ExitWitness()
    q_s = queue_line(abreast_slots(n), G)
    half = np.array([G.cr_half_len, G.cr_half_width, G.cr_half_height])
    grid = [np.array([a, b, c]) * half for a in np.linspace(-1, 1, 5) for b in np.linspace(-1, 1, 5) for c in np.linspace(-1, 1, 3)]
    for lat in (s.lateral for s in abreast_slots(n)):
        for dyaw in (-G.heading_tol_deg, 0.0, G.heading_tol_deg):
            yaw = YAW + math.radians(dyaw)
            p = G06.from_gate_frame([q_s, lat, 0.0])
            R = _rot(yaw)
            for sec in SECTORS:
                son, axis = p + R @ MOUNTS[sec], R @ AXES[sec]
                bound = w.r_far(G06, p, yaw, sec) + ENV.eps_range_far
                for cg in grid:
                    for hy in (YAW - 0.5, YAW, YAW + 0.5):
                        r = _near_range(son, axis, _hull_surface(G06.from_gate_frame(cg), hy, k=4))
                        if r is not None:
                            assert r + ENV.eps_range_far <= bound + 1e-9, (lat, dyaw, sec, cg, r, bound)


def test_absence_of_echoes_never_releases_the_latch():
    gp = perception()
    _, t = run(gp, [[tg({"FRONT"}, 4.6)]] * 3)
    o, t = run(gp, [[]] * int((G.t_occ_max - 0.5) / 0.1), t0=t)
    assert o.occ_state == "BUSY" and o.occ_timeouts == 0
    o, t = run(gp, [[]] * 10, t0=t)
    assert o.occ_state == "FREE" and o.occ_timeouts == 1                     # only the conservative fallback


def test_an_echo_that_a_hull_in_the_cr_can_produce_is_no_witness():
    gp = perception()
    r_far = gp.cr_ranges(G06, G06.from_gate_frame([QS, 0.0, 0.0]), YAW)[1]
    _, t = run(gp, [[tg({"FRONT"}, 4.6)]] * 3)
    o, t = run(gp, [[tg({"FRONT"}, r_far + ENV.eps_range_far - 0.01)]] * 60, t0=t)
    assert o.occ_state == "BUSY"


def test_witness_needs_the_corridor_empty():
    gp = perception()
    _, t = run(gp, [[tg({"FRONT"}, 4.6)]] * 3)
    o, t = run(gp, [[tg({"FRONT"}, 4.8), tg({"LEFT"}, 9.0)]] * 30, t0=t)   # occupant still in the corridor
    assert o.occ_state == "BUSY"


def test_exit_witness_then_free_after_t_clear_and_the_sector_is_reported():
    gp = perception()
    _, t = run(gp, [[tg({"FRONT"}, 4.6)]] * 3)
    _, t = run(gp, [[]] * 20, t0=t)
    o, t = run(gp, [[tg({"FRONT"}, 7.5)]] * 2, t0=t)
    assert o.occ_state == "EXITING" and o.exit_witness["sector"] == "FRONT" and gp.exit_witness["range"] == 7.5
    o, t = run(gp, [[tg({"FRONT"}, 8.0)]] * int(G.t_clear / 0.1 + 2), t0=t)
    assert o.occ_state == "FREE"


def test_a_side_sector_facing_the_exit_side_can_witness_the_exit():
    """Any sector counts: a queued drone turned to the right (e.g. after an avoidance manoeuvre) has its LEFT sonar
    looking downstream; an echo there beyond any CR range of that cone, ahead of the drone, is an exit witness."""
    yaw = YAW - math.pi / 2.0
    gp, w = perception(), ExitWitness()
    p = G06.from_gate_frame([QS, 0.0, 0.0])
    r = max(w.r_far(G06, p, yaw, "LEFT"), 6.0) + ENV.eps_range_far + 0.5
    _, t = run(gp, [[tg({"FRONT"}, 4.6)]] * 3)
    _, t = run(gp, [[]] * 60, t0=t, yaw=yaw)
    o, t = run(gp, [[tg({"LEFT"}, r)]] * 2, t0=t, yaw=yaw)
    assert o.occ_state == "EXITING" and gp.exit_witness["sector"] == "LEFT"


def test_a_drone_behind_the_queue_is_never_a_witness():
    """Regression (raw_gate_bars lost_drone_mutex s0): the lost drone, 9 m behind and 7 m beside the right-hand
    queue point, seen by its LEFT sonar at 10.7 m while the CR is latched, does not release it."""
    lat = -3.5
    gp = perception(lateral=lat)
    _, t = run(gp, [[tg({"FRONT"}, 5.5)]] * 3, lateral=lat)
    _, t = run(gp, [[]] * 60, lateral=lat, t0=t)
    o, t = run(gp, [[tg({"LEFT"}, 10.72)]] * 30, lateral=lat, t0=t)
    assert o.occ_state == "BUSY"
    assert not ExitWitness().ahead(G06, G06.from_gate_frame([QS, lat, 0.0]), YAW, "LEFT", 10.72)


def test_a_contact_outside_the_cr_seen_with_the_occupant_is_background_whatever_its_range():
    """W3 is recorded for every echo outside the CR while the occupant is in the corridor, before any reach test."""
    gp = perception()
    _, t = run(gp, [[tg({"FRONT"}, 4.6), tg({"FRONT"}, 7.4, ranges={"FRONT": 7.4})]] * 3)
    o, t = run(gp, [[tg({"FRONT"}, 7.4, ranges={"FRONT": 7.4})]] * 40, t0=t)   # still there, occupant masked
    assert o.occ_state == "BUSY"


def test_a_drone_already_beyond_is_not_taken_for_the_occupant():
    gp = perception()
    _, t = run(gp, [[tg({"FRONT"}, 4.6), tg({"FRONT"}, 9.5, ranges={"FRONT": 9.5})]] * 5)   # A beyond, O in
    seq = [[tg({"FRONT"}, 9.5 + 0.03 * k, ranges={"FRONT": 9.5 + 0.03 * k})] for k in range(40)]
    o, t = run(gp, seq, t0=t)                                                # O masked, A followed
    assert o.occ_state == "BUSY"
    o, t = run(gp, [[tg({"FRONT"}, 7.0, ranges={"FRONT": 7.0})]] * 2, t0=t)   # a new echo: O
    assert o.occ_state == "EXITING"


def test_a_far_echo_right_after_the_corridor_is_not_reachable():
    gp = perception()
    _, t = run(gp, [[tg({"FRONT"}, 4.6)]] * 3)
    o, t = run(gp, [[tg({"FRONT"}, 11.5, ranges={"FRONT": 11.5})]] * 2, t0=t)
    assert o.occ_state == "BUSY"                                             # 11.5 m: > 3 m beyond the CR in 0.2 s


def test_speed_bound():
    assert speed_bound() == max(ENV.v_escape, ENV.v_max_nominal, G.v_pass) + ENV.w_drift_max
