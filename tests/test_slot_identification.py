"""Association is not identification (C3): regressions from the diagnosed false re-occupations of a lost drone's slot.

In the stopped B2 campaign a healthy neighbour, displaced to the range of the lost drone's vacant slot in a
compatible sector, re-occupied that slot (11 cases, B0 and B2): its own slot was unseen at the same steps, so the
echo could have been it.  The deployed association (formation_perception.check_formation) and lifecycle
(perception.track_missing) are exercised here on a square seen from drone 3: slot 0 diagonal (FRONT+LEFT, 4.95 m),
slot 1 FRONT and slot 2 LEFT (3.5 m)."""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.formations import TEMPLATES
from holo_fleet.mission import FormationClock, MissionPlan, Path
from holo_fleet.perception.formation_perception import HULL_MID, check_formation, estimate_centre_distance
from holo_fleet.perception.perception import Perception
from holo_fleet.perception.sonar_geometry import MOUNTS, SECTORS, far_field_pattern
from holo_fleet.perception.targets import Target

FR = DEFAULT.form
DT = 0.1
HEALTHY = {s: True for s in SECTORS}
AT = {0: (3.5, 3.5, 0.0), 1: (3.5, 0.0, 0.0), 2: (0.0, 3.5, 0.0)}
DISPLACED_0 = (0.8, 3.5, 0.0)


def _plan(k):
    return MissionPlan(drone_id=f"drone_{k}", slot_index=k, slots=list(TEMPLATES["square"].slots),
                       path=Path(np.array([[0.0, 0.0], [80.0, 0.0]]), -5.0), gates=[], structures=[],
                       launch_position=np.zeros(3), launch_yaw_deg=0.0, static_rank=k, s_end=70.0,
                       clock=FormationClock(0.0, 0.3, 0.0, s_end=70.0))


def _targets(rels):
    """One sonar target per drone at the given relative positions (body = map axes), in its far-field sectors."""
    out = []
    for v in rels:
        v = np.asarray(v, float)
        d = float(np.linalg.norm(v))
        u = v / d
        rng = {s: d - HULL_MID - float(MOUNTS[s] @ u) for s in far_field_pattern(u)}
        out.append(Target(pattern=frozenset(rng), r_min=min(rng.values()), d_lower=min(rng.values()), age=0.0,
                          closing_rate=0.0, cls="DYNAMIC", ranges=rng))
    return out


def _observe(p, t, rels, acoustic=None):
    return check_formation(p.plan, DEFAULT, np.zeros(3), np.eye(3), np.eye(3), _targets(rels), HEALTHY, t,
                           p.neighbour_seen, vacant=p.vacant, acoustic=acoustic,
                           missing={k for k, v in p.missing_state.items() if v})


def _run(p, t0, t1, rels, heard=()):
    """Drone 3 keeps its slot and sees drones at ``rels``; ``heard``: neighbours with a fresh accepted fix (B2)."""
    t, o = t0, None
    while t < t1 - 1e-9:
        o = _observe(p, t, rels)
        p.acoustic_fresh = list(heard)
        p.track_missing(t, DT, o, True)
        t = round(t + DT, 6)
    return o, t


def _check(o, slot):
    return next(c for c in o.checks if c.slot == slot)


def _lost_and_vacant():
    p = Perception(_plan(3), DEFAULT)
    _o, t = _run(p, 0.0, 3.0, [AT[0], AT[1], AT[2]])
    _o, t = _run(p, t, t + FR.t_rejoin + 3.0, [AT[0], AT[1]])
    assert 2 in p.vacant
    return p, t


def test_the_scene_is_the_diagnosed_one():
    o = _observe(Perception(_plan(3), DEFAULT), 0.0, [DISPLACED_0, AT[1]])
    assert _check(o, 2).seen and abs(_check(o, 2).residual) <= 0.5 * FR.range_gate_m and not _check(o, 0).seen


def test_a_displaced_neighbour_at_a_vacant_slot_does_not_reoccupy_it():
    p, t = _lost_and_vacant()
    o, t = _run(p, t, t + 10.0, [DISPLACED_0, AT[1]])
    assert _check(o, 2).seen and 2 in p.vacant
    assert not [e for e in p.events if e["type"] == "SLOT_REOCCUPIED"]
    o, t = _run(p, t, t + 3.0, [AT[0], AT[1], AT[2]])
    assert 2 not in p.vacant and p.events[-1]["type"] == "SLOT_REOCCUPIED"


def test_a_displaced_neighbour_does_not_reset_the_timer_of_a_lost_one():
    p = Perception(_plan(3), DEFAULT)
    _o, t = _run(p, 0.0, 3.0, [AT[0], AT[1], AT[2]])
    _o, t = _run(p, t, t + 5.0, [AT[0], AT[1]])
    tau = p.missing_t[2]
    _o, t = _run(p, t, t + 5.0, [DISPLACED_0, AT[1]])
    assert p.missing_state[2] and p.missing_t[2] > tau + 4.5


def test_an_echo_explains_a_present_neighbour_before_a_missing_one():
    p = Perception(_plan(0), DEFAULT)
    tg = _targets([(-4.12, -0.6, 0.0)])
    u2, u3 = np.array([-1.0, 0.0, 0.0]), np.array([-1.0, -1.0, 0.0]) / np.sqrt(2.0)
    r = tg[0].ranges["REAR"]
    assert abs(estimate_centre_distance(r, "REAR", u2) - 3.5) < abs(estimate_centre_distance(r, "REAR", u3) - 4.95)
    by_res = check_formation(p.plan, DEFAULT, np.zeros(3), np.eye(3), np.eye(3), tg, HEALTHY, 0.0, {})
    assert _check(by_res, 2).seen and not _check(by_res, 3).seen
    o = check_formation(p.plan, DEFAULT, np.zeros(3), np.eye(3), np.eye(3), _targets([(-4.12, -0.6, 0.0)]), HEALTHY,
                        0.0, {}, missing={2})
    assert _check(o, 3).seen and not _check(o, 2).seen


def test_b2_the_echo_of_a_heard_drone_is_not_associated_with_an_unheard_slot():
    p = Perception(_plan(3), DEFAULT)
    heard = {0: np.array(DISPLACED_0), 1: np.array(AT[1])}
    o = _observe(p, 0.0, [DISPLACED_0, AT[1]], acoustic=heard)
    assert not _check(o, 2).seen and not _check(o, 0).seen
    assert _check(_observe(p, 0.0, [DISPLACED_0, AT[1]]), 2).seen


def test_b2_a_heard_neighbour_with_a_missed_echo_is_accounted_for():
    p, t = _lost_and_vacant()
    _o, t = _run(p, t, t + 3.0, [AT[1], AT[2]])
    assert 2 in p.vacant
    p, t = _lost_and_vacant()
    _o, t = _run(p, t, t + 3.0, [AT[1], AT[2]], heard=(0, 1))
    assert 2 not in p.vacant and p.events[-1]["type"] == "SLOT_REOCCUPIED"
