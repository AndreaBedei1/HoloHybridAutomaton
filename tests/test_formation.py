"""Generic formation templates and onboard formation perception (one-to-one neighbour association)."""

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.formations import TEMPLATES
from holo_fleet.mission import FormationClock, MissionPlan, Path
from holo_fleet.perception.formation_perception import check_formation, compatible
from holo_fleet.perception.targets import Target

I3 = np.eye(3)


@pytest.mark.parametrize("name,n", [("triangle", 3), ("square", 4), ("line6", 6)])
def test_templates(name, n):
    tm = TEMPLATES[name]
    assert tm.n == n
    assert tm.min_spacing() >= DEFAULT.sep.d_warning_exit + 0.8          # neighbours never in the warning band
    lat = np.array([s.lateral for s in tm.slots])
    assert tm.coverage_gap(lat) == 0.0                                  # nominal swaths are contiguous


def _plan(name, k):
    tm = TEMPLATES[name]
    return MissionPlan(drone_id=f"drone_{k}", slot_index=k, slots=list(tm.slots), path=Path(np.array([[0, 0], [50.0, 0]]), -5),
                       gates=[], structures=[], launch_position=np.zeros(3), launch_yaw_deg=0.0, static_rank=k, s_end=40)


def _tg(pattern, r):
    return Target(pattern=frozenset(pattern), r_min=r, d_lower=r, age=0.0, closing_rate=0.0, cls="DYNAMIC",
                  ranges={s: r for s in pattern})


def test_square_neighbours_seen_and_residuals_near_zero():
    plan = _plan("square", 0)                     # front-left: right neighbour 3.5 m, rear neighbour 3.5 m
    healthy = {s: True for s in ("FRONT", "REAR", "LEFT", "RIGHT", "UP", "DOWN")}
    targets = [_tg({"RIGHT"}, 3.5 - 0.24 - 0.30), _tg({"REAR"}, 3.5 - 0.24 - 0.24), _tg({"REAR", "RIGHT"}, 4.95 - 0.5)]
    o = check_formation(plan, DEFAULT, np.zeros(3), I3, I3, targets, healthy)
    assert o.neighbors_ok and all(c.seen for c in o.checks)
    assert max(abs(c.residual) for c in o.checks) < 0.3


def test_one_target_is_never_two_neighbours():
    plan = _plan("triangle", 2)                   # the leading centre drone: neighbours REAR+LEFT and REAR+RIGHT
    healthy = {s: True for s in ("FRONT", "REAR", "LEFT", "RIGHT", "UP", "DOWN")}
    o = check_formation(plan, DEFAULT, np.zeros(3), I3, I3, [_tg({"LEFT", "REAR"}, 4.1)], healthy)
    assert sum(c.seen for c in o.checks) == 1 and not o.neighbors_ok


def test_pattern_on_the_other_side_is_not_compatible():
    assert not compatible({"LEFT", "REAR"}, np.array([-0.54, -0.84, 0.0]))
    assert compatible({"REAR", "RIGHT"}, np.array([-0.54, -0.84, 0.0]))


def test_formation_clock_jump_and_hold():
    c = FormationClock(5.0, 0.3, 0.0, holds=((40.0, 20.0),), jumps=((10.0, 40.0),), s_end=60.0)
    assert c.s(0.0) == 5.0
    assert c.s(16.0) == pytest.approx(9.8)
    assert c.s(17.0) == 40.0 and c.s(30.0) == 40.0
    assert c.s(37.67) == pytest.approx(40.3, abs=0.01)
    assert c.s(1e4) == 60.0
