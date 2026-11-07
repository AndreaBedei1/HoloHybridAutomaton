"""The onboard conservative distance never exceeds the true distance of the nearest hull (lemma S0, quick version),
for single and merged multi-hull scenes, and it is not needlessly pessimistic in the formation geometries."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.perception import distance_bound as db
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.perception.sonar_processing import DYNAMIC, Echo, SectorReading
from holo_fleet.perception.targets import SectorTracker, build_targets

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "formal"))
import check_separation  # noqa: E402

EST = check_separation.deployed_bound(DEFAULT)
EPS, VC = DEFAULT.env.eps_range_far, DEFAULT.perc.v_close_staleness


@pytest.mark.parametrize("kind,n_hulls,dist,errors,boundary", [
    ("single", (1, 1), (1.0, 8.0), "random", "random"),
    ("corner", (1, 2), (0.7, 3.0), "far", "narrow"),
    ("edge", (2, 3), (1.0, 4.0), "far", "narrow"),
    ("square", (2, 3), (2.5, 4.5), "extreme", "random"),
])
def test_bound_is_sound_on_single_and_merged_hull_scenes(kind, n_hulls, dist, errors, boundary):
    S = check_separation.bound_scenes(DEFAULT, 400, EST, seed=7, kind=kind, n_hulls=n_hulls, dist=dist, errors=errors,
                                      boundary=boundary)
    assert (S[:, 0] - S[:, 1]).min() >= 0.0


def test_uncertified_bound_is_caught():
    S = check_separation.bound_scenes(DEFAULT, 1500, check_separation.v1_bound, seed=9, kind="corner", dist=(1.0, 3.0))
    assert (S[:, 0] - S[:, 1]).min() < 0.0


def _bound(ranges, age=0.0):
    return db.nearest_distance_lower(ranges, {s: age for s in ranges}, EPS, VC)


def test_pure_lateral_neighbour_gets_the_tight_bound():
    # a neighbour 3.5 m abeam: echo ~2.9 m in LEFT only -> centre certified inside the cone
    assert _bound({"LEFT": 2.9}) > DEFAULT.sep.d_warning_exit


def test_square_formation_corner_stays_outside_the_warning_band():
    # left and rear neighbours of a square drone at 3.5 m: echoes 2.92 m (LEFT) and 3.07 m (REAR) merged into one
    # LEFT+REAR target; with a 10 Hz reading the bound must clear the warning exit (the nominal formation is calm)
    assert _bound({"LEFT": 2.92, "REAR": 3.07}, age=0.1) > DEFAULT.sep.d_warning_exit
    assert _bound({"FRONT": 2.92, "LEFT": 3.07}, age=0.1) > DEFAULT.sep.d_warning_exit


def test_near_field_falls_back_to_the_attitude_free_bound():
    ranges = {"LEFT": 0.9, "REAR": 1.0}
    expect = max(db._bound_in_cone(r, s, EPS, DEFAULT.perc.sonar) for s, r in ranges.items())
    assert db.touch_set_bound(frozenset(ranges), ranges, EPS) == pytest.approx(expect)
    assert _bound(ranges) < sg.RHO_MIN


def test_opposite_cones_are_never_one_object():
    b = db.touch_set_bounds({"LEFT": 3.0, "RIGHT": 3.0}, {"LEFT": 0.0, "RIGHT": 0.0}, EPS, VC)
    assert frozenset({"LEFT", "RIGHT"}) not in b and set(b) == {frozenset({"LEFT"}), frozenset({"RIGHT"})}


def test_lattice_covering_radius_is_below_the_declared_one():
    from scipy.spatial import cKDTree

    tree = cKDTree(sg.fibonacci_sphere(db.N_DIRS))
    fine = sg.fibonacci_sphere(2_000_000)
    d, _ = tree.query(fine)
    assert float((2.0 * np.arcsin(d / 2.0)).max()) + 0.0018 <= db.COVER_RAD   # + the fine lattice's covering radius


def test_inscribed_cylinders_lie_inside_the_hull_for_any_heading_and_the_declared_tilt():
    rng = np.random.default_rng(3)
    pts = np.array([[rc * math.cos(t), rc * math.sin(t), z] for rc, hc in db.HULL_CYLINDERS
                    for z in np.linspace(-hc, hc, 5) for t in np.linspace(0.0, 2.0 * math.pi, 37)])
    for _ in range(500):
        R = check_separation._level_rot(rng, db.MAX_REL_TILT_DEG)
        local = pts @ R                                # points expressed in the hull frame
        assert np.all(np.abs(local) <= sg.HULL_HALF + 1e-9)


def _readings(echoes, age=0.1, unhealthy=()):
    out = {}
    for s in sg.SECTORS:
        rd = SectorReading(s, age=age, healthy=s not in unhealthy)
        rd.echoes = [Echo(s, r, r + 0.3, 0.9, cls=DYNAMIC) for sec, r in echoes if sec == s]
        out[s] = rd
    return out


def test_two_merged_neighbours_form_one_target_with_the_scene_bound():
    rd = _readings([("LEFT", 2.92), ("REAR", 3.07)])
    tg = build_targets(0.0, rd, SectorTracker().update(0.0, rd), DEFAULT)
    assert len(tg) == 1 and tg[0].pattern == frozenset({"LEFT", "REAR"})
    assert tg[0].d_lower == pytest.approx(_bound({"LEFT": 2.92, "REAR": 3.07}, age=0.1))


def test_an_unhealthy_sector_falls_back_to_the_per_sector_bound():
    rd = _readings([("LEFT", 2.92), ("REAR", 3.07)], unhealthy=("UP",))
    tg = build_targets(0.0, rd, SectorTracker().update(0.0, rd), DEFAULT)
    expect = db.conservative_bound({"LEFT": 2.92, "REAR": 3.07}, {"LEFT": 0.1, "REAR": 0.1}, EPS, VC)
    assert tg[0].d_lower == pytest.approx(expect) and expect < _bound({"LEFT": 2.92, "REAR": 3.07}, age=0.1)


def test_staleness_lowers_the_guard_distance():
    rf, rs = _readings([("FRONT", 3.0)], age=0.0), _readings([("FRONT", 3.0)], age=0.2)
    fresh = build_targets(0.0, rf, SectorTracker().update(0.0, rf), DEFAULT)[0].d_lower
    stale = build_targets(0.0, rs, SectorTracker().update(0.0, rs), DEFAULT)[0].d_lower
    assert stale == pytest.approx(fresh - VC * 0.2)
