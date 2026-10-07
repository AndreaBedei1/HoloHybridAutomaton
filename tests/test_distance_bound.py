"""The onboard conservative distance never exceeds the true centre distance (lemma S0, quick version)."""

import sys
from pathlib import Path

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.perception.sonar_processing import DYNAMIC, Echo, SectorReading
from holo_fleet.perception.targets import SectorTracker, build_targets

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "formal"))
import check_separation  # noqa: E402


def test_bound_is_sound_on_random_hull_poses():
    S = check_separation.bound_samples(DEFAULT, 1500, lambda r: sg.target_distance_lower(r), seed=7)
    assert (S[:, 0] - S[:, 1]).min() >= 0.0


def test_bound_is_sound_near_the_cube_corners():
    S = check_separation.bound_samples(DEFAULT, 1500, lambda r: sg.target_distance_lower(r), seed=8, corner=True)
    assert (S[:, 0] - S[:, 1]).min() >= 0.0


def test_uncertified_bound_is_caught():
    S = check_separation.bound_samples(DEFAULT, 1500, check_separation.v1_bound, seed=9, corner=True)
    assert (S[:, 0] - S[:, 1]).min() < 0.0


def test_pure_lateral_neighbour_gets_the_tight_bound():
    # a neighbour 3.5 m abeam: echo ~2.9 m in LEFT only -> centre certified inside the cone
    assert sg.target_distance_lower({"LEFT": 2.9}) > DEFAULT.sep.d_warning_exit


def test_staleness_lowers_the_guard_distance():
    def reading(age):
        rd = SectorReading("FRONT", age=age, healthy=True)
        rd.echoes = [Echo("FRONT", 3.0, 3.3, 0.9, cls=DYNAMIC)]
        return {s: (rd if s == "FRONT" else SectorReading(s, age=age, healthy=True)) for s in sg.SECTORS}

    fresh = build_targets(0.0, reading(0.0), SectorTracker().update(0.0, reading(0.0)), DEFAULT)[0].d_lower
    stale = build_targets(0.0, reading(0.2), SectorTracker().update(0.0, reading(0.2)), DEFAULT)[0].d_lower
    assert stale == fresh - DEFAULT.perc.v_close_staleness * 0.2 or abs(stale - (fresh - DEFAULT.perc.v_close_staleness * 0.2)) < 1e-9
