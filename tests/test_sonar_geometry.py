"""Six identical wide-beam single-beam sonars: configuration, mounting, 3-D coverage, sector patterns."""

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.perception.sensor_suite import ONBOARD_NAMES, onboard_sensor_configs, sonar_sensor_configs


def test_six_identical_singlebeam_sonars_and_nothing_else():
    son = sonar_sensor_configs(DEFAULT)
    assert len(son) == 6
    assert {c["sensor_type"] for c in son} == {"SinglebeamSonar"}
    confs = [c["configuration"] for c in son]
    assert all(c == confs[0] for c in confs), "the six sonars must share one model"
    assert confs[0]["OpeningAngle"] == 120 and confs[0]["RangeMin"] == 0.3 and confs[0]["RangeMax"] == 12.0
    assert {c["Hz"] for c in son} == {10}
    types = {c["sensor_type"] for c in onboard_sensor_configs(DEFAULT)}
    assert not types & {"RangeFinderSensor", "ImagingSonar", "ProfilingSonar", "SidescanSonar"}
    # only position / orientation differ
    assert len({tuple(c["location"]) for c in son}) == 6 and len({tuple(c["rotation"]) for c in son}) == 6


def test_mounts_on_the_hull_faces_along_the_sector_axes():
    for s in sg.SECTORS:
        m = sg.MOUNTS[s]
        assert np.allclose(np.cross(m, sg.AXES[s]), 0.0) and float(m @ sg.AXES[s]) > 0
        k = int(np.argmax(np.abs(sg.AXES[s])))
        assert sg.HULL_HALF[k] < float(np.linalg.norm(m)) < sg.HULL_HALF[k] + 0.03


@pytest.mark.parametrize("rho", [1.0, 1.5, 2.4, 5.0])
def test_every_direction_is_covered_by_at_least_one_cone(rho):
    assert sg.coverage(rho, sg.R_IN) == pytest.approx(1.0)


def test_cube_diagonals_are_seen_by_three_sectors():
    for name, u in sg.diagonal_directions().items():
        assert len(sg.far_field_pattern(u)) == 3, name


def test_axes_are_seen_only_by_their_own_sector():
    for s in sg.SECTORS:
        assert sg.far_field_pattern(sg.AXES[s]) == frozenset({s})


def test_region_table_patterns():
    R = sg.regions()
    assert all(len(P) <= 3 for P in R)
    assert frozenset({"FRONT", "REAR"}) not in R          # opposite cones never see the same hull
    for P, dirs in R.items():
        assert np.allclose(np.linalg.norm(dirs, axis=1), 1.0)


def test_onboard_whitelist_has_no_privileged_sensor():
    for key in ("PoseSensor", "VelocitySensor", "CollisionSensor", "LocationSensor", "RotationSensor"):
        assert key not in ONBOARD_NAMES
