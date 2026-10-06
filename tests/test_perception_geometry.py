"""Proximity-sonar geometry: synthetic returns from a box-shaped neighbour are localised correctly."""

import math

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.perception.proximity import ProximityProcessor
from holo_fleet.perception.sensor_suite import ring_beam_directions, ring_name

HALF = np.array([0.29, 0.23, 0.13])        # BlueROV2-like half extents


def ray_box(o, d, c, half):
    """Distance along unit ray d from o to an axis-aligned box centred at c (or -1)."""
    tmin, tmax = -np.inf, np.inf
    for k in range(3):
        if abs(d[k]) < 1e-12:
            if abs(o[k] - c[k]) > half[k]:
                return -1.0
            continue
        t1 = (c[k] - half[k] - o[k]) / d[k]
        t2 = (c[k] + half[k] - o[k]) / d[k]
        tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    return tmin if (tmax >= max(tmin, 0.0) and tmin > 0) else -1.0


def synth_rings(target, yaw_deg=0.0):
    dirs = ring_beam_directions(DEFAULT)
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    out = {}
    for name, D in dirs.items():
        out[name] = np.array([ray_box(np.zeros(3), R @ d, target, HALF) for d in D])
    return out, R


def test_neighbour_localised_within_eps():
    proc = ProximityProcessor([], DEFAULT)
    for target, yaw in (((3.0, 0.0, 0.0), 0.0), ((0.0, 2.6, 0.3), 0.0), ((-2.0, -2.0, -0.5), 30.0), ((1.6, 1.0, 0.0), -60.0)):
        rings, R = synth_rings(np.array(target), yaw)
        res = ProximityProcessor([], DEFAULT).process(0.0, rings, np.zeros(3), R)
        assert len(res.detections) == 1, (target, len(res.detections))
        err = np.abs(res.detections[0].rel - np.array(target))
        assert np.all(err <= DEFAULT.env.eps_rel), (target, err)


def test_ring_naming_and_elevation_convention():
    d = ring_beam_directions(DEFAULT)
    up = d[ring_name(30)][0]
    assert up[2] > 0.49                        # positive ring elevation points up
    beam_left = d[ring_name(0)][DEFAULT.perc.ring_beams * 3 // 4]
    assert beam_left[1] > 0.99                 # beam index 3N/4 points to body +y (left)
