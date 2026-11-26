"""Conditional P1 interval margin for all peers, independent of formation-slot association.

A common sonar capture is usable under the hull visibility, echo completeness and
range-error premises. A previous certified interval can carry that premise through
a short observation gap while the physical ground-speed caps continue to hold.
A missing echo cannot start a distance bound.
"""

import math

import numpy as np

from holo_fleet.perception.distance_bound import conservative_bound, nearest_distance_lower
from holo_fleet.perception.exit_witness import speed_bound
from holo_fleet.perception.sonar_geometry import SECTORS


def swept_margin(distance, own_speed, peer_speed, safety_distance, interval):
    return distance - safety_distance - (own_speed + peer_speed) * interval


def propagate_lower(distance, closing_speed, elapsed):
    return distance - closing_speed * elapsed


def sonar_lower(readings, cfg, closing_speed):
    """A common-snapshot bound, or a partial bound that can only reduce retained evidence."""
    env = cfg.env
    healthy = {s: readings[s] for s in SECTORS
               if s in readings and readings[s].healthy and -1e-9 <= readings[s].age <= env.tau_max + 1e-9}
    ages = {s: max(0.0, rd.age) for s, rd in healthy.items()}
    ranges = {}
    has_return = False
    for s, rd in healthy.items():
        returns = [e.r0 for e in rd.echoes]
        has_return |= bool(returns)
        if rd.blind_from is not None:
            returns.append(rd.blind_from)
        if returns:
            ranges[s] = min(returns)
    if not ranges:
        return None, False
    common = len(healthy) == len(SECTORS) and max(ages.values()) - min(ages.values()) <= 1e-9 and has_return
    bound = nearest_distance_lower if common else conservative_bound
    return bound(ranges, ages, env.eps_range_far, closing_speed, cfg.perc.sonar), common


def interval_record(obs, frame, velocity, thrust, dt, cfg, previous=None):
    env = cfg.env
    speed = speed_bound(cfg)
    distance, common = sonar_lower(obs.readings, cfg, 2 * speed)
    source = "SYNCHRONOUS_SONAR"
    if not common:
        if not previous or previous["status"] != "INTERVAL_CONDITIONAL":
            return {"status": "UNOBSERVED"}
        elapsed = frame.t - previous["t"]
        if not 0.0 < elapsed <= previous["horizon_s"] + 1e-9:
            return {"status": "UNOBSERVED"}
        carried = propagate_lower(previous["lower_m"], 2 * speed, elapsed)
        distance = carried if distance is None else min(distance, carried)
        source = "PROPAGATED"
    if not math.isfinite(distance):
        return {"status": "UNOBSERVED"}
    dvl = frame.sample("DVLSensor", env.tau_max, 3)
    imu = frame.sample("IMUSensor", env.tau_max, 6)
    if dvl is None or imu is None or imu.ndim != 2 or imu.shape[0] < 2 or imu.shape[1] != 3:
        return {"status": "MOTION_UNOBSERVED"}
    if (not np.isfinite(obs.v_world).all() or not np.isfinite(velocity).all() or not np.isfinite(thrust).all()
            or np.linalg.norm(dvl.ravel()[:3]) > speed or np.linalg.norm(obs.v_world) > speed
            or np.linalg.norm(velocity) > speed - env.w_drift_max
            or np.max(abs(thrust)) > cfg.plant.thruster_limit or not 0.0 < dt <= env.dt + 1e-9):
        return {"status": "MOTION_OUTSIDE"}
    margin = swept_margin(distance, speed, speed, cfg.sep.d_safe, env.dt)
    return {"status": "INTERVAL_CONDITIONAL" if margin > 0.0 else "NO_MARGIN", "margin_m": margin,
            "horizon_s": env.dt, "speed_cap_m_s": speed, "source": source, "lower_m": distance, "t": frame.t}
