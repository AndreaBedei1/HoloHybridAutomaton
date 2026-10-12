"""Pure camera geometry and smoothing for the HoloOcean fleet overview (display only)."""

from __future__ import annotations

import math
from typing import Optional, Sequence

import numpy as np


def fleet_camera_pose(positions: Sequence[Sequence[float]], path_direction: Sequence[float], *,
                      gate: Optional[Sequence[float]] = None, intruders=None,
                      fov_deg: float = 68.0, margin_m: float = 3.0,
                      min_distance_m: float = 10.0, max_distance_m: float = 65.0,
                      elevation_deg: float = 28.0, gate_range_m: float = 15.0,
                      intruder_range_m: float = 14.0):
    """Return ``(location, rotation, distance)`` for a rear-oblique spectator camera.

    Positions are referee data passed only by the live visualizer. Nearby gates and intruders
    join the framing bounds; distant ones cannot pull the camera away from the fleet.
    Rotation uses HoloOcean's ``[roll, pitch, yaw]`` convention.
    """
    fleet = np.asarray(positions, dtype=float).reshape((-1, 3))
    if not len(fleet):
        raise ValueError("fleet_camera_pose needs at least one drone position")
    points = list(fleet)
    if gate is not None:
        g = np.asarray(gate, dtype=float).reshape(3)
        if float(np.min(np.linalg.norm(fleet - g, axis=1))) <= gate_range_m:
            points.append(g)
    if intruders is not None:
        for p in np.asarray(intruders, dtype=float).reshape((-1, 3)):
            if float(np.min(np.linalg.norm(fleet - p, axis=1))) <= intruder_range_m:
                points.append(p)
    points = np.asarray(points, dtype=float)
    center = (points.min(axis=0) + points.max(axis=0)) / 2.0
    radius = float(np.max(np.linalg.norm(points - center, axis=1))) + margin_m
    distance = float(np.clip(radius / math.sin(math.radians(fov_deg / 2.0)),
                             min_distance_m, max_distance_m))
    direction = np.asarray(path_direction, dtype=float)[:2]
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        direction = np.array([1.0, 0.0])
    else:
        direction /= norm
    elev = math.radians(elevation_deg)
    back = np.array([-direction[0] * math.cos(elev), -direction[1] * math.cos(elev), math.sin(elev)])
    location = center + distance * back
    # Keep the spectator in the water. HoloOcean's open-water surface is z=0.
    location[2] = min(float(location[2]), -0.35)
    look = center - location
    horizontal = math.hypot(float(look[0]), float(look[1]))
    pitch = math.degrees(math.atan2(float(look[2]), horizontal))
    yaw = math.degrees(math.atan2(float(look[1]), float(look[0])))
    return location, np.array([0.0, pitch, yaw]), distance


class FleetCameraSmoother:
    """Exponential camera smoothing; zooming out responds faster than zooming back in."""

    def __init__(self, zoom_out_tau_s: float = 0.45, zoom_in_tau_s: float = 1.8,
                 rotation_tau_s: float = 0.45):
        self.zoom_out_tau_s = zoom_out_tau_s
        self.zoom_in_tau_s = zoom_in_tau_s
        self.rotation_tau_s = rotation_tau_s
        self.location = None
        self.rotation = None
        self.distance = None

    @staticmethod
    def _blend_angle(old, new, alpha):
        delta = (new - old + 180.0) % 360.0 - 180.0
        return old + alpha * delta

    def update(self, pose, dt_s: float):
        location, rotation, distance = pose
        location, rotation = np.asarray(location, float), np.asarray(rotation, float)
        if self.location is None:
            self.location, self.rotation, self.distance = location.copy(), rotation.copy(), float(distance)
            return self.location.copy(), self.rotation.copy(), self.distance
        dt = max(0.0, float(dt_s))
        tau = self.zoom_out_tau_s if distance > self.distance else self.zoom_in_tau_s
        alpha = 1.0 - math.exp(-dt / max(tau, 1e-6))
        arot = 1.0 - math.exp(-dt / max(self.rotation_tau_s, 1e-6))
        self.location += alpha * (location - self.location)
        self.rotation = np.array([self._blend_angle(a, b, arot) for a, b in zip(self.rotation, rotation)])
        self.distance += alpha * (float(distance) - self.distance)
        return self.location.copy(), self.rotation.copy(), self.distance
