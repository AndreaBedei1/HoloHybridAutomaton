"""HoloOcean viewport camera that frames the whole fleet (display only).

The camera is moved with HoloOcean's ``env.move_viewport(location, rotation)`` (TeleportCamera).
Its convention was measured on the patched 2.3.0 engine, because the client documents it in two
different ways and neither matches:

* ``location`` is in world metres (the client frame, z up, water surface at z = 0);
* ``rotation`` is not ``[roll, pitch, yaw]`` (TeleportCameraCommand's docstring) but the direction the
  camera looks along ("the x-axis that the camera should look down", move_viewport's docstring), with
  the sign inverted by the client-to-engine conversion: ``[a, b, c]`` looks along ``-(a, b, c)``
  (``[0, 0, 10]`` looks straight down, ``[30, 0, 0]`` along -x, ``[0, 30, 0]`` along -y; ``[0, 0, 0]``
  keeps the default, +x).  :func:`viewport_rotation` turns a look direction into that argument.

Framing.  Underwater fog hides a drone beyond about 10-12 m, so the camera never backs off further
than ``MAX_DISTANCE_M``: it frames the drones within ``KEEP_RADIUS_M`` of the fleet median (a drone
lost far away cannot be shown anyway and does not drag the view), the gate when the fleet is near
it, and nearby scripted vehicles.  It looks along the local path direction from behind and slightly
above, stays ``SURFACE_CLEARANCE_M`` below the surface (the surface itself renders as noise), and is
smoothed with one time constant: no jumps, no sudden zoom.  The pose depends only on the set of
positions, never on which drone is first (it does not follow drone 0).  Positions are ground truth
handed over by the live viewer; nothing here reaches a controller.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

HFOV_DEG = 90.0              # conservative horizontal field of view of the 1280 x 720 viewport
MIN_DISTANCE_M = 6.0
MAX_DISTANCE_M = 11.0        # beyond ~10-12 m the fog hides a drone
MARGIN_M = 1.5               # around the framed points
ELEVATION_DEG = 20.0         # camera above the framed centre, looking down
SURFACE_CLEARANCE_M = 1.5
KEEP_RADIUS_M = 12.0         # drones / vehicles farther than this from the fleet median are not framed
GATE_IN_M, GATE_OUT_M = 12.0, 15.0   # the gate joins the frame below GATE_IN_M, leaves above GATE_OUT_M
TAU_S = 2.0                  # smoothing time constant of centre, distance and heading


def viewport_rotation(look) -> np.ndarray:
    """``rotation`` argument of ``move_viewport`` that makes the camera look along ``look`` (measured)."""
    v = np.asarray(look, float).reshape(3)
    return -v / max(float(np.linalg.norm(v)), 1e-9)


def framing(positions, heading: float, gate=None, intruders=None) -> Tuple[np.ndarray, float]:
    """Centre and viewing distance that fit the fleet (plus gate / vehicles) seen along ``heading`` [rad]."""
    P = np.asarray(positions, float).reshape((-1, 3))
    if not len(P):
        raise ValueError("framing needs at least one drone position")
    med = np.median(P, axis=0)
    pts = [p for p in P if np.linalg.norm(p - med) <= KEEP_RADIUS_M] or [med]
    if intruders is not None:
        pts += [q for q in np.asarray(intruders, float).reshape((-1, 3)) if np.linalg.norm(q - med) <= KEEP_RADIUS_M]
    if gate is not None:
        pts.append(np.asarray(gate, float).reshape(3))
    pts = np.asarray(pts)
    centre = (pts.min(axis=0) + pts.max(axis=0)) / 2.0
    h = np.array([math.cos(heading), math.sin(heading)])
    lat = np.array([-h[1], h[0]])
    rel = pts[:, :2] - centre[:2]
    half_width = float(np.max(np.abs(rel @ lat))) + MARGIN_M
    behind = max(0.0, float(np.max(-(rel @ h))))         # framed points closer to the camera than the centre
    distance = half_width / math.tan(math.radians(HFOV_DEG / 2.0)) + behind
    return centre, float(np.clip(distance, MIN_DISTANCE_M, MAX_DISTANCE_M))


def camera_pose(centre, distance: float, heading: float) -> Tuple[np.ndarray, np.ndarray]:
    """Location behind ``centre`` along ``heading`` and the rotation that looks at ``centre``."""
    c = np.asarray(centre, float)
    z = min(c[2] + distance * math.sin(math.radians(ELEVATION_DEG)), -SURFACE_CLEARANCE_M)
    dz = z - c[2]
    back = math.sqrt(max(distance ** 2 - dz ** 2, 1.0))
    loc = np.array([c[0] - back * math.cos(heading), c[1] - back * math.sin(heading), z])
    return loc, viewport_rotation(c - loc)


class FleetCamera:
    """Smoothed fleet framing for the HoloOcean viewport (see the module docstring)."""

    def __init__(self, path=None, tau_s: float = TAU_S):
        self.path = path
        self.tau_s = tau_s
        self.centre: Optional[np.ndarray] = None
        self.distance: Optional[float] = None
        self.heading: Optional[float] = None
        self.with_gate = False

    def _path_heading(self, p) -> float:
        if self.path is None:
            return 0.0 if self.heading is None else self.heading
        s, _, _ = self.path.project(np.asarray(p, float)[:2])
        return float(self.path.heading_at(s))

    def update(self, positions, dt_s: float, gates: Sequence = (), intruders=None):
        """New ``(location, rotation)`` for ``move_viewport`` after ``dt_s`` seconds."""
        P = np.asarray(positions, float).reshape((-1, 3))
        med = np.median(P, axis=0)
        gate = None
        if len(gates):
            g = min((np.asarray(c, float) for c in gates), key=lambda c: float(np.linalg.norm(c[:2] - med[:2])))
            d = float(np.linalg.norm(g[:2] - med[:2]))
            self.with_gate = d <= GATE_IN_M or (self.with_gate and d <= GATE_OUT_M)
            gate = g if self.with_gate else None
        heading = self._path_heading(med)
        centre, distance = framing(P, heading, gate=gate, intruders=intruders)
        if self.centre is None:
            self.centre, self.distance, self.heading = centre, distance, heading
        else:
            a = 1.0 - math.exp(-max(0.0, float(dt_s)) / self.tau_s)
            self.centre = self.centre + a * (centre - self.centre)
            self.distance += a * (distance - self.distance)
            self.heading += a * ((heading - self.heading + math.pi) % (2.0 * math.pi) - math.pi)
        return camera_pose(self.centre, self.distance, self.heading)
