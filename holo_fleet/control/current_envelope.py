"""Control-feasible current envelope (DI-27): one definition shared by the onboard monitor, the offline run
evaluation, the tests and the formal checks.

A current w is acceptable for a requested velocity v_d when the deployed low-level loop can deliver
v_d against it without saturating, and when w lies in the range that has been exercised.

The low level (control/lowlevel.py) commands, in the body frame,

    u = v_d / s + k_v (v_d - v) + k_i i,        |(u_surge, u_sway)| <= AUTHORITY[mode],

with s = surge_speed_per_cmd.  In steady state the velocity error vanishes and u is the command the plant
needs for the through-water velocity r = v_d - w.  Per horizontal axis the plant needs
g(r) = |r| / s + c r^2 (calibrated, results/calibration/head_current_authority.json), so

    required(w, v_d) = |( g(r_surge), g(r_sway) )|  <=  AUTHORITY[mode].

This is the controller's own geometry (a norm bound on the commands), not a bound on |w|: the margin
depends on the direction of w relative to the requested motion and on the requested speed.  With the
nominal authority the BlueROV2 makes about 0.71 m/s through the water along one axis, so at the 0.30 m/s
survey speed a head current is limited to about 0.41 m/s while a lateral one may reach about 0.67 m/s.
The vertical channel is separate (heave clip) and is not binding: the vertical current is bounded by the
declared ``Envelope.current_vertical_max``.  ``Envelope.current_validated_max`` is the largest horizontal
magnitude exercised: nothing beyond it is claimed, in any direction.

Two estimators of the same steady command:
* ground truth (run evaluation, formal checks): the true current and the plant curve g, accurate to
  ``PlantCalibration.cmd_model_tolerance`` on the calibration points;
* onboard (EnvelopeMonitor): the drone's current estimate is the integral action of its own velocity
  loop expressed with the linear model s (LowLevelController.current_estimate), so the steady command is
  exactly |v_d - w_est| / s.  The estimate is in model units (about 1.5 times the true drift at 0.3-0.4 m/s),
  so onboard only the authority test is evaluated, never the range test.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Optional, Sequence

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.control.lowlevel import AUTHORITY

AUTH_NOMINAL = AUTHORITY["nominal"]


def axis_command(r: float, cfg: FleetConfig = DEFAULT) -> float:
    """Steady command needed on one horizontal axis for a through-water speed r [m/s] (plant curve g)."""
    pl = cfg.plant
    return abs(r) / pl.surge_speed_per_cmd + pl.cmd_quadratic * r * r


def axis_speed(command: float, cfg: FleetConfig = DEFAULT) -> float:
    """Inverse of axis_command: through-water speed delivered on one axis by a steady command."""
    pl = cfg.plant
    a, b = pl.cmd_quadratic, 1.0 / pl.surge_speed_per_cmd
    return (-b + math.sqrt(b * b + 4.0 * a * max(command, 0.0))) / (2.0 * a)


def components(w: Sequence[float], v_desired: Sequence[float]):
    """(head, lateral, vertical) of the current relative to the requested horizontal motion.

    head > 0 opposes the motion; lateral is the magnitude of the horizontal part perpendicular to it.
    Without a horizontal request (hover, queue hold) the whole horizontal current is reported as lateral.
    """
    w = np.asarray(w, dtype=float)
    vh = np.asarray(v_desired, dtype=float)[:2]
    n = float(np.linalg.norm(vh))
    if n < 1e-3:
        return 0.0, float(np.linalg.norm(w[:2])), float(w[2])
    t = vh / n
    head = -float(w[:2] @ t)
    lateral = abs(float(-t[1] * w[0] + t[0] * w[1]))
    return head, lateral, float(w[2])


@dataclass(frozen=True)
class CurrentCheck:
    current: tuple                 # w [m/s] (true, or the onboard estimate in model units)
    desired: tuple                 # requested velocity v_d [m/s]
    head: float
    lateral: float
    vertical: float
    magnitude: float               # |w_h|
    through_water: float           # |v_d,h - w_h| [m/s]
    required_cmd: float            # steady horizontal command needed (normalised)
    authority: float               # AUTHORITY of the mode (normalised)
    feasible: bool                 # required_cmd <= authority
    in_range: bool                 # |w_h| <= current_validated_max and |w_z| <= current_vertical_max
    ok: bool
    reason: str

    def as_dict(self) -> dict:
        d = asdict(self)
        for k, v in d.items():
            if isinstance(v, float):
                d[k] = round(v, 3)
            elif isinstance(v, tuple):
                d[k] = [round(float(x), 3) for x in v]
        return d


def _reason(feasible: bool, in_range: bool, head: float, lateral: float, vertical: float, magnitude: float,
            required: float, authority: float, cfg: FleetConfig) -> str:
    if not feasible:
        what = "head" if head >= lateral else "lateral"
        return (f"not control-feasible: steady command {required:.2f} > authority {authority:.2f} "
                f"({what} current: head {head:+.2f}, lateral {lateral:.2f} m/s)")
    if not in_range:
        if magnitude > cfg.env.current_validated_max + 1e-9:
            return f"current {magnitude:.2f} m/s beyond the exercised range {cfg.env.current_validated_max:.2f} m/s"
        return f"vertical current {vertical:+.2f} m/s beyond {cfg.env.current_vertical_max:.2f} m/s"
    return ""


def check_current(w: Sequence[float], v_desired: Sequence[float], cfg: FleetConfig = DEFAULT,
                  heading: Optional[float] = None, authority: float = AUTH_NOMINAL) -> CurrentCheck:
    """Ground truth: the true current w and the plant curve.

    The through-water velocity is split on the drone's body axes (``heading`` [rad]); without a heading the
    axes are the requested motion and its normal (the survey case, where the drone heads along its path).
    """
    w = np.asarray(w, dtype=float)
    v = np.asarray(v_desired, dtype=float)
    r = v[:2] - w[:2]
    if heading is None:
        n = float(np.linalg.norm(v[:2]))
        heading = math.atan2(v[1], v[0]) if n >= 1e-3 else math.atan2(r[1], r[0])
    c, s = math.cos(heading), math.sin(heading)
    r_s, r_w = c * r[0] + s * r[1], -s * r[0] + c * r[1]
    required = math.hypot(axis_command(r_s, cfg), axis_command(r_w, cfg))
    head, lateral, vertical = components(w, v)
    magnitude = float(np.linalg.norm(w[:2]))
    feasible = required <= authority + 1e-12
    in_range = magnitude <= cfg.env.current_validated_max + 1e-9 and abs(vertical) <= cfg.env.current_vertical_max + 1e-9
    return CurrentCheck(tuple(w), tuple(v), head, lateral, vertical, magnitude, float(np.linalg.norm(r)), required,
                        authority, feasible, in_range, feasible and in_range,
                        _reason(feasible, in_range, head, lateral, vertical, magnitude, required, authority, cfg))


def check_onboard(w_est: Sequence[float], v_desired: Sequence[float], cfg: FleetConfig = DEFAULT,
                  authority: float = AUTH_NOMINAL) -> CurrentCheck:
    """Onboard: the steady command of the implemented loop, |v_d - w_est| / s, against the authority."""
    w = np.asarray(w_est, dtype=float)
    v = np.asarray(v_desired, dtype=float)
    tw = float(np.linalg.norm(v[:2] - w[:2]))
    required = tw / cfg.plant.surge_speed_per_cmd
    head, lateral, vertical = components(w, v)
    feasible = required <= authority + 1e-12
    return CurrentCheck(tuple(w), tuple(v), head, lateral, vertical, float(np.linalg.norm(w[:2])), tw, required,
                        authority, feasible, True, feasible,
                        _reason(feasible, True, head, lateral, vertical, 0.0, required, authority, cfg))


def single_axis_speed(cfg: FleetConfig = DEFAULT, authority: float = AUTH_NOMINAL) -> float:
    """Through-water speed available along one axis at the authority (about 0.71 m/s at nominal)."""
    return axis_speed(authority, cfg)


def head_limit(speed: float, cfg: FleetConfig = DEFAULT, authority: float = AUTH_NOMINAL) -> float:
    """Largest current straight against a requested speed that keeps it deliverable (and exercised)."""
    return max(0.0, min(single_axis_speed(cfg, authority) - speed, cfg.env.current_validated_max))


def lateral_limit(speed: float, cfg: FleetConfig = DEFAULT, authority: float = AUTH_NOMINAL) -> float:
    """Largest current perpendicular to a requested speed that keeps it deliverable (and exercised)."""
    rest = authority ** 2 - axis_command(speed, cfg) ** 2
    if rest <= 0.0:
        return 0.0
    return min(axis_speed(math.sqrt(rest), cfg), cfg.env.current_validated_max)


class Persistence:
    """The envelope monitor's persistence rule: a leaky time accumulator of 'bad' steps.

    +dt on a bad step, -dt otherwise; the violation is declared when it reaches t_enter and cleared after
    t_exit consecutive good steps.  Shared by the onboard monitor and by the run evaluation."""

    def __init__(self, t_enter: float = 4.0, t_exit: float = 4.0):
        self.t_enter, self.t_exit = t_enter, t_exit
        self.bad_time = 0.0
        self.ok_time = 0.0
        self.ok = True

    def update(self, bad: bool, dt: float) -> bool:
        if bad:
            self.bad_time += dt
            self.ok_time = 0.0
        else:
            self.ok_time += dt
            self.bad_time = max(0.0, self.bad_time - dt)
        # the same comparisons as the v2 EnvelopeMonitor (the logged runs replay identically)
        if self.ok and self.bad_time >= self.t_enter:
            self.ok = False
        elif not self.ok and self.ok_time >= self.t_exit:
            self.ok = True
        return self.ok
