"""Formation perception (P3): own slot error from navigation + sonar range consistency.

The navigation estimate gives the error between the drone and its own slot.  The sonars give a
*relative* check that the navigation cannot: for every neighbour the template expects within
sonar range, the drone knows in which sectors it should appear and at which centre distance.
Echoes are associated by sector and range (no bearing is assumed inside a cone) and the range
residual ``dr = d_measured - d_expected`` is projected on the template direction of that
neighbour: its along-track part feeds the progress correction, its lateral part the lateral
spacing correction (control/flows.py).  Neighbours expected in range but not seen make the
formation "not consistent" for the automaton guards.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.mission import MissionPlan
from holo_fleet.perception.sonar_geometry import AXES, MOUNTS, SECTORS, far_field_pattern

HULL_MID = 0.24            # typical extent of a BlueROV2 hull towards the observer [m]
PATTERN_TOL_DEG = 35.0     # a displaced neighbour may appear up to this beyond the expected sectors
NEIGHBOUR_MEMORY_S = 1.0   # a neighbour matched within this is present (intermittent weak returns)


@dataclass
class NeighbourCheck:
    slot: int
    expected_d: float
    expected_pattern: str
    measured_d: float = float("nan")
    residual: float = float("nan")
    along_w: float = 0.0
    lateral_w: float = 0.0
    seen: bool = False


@dataclass
class FormationObs:
    slot_err: np.ndarray = field(default_factory=lambda: np.zeros(3))
    slot_err_norm: float = 0.0
    checks: List[NeighbourCheck] = field(default_factory=list)
    along_corr: float = 0.0           # weighted along-track range residual [m]
    lateral_corr: float = 0.0
    form_err: float = 0.0             # onboard formation-error estimate used by the guards [m]
    neighbors_ok: bool = True


def expected_relative(plan: MissionPlan, R_formation: np.ndarray) -> Dict[int, np.ndarray]:
    me = plan.slots[plan.slot_index]
    out = {}
    for k, sl in enumerate(plan.slots):
        if k != plan.slot_index:
            out[k] = R_formation @ np.array([sl.along - me.along, sl.lateral - me.lateral, sl.dz - me.dz])
    return out


def estimate_centre_distance(r_echo: float, sector: str, u_body: np.ndarray) -> float:
    """Best estimate (not a bound) of the centre distance from an echo range in ``sector``."""
    m = MOUNTS[sector]
    return float(r_echo + HULL_MID + float(np.dot(m, u_body)))


def compatible(pattern, u_body: np.ndarray, tol_deg: float = PATTERN_TOL_DEG) -> bool:
    """A target seen by ``pattern`` can be the neighbour expected in direction ``u_body`` only if every
    sector that sees it points within (cone half-angle + tol) of that direction (displaced neighbours
    allowed, a neighbour on the other side of the hull excluded)."""
    lim = np.cos(np.radians(60.0 + tol_deg))
    return all(float(AXES[s] @ u_body) >= lim for s in pattern)


def masked_by_structure(expected_d: float, exp_pat, u_body: np.ndarray, readings, cfg: FleetConfig) -> bool:
    """True when, in every sector where the neighbour is expected, its echo would fall inside the window
    explained by a mapped structure (it would be classified STRUCTURE: unobservable, not missing)."""
    if not readings:
        return False
    pc = cfg.perc
    for sec in exp_pat:
        rd = readings.get(sec)
        sw = None if rd is None else rd.structure_window
        if sw is None:
            return False
        r = expected_d - HULL_MID - float(np.dot(MOUNTS[sec], u_body))
        if not (sw[0] - pc.structure_tol_m - 0.3 <= r <= sw[1] + pc.structure_tol_far_m + 0.3):
            return False
    return True


def check_formation(plan: MissionPlan, cfg: FleetConfig, slot_err: np.ndarray, R_formation: np.ndarray,
                    R_world_body: np.ndarray, targets, healthy: Dict[str, bool], t: float = 0.0,
                    last_seen: Dict[int, float] = None, readings=None) -> FormationObs:
    """Expected neighbours vs sonar echoes, one-to-one per (target, sector).

    Association is done per sector of a target, not per target: two neighbours at similar ranges
    in adjacent sectors are merged into one target by the target builder (e.g. the side and the
    rear neighbour of a square, REAR+RIGHT), and each of its sectors can still confirm one of
    them.  A neighbour counts as present if it was matched within ``NEIGHBOUR_MEMORY_S``; only
    neighbours expected within ``fr.neighbour_range_m`` are required (weak returns near the
    maximum range are intermittent), and not those whose echo would fall inside a mapped structure's
    window (a range-only sensor classifies it STRUCTURE: unobservable, not missing)."""
    fr = cfg.form
    o = FormationObs(slot_err=slot_err, slot_err_norm=float(np.linalg.norm(slot_err)))
    t_hat, n_hat = R_formation[:, 0], R_formation[:, 1]
    sonar_max = cfg.perc.sonar.range_max - 1.0
    last_seen = {} if last_seen is None else last_seen
    errs = [o.slot_err_norm]
    expected = []
    for k, d_world in expected_relative(plan, R_formation).items():
        dist = float(np.linalg.norm(d_world))
        if dist > sonar_max:
            continue
        u_body = R_world_body.T @ (d_world / dist)
        exp_pat = far_field_pattern(u_body)
        expected.append((NeighbourCheck(slot=k, expected_d=dist, expected_pattern="+".join(sorted(exp_pat))),
                         d_world, dist, u_body, exp_pat))
    cands = []                                     # (|residual|, neighbour index, target index, sector, d_measured)
    for i, (_chk, _dw, dist, u_body, exp_pat) in enumerate(expected):
        for j, tg in enumerate(targets):
            if not (set(tg.pattern) & set(exp_pat)) or not compatible(tg.pattern, u_body):
                continue
            for sec in tg.pattern:
                if sec not in exp_pat:
                    continue
                d_m = estimate_centre_distance(tg.ranges.get(sec, tg.r_min), sec, u_body)
                if abs(d_m - dist) <= 3.0:
                    tg.possible_neighbour = True
                if abs(d_m - dist) <= fr.range_gate_m:
                    cands.append((abs(d_m - dist), i, j, sec, d_m))
    used_n, used_ts = set(), set()
    for _r, i, j, sec, d_m in sorted(cands):
        if i in used_n or (j, sec) in used_ts:
            continue
        used_n.add(i)
        used_ts.add((j, sec))
        chk, d_world, dist, _u, _p = expected[i]
        targets[j].expected_neighbour = True
        g = d_world / dist
        chk.measured_d, chk.residual, chk.seen = d_m, d_m - dist, True
        chk.along_w, chk.lateral_w = float(g @ t_hat), float(g @ n_hat)
        errs.append(abs(chk.residual))
        last_seen[chk.slot] = t
    all_ok = True
    for i, (chk, _dw, dist, u_b, exp_pat) in enumerate(expected):
        required = (dist <= fr.neighbour_range_m and all(healthy.get(x, False) for x in exp_pat)
                    and not masked_by_structure(dist, exp_pat, u_b, readings, cfg))
        if required and t - last_seen.get(chk.slot, -1e9) > NEIGHBOUR_MEMORY_S:
            all_ok = False                             # expected nearby, healthy sectors, not seen recently
        o.checks.append(chk)
    seen = [c for c in o.checks if c.seen]
    if seen:
        o.along_corr = float(np.mean([c.residual * c.along_w for c in seen]))
        o.lateral_corr = float(np.mean([c.residual * c.lateral_w for c in seen]))
    o.form_err = float(max(errs))
    o.neighbors_ok = all_ok
    return o
