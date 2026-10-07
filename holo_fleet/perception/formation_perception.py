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


def check_formation(plan: MissionPlan, cfg: FleetConfig, slot_err: np.ndarray, R_formation: np.ndarray,
                    R_world_body: np.ndarray, targets, healthy: Dict[str, bool]) -> FormationObs:
    """Expected neighbours vs sonar targets, one-to-one (greedy on |range residual|)."""
    fr = cfg.form
    o = FormationObs(slot_err=slot_err, slot_err_norm=float(np.linalg.norm(slot_err)))
    t_hat, n_hat = R_formation[:, 0], R_formation[:, 1]
    sonar_max = cfg.perc.sonar.range_max - 1.0
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
    cands = []                                     # (|residual|, neighbour index, target index, d_measured)
    for i, (_chk, _dw, dist, u_body, exp_pat) in enumerate(expected):
        for j, tg in enumerate(targets):
            if not (set(tg.pattern) & set(exp_pat)) or not compatible(tg.pattern, u_body):
                continue
            sec = min((x for x in tg.pattern if x in exp_pat), key=lambda x: tg.ranges.get(x, 99.0))
            d_m = estimate_centre_distance(tg.ranges.get(sec, tg.r_min), sec, u_body)
            if abs(d_m - dist) <= 3.0:
                tg.possible_neighbour = True
            if abs(d_m - dist) <= fr.range_gate_m:
                cands.append((abs(d_m - dist), i, j, d_m))
    used_n, used_t = set(), set()
    for _r, i, j, d_m in sorted(cands):
        if i in used_n or j in used_t:
            continue
        used_n.add(i)
        used_t.add(j)
        chk, d_world, dist, _u, _p = expected[i]
        targets[j].expected_neighbour = True
        g = d_world / dist
        chk.measured_d, chk.residual, chk.seen = d_m, d_m - dist, True
        chk.along_w, chk.lateral_w = float(g @ t_hat), float(g @ n_hat)
        errs.append(abs(chk.residual))
    all_ok = True
    for i, (chk, _dw, _d, _u, exp_pat) in enumerate(expected):
        if not chk.seen and all(healthy.get(x, False) for x in exp_pat):
            all_ok = False                             # expected in range, healthy sectors, not seen
        o.checks.append(chk)
    seen = [c for c in o.checks if c.seen]
    if seen:
        o.along_corr = float(np.mean([c.residual * c.along_w for c in seen]))
        o.lateral_corr = float(np.mean([c.residual * c.lateral_w for c in seen]))
    o.form_err = float(max(errs))
    o.neighbors_ok = all_ok
    return o
