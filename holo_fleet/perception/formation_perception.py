"""Formation perception (P3): own slot error from navigation + sonar range consistency.

The navigation estimate gives the error between the drone and its own slot.  The sonars give a
*relative* check that the navigation cannot: for every neighbour the template expects within
sonar range, the drone knows in which sectors it should appear and at which centre distance.
Echoes are associated by sector and range (no bearing is assumed inside a cone) and the range
residual ``dr = d_measured - d_expected`` is projected on the template direction of that
neighbour: its along-track part feeds the progress correction, its lateral part the lateral
spacing correction (control/flows.py).  Neighbours expected in range but not seen are MISSING; the
perception (perception.py) times them and declares a slot vacant after ``t_rejoin``: a vacant slot is
no longer required (DEGRADED_FORMATION) until its drone is seen there again.  The residual of the neighbour
directly ahead on the same lane (``front_residual``, negative when it is closer than planned with respect to the
own slot) is what a drone follows when that neighbour slows down (control/formation_time.py).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

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
    alias: tuple = ()          # slots a range-only sonar cannot tell apart from this one (confirmed as a group)


@dataclass
class FormationObs:
    slot_err: np.ndarray = field(default_factory=lambda: np.zeros(3))
    slot_err_norm: float = 0.0
    checks: List[NeighbourCheck] = field(default_factory=list)
    along_corr: float = 0.0           # weighted along-track range residual [m]
    lateral_corr: float = 0.0
    lateral_far: Optional[float] = None   # away from the slot: lateral error of the own reference from the fixes [m]
    form_err: float = 0.0             # onboard formation-error estimate used by the guards [m]
    neighbors_ok: bool = True
    missing: List[int] = field(default_factory=list)      # required (expected nearby, observable), not seen recently
    required: List[int] = field(default_factory=list)     # expected nearby and observable (healthy sectors, unmasked)
    front_residual: Optional[float] = None                # range residual of the same-lane neighbour ahead (seen now)
    behind: Optional[float] = None                        # B2: largest certain lag of a non-vacant neighbour [m]


def same_lane_ahead(plan: MissionPlan, lane_tol: float = 1.0) -> List[int]:
    """Slots directly ahead of the own slot on the same lane (template: along-track ahead, same lateral offset and
    depth within ``lane_tol``): the pairs whose spacing an along-track slowdown compresses."""
    me = plan.slots[plan.slot_index]
    return [k for k, sl in enumerate(plan.slots) if k != plan.slot_index and sl.along - me.along > lane_tol
            and abs(sl.lateral - me.lateral) <= lane_tol and abs(sl.dz - me.dz) <= lane_tol]


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
                    last_seen: Dict[int, float] = None, readings=None, vacant=(),
                    acoustic: Optional[Dict[int, np.ndarray]] = None, missing=()) -> FormationObs:
    """Expected neighbours vs sonar echoes, one-to-one per (target, sector).

    Association is done per sector of a target, not per target: two neighbours at similar ranges
    in adjacent sectors are merged into one target by the target builder (e.g. the side and the
    rear neighbour of a square, REAR+RIGHT), and each of its sectors can still confirm one of
    them.  An echo explains a present neighbour before one that is ``missing`` or ``vacant`` (the absence of those
    is what has been observed), then the smallest range residual wins.  A neighbour counts as present if it was
    matched within ``NEIGHBOUR_MEMORY_S``; only neighbours expected within ``fr.neighbour_range_m`` are required
    (weak returns near the maximum range are intermittent), and not those whose echo would fall inside a mapped
    structure's window (a range-only sensor classifies it STRUCTURE: unobservable, not missing).  Neighbours whose
    echoes cannot be separated (same expected sectors and range) are confirmed as a group (``alias``).  Slots in
    ``vacant`` (declared vacant by the perception after t_rejoin) are still associated when an echo fits them, but
    they are not required.  A match is not an identification: the perception confirms a missing or vacant
    neighbour only when no other neighbour can account for the echo (perception.track_missing).

    B2 only: ``acoustic`` maps a slot to the relative position of its drone from a fresh accepted acoustic fix
    (map-frame axes).  For those neighbours an echo is associated where the IDENTIFIED drone is (sector pattern
    compatible with the fix direction, range within the range gate of the fix distance) instead of where the
    template expects it, and only when that echo also lies within the range gate of the template distance, as in B0
    (the fix chooses WHICH echo is the neighbour, never makes a drone far from its slot count as seen there); the
    residual stays the one against the template (it is the formation error).  An echo where the fix of a heard
    neighbour places it belongs to a heard drone, so it is never associated with a neighbour without a fresh fix."""
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
    for chk, _dw, dist, _u, exp_pat in expected:
        chk.alias = tuple(c2.slot for c2, _d2, dist2, _u2, pat2 in expected
                          if c2 is not chk and set(pat2) == set(exp_pat) and abs(dist2 - dist) < fr.range_gate_m)
    fixes = {}                                     # B2: slot -> (distance, body direction, sectors) of a fresh fix
    for chk, _dw, _dist, _u, _p in expected:
        v = None if not acoustic else acoustic.get(chk.slot)
        if v is not None and float(np.linalg.norm(v)) > 0.3:
            da = float(np.linalg.norm(v))
            ua = R_world_body.T @ (np.asarray(v, float) / da)
            fixes[chk.slot] = (da, ua, far_field_pattern(ua))

    def near(look, tg, sec):                       # centre distance of the echo along ``look``, None if incompatible
        d_a, u_a, pat_a = look
        if sec not in pat_a or not compatible(tg.pattern, u_a):
            return None
        return estimate_centre_distance(tg.ranges.get(sec, tg.r_min), sec, u_a)

    def heard_there(tg, sec):                      # B2: the fix of some heard neighbour places its drone at this echo
        for f in fixes.values():
            d = near(f, tg, sec)
            if d is not None and abs(d - f[0]) <= fr.range_gate_m:
                return True
        return False

    cands = []                                     # (priority, |residual|, neighbour index, target index, sector, d_m)
    for i, (chk, _dw, dist, u_body, exp_pat) in enumerate(expected):
        look = fixes.get(chk.slot, (dist, u_body, exp_pat))     # where to look: the fix (B2) or the template
        prio = 2 if chk.slot in vacant else (1 if chk.slot in missing else 0)
        for j, tg in enumerate(targets):
            for sec in tg.pattern:
                d_m = near(look, tg, sec)
                if d_m is None:
                    continue
                if abs(d_m - look[0]) <= 3.0:
                    tg.possible_neighbour = True
                # B2: near the fix AND, as in B0, within the range gate of the slot (a drone heard far from its
                # slot is not seen at it: no formation residual from it, the missing / vacancy path as in B0)
                if abs(d_m - look[0]) > fr.range_gate_m or abs(d_m - dist) > fr.range_gate_m:
                    continue
                if chk.slot not in fixes and heard_there(tg, sec):
                    continue                       # the echo of a heard drone is not that of an unheard one
                cands.append((prio, abs(d_m - look[0]), i, j, sec, d_m))
    used_n, used_ts = set(), set()
    for _p, _r, i, j, sec, d_m in sorted(cands):
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
    # neighbours a range-only sonar cannot separate (same expected sector pattern, expected distances within the
    # range gate: e.g. a stacked pair seen from the side, one echo for both) are observable only as a group: an
    # association of one member confirms the group
    for chk, d_world, dist, _u, _p in expected:
        if chk.seen:
            continue
        for chk2, _dw2, _d2, _u2, _p2 in expected:
            if chk2.seen and chk2.slot in chk.alias:
                g = d_world / dist
                chk.measured_d, chk.residual, chk.seen = chk2.measured_d, chk2.measured_d - dist, True
                chk.along_w, chk.lateral_w = float(g @ t_hat), float(g @ n_hat)
                last_seen[chk.slot] = t
                break
    for i, (chk, _dw, dist, u_b, exp_pat) in enumerate(expected):
        required = (dist <= fr.neighbour_range_m and all(healthy.get(x, False) for x in exp_pat)
                    and not masked_by_structure(dist, exp_pat, u_b, readings, cfg))
        if required:
            o.required.append(chk.slot)
        if required and t - last_seen.get(chk.slot, -1e9) > NEIGHBOUR_MEMORY_S:
            o.missing.append(chk.slot)                 # expected nearby, healthy sectors, not seen recently
        o.checks.append(chk)
    seen = [c for c in o.checks if c.seen]
    if seen:
        o.along_corr = float(np.mean([c.residual * c.along_w for c in seen]))
        o.lateral_corr = float(np.mean([c.residual * c.lateral_w for c in seen]))
    # residual of the neighbour ahead on the same lane relative to the OWN slot: the own along-track tracking error
    # (an avoidance manoeuvre, a drone still closing on its slot) changes the range the same way and is removed, so
    # that the drone compares its reference with the drone ahead (control/formation_time.py)
    ahead = same_lane_ahead(plan)
    e_along = float(np.asarray(slot_err, float) @ t_hat)
    front = [c.residual - e_along for c in seen if c.slot in ahead]
    o.front_residual = float(min(front)) if front else None
    o.form_err = float(max(errs))
    o.neighbors_ok = not [k for k in o.missing if k not in vacant]
    return o
