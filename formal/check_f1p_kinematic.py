"""F1pk: kinematic reconstruction of the F1p counterexample with the deployed gate flow and escape planner (reported, not a proof).

F1p / F1pm (formal/check_fleet.py) find two drones in the CR when two drones commit concurrently (a claim learnt late) and one of
them is moved into the CR by avoidance.  The fleet model has no positions, so it cannot say whether the avoidance layer can do that
while the other drone is inside.  This reconstruction gives the abstraction its positions: two or three drones committed at their
queue points of the abreast queue (n = 2, 3), the claim of each drone known to the others ``delta`` seconds after it commits (the
stage-2 entry_ok of each drone false while it knows another live claim), every pair and order of commitment, commit gaps and claim
delays from 0.5 s to 12 s, with and without a 0.15 m start jitter.  Each drone runs the deployed code of the committed drone:
Flows.gate (stage-2 hold, the RELOC back-off, the give-up after t_occ_max + t_clear) and, by the guards of the automaton with
their hysteresis, Flows.separation_warning / Flows.collision_avoidance (EscapePlanner).  Not deployed, idealised here: the sector
patterns and distance bounds are computed from the true relative positions (sonar_geometry.pattern_of_point, centre_distance_lower:
no noise, no missed echo, no stale sector), the navigation is exact, the plant is a velocity loop limited by a_brake, there is
no current; a drone not yet committed or past exit_s does not move (no avoidance, and a finished drone stays just past exit_s);
commit times are scripted, not produced by the commit guard; entry_ok is the claim alone (no sonar latch, loc_ok always true); a
claim ends at exit_s, not at a verified passage; the give-up of the automaton (pass_abort) and a recommitment are not modelled,
only the retreat target of the flow.  Counted: entries of a centre into the CR without the stage-2 admission (cr_cleared) and
instants with two centres in the CR (the referee's test).  Two companion sets show what the scan can and cannot see.  Control:
claims never known (the stage-2 check never holds anyone); mutation: in addition no avoidance.  The mutation must produce double
occupancy (the scan can detect it).  If the control produces none either, the absence of double occupancy in the scan is due to
separation and the pass path converging on the axis, not to the gate protocol: then F1pk says nothing about the stage-2 logic and
F1p is not addressed by it.  Evidence over a finite set only, in any case.
"""

from __future__ import annotations

import itertools
import time
from types import SimpleNamespace as NS

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.control.flows import Flows
from holo_fleet.perception.sonar_geometry import MOUNTS, R_IN, centre_distance_lower, pattern_of_point

GATE = NS(axis=np.array([1.0, 0.0, 0.0]), center=np.zeros(3), from_gate_frame=lambda c: np.asarray(c, float))
GAPS = (0.0, 1.0, 3.0, 8.0)
DELAYS = (0.5, 2.0, 5.0, 12.0)


def _in_cr(G, p) -> bool:
    return abs(p[0]) <= G.cr_half_len and abs(p[1]) <= G.cr_half_width and abs(p[2]) <= G.cr_half_height


def _target(pi, pj):
    rel = pj - pi
    pat = pattern_of_point(rel, R_IN)
    if not pat:
        return None
    dl = max(centre_distance_lower(max(float(np.linalg.norm(rel - MOUNTS[s])) - R_IN, 0.0), s) for s in pat)
    return NS(cls="DYNAMIC", pattern=pat, d_lower=dl, r_min=float(np.linalg.norm(rel)))


def _mode(prev: str, d: float, sep) -> str:
    if d < sep.d_ca or (prev == "CA" and d < sep.d_ca_exit):
        return "CA"
    if d < sep.d_warning or (prev == "SW" and d < sep.d_warning_exit):
        return "SW"
    return "MUTEX"


def episode(cfg: FleetConfig, n: int, commits, delta: float, jitter=None, horizon: float = 60.0, dt: float = 0.1,
            avoid: bool = True) -> dict:
    """One concurrent episode at the gate (gate frame = world frame, every heading along the axis)."""
    G, sep, env = cfg.gate, cfg.sep, cfg.env
    lanes, qs = list(G.queue_laterals(n)), G.queue_s(n)
    P = [np.array([qs, lanes[k], 0.0]) + (0.0 if jitter is None else jitter[k]) for k in range(n)]
    V = [np.zeros(3) for _ in range(n)]
    fl = [Flows(NS(gates=[GATE], slots=[NS(lateral=lanes[k], dz=0.0)], slot_index=0, path=NS(depth_z=0.0), clock=None), cfg)
          for k in range(n)]
    mode, done = ["MUTEX"] * n, [False] * n
    out = {"pushed": set(), "double": None, "dmin": float("inf")}
    for step in range(int(horizon / dt)):
        t = step * dt
        active = [t >= commits[k] and not done[k] for k in range(n)]
        cmds = []
        for i in range(n):
            if not active[i]:
                cmds.append(np.zeros(3))
                continue
            claim = any(active[j] and t >= commits[j] + delta for j in range(n) if j != i)
            go = NS(s=P[i][0], l=P[i][1], dz=P[i][2], queue_s=qs, queue_l=lanes[i], queue_dz=0.0, loc_ok=True, entry_ok=not claim)
            tg = [x for x in (_target(P[i], P[j]) for j in range(n) if j != i) if x is not None]
            obs = NS(gate=go, gate_index=0, t=t, p=P[i], R_wb=np.eye(3), targets=tg, unhealthy=[], yaw=0.0)
            v_mis, _ = fl[i].gate(obs, "MUTEX_PASS")
            m = _mode(mode[i], min([x.d_lower for x in tg] + [float("inf")]), sep) if avoid else "MUTEX"
            if m == "MUTEX" and mode[i] != "MUTEX":
                fl[i].escape.reset()
            mode[i] = m
            v = (fl[i].separation_warning(obs, v_mis) if m == "SW" else
                 fl[i].collision_avoidance(obs, np.zeros(3), v_mis) if m == "CA" else v_mis)
            cmds.append(v)
            if _in_cr(G, P[i]) and not fl[i].cr_cleared:
                out["pushed"].add(i)
        for i in range(n):
            dv = cmds[i] - V[i]
            nv = float(np.linalg.norm(dv))
            if nv > env.a_brake * dt:
                dv *= env.a_brake * dt / nv
            V[i] = V[i] + dv
            P[i] = P[i] + V[i] * dt
            done[i] = done[i] or P[i][0] > G.exit_s
        for i, j in itertools.combinations(range(n), 2):
            out["dmin"] = min(out["dmin"], float(np.linalg.norm(P[i] - P[j])))
            if out["double"] is None and not (done[i] or done[j]) and _in_cr(G, P[i]) and _in_cr(G, P[j]):
                out["double"] = {"t": round(t, 1), "drones": (i, j)}
    return out


def scan(cfg: FleetConfig = DEFAULT) -> dict:
    """Every pair and order of commitment, commit gap and claim delay; without jitter, with jitter, and (n = 3) with the third
    drone committing between the two."""
    rng = np.random.default_rng(0)
    res = {"episodes": 0, "entries_without_admission": 0, "double_occupancy": 0, "min_centre_distance_m": float("inf"),
           "examples": []}
    for n in (2, 3):
        for first, second in itertools.permutations(range(n), 2):
            for gap, delta, rep in itertools.product(GAPS, DELAYS, range(3)):
                commits = [float("inf")] * n
                commits[first], commits[second] = 0.0, gap
                if n == 3 and rep == 2:
                    commits[({0, 1, 2} - {first, second}).pop()] = gap / 2
                jit = None if rep == 0 else [rng.normal(0.0, 0.15, 3) * np.array([1.0, 1.0, 0.5]) for _ in range(n)]
                o = episode(cfg, n, commits, delta, jitter=jit)
                res["episodes"] += 1
                res["entries_without_admission"] += bool(o["pushed"])
                res["min_centre_distance_m"] = round(min(res["min_centre_distance_m"], o["dmin"]), 2)
                if o["double"]:
                    res["double_occupancy"] += 1
                    if len(res["examples"]) < 5:
                        res["examples"].append({"n": n, "order": (first, second), "gap_s": gap, "delay_s": delta, **o["double"]})
    for name, avoid in (("control_claims_never_known", True), ("mutation_no_claims_no_avoidance", False)):
        hits = 0
        for n in (2, 3):
            for (first, second), gap in itertools.product(itertools.permutations(range(n), 2), GAPS[:3]):
                commits = [float("inf")] * n
                commits[first], commits[second] = 0.0, gap
                hits += episode(cfg, n, commits, float("inf"), avoid=avoid)["double"] is not None
        res[name] = {"episodes": 3 * (2 + 6), "double_occupancy": hits}
    return res


def check(cfg: FleetConfig = DEFAULT):
    from common import CheckResult

    t0 = time.time()
    r = scan(cfg)
    found = r["double_occupancy"] > 0
    blind = r["control_claims_never_known"]["double_occupancy"] == 0
    return CheckResult("F1pk reported (not a proof): kinematic reconstruction of F1p with the deployed Flows.gate and escape planner, "
                       "idealised perception and plant; concurrent commitments of 2-3 drones, commit gaps 0-8 s, claim delays 0.5-12 s",
                       "count instants with two centres in the CR over the finite set of episodes",
                       "formal/check_f1p_kinematic.py", "none", "observed" if found else "none", not found,
                       round(time.time() - t0, 1),
                       note=str(r) + ("; the control without claims has no double occupancy either: in this reconstruction separation "
                                      "and the axis geometry keep the drones apart, so F1pk does not test the stage-2 logic and "
                                      "leaves F1p open" if blind else ""))
