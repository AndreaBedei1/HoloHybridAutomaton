"""Acceptance criteria v3 (REFEREE side): a-priori requirements and deadlines from the mission plan, the protocol's
own bounds and the definition of the sensor profiles, never from runs of the system under test.

Every constant is labelled by its provenance:
  [REQ]   a mission requirement fixed before the campaign (a design choice, not derived);
  [PROT]  a bound or parameter the protocol itself declares (config.py, the flows), reused here;
  [LIT]   a criterion from the literature, applied to a protocol parameter;
  [PROF]  derived from the definition of a sensor profile (sim/sensor_noise.py), never from its effect on a run.

Requirements of a run (the same set is used by the online TerminalMonitor and the offline classifier):
  progress     at the SUCCESS instant every healthy drone is within PROGRESS_TOL_M of its PLANNED end position: its
               slot where the plan clock reaches the end of the path (T_nom, below).  Not a latch: a drone that came
               close and drifted away again does not count;
  gate         every healthy drone has PASSED the judged gate: before it, then through the critical region's cross
               section (inside the CR at some instant), then beyond the exit.  A drone that went around or over the
               frame has not passed (tag GATE_BYPASS);
  P3           formation scenarios: the formation error of the healthy drones is <= E_TOL_M, held T_HOLD_S, and every
               loss episode (error > E_TOL_M) has recovered.  With a permanent fault the formation is that of the
               healthy drones on their original slots (P3-deg), and every healthy drone that expects the lost drone as
               a neighbour (template distance <= neighbour_range_m) has declared its slot vacant;
  vacancy      at the SUCCESS instant no healthy drone holds vacant the slot of a drone that is not permanently
               faulted (ground truth);
  waiting      no healthy drone is still waiting for a neighbour (FORMATION_WAIT_REJOIN);
  gate liveness (bounded response, offline: referee/gate_events.py) no healthy drone waits at an empty, uncommitted
               gate longer than the protocol's slowest release, and no committed healthy drone takes longer than its
               crossing budget (L / v_pass + t_occ_max + t_clear).

One band E_TOL_M opens and closes an episode (recovery needs T_HOLD_S inside the band): the v2 pair e_lost 1.2 /
e_ok 0.5 left a band in which an error never opened an episode.  T_HOLD_S is the 2 % settling time of the
slot-tracking loop, 4 time constants of a first-order loop (Ogata, Modern Control Engineering): a transient crossing of
the band does not count as recovery.  The run stops at the first SUCCESS instant, so a slow limit cycle whose in-band
phase lasts longer than the hold is not excluded (a stated limit of the finite run).

T_nom and deadline (``deadline_v3``):
  T_nom     the instant the plan clock of every drone reaches the end of its path (holds and jumps included): the
            mission's planned end, a property of the plan [PROT];
  gate      the excess of the serialised worst-case passages over the dwell the plan already gives the gate: per
            healthy drone L_k / v_pass + t_occ_max + t_clear, L_k = |queue_lateral_k| + exit_s - queue_s [PROT];
  formation t_rejoin + T_HOLD_S for drones that keep a slot: a displaced drone within the sensing range closes the gap
            within t_rejoin, the protocol's catch-up bound (config.FormationRule.t_rejoin) [PROT];
  avoidance plans without a slot (single drones): the give-way detour (2 x the 2.5 m offset of the traffic rule,
            control/flows.traffic_offset) recovered at v_slot_max - v_plan [PROT];
  faults    a permanent loss: the lag the reduced progress leaves, t_rejoin (1 - v_wait / v_nominal); a temporary
            loss of duration D: the lag (v_nominal + w_drift_max) D closed at the catch-up speed [PROT];
  failsafe  the outages of the sensor profile: for each outage process (sonar blackouts, DVL outages) the 99 %
            quantile of its Poisson count over the horizon times (outage length + the freshness limit that ends the
            FAILSAFE, 0.35 s) [PROF].
DEADLINE means "not done within this allowance", not a proof that a protocol bound was exceeded; deadlocks and
bounded-response violations of the gate are judged separately (gate liveness; a generic no-progress detector outside
the gate does not exist yet).  Comparisons between cells are to use time-to-SUCCESS with right censoring
(Kaplan-Meier, doi:10.1080/01621459.1958.10501452), so that a conclusion does not hinge on the deadline value.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional

import numpy as np

CRITERIA_VERSION = "v8"   # v8 (2026-11-13): v7 with the P3 contract (referee/p3.py: P3a, P3b, P3c, deadlines, premises) and the classes P3_FAILURE and PREMISE_VIOLATED (robustness_metrics.refine_v8).  v7: v7 (2026-11-10): v6, with the collision sensor of a drone after its TOTAL_FAILURE recorded and not a contact (docs/ACCEPTANCE_CRITERIA.md).  v6 (2026-11-09): the requirements and classes of v5; the fault term of the deadline uses the silence
                          # interval of the total failure model (docs/ACCEPTANCE_CRITERIA.md).  v5 (2026-11-05, the user's decision after the v4 campaign): liveness is eventual - no
                          # time bound on a gate passage enters the class; the run horizon is HORIZON_FACTOR x the
                          # a-priori deadline (which stays the reported "on time" reference); scenarios whose mission
                          # is physically infeasible are judged on safety only.
# previous: "v4"   # v4 (2026-11-04): unsafe-action disjunction E2.to_free | E3 | E4, inclusive tolerance,
                          # SAFE_DEGRADED with the hold, crossing obligation from the first commit, bypass participation
E_TOL_M = 1.0              # [REQ] formation tolerance (agreed for v3; the controller keeps e_ok = 0.5 as its margin)
GIVEWAY_OFFSET_M = 2.5     # [PROT] = control/flows.Flows.traffic_offset(offset_max) (tests keep them equal)
DETECT_S = 0.35            # [PROT] DVL freshness limit of sense_ok (perception.update): ends a sensor-fault FAILSAFE
FS_QUANTILE = 0.99         # [LIT] allowance at the 99 % quantile of the outage count (a coverage choice, stated)


def t_hold_s(cfg) -> float:
    """[LIT] 2 % settling time of the first-order slot-tracking loop: 4 / k_slot."""
    return 4.0 / cfg.form.k_slot


def progress_tol_m(cfg) -> float:
    """[PROT] the along-track lag the reduced progress may leave by design, t_rejoin (v_nominal - v_wait) (3.0 m with
    the configured values; v2 used 3.0 m without a derivation)."""
    F = cfg.form
    return F.t_rejoin * (F.v_nominal - F.v_wait)


def profile_family(profile: Optional[str]) -> str:
    """CLEAN / LOW / MEDIUM / HIGH ... from a sensor-profile name such as MEDIUM-SUBSET (None: CLEAN)."""
    if not profile or profile.upper().startswith("CLEAN"):
        return "CLEAN"
    return profile.upper().split("-")[0]


def permanent_faults(sc) -> set:
    return {f["drone"] for f in (getattr(sc.sim, "faults", ()) or ()) if f.get("t_off") is None}


def _clock_end(clock, s_end: float, t_max: float = 3600.0, dt: float = 0.1) -> float:
    lo, hi = 0.0, t_max                         # s(t) is non-decreasing: bisection on s(t) >= s_end
    if clock.s(hi) < s_end - 1e-6:
        return t_max
    while hi - lo > dt / 2:
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if clock.s(mid) < s_end - 1e-6 else (lo, mid)
    return round(hi, 1)


def nominal_end_s(sc) -> float:
    """T_nom: the instant the plan clock of every drone reaches the end of its path (the planned end)."""
    ends = [_clock_end(pl.clock, min(pl.s_end, pl.clock.s_end)) for pl in sc.plans if pl.clock is not None]
    return max(ends) if ends else float(sc.duration_s)


def plan_targets(sc, t_nom: Optional[float] = None) -> List[np.ndarray]:
    """Planned position of every drone at T_nom: its slot at the plan clock's arclength (jumps and holds included)."""
    t_nom = nominal_end_s(sc) if t_nom is None else float(t_nom)
    out = []
    for pl in sc.plans:
        sl = pl.my_slot
        s = (pl.clock.s(t_nom) if pl.clock is not None else pl.s_end) + sl.along
        p, _d, n = pl.path.frame_at(min(s, pl.path.length))
        out.append(np.array([p[0] + sl.lateral * n[0], p[1] + sl.lateral * n[1], pl.path.depth_z + sl.dz]))
    return out


def formation_judged(sc) -> bool:
    return bool(sc.formation_enabled and sc.template is not None and len(sc.plans) > 1)


def expects(sc, cfg, k: int, j: int) -> bool:
    """Drone k expects drone j as a neighbour (template distance within the sensing range of the formation rule)."""
    a, b = sc.plans[k].slots[sc.plans[k].slot_index], sc.plans[k].slots[sc.plans[j].slot_index]
    return math.dist((a.along, a.lateral, a.dz), (b.along, b.lateral, b.dz)) <= cfg.form.neighbour_range_m


def _poisson_quantile(lam: float, q: float) -> int:
    k, p, cdf = 0, math.exp(-lam), math.exp(-lam)
    while cdf < q:
        k += 1
        p *= lam / k
        cdf += p
    return k


def failsafe_allowance_s(profile: Optional[str], horizon_s: float) -> float:
    """[PROF] FAILSAFE time the profile's scheduled outages can impose on one drone over ``horizon_s``."""
    from holo_fleet.sim.sensor_noise import profile as noise_profile

    fam = profile_family(profile)
    if fam == "CLEAN":
        return 0.0
    nz = noise_profile(fam)
    out = 0.0
    for every, length in ((nz.sonar_blackout_every_s, nz.sonar_blackout_len_s), (nz.dvl_outage_every_s, nz.dvl_outage_len_s)):
        if every > 0.0 and length > 0.0:
            out += _poisson_quantile(horizon_s / every, FS_QUANTILE) * (length + DETECT_S)
    return round(out, 1)


def deadline_terms(sc, cfg, profile: Optional[str] = None) -> Dict[str, float]:
    G, F = cfg.gate, cfg.form
    names = [pl.drone_id for pl in sc.plans]
    lost = permanent_faults(sc)
    t_nom = nominal_end_s(sc)
    t_gate = 0.0
    for g in sc.judged_gates:
        serial = sum((abs(pl.queue_lateral) + G.exit_s - pl.queue_s) / G.v_pass + G.t_occ_max + G.t_clear
                     for pl in sc.plans if pl.drone_id not in lost)
        dwell = sum(d for _s, d in (sc.plans[0].clock.holds if sc.plans[0].clock is not None else ()))
        t_gate += max(0.0, serial - dwell)
    keeps_slot = any(pl.formation_enabled and len(pl.slots) > 1 for pl in sc.plans)
    t_form = (F.t_rejoin + t_hold_s(cfg)) if keeps_slot else 0.0
    v_plan = max((pl.clock.v for pl in sc.plans if pl.clock is not None), default=F.v_nominal)
    t_avoid = 0.0 if keeps_slot else 2.0 * GIVEWAY_OFFSET_M / max(F.v_slot_max - v_plan, 1e-3)
    t_fault = 0.0
    for f in (getattr(sc.sim, "faults", ()) or ()):
        if f["drone"] not in names:
            continue
        if f.get("t_off") is None:
            # the drones wait until they have declared the silent drone dead (the silence interval, or t_rejoin)
            t_fault += max(F.t_rejoin, cfg.form.t_silent_s) * (1.0 - F.v_wait / F.v_nominal)
        else:
            t_fault += (F.v_nominal + cfg.env.w_drift_max) * (float(f["t_off"]) - float(f["t_on"])) / F.catch_up
    base = t_nom + t_gate + t_form + t_avoid + t_fault
    return {"T_nom": round(t_nom, 1), "gate": round(t_gate, 1), "formation": round(t_form, 1),
            "avoidance": round(t_avoid, 1), "faults": round(t_fault, 1),
            "failsafe": failsafe_allowance_s(profile, base)}


def deadline_v3(sc, cfg, profile: Optional[str] = None) -> float:
    return round(sum(deadline_terms(sc, cfg, profile).values()), 1)


HORIZON_FACTOR = 3.0      # [PROT] the exploratory horizon of the plan (H.2): eventual completion is observed up to here


def horizon_v5(sc, cfg, profile: Optional[str] = None) -> float:
    """Simulation horizon of a run: HORIZON_FACTOR x the a-priori deadline.  A run ends earlier at SUCCESS or FAILURE;
    reaching the horizon without SUCCESS is INCOMPLETE (not observed to complete), never a proof of a deadlock."""
    return round(HORIZON_FACTOR * deadline_v3(sc, cfg, profile), 1)
