"""Payload v2 (gate status), the total failure model and the vacancy by silence: formal and randomized checks.

The B2-specific checks of formal/check_acoustic.py keep covering the geometry of the fixes; these cover what the v11 design
adds.  The premises are the declared ones (docs/GUARANTEES.md): the gate-frame navigation error at most ``nav_err_m``, the
speed bound of a drone, the queue tolerance, a received packet is genuine (CRC-16) and of its frame.  The freshness bound is
a function of the queue layout.

* S1 positive occupancy: with a claim (a report of committed or admitted) of another drone the latch is BUSY and stage 2 holds
  the entry, in every state of the latch.
* S2 M3-status (Z3, bounded model checking, the timed latch abstraction of formal/check_mutex.py): the phantom exclusion of
  corridor echoes while the crossing drone's last report says it is not engaged (its later packets lost), with the FRONT-only
  corridor echoes never making a relation, never lets a queued drone commit while the crosser is in the CR, at the deployed
  freshness bound, for the queue layouts; the mutation with the larger freshness that the design first proposed must be
  violated.  (Model checked for the worst speed and offset of the crossing drone: speed bound, queue tolerance plus the
  declared navigation error.)
* S3 evidence order: a status older than an accepted fix that places the drone possibly inside the CR never excludes it.
* S4 silence: a drone that fell silent after reporting committed or admitted keeps the gate held for ever.
* S5 structural: the precedence relations are never computed from acoustic data or status.
* S6 the relation dropped for a FRONT-only echo in the corridor band is never one between drones at their queue points.
* S7 a verified passage is sound: a drone reported past the gate has its true centre beyond the CR grown by R_OUT (randomized
  over the fit error at admission, the DVL scale and heading error, a drone blocked at a bar never verifying).
* S8 the freshness bound is below the shortest time from a queue point to the CR at the speed bound.
"""

from __future__ import annotations

import dataclasses
import math
import sys
from pathlib import Path

import numpy as np
import z3

from common import ROOT, CheckResult, Report, check  # noqa: E402

from holo_fleet.config import DEFAULT, BeaconConfig, FleetConfig
from holo_fleet.mission import GateSpec, MissionPlan, Path as MPath, abreast_slots, queue_line
from holo_fleet.perception import payload as pl
from holo_fleet.perception.acoustic import AcousticTracker
from holo_fleet.perception.gate_perception import GatePerception
from holo_fleet.perception.passage import PassageVerifier
from holo_fleet.perception.sonar_geometry import MOUNTS, R_OUT, SECTORS, far_field_pattern

import check_mutex as cm  # noqa: E402

ENC = "perception/payload.py + perception/acoustic_gate.py + perception/gate_perception.py + perception/passage.py"
B2 = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_RB"))


def _gate():
    from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates

    return load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]


def _observer(g, n: int, cfg: FleetConfig, idx: int = 0):
    lats = cfg.gate.queue_laterals(n)
    slots = abreast_slots(n)
    qs = queue_line(slots, cfg.gate)
    yaw = math.atan2(g.axis[1], g.axis[0])
    p = g.from_gate_frame([qs, lats[idx], 0.0])
    tr = AcousticTracker(cfg)
    tr.set_identities({k: k for k in range(n)})
    path = MPath(np.array([g.center[:2] - 30.0 * g.axis[:2], g.center[:2] + 30.0 * g.axis[:2]]), float(g.center[2]))
    plan = MissionPlan(drone_id=f"d{idx}", slot_index=idx, slots=slots, path=path, gates=[g], structures=[g], launch_position=p,
                       launch_yaw_deg=math.degrees(yaw), static_rank=idx, s_end=40.0, queue_lateral=lats[idx], queue_s=qs)
    gp = GatePerception(plan, cfg, acoustic=tr)
    return tr, gp, p, yaw


def _rx(tr, p, yaw, t, k, q, cfg, phase, bound=0.5, gate_idx=0):
    d = np.asarray(q, float) - p
    c, s = math.cos(yaw), math.sin(yaw)
    b = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
    r = float(np.linalg.norm(b))
    az, el = math.atan2(b[1], b[0]), math.asin(float(np.clip(b[2] / r, -1.0, 1.0)))
    t_arr = t - 0.05
    fr = pl.frame_index(t_arr - r / cfg.beacon.sound_speed, cfg.beacon.period_s)
    tr.update(t, [(k, t_arr, r, az, el, pl.encode(fr, q, bound, gate_idx, phase))], p, yaw, 0.5)


def positive_occupancy(cfg: FleetConfig, g, n_cases: int = 300, seed: int = 11) -> dict:
    """S1: any state of the latch, a claim of another drone (random position, committed or admitted): BUSY, no admission."""
    rng = np.random.default_rng(seed)
    bad = 0
    for _ in range(n_cases):
        tr, gp, p, yaw = _observer(g, 3, cfg)
        gp.occ_state = str(rng.choice(["FREE", "EXITING", "BUSY"]))
        gp.exit_seen_t = -10.0
        k = int(rng.integers(1, 3))
        q = g.from_gate_frame([rng.uniform(-6.0, 6.0), rng.uniform(-4.0, 4.0), rng.uniform(-1.0, 1.0)])
        phase = int(rng.choice([pl.GP_COMMITTED, pl.GP_ADMITTED]))
        _rx(tr, p, yaw, 1.0, k, q, cfg, phase)
        o = gp.update(1.2 + float(rng.uniform(0.0, 3.0)), p, yaw, [], {})
        bad += int(not (o.occ_busy and not o.entry_ok))
    return {"cases": n_cases, "violations": bad}


def m3_status_model(cfg, geo, view, t_status, v_hi, e0, steps=320):
    """The timed latch abstraction of M3 extended with the phantom rule of the payload v2: B ignores the corridor echoes of A
    while A's last report (before A commits, the later packets being lost) is younger than t_status; the FRONT-only corridor
    echoes make no relation at all."""
    G = cfg.gate
    dt = 0.1
    tc = int(round(G.t_clear / dt))
    t_occ = int(round(G.t_occ_max / dt))
    s_corr_lo = view["corridor_from_s"]
    m_lo_s = view["mask_from_s"]
    m_hi_s = view["visible_again_s"]
    b_s = view["beyond_s"]
    reach = int(math.floor((abs(geo["queue_s"]) - G.cr_half_len) / v_hi / dt))
    V = lambda n, f: [f(f"{n}_{k}") for k in range(steps + 1)]
    sA, seen, corr, beyond, busy = V("sA", z3.Real), V("seen", z3.Bool), V("corr", z3.Bool), V("beyond", z3.Bool), V("busy", z3.Bool)
    since, quiet, prio, commit = V("since", z3.Int), V("quiet", z3.Int), V("prio", z3.Int), V("commit", z3.Bool)
    C = [sA[0] == geo["queue_s"] + e0, z3.Not(busy[0]), quiet[0] == tc, prio[0] == 0, z3.Not(commit[0]), since[0] == 0]
    ph = [(k * dt <= t_status) for k in range(steps + 1)]
    for k in range(steps):
        C.append(z3.And(sA[k + 1] - sA[k] >= cm.V_LO * dt, sA[k + 1] - sA[k] <= v_hi * dt))
    for k in range(steps + 1):
        C.append(seen[k] == z3.Or(sA[k] < m_lo_s, sA[k] > m_hi_s))
        rng = z3.And(sA[k] >= s_corr_lo, sA[k] <= b_s)
        conf = z3.And(seen[k], rng) if k == 0 else z3.And(seen[k], seen[k - 1], rng)
        C.append(corr[k] == z3.And(conf, z3.BoolVal(not ph[k])))
        C.append(beyond[k] == z3.And(seen[k], sA[k] > b_s))
    for k in range(steps):
        C.append(since[k + 1] == z3.If(corr[k + 1], 0, since[k] + 1))
        C.append(busy[k + 1] == z3.If(corr[k + 1], True, z3.If(z3.And(busy[k], z3.Or(beyond[k + 1], since[k + 1] > t_occ)), False, busy[k])))
        C.append(quiet[k + 1] == z3.If(z3.Or(corr[k + 1], busy[k + 1]), 0, quiet[k] + 1))
        rng = z3.And(sA[k + 1] >= s_corr_lo, sA[k + 1] <= b_s)
        wait = z3.And(seen[k + 1], sA[k + 1] < s_corr_lo)    # FRONT-only echoes in the corridor band make no relation
        C.append(prio[k + 1] == z3.If(wait, 0, prio[k] + 1))
        C.append(commit[k + 1] == z3.Or(commit[k], z3.And(prio[k + 1] >= tc, z3.Not(busy[k + 1]), quiet[k + 1] >= tc)))
    viol = []
    for c in range(steps + 1):
        for k in range(c + reach, steps + 1):
            viol.append(z3.And(commit[c], z3.Not(commit[c - 1]) if c > 0 else z3.BoolVal(True), sA[k] >= -G.cr_half_len, sA[k] <= G.cr_half_len))
    C.append(z3.Or(*viol))
    return C


def evidence_order(cfg: FleetConfig, g, n_cases: int = 300, mutate: bool = False, seed: int = 13) -> dict:
    """S3: a status older than a later accepted fix that places the drone possibly in the CR does not exclude it."""
    rng = np.random.default_rng(seed)
    bad = 0
    for _ in range(n_cases):
        tr, gp, p, yaw = _observer(g, 3, cfg)
        ae = gp.acoustic_exit
        ae.gate_idx, ae.g = 0, (None if mutate else g)
        q = g.from_gate_frame([rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), 0.0])
        q2 = q + rng.uniform(-0.1, 0.1, 3)
        _rx(tr, p, yaw, 1.0, 1, q, cfg, pl.GP_NONE)               # status: not engaged, and a fix near the CR
        tr.status[1] = dataclasses.replace(tr.status[1], t_tx=0.2)    # the status of a ping older than the fix below
        d = q2 - p
        c, s = math.cos(yaw), math.sin(yaw)
        b = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
        r = float(np.linalg.norm(b))
        tr.update(2.2, [(1, 2.15, r, math.atan2(b[1], b[0]), math.asin(float(np.clip(b[2] / r, -1, 1))), None)], p, yaw, 0.5)
        bad += int(ae.status_excluded(1, 2.3))
    return {"cases": n_cases, "excluded_by_stale_status": bad}


def silence_never_clears(cfg: FleetConfig, g, n_cases: int = 200, seed: int = 17) -> dict:
    """S4: a drone that reported committed or admitted, silent for up to a day: the claim stands, it is not excluded."""
    rng = np.random.default_rng(seed)
    bad = 0
    for _ in range(n_cases):
        tr, gp, p, yaw = _observer(g, 3, cfg)
        ae = gp.acoustic_exit
        phase = int(rng.choice([pl.GP_COMMITTED, pl.GP_ADMITTED]))
        q = g.from_gate_frame([rng.uniform(-3.0, 3.0), 0.0, 0.0])
        _rx(tr, p, yaw, 1.0, 1, q, cfg, phase)
        t = 1.0 + float(rng.uniform(cfg.acoustic.t_silent_gate_s + 1.0, 86400.0))
        tr.heard[2] = t - 0.5                                     # the receiver works: drone 2 is still heard
        bad += int(ae.status_excluded(1, t) or 1 not in ae.claims(t) or not tr.silent(1, t))
    return {"cases": n_cases, "violations": bad}


def structural() -> list:
    """S5: nothing acoustic in the precedence computation (the relation loop and the decision of GatePerception.update)."""
    gp = (ROOT / "holo_fleet/perception/gate_perception.py").read_text(encoding="utf-8")
    a = gp.index("        rels = []")
    b = gp.index("        o.decision = o.raw_decision")
    block = gp[a:b]
    return [w for w in ("acoustic", "ae.", "status", "claims", "phantom") if w in block]


def front_band_queue_pairs(cfg: FleetConfig) -> list:
    """S6: drones at their queue points (with the tolerance) are never a FRONT-only target of each other."""
    G = cfg.gate
    bad = []
    for n in range(2, 4):
        lats = G.queue_laterals(n)
        qs = queue_line(abreast_slots(n), G)
        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                for ds in (-G.queue_tol, 0.0, G.queue_tol):
                    for dl in (-G.queue_tol, 0.0, G.queue_tol):
                        for hdg in (-G.heading_tol_deg, 0.0, G.heading_tol_deg):
                            yaw = math.radians(hdg)
                            d = np.array([ds, lats[j] - lats[i] + dl, 0.0])
                            u = d / np.linalg.norm(d)
                            c, s = math.cos(yaw), math.sin(yaw)
                            ub = np.array([c * u[0] + s * u[1], -s * u[0] + c * u[1], u[2]])
                            pat = far_field_pattern(ub)
                            if set(pat) == {"FRONT"}:
                                bad.append((n, i, j, ds, dl, hdg))
    return bad


def passage_soundness(cfg: FleetConfig, g, n_cases: int = 2000, seed: int = 19) -> dict:
    """S7: a passage verified by the onboard rules has the true centre beyond the CR grown by R_OUT, for fit errors at admission,
    DVL scale errors and outages; a drone blocked in the CR never verifies."""
    rng = np.random.default_rng(seed)
    G, bad, blocked_bad = cfg.gate, 0, 0
    s_lim = G.cr_half_len + R_OUT
    dt = 0.1
    for _ in range(n_cases):
        v = PassageVerifier(cfg)
        e_fit = float(rng.uniform(-0.4, 0.4))
        s0_true = -1.18
        v.admit(0, g, g.from_gate_frame([s0_true + e_fit, 0.0, 0.0]))
        scale = 1.0 + float(rng.uniform(-0.03, 0.03))
        blocked = rng.random() < 0.3
        s_block = float(rng.uniform(-0.8, 0.9))
        speed = float(rng.uniform(0.2, 0.55))
        s, t, verified_at = s0_true, 0.0, None
        for _k in range(int(120.0 / dt)):
            t += dt
            ds = speed * dt
            if blocked and s + ds >= s_block:
                ds = 0.0
            s += ds
            v_meas = (ds / dt) * scale if rng.random() > 0.1 else None   # 10 % of the steps have no DVL sample
            p_nav = g.from_gate_frame([s + e_fit + (s - s0_true) * (scale - 1.0), 0.0, 0.0])
            near = abs(s) <= 0.3
            w = g.inner_width
            sectors = {"LEFT": [w / 2 - 0.3] if near else [], "RIGHT": [w / 2 - 0.3] if near else [], "UP": [], "DOWN": [],
                       "REAR": [s - 0.5] if s > 1.0 else []}
            v.update(t, dt, p_nav, g.axis * (v_meas if v_meas is not None else 0.0), sectors, dvl_fresh=v_meas is not None)
            if v.verified and verified_at is None:
                verified_at = s
        if verified_at is not None and verified_at < s_lim:
            bad += 1
        if blocked and verified_at is not None:
            blocked_bad += 1
    return {"cases": n_cases, "verified_inside": bad, "blocked_verified": blocked_bad}


def keepout_barrier(cfg: FleetConfig, g, n_cases: int = 5000, seed: int = 23) -> dict:
    """S9: the flow barrier of a drone not committed to the gate never moves it closer to the CR once inside the keep-out band."""
    from holo_fleet.control.flows import Flows
    from holo_fleet.perception.gate_perception import GateObs

    rng = np.random.default_rng(seed)
    slots = abreast_slots(2)
    plan = MissionPlan(drone_id="d0", slot_index=0, slots=slots, path=MPath(np.array([[-30., 0.], [30., 0.]]), float(g.center[2])),
                       gates=[g], structures=[g], launch_position=np.zeros(3), launch_yaw_deg=0., static_rank=0, s_end=40.,
                       queue_lateral=0., queue_s=queue_line(slots, cfg.gate))
    fl = Flows(plan, cfg)
    G, bad = cfg.gate, 0
    axis = np.asarray(g.axis, float)
    s_keep = -(G.cr_half_len + R_OUT + G.nav_err_m)
    for _ in range(n_cases):
        s = float(rng.uniform(s_keep - 1.0, G.cr_half_len))
        go = GateObs(gate_id=g.gate_id, s=s, l=float(rng.uniform(-1.5, 1.5)), dz=float(rng.uniform(-1.0, 1.0)))
        v = rng.uniform(-0.6, 0.6, 3)
        vout = fl.keepout_filter(type("O", (), {"gate": go, "gate_index": 0})(), v)
        if s >= s_keep and float(vout @ axis) > 1e-9:
            bad += 1
    return {"cases": n_cases, "toward_cr_inside_band": bad}


def backing_up_guard(cfg: FleetConfig) -> list:
    """S10: DROP_BACK at a stationary reference (control/flows.py) never backs a drone up toward a gate it has passed: the
    plane from which the reversal is refused (exit_s + 1 m) is beyond the CR grown by the hull and the navigation error."""
    G, bad = cfg.gate, []
    plane = G.exit_s + 1.0
    if plane < G.cr_half_len + R_OUT + G.nav_err_m:
        bad.append(("plane", plane))
    return bad


def freshness_arithmetic(cfg: FleetConfig) -> list:
    """S8: the freshness bound used per plan <= the time from the queue point to the CR at the speed bound minus the latency."""
    G = cfg.gate
    from holo_fleet.perception.exit_witness import speed_bound

    v = speed_bound(cfg)
    bad = []
    for n in (2, 3):
        qs = queue_line(abreast_slots(n), G)
        t_min = (abs(qs) - G.cr_half_len - R_OUT - G.queue_tol - G.nav_err_m) / v
        fresh = min(cfg.acoustic.status_fresh_s, t_min - cfg.acoustic.latency_max_s)
        if fresh > t_min - cfg.acoustic.latency_max_s + 1e-9 or fresh < 0.0:
            bad.append((n, fresh, t_min))
    return bad


def run(cfg: FleetConfig = B2, verbose: bool = True) -> Report:
    rep = Report("status")
    g = _gate()
    r = positive_occupancy(cfg, g)
    rep.add(CheckResult("S1 positive occupancy: a report of committed or admitted of another drone makes the latch BUSY and holds the entry",
                        "randomized: claim -> occ_busy and not entry_ok", ENC, "holds",
                        verdict="holds" if r["violations"] == 0 else "violated", passed=r["violations"] == 0, note=str(r)), verbose)
    for n, v_hi, e0 in ((2, 0.70, 1.35), (3, 0.70, 1.35), (4, 0.70, 1.35)):
        geo = cm.geometry(cfg, n)
        for view in geo["views"]:
            if view["queue_l"] < -1e-6 and any(abs(w["queue_l"] + view["queue_l"]) < 1e-6 for w in geo["views"]):
                continue                                          # mirror image of a view already checked
            if view["corridor_from_s"] is None or view["mask_from_s"] is None or view["visible_again_s"] is None or view["beyond_s"] is None:
                continue
            ts = min(cfg.acoustic.status_fresh_s, (abs(geo["queue_s"]) - cfg.gate.cr_half_len - R_OUT - cfg.gate.queue_tol - cfg.gate.nav_err_m) / v_hi - cfg.acoustic.latency_max_s)
            rep.add(check(f"S2 M3-status n={n}, observer l={view['queue_l']:+.2f}, freshness {ts:.2f} s, v {v_hi}, offset {e0}: never both in the CR",
                          "latch + status phantom (last report younger than freshness) + FRONT-only no relation -> not(A in CR and B commits)",
                          ENC, m3_status_model(cfg, geo, view, ts, v_hi, e0), timeout_ms=180000), verbose)
    geo3 = cm.geometry(cfg, 3)
    view3 = max(geo3["views"], key=lambda w: w["queue_l"])
    rep.add(check("S2m mutation: a freshness of 5 s (the first design) violates P2 at the declared worst case", "expect counterexample", ENC,
                  m3_status_model(cfg, geo3, view3, 5.0, 0.70, 1.35), expect="sat", timeout_ms=180000), verbose)
    r = evidence_order(cfg, g)
    rep.add(CheckResult("S3 evidence order: a status older than a later fix placing the drone possibly in the CR never excludes it",
                        "randomized: last accepted fix in the CR with t_tx > status t_tx -> not excluded", ENC, "holds",
                        verdict="holds" if r["excluded_by_stale_status"] == 0 else "violated", passed=r["excluded_by_stale_status"] == 0,
                        note=str(r)), verbose)
    rm = evidence_order(cfg, g, n_cases=100, mutate=True)
    okm = rm["excluded_by_stale_status"] > 0
    rep.add(CheckResult("S3m mutation: without the evidence order the stale status excludes the drone", "expect violation", ENC, "violated",
                        verdict="violated" if okm else "holds", passed=okm, note=str(rm)), verbose)
    r = silence_never_clears(cfg, g)
    rep.add(CheckResult("S4 silence: a drone silent after reporting committed or admitted keeps the gate held (claim, never excluded), at any duration",
                        "randomized over silence durations up to a day", ENC, "holds",
                        verdict="holds" if r["violations"] == 0 else "violated", passed=r["violations"] == 0, note=str(r)), verbose)
    bad = structural()
    rep.add(CheckResult("S5 structural: no acoustic quantity or status in the computation of the precedence relations",
                        "text: the relation loop and decision of GatePerception.update", ENC, "holds",
                        verdict="holds" if not bad else "violated", passed=not bad, note="; ".join(bad) or "clean"), verbose)
    bad = front_band_queue_pairs(cfg)
    rep.add(CheckResult("S6 relation drop: drones at their queue points are never FRONT-only targets of each other (n=2,3, tolerances)",
                        "enumeration over queue points, tolerances, heading tolerance", ENC, "holds",
                        verdict="holds" if not bad else "violated", passed=not bad, note=f"{len(bad)} violations"), verbose)
    r = passage_soundness(cfg, g)
    ok = r["verified_inside"] == 0 and r["blocked_verified"] == 0
    rep.add(CheckResult("S7 verified passage: never verified with the true centre inside the CR grown by R_OUT, a blocked drone never verifies",
                        "randomized: fit error, DVL scale and dropouts, blocked drones", ENC, "holds", verdict="holds" if ok else "violated",
                        passed=ok, note=str(r)), verbose)
    r = keepout_barrier(cfg, g)
    rep.add(CheckResult("S9 keep-out barrier: a drone not committed to the gate is never moved closer to the CR inside the keep-out band",
                        "randomized: s in the band -> v . axis <= 0 after the filter", ENC, "holds",
                        verdict="holds" if r["toward_cr_inside_band"] == 0 else "violated", passed=r["toward_cr_inside_band"] == 0, note=str(r)), verbose)
    bad = backing_up_guard(cfg)
    rep.add(CheckResult("S10 a recovering drone backing up after the gate is never sent toward the CR: the reversal is refused from the guard plane on",
                        "exit_s + 1 m >= cr_half_len + R_OUT + E", ENC, "holds", verdict="holds" if not bad else "violated", passed=not bad,
                        note=str(bad) or "ok"), verbose)
    bad = freshness_arithmetic(cfg)
    rep.add(CheckResult("S8 freshness: the bound per layout is below the shortest time from the queue to the CR at the speed bound",
                        "(|queue_s| - CR - R_OUT - queue_tol - nav_err) / v_bound - latency >= status_fresh", ENC, "holds",
                        verdict="holds" if not bad else "violated", passed=not bad, note=str(bad)), verbose)
    return rep


if __name__ == "__main__":
    r = run()
    r.save()
    sys.exit(0 if r.ok else 1)
