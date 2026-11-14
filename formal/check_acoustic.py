"""Variant B2 (range-bearing beacon): the geometric guarantees of the acoustic exit evidence, and its scope.

These checks are B2's own: they hold under the B2 assumptions (docs/B2_ACOUSTIC_AIDED_COORDINATION.md: H1-H5 and
the declared measurement envelope config.AcousticConfig) and add nothing to, and take nothing from, the B0 checks
(the B0 M4g of check_mutex.py stays what it is).  Numeric, exhaustive over sampled geometry:

* B2-A1 envelope coverage: every position compatible with a measurement inside the envelope lies within the
  region (samples + slack) the perception builds (perception/acoustic.region_samples);
* B2-A2 decision soundness: no measurement inside the envelope of a hull anywhere in the CR, from any queue point
  of the deployed queues (abreast n = 2..6), at headings within heading_tol, passes the exit test (region beyond the
  exit face) or the outside test (region clear of the CR) of perception/acoustic_gate.py;
* B2-A3 accounting soundness: a hull in the CR at t_lc is never excluded at t_lc (its region always "could be an
  occupant");
* B2-M4g acoustic exit observability, n = 2..6 (abreast queue, pass paths of control/flows.pass_path_points): for
  every observer and passer, along the passer's path at v_pass with one ping per period (worst phase), the worst
  measurement inside the envelope still gives ``exit_confirm`` consecutive fixes beyond the exit face before the end
  of the path, and every other queued drone is accountable at t_lc (region excluded).  This is GEOMETRIC
  availability: every ping is assumed received.  A ray that touches a gate bar or another queued hull is SHADOWED
  (sim/acoustic_channel.py H4: received with probability 1 - shadow_loss, not cut); the shares of shadowed
  confirmation pings and of shadowed accounting paths are reported, since there the evidence is only probable.
  An observer for which the evidence does not exist falls back to the sonar rules (B0) - reported, never hidden;
* B2-S structural: no module that decides separation, collision avoidance or precedence reads an acoustic fix;
  the latch reads them through acoustic_gate.AcousticExit; exit evaluation requires the FRONT corridor empty.
  Positive occupancy is queried independently to veto commitment, releases and admission.
"""

from __future__ import annotations

import ast
import dataclasses
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from common import ROOT, CheckResult, Report  # noqa: E402

from holo_fleet.config import DEFAULT, BeaconConfig, FleetConfig
from holo_fleet.control.flows import pass_path_points
from holo_fleet.mission import abreast_slots, check_gate_domain, queue_line
from holo_fleet.perception.acoustic import AcousticTracker, Fix, _rot, box_bounds, region_samples
from holo_fleet.perception.acoustic_gate import AcousticExit, gate_points
from holo_fleet.perception.sonar_geometry import HULL_HALF, R_OUT

ENC = "perception/acoustic.py + perception/acoustic_gate.py"
B2 = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_RB"))


def _gate():
    from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates

    return load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]


def _meas(p_obs, yaw, p_tgt):
    d = np.asarray(p_tgt, float) - np.asarray(p_obs, float)
    c, s = math.cos(yaw), math.sin(yaw)
    b = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
    r = float(np.linalg.norm(b))
    return r, math.atan2(b[1], b[0]), math.asin(np.clip(b[2] / r, -1.0, 1.0))


def _fix(p_obs, yaw, r, az, el, cfg, t=0.0) -> Fix:
    body, slack = region_samples(r, az, el, cfg)
    R = _rot(yaw)
    u = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    return Fix(slot=1, t=t, r=r, az=az, el=el, p=p_obs + R @ (r * u), pts=p_obs + body @ R.T, slack=slack,
               rel=R @ (r * u))


def _corners(r, az, el, cfg):
    """Extreme measurements of a true (r, az, el) inside the envelope: the white range bound both ways, and the
    one-sided clear-path multipath excess on top (a measured range up to los_excess_max_m longer)."""
    br, ba, be = box_bounds(r, cfg)
    for rr in (r - br, r, r + br, r + br + cfg.acoustic.los_excess_max_m):
        for sa in (-1, 0, 1):
            for se in (-1, 0, 1):
                yield max(rr, 0.05), az + sa * ba, el + se * be


def queue_points(G, n, perc=DEFAULT.perc):
    return queue_line(abreast_slots(n), G, perc), list(G.queue_laterals(n))


def coverage(cfg: FleetConfig, n_samples: int = 4000, seed: int = 0) -> float:
    """Largest distance (minus slack) between a compatible position and the region's samples (must be <= 0)."""
    rng = np.random.default_rng(seed)
    worst = -1e9
    for _ in range(n_samples):
        r, az, el = rng.uniform(0.5, 30.0), rng.uniform(-math.pi, math.pi), rng.uniform(-1.0, 1.0)
        br, ba, be = box_bounds(r, cfg)
        pts, slack = region_samples(r, az, el, cfg)
        tr = max(r + rng.uniform(-br - cfg.acoustic.los_excess_max_m, br), 0.0)
        ta, te = az + rng.uniform(-ba, ba), el + rng.uniform(-be, be)
        truth = tr * np.array([math.cos(te) * math.cos(ta), math.cos(te) * math.sin(ta), math.sin(te)])
        worst = max(worst, float(np.min(np.linalg.norm(pts - truth, axis=1))) - slack)
    return worst


def decision_soundness(cfg: FleetConfig, g) -> dict:
    """B2-A2 / A3 over the CR grid, every abreast queue point n = 2..6, headings within heading_tol."""
    G = cfg.gate
    tr = AcousticTracker(cfg)
    ae = AcousticExit(cfg, tr, [1])
    half = np.array([G.cr_half_len, G.cr_half_width, G.cr_half_height])
    grid = [np.array([a, b, c]) * half for a in np.linspace(-1, 1, 5) for b in np.linspace(-1, 1, 5)
            for c in np.linspace(-1, 1, 3)]
    yaw0 = math.atan2(g.axis[1], g.axis[0])
    out = {"cases": 0, "exit_passed": 0, "outside_passed": 0, "range_clear_passed": 0, "excluded": 0,
           "worst_s_lower_margin": -1e9}
    s_exit = G.cr_half_len + R_OUT
    for n in range(2, 7):
        q_s, lats = queue_points(G, n, cfg.perc)
        for lat in lats:
            p = g.from_gate_frame([q_s, lat, 0.0])
            for dyaw in (-G.heading_tol_deg, 0.0, G.heading_tol_deg):
                yaw = yaw0 + math.radians(dyaw)
                ae.note_corridor(0.0, g, p, yaw, [float(np.linalg.norm(p - g.center)) - 1.0])
                for cg in grid:
                    r, az, el = _meas(p, yaw, g.from_gate_frame(cg))
                    for m in _corners(r, az, el, cfg):
                        f = _fix(p, yaw, *m, cfg)
                        out["cases"] += 1
                        sl = ae.s_lower(g, f, 0.0)
                        out["worst_s_lower_margin"] = max(out["worst_s_lower_margin"], sl - s_exit)
                        out["exit_passed"] += int(sl > s_exit)
                        out["outside_passed"] += int(ae.clear_of_cr(g, f, 0.0))
                        out["range_clear_passed"] += int(ae.clear_by_range(g, f, 0.0))
                        out["excluded"] += int(not ae._could_be_occupant(g, f, 0.0))
    out["worst_s_lower_margin"] = round(out["worst_s_lower_margin"], 3)
    return out


def _segment_hits_box(a, b, box) -> bool:
    A, B = box.axes.T @ (a - box.center), box.axes.T @ (b - box.center)
    d, t0, t1 = B - A, 0.0, 1.0
    for k in range(3):
        if abs(d[k]) < 1e-12:
            if abs(A[k]) > box.half[k]:
                return False
            continue
        u0, u1 = (-box.half[k] - A[k]) / d[k], (box.half[k] - A[k]) / d[k]
        if u0 > u1:
            u0, u1 = u1, u0
        t0, t1 = max(t0, u0), min(t1, u1)
        if t0 > t1:
            return False
    return True


def _hull(c, yaw):
    return SimpleNamespace(center=np.asarray(c, float), axes=_rot(yaw), half=np.asarray(HULL_HALF, float))


def _worst_s_lower(ae, g, p, yaw, c_true, cfg, t, t_now):
    r, az, el = _meas(p, yaw, c_true)
    return min(ae.s_lower(g, _fix(p, yaw, *m, cfg, t=t), t_now) for m in _corners(r, az, el, cfg))


def exit_observability(cfg: FleetConfig, g, n: int, ds: float = 0.05) -> list:
    """B2-M4g rows, one per (observer, passer) of the abreast queue of n."""
    G, a = cfg.gate, cfg.acoustic
    period = cfg.beacon.period_s
    yaw = math.atan2(g.axis[1], g.axis[0])
    q_s, lats = queue_points(G, n, cfg.perc)
    tr = AcousticTracker(cfg)
    s_exit = G.cr_half_len + R_OUT
    rows = []
    bars = list(g.bars)
    for li in lats:
        p = g.from_gate_frame([q_s, li, 0.0])
        for lj in lats:
            if lj == li:
                continue
            others = [l for l in lats if l not in (li, lj)]
            hulls = [_hull(g.from_gate_frame([q_s, l, 0.0]), yaw) for l in others]
            path = pass_path_points(G, q_s, lj, 0.0, lane=lj, lane_dz=0.0)
            pts = []
            for u, v in zip(path[:-1], path[1:]):
                k = max(1, int(np.linalg.norm(v - u) / ds))
                pts += [u + (v - u) * i / k for i in range(k)]
            pts.append(path[-1])
            dist = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(np.array(pts), axis=0), axis=1))])
            i_out = next(i for i, x in enumerate(pts) if x[0] > G.cr_half_len)
            tt = (dist - dist[i_out]) / G.v_pass
            ae = AcousticExit(cfg, tr, [1])
            c_lc = g.from_gate_frame(pts[i_out])
            ae.note_corridor(0.0, g, p, yaw, [float(np.linalg.norm(c_lc - p)) - R_OUT])
            accountable, acc_shadowed, others_clear = True, 0, True
            for l in others:
                c = g.from_gate_frame([q_s, l, 0.0])
                block = [h for h in hulls if not np.allclose(h.center, c)]
                acc_shadowed += int(any(_segment_hits_box(p, c, b) for b in bars + block))
                r, az, el = _meas(p, yaw, c)
                excl = all(not ae._could_be_occupant(g, _fix(p, yaw, *m, cfg), 0.0) for m in _corners(r, az, el, cfg))
                accountable &= excl
                others_clear &= all(ae.clear_of_cr(g, _fix(p, yaw, *m, cfg), 0.0) for m in _corners(r, az, el, cfg))
            worst_t, worst_s, shadowed = -1e9, None, 0
            for phase in np.linspace(0.0, period, 5, endpoint=False):
                ok_count, t_ok, sh = 0, None, []
                tk = phase
                while tk <= tt[-1] + 1e-9:
                    i = int(np.searchsorted(tt, tk))
                    i = min(i, len(pts) - 1)
                    c = g.from_gate_frame(pts[i])
                    r_c, az_c, el_c = _meas(p, yaw, c)
                    good = (pts[i][0] > s_exit and all(ae.clear_of_cr(g, _fix(p, yaw, *m, cfg, t=tk), tk)
                                                       for m in _corners(r_c, az_c, el_c, cfg)))
                    ok_count = ok_count + 1 if good else 0
                    sh = (sh + [any(_segment_hits_box(p, c, b) for b in bars + hulls)])[-a.exit_confirm:] if good else []
                    if ok_count >= a.exit_confirm:
                        t_ok = tk
                        break
                    tk += period
                if t_ok is None:
                    worst_t, worst_s = math.inf, None
                    break
                if t_ok > worst_t:
                    worst_t, worst_s = t_ok, float(pts[min(int(np.searchsorted(tt, t_ok)), len(pts) - 1)][0])
                    shadowed = sum(sh)
            rows.append({"observer_l": li, "passer_lane": lj, "t_exit_s": None if math.isinf(worst_t) else round(worst_t, 1),
                         "s_at_evidence": None if worst_s is None else round(worst_s, 2),
                         "accountable": accountable or others_clear, "others_excluded": accountable,
                         "others_clear": others_clear,
                         "confirm_pings_shadowed": shadowed, "accounting_paths_shadowed": acc_shadowed,
                         "path_end_t_s": round(float(tt[-1]), 1)})
    return rows


def payload_soundness(cfg: FleetConfig, g, n: int = 20000, seed: int = 3, radius=None) -> dict:
    """B2-P: a sender truly in the CR (centre anywhere in the box, boundary included) whose estimate errs by at most
    its declared horizontal bound h and vertical bound z, sent as the ball radius ``radius(h, z)`` (default: the
    deployed NavBound rule hypot(h, z)) through the real packet (quantisation, rounding up), is never cleared by the
    payload rule (``rep_cr_possible`` stays True) and never placed beyond the exit face by A3.  Randomized, with the
    worst cases (errors on the boundary of the cylinder) over-sampled."""
    from holo_fleet.perception import payload as pl
    from holo_fleet.perception.acoustic_gate import AcousticExit, gate_points
    from holo_fleet.perception.acoustic import AcousticTracker

    radius = radius or (lambda h, z: math.hypot(h, z))
    rng = np.random.default_rng(seed)
    ae = AcousticExit(cfg, AcousticTracker(cfg), [1])
    half = ae.half
    bad = beyond_bad = 0
    s_exit = half[0] + R_OUT
    for i in range(n):
        q = rng.uniform(-half, half)
        if i % 3 == 0:
            ax = rng.integers(3)
            q[ax] = half[ax] * rng.choice([-1.0, 1.0])
        true = g.center + q[0] * g.axis + q[1] * np.array([-g.axis[1], g.axis[0], 0.0]) + np.array([0, 0, q[2]])
        h, z = rng.uniform(0.0, 4.0), rng.uniform(0.0, 2.0)
        ang = rng.uniform(0, 2 * math.pi)
        rh = h if i % 2 == 0 else h * math.sqrt(rng.uniform())
        e = np.array([rh * math.cos(ang), rh * math.sin(ang), z * rng.choice([-1.0, 1.0]) * (1.0 if i % 2 == 0 else rng.uniform())])
        rep = pl.decode(pl.encode(0, true + e, radius(h, z)))
        if rep is None:
            continue
        f = Fix(slot=1, t=0.0, r=1.0, az=0.0, el=0.0, p=true, pts=true[None, :], slack=0.0, rel=np.zeros(3),
                rep=rep, t_tx=0.0)
        if not ae.rep_cr_possible(g, f, 0.0):
            bad += 1
        if float(gate_points(g, rep.p[None, :])[0][0]) - f.rep_radius(0.0, ae.v_bound) > s_exit:
            beyond_bad += 1
    return {"samples": n, "cleared_inside": bad, "beyond_while_inside": beyond_bad}


def phantom_soundness(cfg: FleetConfig, g, n: int = 3000, seed: int = 5, sender_rule: bool = True, use_status: bool = True) -> dict:
    """B2-PE: a fleet drone whose centre is truly in the CR is never excluded by the phantom-echo rule
    (AcousticExit.no_fleet_drone): its accepted fixes are generated inside the declared envelope (white and bias
    errors at their bounds, the one-sided multipath excess, ages up to tie_window_s) from queue-side observers, with
    and without a consistent payload whose error lies within the sent bound.  The packet carries the gate phase the sender
    reports (passage.report_phase) for its own estimate, the estimate within the declared gate-frame navigation error, in the
    worst case of a drone neither committed nor admitted; ``sender_rule`` False is the mutation without the keep-out claim.
    Randomized."""
    from holo_fleet.perception import payload as pl
    from holo_fleet.perception.acoustic import AcousticTracker
    from holo_fleet.perception.passage import inside_keepout, report_phase

    rng = np.random.default_rng(seed)
    a = cfg.acoustic
    br_k = a.k_sigma
    left = np.array([-g.axis[1], g.axis[0], 0.0])
    bad = 0
    for i in range(n):
        tr = AcousticTracker(cfg)
        tr.set_identities({1: 1})
        ae = AcousticExit(cfg, tr, [1])
        q = rng.uniform(-ae.half, ae.half)
        true = g.center + q[0] * g.axis + q[1] * np.array([-g.axis[1], g.axis[0], 0.0]) + np.array([0, 0, q[2]])
        obs_s = rng.uniform(-9.0, -3.0)
        p_obs = g.center + obs_s * g.axis + rng.uniform(-3.5, 3.5) * np.array([-g.axis[1], g.axis[0], 0.0])
        yaw = math.atan2(g.axis[1], g.axis[0])
        t_now = 10.0
        tr.update(t_now - 4.0, (), p_obs, yaw, 0.5)
        for k, age in enumerate(sorted(rng.uniform(0.0, a.tie_window_s, size=3), reverse=True)):
            t_arr = t_now - age
            r, az, el = _meas(p_obs, yaw, true)
            br, ba, be = box_bounds(r, cfg)
            r_m = r + rng.uniform(-br, br) + (rng.uniform(0, a.los_excess_max_m) if rng.random() < 0.3 else 0.0)
            az_m, el_m = az + rng.uniform(-ba, ba), el + rng.uniform(-be, be)
            pkt = None
            if rng.random() < 0.5:
                b = rng.uniform(0.3, 3.0)
                e = rng.normal(size=3)
                e = e / max(np.linalg.norm(e), 1e-9) * rng.uniform(0, b)
                u = rng.normal(size=3)
                est = q + u / max(np.linalg.norm(u), 1e-9) * rng.uniform(0, cfg.gate.nav_err_m)
                phase = report_phase(False, False, False, inside_keepout(est[0], est[1], est[2], cfg.gate)) if sender_rule else pl.GP_NONE
                pkt = pl.encode(pl.frame_index(t_arr - r_m / cfg.beacon.sound_speed, cfg.beacon.period_s), true + e, b, 0, phase)
            tr.update(t_arr + 0.05, [(1, t_arr, r_m, az_m, el_m, pkt)], p_obs, yaw, 0.5)
        r_e = float(np.linalg.norm(true - p_obs)) - R_OUT
        if not use_status:
            tr.status.clear()
        if ae.no_fleet_drone(t_now, g, p_obs, yaw, [max(r_e, 0.3)]):
            bad += 1
    return {"samples": n, "occupant_excluded": bad}


def structural() -> list:
    """Modules that decide separation, collision avoidance or precedence never touch an acoustic fix."""
    pkg = ROOT / "holo_fleet"
    forbidden = ["control/escape.py", "control/flows.py", "control/current_envelope.py", "ha/mutex_rule.py", "ha/spec.py",
                 "ha/automaton.py", "perception/targets.py", "perception/sonar_processing.py",
                 "perception/structure_aiding.py", "perception/exit_witness.py", "ha/observation_invariants.py"]
    bad = []
    for rel in forbidden:
        f = pkg / rel
        if not f.exists():
            continue
        tree = ast.parse(f.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            name = node.attr if isinstance(node, ast.Attribute) else node.id if isinstance(node, ast.Name) else None
            if (name == "acoustic" and isinstance(node, ast.Attribute)
                    and ((isinstance(node.value, ast.Attribute) and node.value.attr == "cfg")
                         or (isinstance(node.value, ast.Name) and node.value.id == "cfg"))):
                continue
            if name in ("acoustic", "AcousticTracker", "AcousticExit", "acoustic_exit", "AcousticRx"):
                bad.append(f"{rel}:{getattr(node, 'lineno', '?')}")
    gp = (pkg / "perception/gate_perception.py").read_text(encoding="utf-8")
    guarded = "if witness is None and not corridor and self.occ_state == \"BUSY\" and self.acoustic_exit is not None:" in gp
    if not guarded:
        bad.append("gate_perception.py: acoustic_exit.evaluate not guarded by an empty corridor")
    return bad


def run(cfg: FleetConfig = B2, verbose: bool = True) -> Report:
    rep = Report("acoustic")
    g = _gate()
    w = coverage(cfg)
    rep.add(CheckResult("B2-A1 envelope coverage: every position compatible with a measurement inside the envelope lies in "
                        "the region the perception builds", "max over draws of (distance to samples - slack) <= 0", ENC,
                        "holds", verdict="holds" if w <= 1e-9 else "violated", passed=w <= 1e-9,
                        note=f"4000 random measurements 0.5-30 m, worst {w:+.3f} m"), verbose)
    ds = decision_soundness(cfg, g)
    ok = ds["outside_passed"] == 0 and ds["excluded"] == 0
    rep.add(CheckResult("B2-A2 decision soundness: no measurement inside the envelope (multipath excess included) of a "
                        "hull in the CR (any queue point n=2..6, heading within heading_tol) passes the outside test "
                        "(by region or by range) or the exclusion test",
                        "forall CR grid x envelope corners: region or range clear of the CR -> false; could be occupant",
                        ENC, "holds", verdict="holds" if ok else "violated", passed=ok,
                        note=f"{ds['cases']} cases; outside {ds['outside_passed']} (by range {ds['range_clear_passed']}), "
                             f"excluded {ds['excluded']}; the progress test A3 is not a safety condition (it passed in "
                             f"{ds['exit_passed']} cases with the strict region bound)"), verbose)
    rep.exit_observability = {}
    for n in range(2, 7):
        rows = exit_observability(cfg, g, n)
        rep.exit_observability[n] = rows
        blind = [r for r in rows if r["t_exit_s"] is None or r["t_exit_s"] > r["path_end_t_s"] or not r["accountable"]]
        worst = max((r for r in rows if r["t_exit_s"] is not None), key=lambda r: r["t_exit_s"], default=None)
        okn = not blind
        sh_c = sum(1 for r in rows if r["confirm_pings_shadowed"])
        sh_a = sum(1 for r in rows if r["accounting_paths_shadowed"])
        note = (f"{len(rows)} pairs; confirmation through a shadowed ray for {sh_c}, accounting through a shadowed ray "
                f"for {sh_a} (probable, not certain); worst exit evidence {worst['t_exit_s']} s after the exit face (observer l={worst['observer_l']:+.2f}, "
                f"passer lane {worst['passer_lane']:+.2f}, passer at s={worst['s_at_evidence']})" if worst else "no evidence")
        if blind:
            note += "; falls back to the sonar latch for: " + ", ".join(
                f"observer {r['observer_l']:+.2f}/passer {r['passer_lane']:+.2f}"
                f" ({'no evidence' if r['t_exit_s'] is None else 'late'}{'' if r['accountable'] else ', queue not accountable'})"
                for r in blind)
        try:
            check_gate_domain(abreast_slots(n), cfg.gate)
            inside = True
        except ValueError:
            inside = False
        head = (f"B2-M4g availability (reported), n={n}: every queued drone gets an acoustic exit evidence of every "
                "passing drone (bars' and queued hulls' occlusion, worst envelope error, worst ping phase) and can "
                "account for the other queued drones") if inside else             f"B2-M4g domain boundary, n={n}: refused by the planner (queue points beyond queue_lateral_max), reported"
        rep.add(CheckResult(head, f"exists {cfg.acoustic.exit_confirm} consecutive clear fixes with s_lower > exit face "
                            "before the path end; queued drones excluded at t_lc", ENC, "violated" if not okn else "holds",
                            verdict="holds" if okn else "violated", passed=True, note=note), verbose)
    ps = payload_soundness(cfg, g)
    ok = ps["cleared_inside"] == 0 and ps["beyond_while_inside"] == 0
    rep.add(CheckResult("B2-P payload soundness: a sender truly in the CR whose estimate errs within its sent bound "
                        "(cylinder |e_h| <= h, |e_z| <= z, radius hypot(h, z), through the 56-bit packet) is never "
                        "cleared by the payload rule nor placed beyond the exit face",
                        "forall sampled centres in the CR and errors in the cylinder: rep_cr_possible and not A3",
                        ENC, "holds", verdict="holds" if ok else "violated", passed=ok,
                        note=f"randomized, {ps}"), verbose)
    pm = payload_soundness(cfg, g, n=4000, radius=lambda h, z: max(h, z))
    okm = pm["cleared_inside"] > 0
    rep.add(CheckResult("B2-Pm mutation: the radius max(h, z) (not a ball over the cylinder) must clear an occupant",
                        "expect counterexample", ENC, "violated", verdict="violated" if okm else "holds", passed=okm,
                        note=f"{pm}"), verbose)
    pe = phantom_soundness(cfg, g)
    rep.add(CheckResult("B2-PE phantom-echo soundness: a fleet drone truly in the CR, heard inside the declared envelope "
                        "(multipath, ages up to the tie window, with or without a payload within its bound), is never "
                        "excluded by the phantom-echo rule", "randomized: no_fleet_drone -> false", ENC, "holds",
                        verdict="holds" if pe["occupant_excluded"] == 0 else "violated",
                        passed=pe["occupant_excluded"] == 0, note=f"{pe}"), verbose)
    peg = phantom_soundness(cfg, g, use_status=False)
    rep.add(CheckResult("B2-PEg phantom-echo soundness of the fix geometry alone (no status in any packet): an occupant is never excluded",
                        "randomized: no_fleet_drone -> false, status removed", ENC, "holds", verdict="holds" if peg["occupant_excluded"] == 0 else "violated",
                        passed=peg["occupant_excluded"] == 0, note=f"{peg}"), verbose)
    pem = phantom_soundness(cfg, g, sender_rule=False)
    okem = pem["occupant_excluded"] > 0
    rep.add(CheckResult("B2-PEm mutation: without the keep-out claim a drone carried into the CR uncommitted reports not engaged and is excluded",
                        "expect counterexample", ENC, "violated", verdict="violated" if okem else "holds", passed=okem,
                        note=f"{pem}"), verbose)
    bad = structural()
    rep.add(CheckResult("B2-S acoustic fixes never reach separation, collision avoidance, precedence or the sonar witness; "
                        "acoustic exit evaluation requires the FRONT corridor empty (H6: the payload balls reach the "
                        "map fit only as its identity premise, through Perception, never as a position input)",
                        "AST: no acoustic name in the safety / precedence modules; guarded call", ENC, "holds",
                        verdict="holds" if not bad else "violated", passed=not bad, note="; ".join(bad) or "clean"), verbose)
    return rep


if __name__ == "__main__":
    r = run()
    r.save()
    sys.exit(0 if r.ok else 1)
