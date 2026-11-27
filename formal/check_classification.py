"""Echo classification at the corrected pose and the gate beacon (perception/perception.py, perception/gate_beacon.py,
perception/structure_aiding.py, perception/sonar_processing.py).

The six sonars give a range per cone, no bearing: an echo is STRUCTURE when it lies in the range window of the gate map seen
from a pose.  At the estimated pose a lateral error of 0.72 m (v39 B0 gate MEDIUM, within nav_err_m) put the bars ahead outside
the window: the drone took them for an obstacle at the CR entry and never crossed.  The classifier compares the echoes with the
map at a corrected pose - the gate-beacon fix (preferred) or a structure fit the aiding would accept (safe or identified) -
and widens the cone edge and the range of the window by the error bound of that pose.  A sensor net for a side bar is
not possible - the side sonar is blind below range_min (0.30 m); the hull clears the bars by the structure keepout of
the estimate, the estimate corrected by the beacon (laterally only once admitted; v41: a 0.42 m lateral error, a contact).

CL1 (numeric, deployed EchoClassifier, G06 bars, every sector): bar echoes of the true pose (the nearest bar point in the true
cone, a direct profile model independent of the window function) are STRUCTURE at a classification pose in error by E with
the window widened by E, for E = 0.15, 0.3, 0.5 m (fit) - the beacon bounds are covered by CL4.
CL1m (mutation): at the uncorrected estimate with the v39 lateral error (0.72 m) bar echoes are not all explained.
CL2 (numeric): an echo off the gate map (outside the window of the true pose widened by the largest E, plus the tolerances) is
never STRUCTURE.  CL2r (reported): how far the widening extends the window beyond the unwidened one (what more it can mask).
CL4 (numeric): the bar echoes are STRUCTURE at a beacon pose in error by its deployed bound (median of min_fixes fixes) in 8
directions, wherever that bound is below max_bound_m.
CL5 (numeric, Monte Carlo of the model): the median fix is within its bound with an onboard heading biased up to
heading_bound_deg (the onboard R differs from the true one).
CL6 (numeric): the beacon bound in the gate plane is below the lateral tolerance of the structure keepout.
CL3 (reported): the acceptance of a classification pose.
The masking premises of the P1 / P2 checks (formal M4, stage-2, admit_clear) compute the window at the true pose; the
deployed window is at the true pose within the bound E of the classification pose (before: within the navigation error).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common import CheckResult, Report  # noqa: E402
from holo_fleet.config import DEFAULT, FleetConfig  # noqa: E402


def _setup(cfg):
    from holo_fleet.perception import sonar_geometry as sg
    from holo_fleet.perception.sonar_processing import EchoClassifier
    from holo_fleet.sim.scenarios import SCENARIOS

    plan = SCENARIOS["line_parallel_mutex"](cfg).plans[0]
    g = plan.gates[0]
    bars = [b for st in plan.structures for b in st.bars]
    yaw = float(np.arctan2(g.axis[1], g.axis[0]))
    return sg, EchoClassifier, bars, g, yaw


def _R(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _bar_profile(clf, sg, p_true, R, extra=None):
    """Direct model: the nearest bar surface point inside the true cone returns at its range (not the window function)."""
    rng = clf.model.ranges()
    half = math.radians(clf.model.half_angle_deg)
    out = {}
    for sec in sg.SECTORS:
        prof = np.zeros_like(rng)
        a = R @ sg.AXES[sec]
        spos = p_true + R @ sg.MOUNTS[sec]
        d = clf.struct_pts - spos
        r = np.linalg.norm(d, axis=1)
        ang = np.arccos(np.clip((d @ a) / np.maximum(r, 1e-9), -1.0, 1.0))
        sel = (ang <= half) & (r >= clf.model.range_min) & (r <= clf.model.range_max)
        if sel.any():
            r0 = float(r[sel].min())
            prof[(rng >= r0) & (rng <= r0 + 0.15)] = 0.9
        if extra and sec in extra:
            prof[(rng >= extra[sec]) & (rng <= extra[sec] + 0.15)] = 0.9
        out[sec] = prof
    return out


def _classify(EchoClassifier, bars, cfg, prof, pose, R, pose_err):
    clf = EchoClassifier(gate_bars=bars, cfg=cfg)
    ages = {k: 0.0 for k in prof}
    out = None
    for _ in range(3):
        out = clf.classify(prof, ages, pose, R, None, pose_err_m=pose_err)
    return out


def _non_structure(out):
    from holo_fleet.perception.sonar_processing import STRUCTURE

    return [(s, round(x.r0, 2)) for s, rd in out.items() for x in rd.echoes if x.cls != STRUCTURE]


def _dirs(n):
    return [np.array([math.cos(2 * math.pi * k / n), math.sin(2 * math.pi * k / n), 0.0]) for k in range(n)]


def run(cfg: FleetConfig = DEFAULT, verbose: bool = True) -> Report:
    from holo_fleet.perception.gate_beacon import GateBeaconFix, beacon_position, fix_bound
    from holo_fleet.perception.sonar_processing import STRUCTURE
    from holo_fleet.perception.structure_aiding import CLS_HOLD_S, CLS_MIN_MATCHED, FIT_RMS_M, UNIQUE_M
    from holo_fleet.sim.gate_beacon import GateBeaconSim

    rep = Report("classification")
    sg, EchoClassifier, bars, g, yaw = _setup(cfg)
    R = _R(yaw)
    base = EchoClassifier(gate_bars=bars, cfg=cfg)
    gb = cfg.gate_beacon
    bp = beacon_position(g, cfg)
    poses = [(s, l, dz) for s in (-6.0, -4.5, -3.0, -2.55, -2.0, -1.5, -1.2, -0.9) for l in (-0.35, 0.0, 0.35)
             for dz in (-0.8, 0.0, 0.8)]

    bad, n = [], 0
    for q in poses:
        truth = np.asarray(g.from_gate_frame(list(q)), float)
        prof = _bar_profile(base, sg, truth, R)
        for E in (0.15, 0.3, 0.5):
            for u in _dirs(4):
                n += 1
                nb = _non_structure(_classify(EchoClassifier, bars, cfg, prof, truth + R @ (E * u), R, E))
                if nb:
                    bad.append((q, E, u[:2].round(2).tolist(), nb[:2]))
    r = CheckResult(prop=f"CL1 bar echoes of the true pose are STRUCTURE at a classification pose in error by E with the window "
                         f"widened by E (E = 0.15, 0.3, 0.5 m; {n} cases near G06, every sector, direct bar model)",
                    formula="for every sampled pose, E and direction: every bar echo classified STRUCTURE",
                    encoding="numeric over the deployed classifier and the arena gate map", expect="holds")
    r.verdict, r.passed = ("holds" if not bad else "violated"), not bad
    r.note = f"{len(bad)} failing: {bad[:3]}" if bad else ""
    rep.add(r, verbose)

    miss = 0
    for q in poses:
        truth = np.asarray(g.from_gate_frame(list(q)), float)
        prof = _bar_profile(base, sg, truth, R)
        miss += bool(_non_structure(_classify(EchoClassifier, bars, cfg, prof, truth + R @ np.array([0.0, 0.72, 0.0]), R, 0.0)))
    r = CheckResult(prop="CL1m mutation: at the uncorrected estimate with the v39 lateral error (0.72 m) bar echoes are "
                         f"unexplained (an obstacle at the CR entry) in {miss} of {len(poses)} poses",
                    formula="expect some pose with a non-STRUCTURE bar echo", encoding="numeric", expect="caught")
    r.verdict, r.passed = ("caught" if miss else "missed"), miss > 0
    rep.add(r, verbose)

    pc = cfg.perc
    e_max = max(0.5, gb.max_bound_m)
    bad2, n2, ext = [], 0, 0.0
    for q in poses:
        truth = np.asarray(g.from_gate_frame(list(q)), float)
        for sec in sg.SECTORS:
            a = R @ sg.AXES[sec]
            spos = truth + R @ sg.MOUNTS[sec]
            w0 = base.structure_window(spos, a, 0.0)
            w = base.structure_window(spos, a, e_max)
            if w0 is not None and w is not None:
                ext = max(ext, w0[0] - (w[0] - e_max), (w[1] + e_max) - w0[1])
            for r_e in (0.6, 1.0, 1.5, 2.0, 3.0, 4.0):
                if w is not None and (w[0] - pc.structure_tol_m - e_max - 0.2) <= r_e <= (w[1] + pc.structure_tol_far_m + e_max + 0.2):
                    continue
                n2 += 1
                prof = _bar_profile(base, sg, truth, R, extra={sec: r_e})
                out = _classify(EchoClassifier, bars, cfg, prof, truth, R, e_max)
                if any(abs(x.r0 - r_e) < 0.08 and x.cls == STRUCTURE for x in out[sec].echoes):
                    bad2.append((q, sec, r_e))
    r = CheckResult(prop=f"CL2 an echo off the gate map (outside the window of the true pose widened by {e_max} m, plus the "
                         f"tolerances) is never STRUCTURE ({n2} cases, every sector)",
                    formula="echo range outside [w0 - tol, w1 + tol_far] -> not STRUCTURE",
                    encoding="numeric over the deployed classifier", expect="holds")
    r.verdict, r.passed = ("holds" if not bad2 else "violated"), not bad2
    r.note = "" if not bad2 else str(bad2[:3])
    rep.add(r, verbose)
    r = CheckResult(prop=f"CL2r reported: the cone-edge widening by {e_max} m extends the structure window by at most {ext:.2f} m "
                         "beyond the unwidened one at the sampled poses (the range where a hull can be masked grows by it)",
                    formula="reported", encoding="numeric", expect="holds")
    r.verdict, r.passed = "holds", True
    rep.add(r, verbose)

    bad4, n4, used = [], 0, 0
    for q in poses:
        truth = np.asarray(g.from_gate_frame(list(q)), float)
        e_b = fix_bound(cfg, float(np.linalg.norm(bp - truth)), gb.min_fixes)
        if e_b > gb.max_bound_m:
            continue
        used += 1
        prof = _bar_profile(base, sg, truth, R)
        for u in _dirs(8):
            n4 += 1
            nb = _non_structure(_classify(EchoClassifier, bars, cfg, prof, truth + R @ (e_b * u), R, e_b))
            if nb:
                bad4.append((q, round(e_b, 2), u[:2].round(2).tolist(), nb[:2]))
    r = CheckResult(prop=f"CL4 at a beacon pose in error by its deployed bound (median of {gb.min_fixes} fixes, 8 directions) the "
                         f"bar echoes are STRUCTURE ({n4} cases at the {used} poses where the bound is <= {gb.max_bound_m} m)",
                    formula="classification pose error = deployed bound -> bar echoes STRUCTURE",
                    encoding="numeric over the deployed classifier and fix_bound", expect="holds")
    r.verdict, r.passed = ("holds" if not bad4 and n4 > 0 else "violated"), not bad4 and n4 > 0
    r.note = f"{len(bad4)} failing: {bad4[:3]}" if bad4 else ""
    rep.add(r, verbose)

    over, n5 = 0, 0
    rng = np.random.default_rng(5)
    for k, q in enumerate([(s_, l_, 0.0) for s_ in (-3.0, -2.55, -1.5, -0.9) for l_ in (-0.35, 0.35)]):
        truth = np.asarray(g.from_gate_frame(list(q)), float)
        for seed in range(40):
            sim, fix = GateBeaconSim(["d"], [g], cfg, seed=1000 * k + seed), GateBeaconFix(cfg)
            bias = math.radians(rng.uniform(-gb.heading_bound_deg, gb.heading_bound_deg))
            R_on = _R(yaw + bias)
            est = truth + R @ np.array([rng.uniform(-0.7, 0.7), rng.uniform(-0.7, 0.7), 0.0])
            t = 0.0
            while t < gb.hold_s:
                for _gid, _tm, rr, az, el in sim.measure(t, {"d": (truth, yaw)})["d"]:
                    fix.add(t, g, rr, az, el, est, R_on)
                t += 0.1
            res = fix.offset(t - 0.1)
            if res is None or res[1] > gb.max_bound_m:
                continue
            n5 += 1
            over += int(np.linalg.norm((est + res[0] - truth)[:2]) > res[1])
    r = CheckResult(prop=f"CL5 the median beacon fix is within its bound in {n5 - over} of {n5} simulated windows (onboard heading "
                         f"biased up to {gb.heading_bound_deg} deg, drift up to 0.7 m, sigma levels of GateBeaconConfig)",
                    formula="|fix - truth| <= bound for every sampled window", encoding="numeric (Monte Carlo of the model)",
                    expect="holds")
    r.verdict, r.passed = ("holds" if over == 0 and n5 > 0 else "violated"), over == 0 and n5 > 0
    rep.add(r, verbose)

    # CL6: the hull clears the side bars by the structure keepout of the estimate (control/flows.py structure_filter):
    # a lateral error below keepout - hull half width keeps it off the bars; the beacon bound in the gate plane (where
    # the admitted drone is corrected laterally) must stay below that tolerance.  The side sonar is blind below
    # range_min, so no sensor net can replace it.
    tol = float(cfg.env.struct_keepout) - float(sg.HULL_HALF[1])
    worst6 = 0.0
    for s_ in (-0.8, -0.4, 0.0, 0.3):
        for l_ in (-0.3, 0.0, 0.3):
            for dz_ in (-0.8, 0.0, 0.8):
                truth = np.asarray(g.from_gate_frame([s_, l_, dz_]), float)
                worst6 = max(worst6, fix_bound(cfg, float(np.linalg.norm(bp - truth)), gb.min_fixes))
    ok6 = worst6 < tol
    r = CheckResult(prop=f"CL6 in the gate plane the beacon bound ({worst6:.2f} m, median of {gb.min_fixes} fixes) is below the "
                         f"lateral tolerance of the structure keepout ({tol:.2f} m = struct_keepout - hull half width); the side "
                         f"sonar is blind below {base.model.range_min} m",
                    formula="max bound over the gate plane < struct_keepout - hull half width",
                    encoding="numeric over fix_bound and the config", expect="holds")
    r.verdict, r.passed = ("holds" if ok6 else "violated"), ok6
    rep.add(r, verbose)

    r = CheckResult(prop=f"CL3 reported: the classification pose is the beacon fix (bound <= {gb.max_bound_m} m, >= {gb.min_fixes} "
                         f"fixes in {gb.hold_s} s; it corrects the estimate before admission, laterally only once admitted) or a structure fit the aiding "
                         f"accepts (>= {CLS_MIN_MATCHED} sectors, RMS <= {FIT_RMS_M} m, uncertainty <= "
                         f"{UNIQUE_M + FIT_RMS_M:.2f} m, safe or identified, held {CLS_HOLD_S} s); the cone edge is widened by "
                         "its bound", formula="reported", encoding="config", expect="holds")
    r.verdict, r.passed = "holds", True
    rep.add(r, verbose)
    return rep


if __name__ == "__main__":
    rep = run()
    rep.save()
    sys.exit(0 if rep.ok else 1)
