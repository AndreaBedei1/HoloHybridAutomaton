"""Echo classification at the structure-fit pose (perception/perception.py, perception/structure_aiding.py).

The six sonars give a range per cone, no bearing: an echo is STRUCTURE when it lies in the range window of the gate map seen
from a pose.  At the estimated pose a lateral error of 0.72 m (v39 B0 gate MEDIUM, within nav_err_m) put the bars ahead outside
the window: the drone took them for an obstacle at the CR entry and never crossed.  The classifier now compares the echoes with
the map at the pose a structure fit explains (estimate + fit offset, CLS_MIN_MATCHED sectors within FIT_RMS_M), held CLS_HOLD_S.

CL1 (numeric, the deployed EchoClassifier on the G06 bars): from every sampled pose near the gate, the bar echoes of the true
pose are STRUCTURE at a classification pose within CL_RES_M of the truth (the fit residual).
CL1m (mutation): at the uncorrected estimate with the v39 lateral error the bar echoes are not all explained.
CL2 (numeric): the fit pose moves the comparison, not its tolerances: an echo outside the window by more than the tolerances
is never STRUCTURE (what the window can mask is unchanged; the masking stays in the P1 detection premise).
CL3 (reported): the acceptance of a fit for the classification pose.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common import CheckResult, Report  # noqa: E402
from holo_fleet.config import DEFAULT, FleetConfig  # noqa: E402

CL_RES_M = 0.15


def _setup(cfg):
    from holo_fleet.perception import sonar_geometry as sg
    from holo_fleet.perception.sonar_processing import EchoClassifier
    from holo_fleet.sim.scenarios import SCENARIOS

    plan = SCENARIOS["line_parallel_mutex"](cfg).plans[0]
    g = plan.gates[0]
    bars = [b for st in plan.structures for b in st.bars]
    yaw = float(np.arctan2(g.axis[1], g.axis[0]))
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return sg, EchoClassifier, bars, g, R


def _profiles(clf, sg, p_true, R, extra=None):
    rng = clf.model.ranges()
    out = {}
    for sec in sg.SECTORS:
        prof = np.zeros_like(rng)
        a, _ = clf._sector_frame(sec, R)
        w = clf.structure_window(p_true + R @ sg.MOUNTS[sec], a)
        if w is not None:
            prof[(rng >= w[0]) & (rng <= w[0] + 0.15)] = 0.9
        if extra and sec in extra:
            prof[(rng >= extra[sec]) & (rng <= extra[sec] + 0.15)] = 0.9
        out[sec] = prof
    return out


def _classify(EchoClassifier, bars, cfg, prof, pose, R, pose_err=None):
    from holo_fleet.perception.structure_aiding import CLS_POSE_ERR_M

    clf = EchoClassifier(gate_bars=bars, cfg=cfg)
    ages = {k: 0.0 for k in prof}
    out = None
    for _ in range(3):
        out = clf.classify(prof, ages, pose, R, None, pose_err_m=CLS_POSE_ERR_M if pose_err is None else pose_err)
    return out


def run(cfg: FleetConfig = DEFAULT, verbose: bool = True) -> Report:
    from holo_fleet.perception.sonar_processing import STRUCTURE
    from holo_fleet.perception.structure_aiding import CLS_HOLD_S, CLS_MIN_MATCHED, FIT_RMS_M, UNIQUE_M

    rep = Report("classification")
    sg, EchoClassifier, bars, g, R = _setup(cfg)
    base = EchoClassifier(gate_bars=bars, cfg=cfg)
    poses = [(s, l, dz) for s in (-6.0, -4.5, -3.0, -2.55, -2.0, -1.5, -1.2, -0.9) for l in (-0.35, 0.0, 0.35)
             for dz in (-0.8, 0.0, 0.8)]
    res = [np.array(v) for v in ((CL_RES_M, 0, 0), (-CL_RES_M, 0, 0), (0, CL_RES_M, 0), (0, -CL_RES_M, 0))]
    bad, n = [], 0
    for q in poses:
        truth = np.asarray(g.from_gate_frame(list(q)), float)
        prof = _profiles(base, sg, truth, R)
        for e in res:
            n += 1
            out = _classify(EchoClassifier, bars, cfg, prof, truth + R @ e, R)
            nb = [(sec, round(x.r0, 2)) for sec, rd in out.items() for x in rd.echoes if x.cls != STRUCTURE]
            if nb:
                bad.append((q, e.round(2).tolist(), nb[:2]))
    r = CheckResult(prop=f"CL1 the bar echoes of the true pose are STRUCTURE at a classification pose within {CL_RES_M} m of the "
                         f"truth ({n} pose / residual cases near G06, deployed EchoClassifier)",
                    formula="for every sampled pose and residual: every bar echo classified STRUCTURE",
                    encoding="numeric over the deployed classifier and the arena gate map", expect="holds")
    r.verdict, r.passed = ("holds" if not bad else "violated"), not bad
    r.note = "" if not bad else str(bad[:3])
    rep.add(r, verbose)
    miss = 0
    for q in poses:
        truth = np.asarray(g.from_gate_frame(list(q)), float)
        prof = _profiles(base, sg, truth, R)
        out = _classify(EchoClassifier, bars, cfg, prof, truth + R @ np.array([0.0, 0.72, 0.0]), R, pose_err=0.0)
        miss += any(x.cls != STRUCTURE for rd in out.values() for x in rd.echoes)
    r = CheckResult(prop="CL1m mutation: at the uncorrected estimate with the v39 lateral error (0.72 m) bar echoes are "
                         f"unexplained (obstacle at the CR entry) in {miss} of {len(poses)} poses",
                    formula="expect some pose with a non-STRUCTURE bar echo", encoding="numeric", expect="caught")
    r.verdict, r.passed = ("caught" if miss else "missed"), miss > 0
    rep.add(r, verbose)
    bad2, n2 = [], 0
    pc = cfg.perc
    for q in poses:
        truth = np.asarray(g.from_gate_frame(list(q)), float)
        for sec in ("FRONT", "REAR", "LEFT", "RIGHT"):
            a, _ = base._sector_frame(sec, R)
            w = base.structure_window(truth + R @ sg.MOUNTS[sec], a, 0.3)
            for r_e in (0.6, 1.0, 2.0, 4.0):
                if w is not None and (w[0] - pc.structure_tol_m - 0.2) <= r_e <= (w[1] + pc.structure_tol_far_m + 0.2):
                    continue
                n2 += 1
                prof = _profiles(base, sg, truth, R, extra={sec: r_e})
                out = _classify(EchoClassifier, bars, cfg, prof, truth, R, pose_err=0.3)
                if any(abs(x.r0 - r_e) < 0.08 and x.cls == STRUCTURE for x in out[sec].echoes):
                    bad2.append((q, sec, r_e))
    r = CheckResult(prop=f"CL2 an echo outside the structure window by more than the tolerances is never STRUCTURE at the "
                         f"classification pose ({n2} cases): the fit pose does not widen what the window can mask",
                    formula="echo range outside [w0 - tol, w1 + tol_far] -> not STRUCTURE",
                    encoding="numeric over the deployed classifier", expect="holds")
    r.verdict, r.passed = ("holds" if not bad2 else "violated"), not bad2
    r.note = "" if not bad2 else str(bad2[:3])
    rep.add(r, verbose)
    from holo_fleet.perception.gate_beacon import GateBeaconFix, beacon_position, fix_bound
    from holo_fleet.sim.gate_beacon import GateBeaconSim

    bp = beacon_position(g, cfg)
    bad4, n4 = [], 0
    for q in poses:
        truth = np.asarray(g.from_gate_frame(list(q)), float)
        e_b = min(fix_bound(cfg, float(np.linalg.norm(bp - truth))), 0.3)
        prof = _profiles(base, sg, truth, R)
        for u in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n4 += 1
            out = _classify(EchoClassifier, bars, cfg, prof, truth + R @ np.array([u[0] * e_b, u[1] * e_b, 0.0]), R,
                            pose_err=e_b)
            if any(x.cls != STRUCTURE for rd in out.values() for x in rd.echoes):
                bad4.append((q, u, round(e_b, 2)))
    r = CheckResult(prop=f"CL4 at the gate-beacon pose (error up to its bound, capped 0.3 m) every bar echo is STRUCTURE ({n4} "
                         "cases)", formula="classification pose within the beacon bound -> bar echoes STRUCTURE",
                    encoding="numeric over the deployed classifier", expect="holds")
    r.verdict, r.passed = ("holds" if not bad4 else "violated"), not bad4
    r.note = "" if not bad4 else str(bad4[:3])
    rep.add(r, verbose)
    yaw = float(np.arctan2(g.axis[1], g.axis[0]))
    over, n5 = 0, 0
    for k, q in enumerate([(s_, l_, 0.0) for s_ in (-6.0, -2.55, -1.0) for l_ in (-0.35, 0.35)]):
        truth = np.asarray(g.from_gate_frame(list(q)), float)
        for seed in range(30):
            sim, fix = GateBeaconSim(["d"], [g], cfg, seed=1000 * k + seed), GateBeaconFix(cfg)
            est = truth + R @ np.array([0.0, 0.7, 0.0])
            t = 0.0
            while t < cfg.gate_beacon.hold_s:
                for _gid, _tm, rr, az, el in sim.measure(t, {"d": (truth, R)})["d"]:
                    fix.add(t, g, rr, az, el, est, R)
                t += 0.1
            res = fix.offset(t - 0.1)
            if res is None:
                continue
            n5 += 1
            over += int(np.linalg.norm(est + res[0] - truth) > res[1])
    r = CheckResult(prop=f"CL5 the median beacon fix is within its bound in {n5 - over} of {n5} simulated windows (sigma levels of "
                         "config.GateBeaconConfig, a 0.7 m lateral drift; yaw from the gate axis)",
                    formula="|fix - truth| <= bound for every sampled window", encoding="numeric (Monte Carlo of the model)",
                    expect="holds")
    r.verdict, r.passed = ("holds" if over == 0 and n5 > 0 else "violated"), over == 0 and n5 > 0
    rep.add(r, verbose)
    r = CheckResult(prop=f"CL3 reported: a fit sets the classification pose with >= {CLS_MIN_MATCHED} matched sectors, RMS <= "
                         f"{FIT_RMS_M} m, uncertainty <= {UNIQUE_M + FIT_RMS_M:.2f} m, held {CLS_HOLD_S} s; never applied to the "
                         "navigation estimate", formula="reported", encoding="config", expect="holds")
    r.verdict, r.passed = "holds", True
    rep.add(r, verbose)
    return rep


if __name__ == "__main__":
    rep = run()
    rep.save()
    sys.exit(0 if rep.ok else 1)
