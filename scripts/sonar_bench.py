"""Six-sonar bench in HoloOcean (pinned agents, ground truth allowed: this is a test bench).

  python scripts/sonar_bench.py coverage   # second BlueROV2 along 26 directions (8 diagonals) x distances
  python scripts/sonar_bench.py perf       # tick cost with 1 / 3 / 4 / 6 drones x 6 sonars
  python scripts/sonar_bench.py shot       # screenshot of the six cones (ViewRegion) around one drone

Writes results/v2/sonar_bench/<mode>.json (+ figures/v2/sensor/*.png for ``shot``).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.config import DEFAULT  # noqa: E402
from holo_fleet.perception import sonar_geometry as sg  # noqa: E402
from holo_fleet.perception.sensor_suite import SONAR_NAMES  # noqa: E402
from holo_fleet.sim.holo_env import HoloFleetSim, TICKS_PER_SEC, agent_origin_from_pose  # noqa: E402
from holo_fleet.sim.spec import bench_spec  # noqa: E402

OUT = ROOT / "results" / "v2" / "sonar_bench"
FIG = ROOT / "figures" / "v2" / "sensor"
RANGES = DEFAULT.perc.sonar.ranges()
THR = DEFAULT.perc.sonar.threshold
O = np.array([0.0, -20.0, -8.0])          # deep enough to stage targets 6 m above


def first_echo(profile, lo=0.0, hi=99.0):
    p = np.asarray(profile, dtype=float)
    idx = np.flatnonzero((p > THR) & (RANGES >= lo) & (RANGES <= hi))
    return None if idx.size == 0 else float(RANGES[idx[0]])


def wait_captures(sim, n=2):
    sim.step({nm: None for nm in sim.names}, 0.1 * (n + 1))


def _seg_dist(a, b, c):
    ab = b - a
    t = float(np.clip(np.dot(c - a, ab) / max(float(np.dot(ab, ab)), 1e-9), 0.0, 1.0))
    return float(np.linalg.norm(a + t * ab - c))


def place(sim, name, target_pos, rot=(0.0, 0.0, 0.0), avoid=None, clearance=1.6, via_z=None):
    """Move a pinned agent to ``target_pos`` without crossing ``avoid`` (set_physics_state sweeps:
    a move through another hull stops at contact).  Detours via a waypoint 10 m out if needed;
    with ``via_z`` it climbs to that depth, translates and descends (clear of the seabed)."""
    avoid = O if avoid is None else avoid
    cur = agent_origin_from_pose(sim.latest[name]["PoseSensor"])
    if via_z is not None:
        dst = np.asarray(target_pos, float)
        for wp in (np.array([cur[0], cur[1], via_z]), np.array([dst[0], dst[1], via_z]), dst):
            sim.pin(name, wp, rot)
            wait_captures(sim, 0)
        return float(np.linalg.norm(agent_origin_from_pose(sim.latest[name]["PoseSensor"]) - dst))
    path = [np.asarray(target_pos, float)]
    if _seg_dist(cur, path[0], avoid) < clearance:
        a, b = cur - avoid, path[0] - avoid
        w = np.cross(a, b)
        if np.linalg.norm(w) < 1e-6:
            w = np.cross(a, [1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 * np.linalg.norm(a) else np.cross(a, [0.0, 1.0, 0.0])
        w = avoid + 10.0 * w / np.linalg.norm(w)
        path = [w] + path
    for wp in path:
        sim.pin(name, wp, rot)
        wait_captures(sim, 0)
    return float(np.linalg.norm(agent_origin_from_pose(sim.latest[name]["PoseSensor"]) - path[-1]))


def directions():
    dirs = {}
    for x in (-1, 0, 1):
        for y in (-1, 0, 1):
            for z in (-1, 0, 1):
                if (x, y, z) == (0, 0, 0):
                    continue
                kind = {1: "axis", 2: "edge", 3: "diagonal"}[abs(x) + abs(y) + abs(z)]
                v = np.array([x, y, z], float)
                dirs[f"{kind}({x:+d},{y:+d},{z:+d})"] = v / np.linalg.norm(v)
    return dirs


def mode_coverage():
    spec = bench_spec("bench_coverage", [O, O + np.array([0, 40.0, 0])])
    sim = HoloFleetSim(spec)
    sim.start()
    obs, tgt = sim.names
    sim.pin(obs, O, (0, 0, 0))
    rows = []
    t0 = time.time()
    for label, u in directions().items():
        for rho in (1.0, 1.25, 1.5, 2.0, 3.0, 5.0):
            for tyaw in (0.0, 45.0):
                if tyaw and not label.startswith("diagonal"):
                    continue
                p = O + rho * u
                # set_physics_state sweeps: stage 6 m out along u, detouring around the observer if needed,
                # then come in radially; check that the target really is where it was put
                place(sim, tgt, O + 6.0 * u, (0, 0, tyaw))
                place(sim, tgt, p, (0, 0, tyaw))
                wait_captures(sim, 3)
                pos_err = float(np.linalg.norm(agent_origin_from_pose(sim.latest[tgt]["PoseSensor"]) - p))
                seen, ages = {}, {}
                for s in sg.SECTORS:
                    key = sg.sonar_sensor_name(s)
                    prof = sim.latest[obs].get(key)
                    ages[s] = round(sim.t - sim.stamp[obs].get(key, -1e9), 3)
                    r = first_echo(prof, 0.0, rho + 0.6) if prof is not None else None
                    if r is not None:
                        seen[s] = r
                must = sorted(sg.pattern_of_point(rho * u, sg.R_IN))
                maybe = sorted(sg.pattern_of_point(rho * u, sg.R_OUT))
                rows.append({"dir": label, "rho": rho, "target_yaw": tyaw, "seen": seen, "age": ages, "t": sim.t,
                             "target_pos_err": round(pos_err, 3), "valid": pos_err < 0.05,
                             "must": must, "maybe": maybe,
                             "consistent": set(must) <= set(seen) <= set(maybe), "detected": bool(seen)})
    sim.close()
    summary = {}
    for rho in sorted({r["rho"] for r in rows}):
        sel = [r for r in rows if r["rho"] == rho]
        summary[str(rho)] = {"cases": len(sel), "valid": sum(r["valid"] for r in sel),
                             "detected": sum(r["detected"] for r in sel),
                             "pattern_within_prediction": sum(r["consistent"] for r in sel),
                             "diagonals_detected": sum(r["detected"] for r in sel if r["dir"].startswith("diagonal")),
                             "diagonal_cases": sum(1 for r in sel if r["dir"].startswith("diagonal"))}
    return {"rows": rows, "summary": summary, "wall_s": round(time.time() - t0, 1), "setup": sim.setup_report}


NAV_ERR = np.array([0.10, -0.10, 0.0])     # emulated dead-reckoning error of the observer (bench)


def classify_case(sim, clf, obs, yaw_deg, captures=3):
    """Run the onboard classifier on the observer's next captures (own pose = truth + NAV_ERR)."""
    from holo_fleet.perception.sonar_processing import dvl_altitude

    out = None
    for _ in range(captures):
        wait_captures(sim, 0)
        pose = np.asarray(sim.latest[obs]["PoseSensor"], float)
        own = agent_origin_from_pose(pose) + NAV_ERR
        y = math.radians(yaw_deg)
        Rwb = np.array([[math.cos(y), -math.sin(y), 0.0], [math.sin(y), math.cos(y), 0.0], [0.0, 0.0, 1.0]])
        alt = dvl_altitude(sim.latest[obs].get("DVLSensor"))
        seabed_z = None if alt is None else float(own[2] - 0.15 - alt)       # DVL head ~0.15 m below the origin
        profiles = {s_: sim.latest[obs].get(sg.sonar_sensor_name(s_)) for s_ in sg.SECTORS}
        ages = {s_: sim.t - sim.stamp[obs].get(sg.sonar_sensor_name(s_), -1e9) for s_ in sg.SECTORS}
        out = clf.classify(profiles, ages, own, Rwb, seabed_z)
    res = {s_: [{"r0": round(e.r0, 3), "r1": round(e.r1, 3), "cls": e.cls, "why": e.why} for e in rd.echoes]
           for s_, rd in out.items() if rd.echoes}
    truth = {nm: np.round(agent_origin_from_pose(sim.latest[nm]["PoseSensor"]), 2).tolist() for nm in sim.names}
    prof = {s_: np.round(np.asarray(sim.latest[obs].get(sg.sonar_sensor_name(s_)), float), 4).tolist()
            for s_ in sg.SECTORS}
    return res, prof, {"seabed_z_est": None if seabed_z is None else round(seabed_z, 2), "truth": truth}


def mode_classify():
    from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates
    from holo_fleet.perception.sonar_processing import EchoClassifier

    cases = []
    g6 = {g.gate_id: g for g in load_arena_gates(HORSESHOE_TRACK)}["G06"]
    gyaw = math.degrees(math.atan2(g6.axis[1], g6.axis[0]))
    far = [O + np.array([-40.0, -30.0, 0]), O + np.array([-46.0, -30.0, 0])]     # beyond RangeMax of every test pose
    # ---------------------------------------------------------------- arena session: A, B, D, F
    spec = bench_spec("bench_classify", [O] + far, gate_ids=("G06",))
    sim = HoloFleetSim(spec)
    sim.start()
    obs, t1, t2 = sim.names
    clf = EchoClassifier(gate_bars=g6.bars)

    def at_gate(d):
        return g6.center - d * g6.axis

    sim.pin(obs, at_gate(3.0), (0, 0, gyaw))
    res, prof, extra = classify_case(sim, clf, obs, gyaw)
    cases.append({"case": "A_gate_only", "expect": "FRONT: STRUCTURE only", "echoes": res, "profiles": prof, **extra})
    place(sim, obs, O, (0, 0, 0), avoid=O + np.array([50.0, 0, 0]))
    place(sim, t1, O + np.array([6.0, 0, 0]), (0, 0, 180)); place(sim, t1, O + np.array([3.0, 0, 0]), (0, 0, 180))
    res, prof, extra = classify_case(sim, clf, obs, 0.0)
    cases.append({"case": "B_drone_only", "expect": "FRONT: DYNAMIC at ~2.5 m", "echoes": res, "profiles": prof, **extra})
    place(sim, obs, at_gate(4.5), (0, 0, gyaw), avoid=O + np.array([3.0, 0, 0]))
    place(sim, t1, at_gate(2.5), (0, 0, gyaw + 180.0), avoid=at_gate(4.5))
    res, prof, extra = classify_case(sim, clf, obs, gyaw)
    cases.append({"case": "D_gate_plus_drone", "expect": "FRONT: DYNAMIC ~1.8 m + STRUCTURE ~4.4 m", "echoes": res,
                  "profiles": prof, **extra})
    place(sim, obs, O, (0, 0, 0), avoid=at_gate(2.5))
    place(sim, t1, O + np.array([6.0, 0.6, 0]), (0, 0, 180)); place(sim, t1, O + np.array([2.2, 0.6, 0]), (0, 0, 180))
    place(sim, t2, O + np.array([8.0, -1.2, 0]), (0, 0, 180)); place(sim, t2, O + np.array([5.5, -1.2, 0]), (0, 0, 180))
    res, prof, extra = classify_case(sim, clf, obs, 0.0)
    cases.append({"case": "F_two_drones_one_cone", "expect": "FRONT: two DYNAMIC echoes (~1.7 m and ~5.0 m)",
                  "echoes": res, "profiles": prof, **extra})
    sim.close()
    # ---------------------------------------------------------------- seabed session: C, E
    SB = np.array([0.0, -40.0, -292.7])                      # ~2 m above the seabed (z ~ -294.7 here)
    box = ([-50.0, -90.0, -310.0], [50.0, 10.0, -260.0])
    spec = bench_spec("bench_seabed", [SB, SB + np.array([0, 25.0, 0])], env_box=box)
    sim = HoloFleetSim(spec)
    sim.start()
    obs, t1 = sim.names
    clf = EchoClassifier(gate_bars=())
    sim.pin(obs, SB, (0, 0, 0))
    res, prof, extra = classify_case(sim, clf, obs, 0.0)
    cases.append({"case": "C_seabed_only", "expect": "DOWN and lower sectors: SEABED only", "echoes": res,
                  "profiles": prof, **extra})
    place(sim, t1, SB + np.array([1.9, 0, 0]), (0, 0, 180), via_z=-280.0)
    res, prof, extra = classify_case(sim, clf, obs, 0.0)
    cases.append({"case": "E1_seabed_plus_near_drone", "expect": "FRONT: DYNAMIC before the seabed onset + SEABED",
                  "echoes": res, "profiles": prof, **extra})
    place(sim, t1, SB + np.array([4.0, 0, 0]), (0, 0, 180), via_z=-280.0)
    res, prof, extra = classify_case(sim, clf, obs, 0.0)
    cases.append({"case": "E2_seabed_plus_far_drone", "expect": "FRONT: drone inside the seabed clutter -> UNKNOWN (not SEABED)",
                  "echoes": res, "profiles": prof, **extra})
    sim.close()
    for c in cases:
        print(c["case"], "| expect:", c["expect"], "| truth", c["truth"])
        for sec, ech in c["echoes"].items():
            print("    ", sec, [(e["r0"], e["cls"]) for e in ech])
    return {"cases": cases, "nav_error_m": NAV_ERR.tolist()}


def mode_perf():
    res = {}
    for n in (1, 3, 4, 6):
        for with_gate in (False, True):
            base = np.array([-12.0, 16.0, -4.3]) if with_gate else O
            pos = [base + np.array([-3.0 * (k // 3), 3.0 * (k % 3) - 3.0, 0.0]) for k in range(n)]
            spec = bench_spec(f"bench_perf_{n}", pos, gate_ids=("G06",) if with_gate else ())
            sim = HoloFleetSim(spec)
            sim.start()
            for k, nm in enumerate(sim.names):
                sim.pin(nm, pos[k], (0, 0, 8.5 if with_gate else 0.0))
            sim.step({nm: None for nm in sim.names}, 1.0)       # warm-up
            sim.tick_wall_ms.clear()
            c0 = sim.sonar_captures
            t0 = time.time()
            sim.step({nm: None for nm in sim.names}, 6.0)
            wall = time.time() - t0
            captures_per_s = (sim.sonar_captures - c0) / 6.0 / (n * 6)
            key = f"{n}x6{'_gate' if with_gate else ''}"
            res[key] = {**sim.perf(), "sonars": n * 6, "sonar_hz_per_sensor_sim_time": round(captures_per_s, 2),
                        "wall_s_for_6s_sim": round(wall, 2)}
            print(key, res[key], flush=True)
            sim.close()
    return res


def mode_shot():
    import cv2

    spec = bench_spec("bench_shot", [O], camera_drone=0)
    spec.chase_offset = (-4.0, -3.0, 2.4)
    sim = HoloFleetSim(spec, view_region=True)
    sim.start()
    sim.pin(sim.names[0], O, (0, 0, 20.0))
    imgs = []
    for _ in range(12):
        sim.step({sim.names[0]: None}, 0.1)
        img = sim.image("ChaseCamera")
        if img is not None:
            imgs.append(np.asarray(img)[:, :, :3].copy())
    sim.close()
    FIG.mkdir(parents=True, exist_ok=True)
    # the cone lines are drawn on capture ticks only: keep the frame with the most green pixels
    best = max(imgs, key=lambda im: int(((im[:, :, 1] > 200) & (im[:, :, 0] < 80) & (im[:, :, 2] < 80)).sum()))
    cv2.imwrite(str(FIG / "six_sonar_cones_holoocean.png"), best)
    return {"frames": len(imgs)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["coverage", "perf", "shot", "classify"])
    a = ap.parse_args()
    res = {"coverage": mode_coverage, "perf": mode_perf, "shot": mode_shot, "classify": mode_classify}[a.mode]()
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{a.mode}.json").write_text(json.dumps(res, indent=1, default=str), encoding="utf-8")
    if a.mode == "coverage":
        print(json.dumps(res["summary"], indent=1))
        bad = [r for r in res["rows"] if not r["detected"] or not r["consistent"]]
        for r in bad[:40]:
            print("  ", r["dir"], r["rho"], r["target_yaw"], "t", r["t"], "seen", r["seen"], "must", r["must"], "max age", max(r["age"].values()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
