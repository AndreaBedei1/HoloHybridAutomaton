"""sonar_classification demo (about 28 s of simulated time): an echo is not a drone.

Part 1, at Marine Race Arena gate G06: an observer drone holds 4.5 m in front of the gate; a second
BlueROV2 is moved along a scripted path: out of range -> into the FRONT cone between observer and
gate -> behind the gate -> abeam on the left.  Part 2, two metres above the seabed: a second drone
comes towards the observer from inside the seabed clutter.

The observer runs the onboard echo classifier on its six profiles (own pose: the pinned pose plus a
10 cm navigation error, like the bench).  The left half of the frame shows the six profiles with
the classified segments; the right half shows the ground truth (referee only).

    python scripts/run_demo.py --scenario sonar_classification
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.config import DEFAULT  # noqa: E402
from holo_fleet.perception import sonar_geometry as sg  # noqa: E402
from holo_fleet.perception.sonar_processing import EchoClassifier, dvl_altitude  # noqa: E402
from holo_fleet.sim.holo_env import HoloFleetSim, agent_origin_from_pose  # noqa: E402
from holo_fleet.sim.spec import bench_spec  # noqa: E402
from holo_fleet.ui import dashboard as D  # noqa: E402
from holo_fleet.ui.gif import make_gif  # noqa: E402

NAV_ERR = np.array([0.10, -0.10, 0.0])
RANGES = DEFAULT.perc.sonar.ranges()
THR = DEFAULT.perc.sonar.threshold
W, H = 1600, 900


def _rot(yaw_deg):
    y = math.radians(yaw_deg)
    return np.array([[math.cos(y), -math.sin(y), 0.0], [math.sin(y), math.cos(y), 0.0], [0.0, 0.0, 1.0]])


def _profile_panel(img, x, y, w, h, sector, prof, rd):
    cv2.rectangle(img, (x, y), (x + w, y + h), D.PANEL, -1)
    D._text(img, sector, x + 8, y + 18, 0.5, D.INK)
    px0, px1, py0, py1 = x + 40, x + w - 10, y + h - 22, y + 26
    cv2.line(img, (px0, py0), (px1, py0), D.LINE, 1)
    for r in range(0, 13, 2):
        u = int(px0 + (px1 - px0) * r / 12.0)
        cv2.line(img, (u, py0), (u, py0 + 4), D.MUTED, 1)
        D._text(img, f"{r}", u - 4, py0 + 17, 0.38, D.MUTED)
    ymax = 1.2
    ty = int(py0 - (py0 - py1) * THR / ymax)
    cv2.line(img, (px0, ty), (px1, ty), D.WARN, 1)
    if prof is not None:
        p = np.clip(np.asarray(prof, float).ravel(), 0, ymax)
        pts = np.stack([px0 + (px1 - px0) * (RANGES - 0.0) / 12.0, py0 - (py0 - py1) * p / ymax], axis=1).astype(np.int32)
        cv2.polylines(img, [pts], False, D.INK2, 1, cv2.LINE_AA)
    if rd is not None:
        last_label, n_labels = -9.0, 0
        for e in rd.echoes:
            u0 = int(px0 + (px1 - px0) * e.r0 / 12.0)
            u1 = max(u0 + 3, int(px0 + (px1 - px0) * e.r1 / 12.0))
            col = D.CLS_COLOR.get(e.cls, D.INK2)
            cv2.rectangle(img, (u0, py1 - 4), (u1, py0 - 1), col, 2)
            if e.r0 - last_label >= 1.6 and n_labels < 3:          # no overlapping labels
                D._text(img, f"{D.CLS_SHORT.get(e.cls, e.cls)} {e.r0:.1f}", u0, py1 - 8, 0.42, col)
                last_label, n_labels = e.r0, n_labels + 1
        if rd.structure_window is not None:
            a, b = rd.structure_window
            cv2.line(img, (int(px0 + (px1 - px0) * a / 12.0), py0 + 2), (int(px0 + (px1 - px0) * b / 12.0), py0 + 2),
                     D.CLS_COLOR["STRUCTURE"], 3)
        if rd.blind_from is not None:
            u = int(px0 + (px1 - px0) * rd.blind_from / 12.0)
            cv2.line(img, (u, py1), (u, py0), D.CLS_COLOR["SEABED"], 2)
            if rd.blind_from <= 4.0:
                D._text(img, f"clutter from {rd.blind_from:.1f} m: UNKNOWN target", x + 70, y + 18, 0.4, D.WARN)


def _frame(title, caption, t, profiles, readings, truth_lines, top_pts, cam):
    img = np.full((H, W, 3), D.SURFACE, np.uint8)
    D._text(img, "sonar_classification - an echo is not a drone", 16, 30, 0.75, D.INK, 2)
    D._text(img, f"t = {t:5.1f} s", W - 180, 30, 0.7, D.INK, 2)
    D._text(img, title, 16, 50, 0.45, D.MUTED)
    cv2.rectangle(img, (8, 58), (D.SPLIT - 8, 82), D.ONBOARD_HDR, -1)
    D._text(img, "ONBOARD - observer's six echo profiles, classified (gate map + own pose + depth + DVL)", 16, 76, 0.5, D.INK)
    order = ["FRONT", "LEFT", "RIGHT", "REAR", "UP", "DOWN"]
    pw, ph = (D.SPLIT - 24) // 2, 205
    for i, s in enumerate(order):
        _profile_panel(img, 8 + (i % 2) * (pw + 8), 90 + (i // 2) * (ph + 8), pw, ph, s, profiles.get(s), readings.get(s))
    D._legend(img, 16, 90 + 3 * (ph + 8) + 18)
    D._text(img, "grey curve: intensity per 5 cm bin; amber: threshold 0.30; grey bar under the axis: gate map window; "
                 "brown line: seabed clutter onset", 16, H - 46, 0.4, D.MUTED)
    cv2.rectangle(img, (D.SPLIT + 4, 58), (W - 8, 82), D.REFEREE_HDR, -1)
    D._text(img, "REFEREE / GROUND TRUTH - never given to the observer", D.SPLIT + 12, 76, 0.5, (16, 16, 16))
    # top view of observer / target / gate, drawn on its own canvas (clipped to the panel)
    mx, my, mw, mh = D.SPLIT + 4, 90, W - D.SPLIT - 12, 330
    pan = np.full((mh, mw, 3), D.PANEL, np.uint8)
    c = top_pts["center"]
    s_ = min(mw, mh) / 16.0

    def tx(q):
        return int(mw / 2 + (q[0] - c[0]) * s_), int(mh / 2 - (q[1] - c[1]) * s_)

    for poly in top_pts.get("bars", []):
        cv2.fillPoly(pan, [np.array([tx(q) for q in poly], np.int32)], D.INK2)
    o, yaw = top_pts["observer"], top_pts["observer_yaw"]
    R = _rot(yaw)
    for sct, col in (("FRONT", (90, 90, 84)), ("LEFT", (70, 70, 66)), ("RIGHT", (70, 70, 66))):
        a = R @ sg.AXES[sct]
        ang = math.atan2(a[1], a[0])
        poly = [tx(o)] + [tx(o + 6.0 * np.array([math.cos(ang + d), math.sin(ang + d), 0.0]))
                          for d in np.radians(np.linspace(-60, 60, 13))]
        cv2.polylines(pan, [np.array(poly, np.int32)], True, col, 1, cv2.LINE_AA)
        q = tx(o + 6.4 * np.array([math.cos(ang), math.sin(ang), 0.0]))
        D._text(pan, sct.lower(), q[0] - 14, q[1], 0.4, D.MUTED)
    cv2.circle(pan, tx(o), 7, D.DRONE[0], -1)
    D._text(pan, "observer", tx(o)[0] + 9, tx(o)[1] + 18, 0.45, D.INK)
    for q in top_pts.get("targets", []):
        cv2.circle(pan, tx(q), 7, D.DRONE[1], -1)
        D._text(pan, "drone", tx(q)[0] + 9, tx(q)[1] - 6, 0.45, D.INK)
    D._text(pan, "top view (ground truth); outlines: FRONT / LEFT / RIGHT cones to 6 m", 8, 18, 0.42, D.MUTED)
    img[my:my + mh, mx:mx + mw] = pan
    yy = my + mh + 30
    words, cap_lines, cur = caption.split(), [], ""
    for w_ in words:                                       # wrap the caption to the panel width
        if len(cur) + len(w_) + 1 > 62:
            cap_lines.append(cur)
            cur = w_
        else:
            cur = (cur + " " + w_).strip()
    cap_lines.append(cur)
    for i, line in enumerate(cap_lines):
        D._text(img, line, D.SPLIT + 12, yy + 24 * i, 0.5, D.INK)
    yy += 24 * (len(cap_lines) - 1)
    for i, line in enumerate(truth_lines):
        D._text(img, line, D.SPLIT + 12, yy + 26 + 22 * i, 0.45, D.INK2)
    if cam is not None:
        ih = 220
        iw = int(ih * cam.shape[1] / cam.shape[0])
        iw = min(iw, W - D.SPLIT - 16)
        ih = int(iw * cam.shape[0] / cam.shape[1])
        img[H - D.FOOTER - ih - 4:H - D.FOOTER - 4, D.SPLIT + 8:D.SPLIT + 8 + iw] = cv2.resize(cam[:, :, :3], (iw, ih))
    cv2.rectangle(img, (0, H - D.FOOTER), (W, H), D.PANEL, -1)
    D._text(img, "classes: STRUCTURE (explained by the gate map), SEABED (explained by depth + DVL altitude + cone), "
                 "DYNAMIC (possible vehicle), UNKNOWN (obstacle, cannot be explained)", 16, H - 12, 0.45, D.INK2)
    return img


def _classify(sim, clf, obs, yaw):
    pose = np.asarray(sim.latest[obs]["PoseSensor"], float)
    own = agent_origin_from_pose(pose) + NAV_ERR
    alt = dvl_altitude(sim.latest[obs].get("DVLSensor"))
    seabed_z = None if alt is None else float(own[2] - 0.15 - alt)
    profiles = {s: sim.latest[obs].get(sg.sonar_sensor_name(s)) for s in sg.SECTORS}
    ages = {s: sim.t - sim.stamp[obs].get(sg.sonar_sensor_name(s), -1e9) for s in sg.SECTORS}
    return profiles, clf.classify(profiles, ages, own, _rot(yaw), seabed_z)


def _path(waypoints, speed, dt=0.1):
    out = []
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(np.linalg.norm(b - a) / (speed * dt)))
        out += [a + (b - a) * k / n for k in range(n)]
    out.append(np.asarray(waypoints[-1], float))
    return out


def run_classification_demo(out_root: Path, headless: bool = True, show: bool = True) -> int:
    from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates

    out = Path(out_root) / "sonar_classification"
    (out / "dashboard").mkdir(parents=True, exist_ok=True)
    for f in (out / "dashboard").glob("*.jpg"):
        f.unlink()
    log = []
    g6 = {g.gate_id: g for g in load_arena_gates(HORSESHOE_TRACK)}["G06"]
    gyaw = math.degrees(math.atan2(g6.axis[1], g6.axis[0]))
    left = np.array([-g6.axis[1], g6.axis[0], 0.0])
    bars = []
    for b in g6.bars:
        corners = np.array([b.center + b.axes @ (b.half * np.array([sx, sy, sz])) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        bars.append(cv2.convexHull((corners[:, :2] * 1000).astype(np.float32)).reshape(-1, 2) / 1000.0)
    t_global = 0.0
    # ------------------------------------------------------------------------------------------ part 1: gate
    obs_pos = g6.center - 4.5 * g6.axis
    far = obs_pos + 14.0 * left
    spec = bench_spec("demo_classification", [obs_pos, far], yaws=[gyaw, gyaw], gate_ids=("G06",), camera_drone=0)
    spec.chase_offset = (-3.5, -2.5, 1.4)
    sim = HoloFleetSim(spec, headless=headless)
    sim.start()
    o, tg = sim.names
    clf = EchoClassifier(gate_bars=g6.bars)
    sim.pin(o, obs_pos, (0, 0, gyaw))
    sim.pin(tg, far, (0, 0, gyaw - 90))
    sim.step({nm: None for nm in sim.names}, 0.5)
    legs = [
        ("1) only the gate in the FRONT cone -> STRUCTURE", [far, far], 1.5),
        ("2) a drone enters the FRONT cone between observer and gate -> DYNAMIC + STRUCTURE",
         [obs_pos + 7.0 * left + 2.2 * g6.axis, obs_pos + 2.2 * g6.axis + 0.3 * left, obs_pos + 2.2 * g6.axis - 1.2 * left], 1.0),
        ("3) the drone moves behind the gate: STRUCTURE first, the drone beyond it",
         [obs_pos + 2.2 * g6.axis - 3.0 * left, obs_pos + 7.5 * g6.axis - 3.0 * left, obs_pos + 7.5 * g6.axis], 0.0),
        ("4) the drone comes abeam on the left: LEFT sees a DYNAMIC echo, FRONT only the gate",
         [obs_pos + 7.5 * g6.axis + 3.5 * left, obs_pos + 3.0 * left + 0.5 * g6.axis], 1.5),
    ]
    for caption, wps, hold in legs:
        pts = _path(wps, 1.6) if len(wps) > 1 and np.linalg.norm(np.asarray(wps[0]) - np.asarray(wps[-1])) > 1e-6 else []
        steps = [(p, False) for p in pts] + [(np.asarray(wps[-1], float), True)] * int(hold / 0.1)
        for p, _h in steps:
            sim.pin(tg, p, (0, 0, gyaw - 90))
            sim.step({nm: None for nm in sim.names}, 0.1)
            t_global += 0.1
            profiles, rd = _classify(sim, clf, o, gyaw)
            P_t = agent_origin_from_pose(np.asarray(sim.latest[tg]["PoseSensor"], float))
            P_o = agent_origin_from_pose(np.asarray(sim.latest[o]["PoseSensor"], float))
            d = float(np.linalg.norm(P_t - P_o))
            lines = [f"drone at {d:.2f} m from the observer (centre to centre)",
                     f"gate centre at {np.linalg.norm(g6.center - P_o):.2f} m; drone {'behind' if g6.to_gate_frame(P_t)[0] > 0.3 else 'in front of'} the gate plane"]
            img = _frame("observer pinned 4.5 m before gate G06; the second drone follows a scripted path",
                         caption, t_global, profiles, rd, lines,
                         {"center": P_o + 3.0 * g6.axis, "bars": bars, "observer": P_o, "observer_yaw": gyaw, "targets": [P_t]},
                         sim.image("ChaseCamera"))
            k = int(round(t_global * 10))
            if k % 4 == 0:
                cv2.imwrite(str(out / "dashboard" / f"dash_{k:05d}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 88])
            if show:
                cv2.imshow("sonar_classification", img)
                cv2.waitKey(1)
            log.append({"t": round(t_global, 1), "part": "gate", "caption": caption, "true_distance": round(d, 3),
                        "echoes": {s: [(round(e.r0, 2), e.cls) for e in r.echoes] for s, r in rd.items() if r.echoes}})
    sim.close()
    # ------------------------------------------------------------------------------------------ part 2: seabed
    SB = np.array([0.0, -40.0, -292.7])                    # about 2 m above the seabed
    box = ([-50.0, -90.0, -310.0], [50.0, 10.0, -260.0])
    spec = bench_spec("demo_seabed", [SB, SB + np.array([25.0, 0, 12.0])], env_box=box, camera_drone=0)
    spec.chase_offset = (-3.5, -2.5, 1.0)
    sim = HoloFleetSim(spec, headless=headless)
    sim.start()
    o, tg = sim.names
    clf = EchoClassifier(gate_bars=())
    sim.pin(o, SB, (0, 0, 0))
    sim.pin(tg, SB + np.array([25.0, 0, 12.0]), (0, 0, 180))
    sim.step({nm: None for nm in sim.names}, 0.5)
    for wp in (SB + np.array([7.0, 0.0, 12.0]), SB + np.array([7.0, 0.0, 0.0])):     # descend far away (no sweep)
        sim.pin(tg, wp, (0, 0, 180))
        sim.step({nm: None for nm in sim.names}, 0.3)
    legs = [("5) near the seabed: the lower part of every cone hits the bottom -> SEABED (DVL altitude + cone)",
             [SB + np.array([7.0, 0.0, 0.0])], 1.5),
            ("6) a drone inside the seabed clutter cannot be separated from the bottom -> UNKNOWN (conservative)",
             [SB + np.array([7.0, 0.0, 0.0]), SB + np.array([3.6, 0.0, 0.0])], 1.5),
            ("7) closer than the clutter onset the drone is a separate echo -> DYNAMIC before SEABED",
             [SB + np.array([3.6, 0.0, 0.0]), SB + np.array([1.9, 0.0, 0.0])], 1.5)]
    for caption, wps, hold in legs:
        pts = _path(wps, 1.0) if len(wps) > 1 else []
        steps = [(p, False) for p in pts] + [(np.asarray(wps[-1], float), True)] * int(hold / 0.1)
        for p, _h in steps:
            sim.pin(tg, p, (0, 0, 180))
            sim.step({nm: None for nm in sim.names}, 0.1)
            t_global += 0.1
            profiles, rd = _classify(sim, clf, o, 0.0)
            P_t = agent_origin_from_pose(np.asarray(sim.latest[tg]["PoseSensor"], float))
            P_o = agent_origin_from_pose(np.asarray(sim.latest[o]["PoseSensor"], float))
            alt = dvl_altitude(sim.latest[o].get("DVLSensor"))
            lines = [f"drone at {np.linalg.norm(P_t - P_o):.2f} m from the observer",
                     f"observer altitude above the seabed (DVL, onboard): {alt:.2f} m" if alt is not None else "no DVL lock"]
            img = _frame("observer pinned about 2 m above the vegetated seabed (no gate map here)", caption, t_global,
                         profiles, rd, lines, {"center": P_o + np.array([3.0, 0, 0]), "bars": [], "observer": P_o,
                                               "observer_yaw": 0.0, "targets": [P_t]}, sim.image("ChaseCamera"))
            k = int(round(t_global * 10))
            if k % 4 == 0:
                cv2.imwrite(str(out / "dashboard" / f"dash_{k:05d}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 88])
            if show:
                cv2.imshow("sonar_classification", img)
                cv2.waitKey(1)
            log.append({"t": round(t_global, 1), "part": "seabed", "caption": caption,
                        "true_distance": round(float(np.linalg.norm(P_t - P_o)), 3),
                        "echoes": {s: [(round(e.r0, 2), e.cls) for e in r.echoes] for s, r in rd.items() if r.echoes}})
    sim.close()
    (out / "classification_log.json").write_text(json.dumps(log, indent=0), encoding="utf-8")
    (out / "run_status.json").write_text(json.dumps({"status": "COMPLETE", "sim_time_s": round(t_global, 1)}), encoding="utf-8")
    gif = make_gif((out / "dashboard").glob("dash_*.jpg"), out / "sonar_classification.gif", fps=5.0)
    print(f"sonar_classification: {t_global:.1f} s simulated, frames in {out / 'dashboard'}, GIF {gif}")
    for cap in dict.fromkeys(e["caption"] for e in log):
        last = [e for e in log if e["caption"] == cap][-1]
        print(f"  {cap}\n      last capture: {last['echoes']}  (true distance {last['true_distance']} m)")
    return 0


if __name__ == "__main__":
    raise SystemExit(run_classification_demo(ROOT / "results" / "v2" / "demos", headless=True, show=False))
