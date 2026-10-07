"""Demo dashboard (OpenCV): ONBOARD knowledge of every drone vs REFEREE / GROUND TRUTH.

The left half shows only what each drone knows - its own estimated pose, its mode, its six sonar
sectors (nearest echo range, class, age), the conservative nearest distance used by its guards, the
escape direction, its onboard current estimate, the gate decision and occupancy belief, its own
formation-error estimate.  The right half is the referee: ground-truth positions, true distances,
critical-region occupancy, formation error, true current.  Nothing on the right is ever sent to a
drone.  The same renderer is used live (scripts/run_demo.py) and on the logs of a finished run
(scripts/render_demo.py), so a GIF shows exactly what the live window showed.

Colours: dark chart surface, categorical slots of the reference palette for drone identity (always
paired with the drone number), status colours only for safety states, always with a text label.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np

W, H = 1600, 900
SPLIT = 880                                  # ONBOARD | REFEREE
HEADER = 54
FOOTER = 34


def _bgr(hexs: str):
    h = hexs.lstrip("#")
    return (int(h[4:6], 16), int(h[2:4], 16), int(h[0:2], 16))


SURFACE = _bgr("#1a1a19")
PANEL = _bgr("#232321")
LINE = _bgr("#383835")
INK = _bgr("#ffffff")
INK2 = _bgr("#c3c2b7")
MUTED = _bgr("#898781")
DRONE = [_bgr(c) for c in ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300")]
GOOD, WARN, SERIOUS, CRIT = _bgr("#0ca30c"), _bgr("#fab219"), _bgr("#ec835a"), _bgr("#d03b3b")
ONBOARD_HDR = _bgr("#3987e5")
REFEREE_HDR = _bgr("#c98500")
CLS_COLOR = {"STRUCTURE": _bgr("#6f6e69"), "SEABED": _bgr("#8c6d46"), "DYNAMIC": CRIT, "UNKNOWN": WARN,
             "UNCONFIRMED": _bgr("#4a4a46")}
CLS_SHORT = {"STRUCTURE": "STR", "SEABED": "SEA", "DYNAMIC": "DYN", "UNKNOWN": "UNK", "UNCONFIRMED": "unc"}
MODE_COLOR = {"FORMATION_FOLLOW": GOOD, "FORMATION_RECOVERY": WARN, "SEPARATION_WARNING": SERIOUS,
              "COLLISION_AVOIDANCE": CRIT, "FAILSAFE_HOLD_OR_RETREAT": _bgr("#9085e9"),
              "GATE_APPROACH": _bgr("#5598e7"), "GATE_YIELD": _bgr("#86b6ef"), "GATE_PASS": _bgr("#e87ba4")}
MODE_SHORT = {"FORMATION_FOLLOW": "FOLLOW", "FORMATION_RECOVERY": "RECOVERY", "SEPARATION_WARNING": "SEP WARNING",
              "COLLISION_AVOIDANCE": "COLL AVOID", "FAILSAFE_HOLD_OR_RETREAT": "FAILSAFE", "GATE_APPROACH": "GATE APPROACH",
              "GATE_YIELD": "GATE YIELD", "GATE_PASS": "GATE PASS"}
FONT = cv2.FONT_HERSHEY_SIMPLEX


def _text(img, s, x, y, scale=0.5, color=INK, thick=1):
    cv2.putText(img, str(s), (int(x), int(y)), FONT, scale, color, thick, cv2.LINE_AA)


def _badge(img, s, x, y, color, scale=0.48, pad=5, text_color=(16, 16, 16)):
    (tw, th), _ = cv2.getTextSize(s, FONT, scale, 1)
    cv2.rectangle(img, (int(x), int(y - th - pad)), (int(x + tw + 2 * pad), int(y + pad - 1)), color, -1)
    _text(img, s, x + pad, y, scale, text_color, 1)
    return x + tw + 2 * pad + 6


def _fmt(v, nd=2):
    return "-" if v is None else f"{v:.{nd}f}"


# ---------------------------------------------------------------------------------------------- onboard
def _sector_glyph(img, rec, cx, cy):
    """Six sector boxes around a hull: F up, B down, L left, R right; U and D to the right."""
    bw, bh = 66, 30
    pos = {"FRONT": (cx - bw // 2, cy - 2 * bh - 4), "REAR": (cx - bw // 2, cy + bh // 2 + 4),
           "LEFT": (cx - bw - bw // 2 - 4, cy - bh // 2 - 8), "RIGHT": (cx + bw // 2 + 4, cy - bh // 2 - 8),
           "UP": (cx + 2 * bw - 6, cy - 2 * bh - 4), "DOWN": (cx + 2 * bw - 6, cy + bh // 2 + 4)}
    cv2.rectangle(img, (cx - 12, cy - bh - 2), (cx + 12, cy + 4), INK2, 1)          # hull (top view)
    cv2.line(img, (cx, cy - bh - 2), (cx, cy - bh - 9), INK2, 1)
    for s, (x, y) in pos.items():
        sec = (rec.get("sectors") or {}).get(s, {})
        ech = [e for e in sec.get("echoes", []) if e[1] in CLS_COLOR]
        healthy = sec.get("healthy", True)
        near = min(ech, key=lambda e: e[0]) if ech else None
        fill = CLS_COLOR[near[1]] if near else PANEL
        cv2.rectangle(img, (x, y), (x + bw, y + bh), fill, -1)
        cv2.rectangle(img, (x, y), (x + bw, y + bh), LINE if healthy else CRIT, 1)
        _text(img, {"FRONT": "F", "REAR": "B", "LEFT": "L", "RIGHT": "R", "UP": "UP", "DOWN": "DN"}[s], x + 3, y + 12, 0.36, INK2)
        if near:
            _text(img, f"{near[0]:.1f}", x + 22, y + 13, 0.42, INK)
            _text(img, CLS_SHORT[near[1]], x + 22, y + 26, 0.36, INK)
        else:
            _text(img, "-" if healthy else "STALE", x + 22, y + 20, 0.38, MUTED if healthy else CRIT)
        age = sec.get("age", 0.0)
        if age is not None and age > 0.15:
            _text(img, f"{age:.1f}s", x + bw - 30, y + 12, 0.32, WARN)


def _drone_card(img, k, name, rec, x, y, w, h, show_form=True):
    cv2.rectangle(img, (x, y), (x + w, y + h), PANEL, -1)
    cv2.rectangle(img, (x, y), (x + 6, y + h), DRONE[k % len(DRONE)], -1)
    mode = rec.get("mode", "?")
    xx = _badge(img, f"{name}", x + 14, y + 22, DRONE[k % len(DRONE)], 0.5, text_color=INK)
    xx = _badge(img, MODE_SHORT.get(mode, mode), xx, y + 22, MODE_COLOR.get(mode, INK2), 0.5)
    g = rec.get("gate") or {}
    if g.get("id") and mode.startswith("GATE"):
        _text(img, f"gate {g['id']}: {g.get('decision', '')}  CR belief {g.get('occ', '')}", xx, y + 22, 0.42, INK2)
    elif rec.get("giveway"):
        _text(img, f"give-way {rec['giveway']}", xx, y + 22, 0.45, WARN)
    p = rec.get("nav_p") or [0, 0, 0]
    ce = rec.get("current_est") or [0, 0, 0]
    _text(img, f"nav pose  x {p[0]:6.1f}  y {p[1]:6.1f}  z {p[2]:5.1f}  yaw {rec.get('nav_yaw_deg', 0):5.0f}",
          x + 14, y + 44, 0.42, INK2)
    _text(img, f"current est  ({ce[0]:+.2f}, {ce[1]:+.2f}, {ce[2]:+.2f}) m/s", x + 14, y + 62, 0.42, INK2)
    dmin = rec.get("d_min")
    esc = rec.get("escape") or {}
    _text(img, f"nearest (conservative) d >= {_fmt(dmin)} m", x + 14, y + 80, 0.42,
          CRIT if dmin is not None and dmin < 1.7 else (SERIOUS if dmin is not None and dmin < 2.4 else INK2))
    if esc.get("dir"):
        _text(img, f"escape {esc['dir']}", x + 300, y + 80, 0.45, CRIT if mode == "COLLISION_AVOIDANCE" else SERIOUS)
    f = rec.get("form") or {}
    if f and show_form:
        _text(img, f"formation err est {_fmt(f.get('form_err'))} m  neighbours {'ok' if f.get('neighbors_ok') else 'missing'}",
              x + 14, y + 98, 0.42, INK2)
    _sector_glyph(img, rec, x + w - 190, y + h // 2 + 14)


def _legend(img, x, y):
    _text(img, "sonar sector: nearest echo range [m] + class", x, y - 4, 0.4, MUTED)
    xx = x
    for c in ("STRUCTURE", "SEABED", "DYNAMIC", "UNKNOWN", "UNCONFIRMED"):
        label = {"STRUCTURE": "STR structure (map)", "SEABED": "SEA seabed", "DYNAMIC": "DYN possible vehicle",
                 "UNKNOWN": "UNK unknown (treated as obstacle)", "UNCONFIRMED": "unc single capture"}[c]
        cv2.rectangle(img, (xx, y + 4), (xx + 14, y + 18), CLS_COLOR[c], -1)
        _text(img, label, xx + 18, y + 16, 0.38, INK2)
        xx += 26 + cv2.getTextSize(label, FONT, 0.38, 1)[0][0]


# ---------------------------------------------------------------------------------------------- referee
def _map(img, meta, ref, trails, x0, y0, w, h):
    cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), PANEL, -1)
    row = ref.get("row") or {}
    n = len(meta["names"])
    P = np.array([[row.get(f"x{k}", 0.0), row.get(f"y{k}", 0.0)] for k in range(n)], float)
    pts = [P]
    if meta.get("gate_pts") is not None:
        pts.append(np.asarray(meta["gate_pts"])[:, :2])
    allp = np.concatenate(pts)
    c = allp.mean(axis=0) if meta.get("map_center") is None else np.asarray(meta["map_center"], float)
    span = max(float(np.max(np.abs(allp - c))) * 2.0 + 6.0, meta.get("map_span", 18.0))
    s = min(w, h) / span

    def tx(q):
        return int(x0 + w / 2 + (q[0] - c[0]) * s), int(y0 + h / 2 - (q[1] - c[1]) * s)

    # 1 m grid
    for gx in range(int(c[0] - span), int(c[0] + span) + 1, 2):
        a, b = tx((gx, c[1] - span)), tx((gx, c[1] + span))
        cv2.line(img, (a[0], y0), (a[0], y0 + h), (40, 40, 38), 1)
    for gy in range(int(c[1] - span), int(c[1] + span) + 1, 2):
        a = tx((c[0], gy))
        cv2.line(img, (x0, a[1]), (x0 + w, a[1]), (40, 40, 38), 1)
    if meta.get("path") is not None:
        wp = np.asarray(meta["path"])
        for a, b in zip(wp[:-1], wp[1:]):
            cv2.line(img, tx(a), tx(b), LINE, 2)
    for poly in meta.get("cr_polys", []):
        cv2.polylines(img, [np.array([tx(q) for q in poly], np.int32)], True, _bgr("#5598e7"), 1)
    for poly in meta.get("bar_polys", []):
        cv2.fillPoly(img, [np.array([tx(q) for q in poly], np.int32)], INK2)
    for k in range(n):
        tr = trails[k] if trails else []
        for a, b in zip(tr[:-1], tr[1:]):
            cv2.line(img, tx(a), tx(b), DRONE[k % len(DRONE)], 1, cv2.LINE_AA)
    rs = meta.get("d_safe", 1.0) / 2.0 * s
    for k in range(n):
        q = tx(P[k])
        cv2.circle(img, q, max(3, int(rs)), DRONE[k % len(DRONE)], 1, cv2.LINE_AA)
        cv2.circle(img, q, 6, DRONE[k % len(DRONE)], -1, cv2.LINE_AA)
        _text(img, str(k), q[0] + 8, q[1] - 6, 0.5, INK)
    m = 0
    while f"ix{m}" in row:
        q = tx((row[f"ix{m}"], row[f"iy{m}"]))
        cv2.rectangle(img, (q[0] - 7, q[1] - 7), (q[0] + 7, q[1] + 7), INK2, -1)
        _text(img, "intruder", q[0] + 10, q[1] + 4, 0.45, INK2)
        m += 1
    # scale bar and current
    a = (x0 + 14, y0 + h - 14)
    cv2.line(img, a, (int(a[0] + 2 * s), a[1]), INK2, 2)
    _text(img, "2 m", a[0], a[1] - 6, 0.4, INK2)
    cur = ref.get("true_current")
    if cur is not None and np.linalg.norm(cur[:2]) > 0.02:
        o = (x0 + w - 60, y0 + 50)
        e = (int(o[0] + 60 * cur[0]), int(o[1] - 60 * cur[1]))
        cv2.arrowedLine(img, o, e, _bgr("#5598e7"), 2, cv2.LINE_AA, tipLength=0.3)
        _text(img, f"true current {np.linalg.norm(cur):.2f} m/s", x0 + w - 230, y0 + 20, 0.42, INK2)
    _text(img, "top view (ground truth), circles = d_safe / 2", x0 + 10, y0 + 18, 0.42, MUTED)


def _status(img, meta, ref, recs, x, y):
    live = ref.get("live") or {}
    sep = meta
    dtrue = live.get("d_min_true")
    d_on = min([r.get("d_min") for r in recs if r.get("d_min") is not None], default=None)
    ok = live.get("p1_ok", True)
    xx = _badge(img, "P1 SAFE" if ok else "P1 VIOLATED", x, y, GOOD if ok else CRIT, 0.55)
    _text(img, f"nearest pair, true {_fmt(dtrue)} m [REFEREE]   onboard conservative {_fmt(d_on)} m",
          xx, y, 0.45, INK)
    _text(img, f"d_warning {sep['d_warning']:.2f} (onboard guard)   d_ca {sep['d_ca']:.2f}   d_safe {sep['d_safe']:.2f} "
               f"(ground truth)   contacts {ref.get('contacts', 0)}", x, y + 24, 0.42, INK2)
    di = (ref.get("row") or {}).get("d_intruder")
    if di not in (None, ""):
        _text(img, f"intruder clearance (true) {float(di):.2f} m [REFEREE]", x + 470, y + 24, 0.42, INK)
    y2 = y + 56
    occ = live.get("occupancy") or {}
    if occ:
        p2ok = live.get("p2_ok", True)
        xx = _badge(img, "P2 OK" if p2ok else "P2 VIOLATED", x, y2, GOOD if p2ok else CRIT, 0.55)
        order = ref.get("entry_order") or {}
        _text(img, "  ".join(f"{g}: occupancy {o}  max {ref.get('max_occ', {}).get(g, o)}  order {'>'.join(nm[-1] for nm in order.get(g, []))}"
                             for g, o in occ.items()), xx, y2, 0.45, INK)
    else:
        _badge(img, "P2 n/a", x, y2, LINE, 0.55, text_color=INK2)
        _text(img, "no gate in this scenario", x + 90, y2, 0.45, MUTED)
    y3 = y2 + 34
    fst = live.get("formation")
    if fst is None:
        _badge(img, "P3 n/a", x, y3, LINE, 0.55, text_color=INK2)
        _text(img, "no formation judged in this scenario", x + 90, y3, 0.45, MUTED)
    else:
        label = {"OK": "P3 OK", "LOST": "P3 LOST", "RECOVERING": "P3 RECOVERING"}[fst]
        if fst == "OK" and live.get("episodes", 0) > 0:
            label = "P3 RECOVERED"
        col = {"OK": GOOD, "LOST": SERIOUS, "RECOVERING": WARN}[fst]
        xx = _badge(img, label, x, y3, col, 0.55)
        _text(img, f"formation error (true) {_fmt(live.get('form_err'))} m   lost > {meta['e_lost']:.1f}, "
                   f"recovered < {meta['e_ok']:.1f} for {meta['t_ok_hold']:.0f} s   episodes {live.get('episodes', 0)}",
              xx, y3, 0.45, INK)


# ---------------------------------------------------------------------------------------------- frame
def render(meta: Dict, recs: Sequence[Dict], ref: Dict, cam: Optional[np.ndarray] = None,
           trails: Optional[List[List]] = None) -> np.ndarray:
    img = np.full((H, W, 3), SURFACE, np.uint8)
    _text(img, meta.get("title", meta.get("scenario", "")), 16, 30, 0.75, INK, 2)
    _text(img, f"t = {meta.get('t', 0.0):5.1f} s / {meta.get('duration', 0):.0f} s", W - 250, 30, 0.7, INK, 2)
    _text(img, meta.get("focus", ""), 16, 48, 0.42, MUTED)
    # ---- onboard
    cv2.rectangle(img, (8, HEADER + 4), (SPLIT - 8, HEADER + 28), ONBOARD_HDR, -1)
    _text(img, "ONBOARD  -  what each drone knows (own sensors + mission plan)", 16, HEADER + 22, 0.55, INK, 1)
    n = len(recs)
    top = HEADER + 34
    avail = H - FOOTER - top - 6
    ch = int(min(132, (avail - 6 * (n - 1)) / max(n, 1)))
    for k, rec in enumerate(recs):
        _drone_card(img, k, meta["names"][k], rec, 8, top + k * (ch + 6), SPLIT - 16, ch, meta.get("formation", True))
    _legend(img, 16, H - FOOTER - 40)
    # ---- referee
    cv2.rectangle(img, (SPLIT + 4, HEADER + 4), (W - 8, HEADER + 28), REFEREE_HDR, -1)
    _text(img, "REFEREE / GROUND TRUTH  -  never sent to the drones", SPLIT + 12, HEADER + 22, 0.55, (16, 16, 16), 1)
    mx, my, mw, mh = SPLIT + 4, HEADER + 34, W - SPLIT - 12, 380
    _map(img, meta, ref, trails, mx, my, mw, mh)
    _status(img, meta, ref, recs, SPLIT + 12, my + mh + 30)
    if cam is not None:
        ih = H - FOOTER - (my + mh + 140) - 6
        iw = int(ih * cam.shape[1] / cam.shape[0])
        iw = min(iw, W - SPLIT - 16)
        ih = int(iw * cam.shape[0] / cam.shape[1])
        thumb = cv2.resize(cam[:, :, :3], (iw, ih))
        y0 = H - FOOTER - ih - 4
        img[y0:y0 + ih, SPLIT + 8:SPLIT + 8 + iw] = thumb
        _text(img, "chase camera (visualisation only)", SPLIT + 14, y0 + 16, 0.42, INK)
    # ---- footer
    cv2.rectangle(img, (0, H - FOOTER), (W, H), PANEL, -1)
    _text(img, f"COMMUNICATION: {meta.get('messages', 0)} messages     GROUND TRUTH USED BY CONTROLLERS: "
               f"{'YES' if meta.get('gt_used') else 'NO'}     {meta.get('n_sonars', 6 * n)} sonars, 10 Hz     "
               f"envelope: {meta.get('envelope', '-')}", 16, H - 12, 0.5, INK2)
    return img


def scenario_meta(sc, cfg, names) -> Dict:
    """Static part of the dashboard state (map geometry, thresholds) from a Scenario."""
    gate_pts, bar_polys, cr_polys = [], [], []
    G = cfg.gate
    for g in sc.judged_gates:
        for b in g.bars:
            corners = np.array([b.center + b.axes @ (b.half * np.array([sx, sy, sz]))
                                for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            hull = cv2.convexHull((corners[:, :2] * 1000).astype(np.float32)).reshape(-1, 2) / 1000.0
            bar_polys.append([q for q in hull])
            gate_pts += list(corners)
        cr = [g.from_gate_frame([a * G.cr_half_len, b * G.cr_half_width, 0.0])[:2] for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        cr_polys.append(cr)
    return {"scenario": sc.name, "title": sc.title, "focus": sc.focus, "duration": sc.duration_s, "names": list(names),
            "path": sc.path.waypoints, "gate_pts": np.array(gate_pts) if gate_pts else None, "bar_polys": bar_polys,
            "cr_polys": cr_polys, "d_safe": cfg.sep.d_safe, "d_warning": cfg.sep.d_warning, "d_ca": cfg.sep.d_ca,
            "e_lost": cfg.ref.e_lost, "e_ok": cfg.ref.e_ok, "t_ok_hold": cfg.ref.t_ok_hold,
            "messages": 0, "gt_used": False, "n_sonars": 6 * len(names), "formation": bool(sc.formation_enabled)}
