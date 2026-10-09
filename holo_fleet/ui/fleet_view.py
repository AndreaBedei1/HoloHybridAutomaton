"""Fleet-centred demo view (DI-30): one clean picture of the whole fleet for a supervisor.

* main map, top-down: the common survey path, the formation slots of the shared plan (empty rings; a
  slot declared vacant is marked), every drone at its true position coloured by its automaton state
  with a short recent trail, the gate with its critical region and the queue points numbered in
  precedence order (left first, then top first);  a side view (along-track vs depth) is added when
  the formation has depth layers;
* fleet panel: P1 / P2 / P3 in one line each, formation error, critical-region occupancy, queue
  progress (or deadlock risk), drones active / missing / assumed failed;
* one row per drone: state, ok / missing / rejoining / assumed failed, distance only when relevant;
* readable events ("D0: D2 missing for 37.5 s, slot vacant -> DEGRADED_FORMATION");
* a timeline of the automaton states of every drone.

Positions are simulator ground truth (for the viewer); states, missing / vacant slots and queue
decisions are what the drones' own automata and perception logged.  No drone receives anything from
this view.  The same class renders live (holo_fleet/ui/live.py) and from the logs of a finished run
(scripts/render_demo.py).
"""

from __future__ import annotations

import math
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H = 1600, 900
MAP = (24, 84, 1030, 690)            # x0, y0, x1, y1 of the map card
PANEL_X = 1048
TL = (24, 708, 1576, 884)           # timeline card


def _rgb(h: str):
    h = h.lstrip("#")
    return (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16))


BG = _rgb("#f4f3ef")
CARD = _rgb("#ffffff")
BORDER = _rgb("#dedcd4")
INK = _rgb("#1f1e1c")
INK2 = _rgb("#55534d")
MUTED = _rgb("#8e8c84")
GRID = _rgb("#ecebe5")
PATH = _rgb("#b9b7ae")
TRAIL = _rgb("#a7a59c")
GATE = _rgb("#3b3a36")
CR_FILL = _rgb("#cfe0f5")
ZONE_FILL = _rgb("#eef3fa")
OK_C, WARN_C, BAD_C, NA_C = _rgb("#2f9e5b"), _rgb("#d99a12"), _rgb("#c8322f"), _rgb("#9a988f")

SHORT = {"FORMATION_FOLLOW": "FOLLOW", "FORMATION_WAIT_REJOIN": "WAIT", "DEGRADED_FORMATION": "DEGRADED",
         "FORMATION_RECOVERY": "RECOVERY", "MUTEX_APPROACH": "APPROACH", "MUTEX_YIELD": "YIELD",
         "MUTEX_PASS": "PASS", "SEPARATION_WARNING": "WARNING", "COLLISION_AVOIDANCE": "AVOID",
         "FAILSAFE_HOLD_OR_RETREAT": "FAILSAFE"}
STATE_COLOR = {"FOLLOW": _rgb("#2f9e5b"), "WAIT": _rgb("#e0a21c"), "DEGRADED": _rgb("#8a63c9"),
               "RECOVERY": _rgb("#ef7d32"), "APPROACH": _rgb("#7fb2e5"), "YIELD": _rgb("#3d85c6"),
               "PASS": _rgb("#1c4e9c"), "WARNING": _rgb("#e8577a"), "AVOID": _rgb("#c62828"),
               "FAILSAFE": _rgb("#4b4a46")}
LEGEND = ["FOLLOW", "WAIT", "DEGRADED", "RECOVERY", "APPROACH", "YIELD", "PASS", "WARNING", "AVOID", "FAILSAFE"]
KEEPING = ("FORMATION_FOLLOW", "FORMATION_WAIT_REJOIN", "DEGRADED_FORMATION")


def _font(size: int, bold: bool = False):
    for p in ((r"C:\Windows\Fonts\segoeuib.ttf" if bold else r"C:\Windows\Fonts\segoeui.ttf"),
              ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf")):
        try:
            return ImageFont.truetype(p, size)
        except OSError:
            continue
    try:
        import matplotlib

        base = Path(matplotlib.get_data_path()) / "fonts" / "ttf"
        return ImageFont.truetype(str(base / ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf")), size)
    except Exception:                                   # pragma: no cover - last resort
        return ImageFont.load_default()


F = {k: _font(s, b) for k, (s, b) in {"title": (24, True), "sub": (14, False), "h": (15, True), "b": (14, False),
                                       "bb": (14, True), "s": (12, False), "sb": (12, True), "xs": (11, False),
                                       "chip": (12, True), "tag": (11, True)}.items()}


def _bgr(c):
    return (c[2], c[1], c[0])


def short(mode: str) -> str:
    return SHORT.get(mode, mode)


def dname(name: str) -> str:
    return "D" + name.split("_")[-1]


# ---------------------------------------------------------------------------------------------- events
def describe(e: Dict, slot_name: Dict[int, str]) -> Optional[str]:
    """One readable sentence for a logged event (None: not shown)."""
    d = dname(e["drone"]) if e.get("drone") else ""
    typ = e.get("type")
    sl = lambda k: dname(slot_name.get(int(k), f"slot {k}"))      # noqa: E731
    if typ == "FAULT_INJECTED":
        return f"[sim] {d} thrusters fail{' for good' if e.get('permanent') else ''} (no drone is told)"
    if typ == "FAULT_CLEARED":
        return f"[sim] {d} thrusters work again"
    if typ == "SLOT_DECLARED_VACANT":
        return f"{d}: {sl(e['slot'])} missing for {e.get('missing_s', 37.5):.1f} s, slot vacant -> DEGRADED_FORMATION"
    if typ == "SLOT_REOCCUPIED":
        return f"{d}: {sl(e['slot'])} back in its slot, slot re-included"
    if typ == "ENVELOPE_VIOLATION":
        return f"{d} cannot deliver its commands -> FAILSAFE"
    if typ == "ENVELOPE_OK":
        return f"{d} moves as commanded again (probe followed)"
    if typ == "OBSERVATION_INCONSISTENT":
        return f"{d}: inconsistent observation {e.get('violated')} -> FAILSAFE"
    if typ != "decision":
        return None
    dec, to = e.get("decision"), short(e.get("to", ""))
    text = {"NEIGHBOUR_MISSING": f"{d}: a neighbour is missing -> keeps its slot and waits (WAIT_REJOIN)",
            "NEIGHBOUR_REJOINED": f"{d}: missing neighbour back -> FOLLOW",
            "SLOT_VACANT": None,
            "FORMATION_RESTORED": f"{d}: formation complete again -> FOLLOW",
            "FORMATION_LOST": f"{d} off its slot -> RECOVERY (rejoins from behind)",
            "FORMATION_RECOVERED": f"{d} back on its slot -> {to}",
            "PASS": f"{d} has priority, critical region free -> passes the gate",
            "RETRY_PASS": f"{d} has priority, critical region free -> passes the gate",
            "YIELD": f"{d} at its queue point, yields",
            "EXITED": f"{d} through the gate -> re-forms beyond it",
            "APPROACH": None, "RESUME_PASS": f"{d} resumes its passage"}.get(dec)
    return text


class FleetView:
    """Accumulates the history of a run and renders the fleet view (RGB -> BGR numpy image)."""

    def __init__(self, scenario, cfg, names: Sequence[str], every_trail_s: float = 0.5, trail_s: float = 25.0):
        self.sc, self.cfg, self.names = scenario, cfg, list(names)
        self.n = len(self.names)
        self.plans = list(scenario.plans)
        self.slot_name = {p.slot_index: p.drone_id for p in self.plans} if scenario.template is not None else {}
        self.duration = float(scenario.duration_s)
        self.every_trail_s, self.trail_s = every_trail_s, trail_s
        self.trails: List[deque] = [deque() for _ in range(self.n)]
        self.timeline: List[List] = [[] for _ in range(self.n)]
        self.events: deque = deque(maxlen=7)
        self.last_trail_t = -1e9
        self.t = 0.0
        self.recs: List[Dict] = [{} for _ in range(self.n)]
        self.P = np.array([np.asarray(p, float) for p in scenario.sim.spawn_positions])
        self.yaw = np.array([math.radians(y) for y in scenario.sim.spawn_yaw_deg])
        self.ref: Dict = {}
        self.min_d = 1e9
        self.no_leader_since: Optional[float] = None
        self.entered: List[str] = []
        self.inside_prev: set = set()
        self.marks: List = []                               # (t, text) on the timeline
        self._ever_missing: set = set()                     # drones missed by a neighbour at some time (rejoining)
        G = cfg.gate
        self.G = G
        # view frame: along / lateral of the scenario path
        wp = scenario.path.waypoints
        self.o = np.asarray(wp[0], float)
        ex = np.asarray(wp[-1], float) - self.o
        self.ex = ex / np.linalg.norm(ex)
        self.ey = np.array([-self.ex[1], self.ex[0]])
        self.depth0 = scenario.path.depth_z
        dz = [sl.dz for sl in scenario.template.slots] if scenario.template is not None else [0.0]
        self.vertical = (max(dz) - min(dz) > 1.0) or bool(getattr(scenario.sim, "intruders", ()))
        self._fit()

    # ------------------------------------------------------------------ geometry
    def uv(self, p) -> np.ndarray:
        q = np.asarray(p, float)[:2] - self.o
        return np.array([q @ self.ex, q @ self.ey])

    def _fit(self) -> None:
        pts = [self.uv(p) for p in self.P]
        for pl in self.plans:
            if pl.clock is None:
                continue
            for t in np.linspace(0.0, self.duration, 25):
                s = pl.clock.s(t)
                for sl in pl.slots:
                    pp, _d, nn = pl.path.frame_at(s + sl.along)
                    pts.append(self.uv(pp + sl.lateral * nn))
        for g in self.sc.judged_gates:
            for a in (-9.0, 4.0):
                for b in (-6.0, 6.0):
                    pts.append(self.uv(g.from_gate_frame([a, b, 0.0])))
        for d in getattr(self.sc.sim, "intruders", ()) or ():
            for w in d.get("waypoints", []):
                pts.append(self.uv(w[1:3]))
        pts = np.array(pts)
        lo, hi = pts.min(axis=0) - 2.5, pts.max(axis=0) + 2.5
        x0, y0, x1, y1 = MAP
        top, bottom = y0 + 40, y1 - (150 if self.vertical else 34)
        self.scale = min((x1 - x0 - 48) / (hi[0] - lo[0]), (bottom - top) / (hi[1] - lo[1]))
        cu, cv = (lo + hi) / 2.0
        self.cx, self.cy = (x0 + x1) / 2.0, (top + bottom) / 2.0
        self.cu, self.cv = cu, cv
        self.side_y = y1 - 78                                # depth axis centre of the side view
        self.zscale = min(self.scale, 46.0 / 3.6)

    def px(self, p) -> tuple:
        u, v = self.uv(p)
        return (int(round(self.cx + (u - self.cu) * self.scale)), int(round(self.cy - (v - self.cv) * self.scale)))

    def pz(self, p) -> tuple:
        u, _v = self.uv(p)
        return (int(round(self.cx + (u - self.cu) * self.scale)),
                int(round(self.side_y - (float(p[2]) - self.depth0) * self.zscale)))

    # ------------------------------------------------------------------ state
    def step(self, t: float, recs: Sequence[Dict], positions, referee_live: Dict, events: Sequence[Dict] = (),
             yaws=None) -> None:
        self.t = t
        self.recs = [dict(r or {}) for r in recs]
        self.P = np.asarray(positions, float)
        if yaws is not None:
            self.yaw = np.asarray(yaws, float)
        else:
            self.yaw = np.array([math.radians(r.get("nav_yaw_deg", 0.0)) for r in self.recs])
        self.ref = dict(referee_live or {})
        if self.ref.get("d_min_true") is not None:
            self.min_d = min(self.min_d, float(self.ref["d_min_true"]))
        if t - self.last_trail_t >= self.every_trail_s:
            self.last_trail_t = t
            for k in range(self.n):
                self.trails[k].append((t, self.P[k].copy()))
                while self.trails[k] and t - self.trails[k][0][0] > self.trail_s:
                    self.trails[k].popleft()
        for k, r in enumerate(self.recs):
            st = short(r.get("mode", ""))
            if st and (not self.timeline[k] or self.timeline[k][-1][1] != st):
                self.timeline[k].append((t, st))
        for e in events:
            txt = describe(e, self.slot_name)
            if txt:
                self.events.append((float(e.get("t", t)), txt))
            if e.get("type") in ("FAULT_INJECTED", "SLOT_DECLARED_VACANT", "FAULT_CLEARED"):
                self.marks.append((float(e.get("t", t)), e.get("type")))
        for g in self.sc.judged_gates:
            G = self.G
            inside = {self.names[k] for k in range(self.n)
                      if (lambda q: abs(q[0]) <= G.cr_half_len and abs(q[1]) <= G.cr_half_width and abs(q[2]) <= G.cr_half_height)(
                          g.to_gate_frame(self.P[k]))}
            for nm in sorted(inside - self.inside_prev):
                if nm not in self.entered:
                    self.entered.append(nm)
            self.inside_prev = inside
        # queue progress: some queued drone may go, or nobody can (deadlock risk)
        miss, vac = self.beliefs()
        self._ever_missing |= miss | vac
        for k, r in enumerate(self.recs):
            if r.get("mode") in KEEPING and k not in miss and k not in vac:
                self._ever_missing.discard(k)
        queued = [k for k, r in enumerate(self.recs) if r.get("mode") == "MUTEX_YIELD"]
        passing = [k for k, r in enumerate(self.recs) if r.get("mode") == "MUTEX_PASS"]
        leader = [k for k in queued if (self.recs[k].get("gate") or {}).get("decision") == "PRIORITY"]
        if queued and not passing and not leader:
            self.no_leader_since = t if self.no_leader_since is None else self.no_leader_since
        else:
            self.no_leader_since = None

    # ------------------------------------------------------------------ fleet beliefs
    def beliefs(self):
        """missing[k] / vacant[k]: drone k is missing / declared vacant for at least one neighbour that is itself
        keeping the formation (the beliefs of a drone that is lost, avoiding or in FAILSAFE are not its
        neighbours' state: its timers do not run either)."""
        missing, vacant = set(), set()
        for r in self.recs:
            if r.get("mode") not in KEEPING:
                continue
            f = r.get("form") or {}
            for s in f.get("missing", []) or []:
                missing.add(self._drone_of_slot(s))
            for s in f.get("vacant", []) or []:
                vacant.add(self._drone_of_slot(s))
        missing.discard(None)
        vacant.discard(None)
        return missing - vacant, vacant

    def _drone_of_slot(self, s):
        nm = self.slot_name.get(int(s))
        return self.names.index(nm) if nm in self.names else None

    def status(self, k: int, missing, vacant) -> str:
        mode = self.recs[k].get("mode", "")
        if k in vacant:
            return "assumed failed"
        if k in missing:
            return "missing"
        if mode == "FORMATION_RECOVERY":
            return "rejoining" if k in self._ever_missing else "re-forming"
        if mode == "FAILSAFE_HOLD_OR_RETREAT":
            return "holding (failsafe)"
        return "ok"

    # ------------------------------------------------------------------ rendering
    def render(self) -> np.ndarray:
        img = np.full((H, W, 3), BG[::-1], np.uint8)
        self._cards_cv(img)
        x0, y0, x1, y1 = MAP
        layer = img.copy()
        self._map_cv(layer)
        img[y0 + 1:y1, x0 + 1:x1] = layer[y0 + 1:y1, x0 + 1:x1]          # the map stays inside its card
        self._timeline_cv(img)
        pil = Image.fromarray(img[:, :, ::-1])
        mp = Image.new("RGBA", (x1 - x0 - 2, y1 - y0 - 2), (0, 0, 0, 0))
        self._map_txt(ImageDraw.Draw(mp), (x0 + 1, y0 + 1))
        pil.paste(mp, (x0 + 1, y0 + 1), mp)
        dr = ImageDraw.Draw(pil)
        self._header_txt(dr)
        dr.text((x0 + 14, y0 + 10), "Fleet, top view", font=F["h"], fill=INK)
        dr.text((x0 + 150, y0 + 12), "true positions (simulator)  ·  states, missing / vacant slots and queue decisions: "
                "onboard", font=F["s"], fill=MUTED)
        self._panel(dr, pil)
        self._timeline_txt(dr)
        return np.asarray(pil)[:, :, ::-1].copy()

    def _card(self, img, x0, y0, x1, y1):
        cv2.rectangle(img, (x0, y0), (x1, y1), _bgr(CARD), -1)
        cv2.rectangle(img, (x0, y0), (x1, y1), _bgr(BORDER), 1)

    def _cards_cv(self, img):
        self._card(img, *MAP)
        self._card(img, PANEL_X, 84, W - 24, 330)
        self._card(img, PANEL_X, 342, W - 24, 512)
        self._card(img, PANEL_X, 524, W - 24, 690)
        self._card(img, *TL)

    def _header_txt(self, dr):
        dr.text((24, 14), self.sc.title, font=F["title"], fill=INK)
        sub = self.sc.focus or self.sc.description
        dr.text((24, 50), sub[:150] + ("..." if len(sub) > 150 else ""), font=F["sub"], fill=INK2)
        tt = f"t = {self.t:5.1f} s / {self.duration:.0f} s"
        w = dr.textlength(tt, font=F["h"])
        dr.text((W - 24 - w, 16), tt, font=F["h"], fill=INK)
        chip = "no communication  ·  onboard sensors + shared mission plan"
        w2 = dr.textlength(chip, font=F["s"]) + 18
        dr.rounded_rectangle((W - 24 - w2, 44, W - 24, 66), radius=11, fill=_rgb("#e7efe9"), outline=_rgb("#bcd3c3"))
        dr.text((W - 24 - w2 + 9, 47), chip, font=F["s"], fill=_rgb("#2c6b45"))

    # ---------------------------------------------------------------- map
    def _map_cv(self, img):
        x0, y0, x1, y1 = MAP
        sub = img[y0 + 1:y1, x0 + 1:x1]
        # grid every 5 m
        u0 = self.cu - (self.cx - x0) / self.scale
        u1 = self.cu + (x1 - self.cx) / self.scale
        for u in np.arange(math.floor(u0 / 5.0) * 5.0, u1, 5.0):
            X = int(round(self.cx + (u - self.cu) * self.scale))
            if x0 + 2 < X < x1 - 2:
                cv2.line(img, (X, y0 + 34), (X, y1 - (140 if self.vertical else 26)), _bgr(GRID), 1)
        # mutex zone: approach band, CR, gate bars, queue points
        G = self.G
        for g in self.sc.judged_gates:
            q_s = self.plans[0].queue_s
            half_w = max(G.corridor_half_width, max(abs(p.queue_lateral) for p in self.plans) + 1.0)
            band = [g.from_gate_frame([a, b, 0.0]) for a, b in ((q_s - G.approach_len, -half_w), (G.exit_s, -half_w),
                                                                 (G.exit_s, half_w), (q_s - G.approach_len, half_w))]
            cv2.fillPoly(img, [np.array([self.px(p) for p in band], np.int32)], _bgr(ZONE_FILL), cv2.LINE_AA)
            cr = [g.from_gate_frame([a * G.cr_half_len, b * G.cr_half_width, 0.0]) for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            cv2.fillPoly(img, [np.array([self.px(p) for p in cr], np.int32)], _bgr(CR_FILL), cv2.LINE_AA)
            cv2.polylines(img, [np.array([self.px(p) for p in cr], np.int32)], True, _bgr(_rgb("#7ea6d8")), 1, cv2.LINE_AA)
            for b in g.bars:
                corners = np.array([b.center + b.axes @ (b.half * np.array([sx, sy, sz]))
                                    for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
                hull = cv2.convexHull(np.array([self.px(c) for c in corners], np.int32))
                cv2.fillPoly(img, [hull], _bgr(GATE), cv2.LINE_AA)
            if self.vertical:
                top = [g.from_gate_frame([0.0, 0.0, z]) for z in (-0.93, 0.93)]
                for zz in (-0.93, 0.93):
                    a = self.pz(g.from_gate_frame([-0.11, 0.0, zz]))
                    cv2.rectangle(img, (a[0] - 2, a[1] - 2), (a[0] + 4, a[1] + 2), _bgr(GATE), -1)
                _ = top
        # common path (dashed)
        wp = self.sc.path.waypoints
        a, b = np.asarray(wp[0], float), np.asarray(wp[-1], float)
        L = float(np.linalg.norm(b - a))
        for s in np.arange(0.0, L, 1.2):
            p1, p2 = a + (b - a) * s / L, a + (b - a) * min(s + 0.6, L) / L
            cv2.line(img, self.px(p1), self.px(p2), _bgr(PATH), 2, cv2.LINE_AA)
        if self.vertical:
            yz = self.side_y
            cv2.line(img, (x0 + 14, yz + int(self.zscale * 0)), (x1 - 14, yz), _bgr(GRID), 1)
        # planned slots (shared plan, clock s(t)), vacant ones marked
        _missing, vacant = self.beliefs()
        for pl in self.plans:
            if pl.clock is None or self.sc.template is None:
                continue
            s = pl.clock.s(self.t)
            for k, sl in enumerate(pl.slots):
                pp, _d, nn = pl.path.frame_at(s + sl.along)
                p3 = np.array([pp[0] + sl.lateral * nn[0], pp[1] + sl.lateral * nn[1], pl.path.depth_z + sl.dz])
                k_dr = self._drone_of_slot(k)
                col = BAD_C if k_dr in vacant else _rgb("#9c9a92")
                c = self.px(p3)
                self._ring(img, c, 11, col, dashed=k_dr in vacant)
                if self.vertical:
                    self._ring(img, self.pz(p3), 9, col, dashed=k_dr in vacant)
            break                                               # one shared plan
        # trails
        for k in range(self.n):
            pts = [self.px(p) for _t, p in self.trails[k]] + [self.px(self.P[k])]
            for i in range(1, len(pts)):
                f = i / len(pts)
                c = tuple(int(BG[j] + (TRAIL[j] - BG[j]) * (0.25 + 0.75 * f)) for j in range(3))
                cv2.line(img, pts[i - 1], pts[i], _bgr(c), 2, cv2.LINE_AA)
        # drones
        missing, vacant = self.beliefs()
        r_px = max(7, int(round(0.38 * self.scale)))
        for k in range(self.n):
            st = short(self.recs[k].get("mode", "FORMATION_FOLLOW"))
            col = STATE_COLOR.get(st, MUTED)
            for (c, yawd) in ((self.px(self.P[k]), True),) + (((self.pz(self.P[k]), False),) if self.vertical else ()):
                if k in missing or k in vacant:
                    self._ring(img, c, r_px + 7, BAD_C if k in vacant else WARN_C, dashed=True, thick=2)
                cv2.circle(img, c, r_px + 2, (255, 255, 255), -1, cv2.LINE_AA)
                cv2.circle(img, c, r_px, _bgr(col), -1, cv2.LINE_AA)
                if yawd:
                    yaw = float(self.yaw[k]) if k < len(self.yaw) else 0.0
                    hv = np.array([math.cos(yaw), math.sin(yaw)])
                    dv = np.array([hv @ self.ex, hv @ self.ey])
                    tip = (int(c[0] + dv[0] * (r_px + 7)), int(c[1] - dv[1] * (r_px + 7)))
                    cv2.line(img, c, tip, _bgr(INK), 2, cv2.LINE_AA)
                if k in vacant:
                    d = r_px - 2
                    cv2.line(img, (c[0] - d, c[1] - d), (c[0] + d, c[1] + d), (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.line(img, (c[0] - d, c[1] + d), (c[0] + d, c[1] - d), (255, 255, 255), 2, cv2.LINE_AA)
        # intruder (scripted, not part of the fleet)
        row = (self.ref or {}).get("row") or {}
        for m in range(3):
            if f"ix{m}" in row:
                q = np.array([float(row[f"ix{m}"]), float(row[f"iy{m}"]), float(row[f"iz{m}"])])
                c = self.px(q)
                cv2.drawMarker(img, c, _bgr(BAD_C), cv2.MARKER_DIAMOND, 18, 2, cv2.LINE_AA)
                if self.vertical:
                    cv2.drawMarker(img, self.pz(q), _bgr(BAD_C), cv2.MARKER_DIAMOND, 14, 2, cv2.LINE_AA)
        # scale bar 5 m
        L5 = int(round(5.0 * self.scale))
        bx, by = x0 + 20, y1 - 16
        cv2.line(img, (bx, by), (bx + L5, by), _bgr(INK2), 2, cv2.LINE_AA)
        for xx in (bx, bx + L5):
            cv2.line(img, (xx, by - 4), (xx, by + 4), _bgr(INK2), 2, cv2.LINE_AA)
        _ = sub

    def _ring(self, img, c, r, col, dashed=False, thick=1):
        if not dashed:
            cv2.circle(img, c, r, _bgr(col), thick, cv2.LINE_AA)
            return
        for a in range(0, 360, 30):
            cv2.ellipse(img, c, (r, r), 0, a, a + 18, _bgr(col), thick, cv2.LINE_AA)

    def _map_txt(self, dr0, off):
        """Map labels, drawn on a transparent layer the size of the map card (clipped); ``off`` = its origin."""
        x0, y0, x1, y1 = MAP

        class _D:                                           # global coordinates -> layer coordinates
            def __init__(self, d):
                self.d = d

            def textlength(self, *a, **k):
                return self.d.textlength(*a, **k)

            def text(self, xy, *a, **k):
                return self.d.text((xy[0] - off[0], xy[1] - off[1]), *a, **k)

            def rounded_rectangle(self, box, *a, **k):
                return self.d.rounded_rectangle((box[0] - off[0], box[1] - off[1], box[2] - off[0], box[3] - off[1]),
                                                *a, **k)

        dr = _D(dr0)
        dr.text((x0 + 22, y1 - 36), "5 m", font=F["xs"], fill=INK2)
        if self.vertical:
            dr.text((x0 + 14, y1 - 140), "side view (along-track vs depth)", font=F["s"], fill=MUTED)
        # queue points numbered in precedence order (left first, then top first), grouped when stacked
        G = self.G
        for g in self.sc.judged_gates:
            groups: Dict = {}
            for p in self.plans:
                key = round(p.queue_lateral, 2)
                groups.setdefault(key, []).append(p)
            for key, ps in groups.items():
                ps = sorted(ps, key=lambda p: -p.queue_dz)
                q = g.from_gate_frame([ps[0].queue_s, ps[0].queue_lateral, 0.0])
                c = self.px(q)
                label = "/".join(str(p.static_rank + 1) for p in ps)
                w = dr.textlength(label, font=F["tag"]) + 10
                dr.rounded_rectangle((c[0] - w / 2, c[1] - 8, c[0] + w / 2, c[1] + 8), radius=8, fill=_rgb("#ffffff"),
                                     outline=_rgb("#7ea6d8"))
                dr.text((c[0] - w / 2 + 5, c[1] - 8), label, font=F["tag"], fill=_rgb("#2f5f9e"))
                if self.vertical:
                    for p in ps:
                        cz = self.pz(g.from_gate_frame([p.queue_s, p.queue_lateral, p.queue_dz]))
                        dr.text((cz[0] - 4, cz[1] - 8), str(p.static_rank + 1), font=F["tag"], fill=_rgb("#2f5f9e"))
            gc = self.px(g.center)
            dr.text((gc[0] - 30, gc[1] - 62), f"gate {g.gate_id}", font=F["sb"], fill=INK2)
            dr.text((gc[0] - 40, gc[1] + 34), "critical region", font=F["xs"], fill=_rgb("#5b7fae"))
            _ = G
        # drone labels
        missing, vacant = self.beliefs()
        for k in range(self.n):
            c = self.px(self.P[k])
            txt = dname(self.names[k])
            dr.rounded_rectangle((c[0] + 11, c[1] - 26, c[0] + 15 + dr.textlength(txt, font=F["tag"]) + 6, c[1] - 10),
                                 radius=6, fill=_rgb("#ffffff"), outline=BORDER)
            dr.text((c[0] + 14, c[1] - 26), txt, font=F["tag"], fill=INK)
            note = "assumed failed" if k in vacant else ("missing" if k in missing else "")
            if note:
                dr.text((c[0] + 12, c[1] + 10), note, font=F["xs"], fill=BAD_C if k in vacant else WARN_C)
            if self.vertical:
                cz = self.pz(self.P[k])
                dr.text((cz[0] + 10, cz[1] - 18), txt, font=F["tag"], fill=INK2)

    # ---------------------------------------------------------------- panel
    def _chip(self, dr, x, y, text, col):
        w = dr.textlength(text, font=F["chip"]) + 16
        dr.rounded_rectangle((x, y, x + w, y + 20), radius=10, fill=col)
        dr.text((x + 8, y + 2), text, font=F["chip"], fill=(255, 255, 255))
        return x + w + 8

    def _panel(self, dr, pil):
        x = PANEL_X + 16
        r = self.ref or {}
        recs = self.recs
        missing, vacant = self.beliefs()
        modes = [rr.get("mode", "") for rr in recs]
        # ---- fleet card
        dr.text((x, 96), "Fleet status", font=F["h"], fill=INK)
        y = 124
        sep = cfg_sep = self.cfg.sep
        warn = [dname(self.names[k]) for k, m in enumerate(modes) if m in ("SEPARATION_WARNING", "COLLISION_AVOIDANCE")]
        p1_ok = r.get("p1_ok", True)
        chip, col = (("VIOLATION", BAD_C) if not p1_ok else (("WARNING", WARN_C) if warn else ("OK", OK_C)))
        self._row(dr, x, y, "P1  separation", chip, col,
                  (f"min {self.min_d:.2f} m so far (>= {sep.d_safe:.1f} m)" if self.min_d < 1e8 else "")
                  + (f" · warning: {', '.join(warn)}" if warn else ""))
        y += 34
        if self.sc.judged_gates:
            occ = sum((r.get("occupancy") or {}).values()) if r.get("occupancy") else 0
            ok2 = r.get("p2_ok", True)
            self._row(dr, x, y, "P2  mutual exclusion", "OK" if ok2 else "VIOLATION", OK_C if ok2 else BAD_C,
                      f"critical region {occ}/1 · entered: {', '.join(dname(nm) for nm in self.entered) or '-'}")
        else:
            self._row(dr, x, y, "P2  mutual exclusion", "n/a", NA_C, "no gate in this scenario")
        y += 34
        if self.sc.formation_enabled and self.sc.template is not None:
            absent = r.get("absent") or []
            fe = r.get("form_err")
            if vacant or absent:
                fp = r.get("form_err_present")
                chip, col = ("DEGRADED", STATE_COLOR["DEGRADED"])
                who = ', '.join(dname(self.names[k]) for k in sorted(vacant)) or ', '.join(dname(a) for a in absent)
                det = f"slot of {who} vacant · others err {fp:.2f} m" if fp is not None else f"slot of {who} vacant"
            elif any(m.startswith("MUTEX") for m in modes):
                chip, col, det = "GATE", NA_C, "planned: one at a time, re-forms beyond the gate"
            elif r.get("formation") == "LOST":
                chip, col, det = "LOST", WARN_C, f"error {fe:.2f} m (> {self.cfg.ref.e_lost} m)" if fe is not None else ""
            elif r.get("formation") == "RECOVERING":
                chip, col, det = "RE-FORMING", WARN_C, f"error {fe:.2f} m" if fe is not None else ""
            else:
                chip, col, det = "OK", OK_C, f"error {fe:.2f} m" if fe is not None else ""
            self._row(dr, x, y, "P3  formation", chip, col, det)
        else:
            self._row(dr, x, y, "P3  formation", "n/a", NA_C, "no formation in this scenario")
        y += 34
        if self.sc.judged_gates:
            queued = [k for k, m in enumerate(modes) if m == "MUTEX_YIELD"]
            passing = [k for k, m in enumerate(modes) if m == "MUTEX_PASS"]
            lead = [k for k in queued if (recs[k].get("gate") or {}).get("decision") == "PRIORITY"]
            if self.no_leader_since is not None and self.t - self.no_leader_since > 5.0:
                chip, col, det = "DEADLOCK RISK", BAD_C, f"nobody has priority for {self.t - self.no_leader_since:.0f} s"
            elif passing:
                chip, col = "PROGRESS", OK_C
                det = f"{', '.join(dname(self.names[k]) for k in passing)} passing" + (
                    f" · next: {dname(self.names[lead[0]])}" if lead else (f" · {len(queued)} queued" if queued else ""))
            elif lead:
                chip, col, det = "PROGRESS", OK_C, f"next: {dname(self.names[lead[0]])} (left first, then top)"
            elif queued:
                chip, col, det = "WAITING", WARN_C, "queue forming"
            else:
                chip, col, det = "IDLE", NA_C, "no drone queued"
            self._row(dr, x, y, "Queue", chip, col, det)
            y += 34
        active = [k for k in range(self.n) if modes[k] != "FAILSAFE_HOLD_OR_RETREAT" and k not in vacant]
        dr.text((x, y + 4), f"{self.n} drones  ·  {len(active)} active  ·  {len(missing)} missing  ·  "
                            f"{len(vacant)} assumed failed", font=F["bb"], fill=INK)
        # ---- drones card
        dr.text((x, 354), "Drones", font=F["h"], fill=INK)
        y = 380
        rowh = min(26, int(122 / max(1, self.n)))
        for k in range(self.n):
            st = short(modes[k]) or "-"
            col = STATE_COLOR.get(st, MUTED)
            dr.ellipse((x, y + 4, x + 12, y + 16), fill=col)
            dr.text((x + 20, y), dname(self.names[k]), font=F["bb"], fill=INK)
            self._chip(dr, x + 56, y, st, col)
            stt = self.status(k, missing, vacant)
            dr.text((x + 170, y + 1), stt, font=F["b"], fill=BAD_C if stt == "assumed failed" else (
                WARN_C if stt in ("missing", "rejoining") else INK2))
            dmin = recs[k].get("d_min")
            if dmin is not None and (dmin < self.cfg.sep.d_warning or modes[k] in ("SEPARATION_WARNING",
                                                                                  "COLLISION_AVOIDANCE")):
                dr.text((x + 330, y + 1), f"nearest {dmin:.1f} m", font=F["b"], fill=WARN_C)
            y += rowh
        # ---- events card
        dr.text((x, 536), "Events", font=F["h"], fill=INK)
        y = 562
        for te, txt in list(self.events)[-6:][::-1]:
            dr.text((x, y), f"{te:5.1f} s", font=F["sb"], fill=MUTED)
            dr.text((x + 52, y), txt[:68] + ("..." if len(txt) > 68 else ""), font=F["s"], fill=INK2)
            y += 20
        _ = (pil, cfg_sep)

    def _row(self, dr, x, y, label, chip, col, detail):
        dr.text((x, y + 1), label, font=F["bb"], fill=INK)
        nx = self._chip(dr, x + 160, y, chip, col)
        dr.text((nx, y + 2), detail[:52] + ("..." if len(detail) > 52 else ""), font=F["s"], fill=INK2)

    # ---------------------------------------------------------------- timeline
    def _tl_geom(self):
        x0, y0, x1, y1 = TL
        lx0, lx1 = x0 + 64, x1 - 16
        lane_h = min(18, int((y1 - y0 - 60) / max(1, self.n)))
        return lx0, lx1, y0 + 34, lane_h

    def _timeline_cv(self, img):
        lx0, lx1, ly0, lh = self._tl_geom()
        T = max(self.duration, 1e-6)
        tx = lambda t: int(round(lx0 + (lx1 - lx0) * min(max(t, 0.0), T) / T))     # noqa: E731
        for k in range(self.n):
            y = ly0 + k * (lh + 4)
            cv2.rectangle(img, (lx0, y), (lx1, y + lh), _bgr(GRID), -1)
            segs = self.timeline[k]
            for i, (t0, st) in enumerate(segs):
                t1 = segs[i + 1][0] if i + 1 < len(segs) else self.t
                cv2.rectangle(img, (tx(t0), y), (max(tx(t1), tx(t0) + 1), y + lh), _bgr(STATE_COLOR.get(st, MUTED)), -1)
        for tm, typ in self.marks:
            X = tx(tm)
            cv2.line(img, (X, ly0 - 6), (X, ly0 + self.n * (lh + 4)), _bgr(BAD_C if typ != "FAULT_CLEARED" else OK_C), 1,
                     cv2.LINE_AA)
        X = tx(self.t)
        cv2.line(img, (X, ly0 - 8), (X, ly0 + self.n * (lh + 4) + 2), _bgr(INK), 2, cv2.LINE_AA)

    def _timeline_txt(self, dr):
        x0, y0, x1, y1 = TL
        lx0, lx1, ly0, lh = self._tl_geom()
        dr.text((x0 + 14, y0 + 8), "Automaton states", font=F["h"], fill=INK)
        lx = x0 + 170
        for st in LEGEND:
            dr.rectangle((lx, y0 + 13, lx + 12, y0 + 25), fill=STATE_COLOR[st])
            dr.text((lx + 16, y0 + 10), st, font=F["xs"], fill=INK2)
            lx += 26 + dr.textlength(st, font=F["xs"]) + 8
        for k in range(self.n):
            y = ly0 + k * (lh + 4)
            dr.text((x0 + 18, y + lh / 2 - 8), dname(self.names[k]), font=F["sb"], fill=INK)
        T = self.duration
        for tt in np.arange(0.0, T + 1e-6, 10.0):
            X = lx0 + (lx1 - lx0) * tt / T
            dr.text((X - 8, ly0 + self.n * (lh + 4) + 2), f"{tt:.0f}", font=F["xs"], fill=MUTED)
        if self.marks:
            dr.text((lx1 - 330, y0 + 10), "| red: fault injected / slot declared vacant   green: fault cleared",
                    font=F["xs"], fill=MUTED)
