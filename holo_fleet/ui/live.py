"""Live demo hook for :func:`holo_fleet.runner.run`: fleet view window and frames, viewport drawing.

The hook only *reads*: controller records and events (onboard knowledge), the referee (ground truth
for the viewer) and draws the onboard belief in the HoloOcean viewport.  It never writes into a
controller.  The fleet view (holo_fleet/ui/fleet_view.py) is the same one scripts/render_demo.py
rebuilds from the logs.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Dict, List

import cv2
import numpy as np

from holo_fleet.perception.sonar_geometry import AXES
from holo_fleet.ui.fleet_camera import FleetCamera
from holo_fleet.ui.fleet_view import FleetView

VP_CLS = {"STRUCTURE": [150, 150, 150], "SEABED": [140, 109, 70], "DYNAMIC": [230, 40, 40], "UNKNOWN": [250, 178, 25]}
VP_ESCAPE = [230, 60, 230]


def _arrow_cursor(title: str) -> None:
    """Give an OpenCV window the normal arrow cursor.

    OpenCV's Win32 backend registers its window classes ("Main HighGUI class", "HighGUI class") with
    the crosshair cursor (IDC_CROSS), which makes the fleet view look like a drawing canvas.  Cosmetic
    only: a no-op on other platforms or if anything fails.
    """
    if sys.platform != "win32":
        return
    try:
        import ctypes

        u = ctypes.windll.user32
        u.FindWindowW.restype = ctypes.c_void_p
        u.FindWindowW.argtypes = [ctypes.c_wchar_p, ctypes.c_wchar_p]
        u.FindWindowExW.restype = ctypes.c_void_p
        u.FindWindowExW.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_wchar_p, ctypes.c_wchar_p]
        u.LoadCursorW.restype = ctypes.c_void_p
        u.LoadCursorW.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        u.SetClassLongPtrW.restype = ctypes.c_void_p
        u.SetClassLongPtrW.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
        main = u.FindWindowW("Main HighGUI class", title)
        if not main:
            return
        arrow = u.LoadCursorW(None, 32512)
        for h in (main, u.FindWindowExW(main, None, "HighGUI class", None)):
            if h:
                u.SetClassLongPtrW(h, -12, arrow)
    except Exception:  # noqa: BLE001 - cosmetic only
        pass


# cv2.waitKeyEx codes (Windows) and letter fallbacks; replay moves in control steps (0.1 s)
KEY_STEPS = {2424832: -1, 2555904: 1, 2490368: 10, 2621440: -10, 2162688: 100, 2228224: -100,
             ord("a"): -1, ord("d"): 1, ord("w"): 10, ord("s"): -10}
KEY_HOME, KEY_END, KEY_SPACE = (2359296, ord("h")), (2293760, ord("e")), ord(" ")
KEY_QUIT = (ord("q"), 27)
CHECKPOINT_EVERY = 10
BAR_H = 30


class DemoUI:
    """Fleet view window of a run.  With replay=True every step is kept: the view can be moved back and forth while the
    run goes on and after it ends, and the window stays open until it is closed (X, q or Esc)."""

    def __init__(self, scenario, cfg, names, show: bool = True, draw_viewport: bool = True,
                 window: str = "holo_fleet - fleet view", selected=None, replay: bool = False):
        self.view = FleetView(scenario, cfg, names, selected=selected)
        self.names = list(names)
        self.show = show
        self.draw_viewport = draw_viewport
        self.window = window
        self.frames_written = 0
        self._closed = False
        self._window_created = False
        self._camera = FleetCamera(scenario.path)
        self._last_camera_t = None
        self.replay = replay
        self.history: List[tuple] = []
        self.checkpoints: Dict[int, Dict] = {}
        self.cursor = None
        self.finished = False
        self._rview = None
        self._dirty = False

    def __call__(self, sim, scenario, ctrls, referee, truth, t, out, save_frame, events=()) -> None:
        recs = [ctrls[nm].last_record for nm in self.names]
        live = dict(referee.live(), row=referee.rows[-1] if referee.rows else {})
        self.view.step(t, recs, truth.positions, live, list(events))
        if self.replay:
            self.history.append((t, [dict(r or {}) for r in recs], np.asarray(truth.positions, float).copy(), live,
                                 list(events)))
            if (len(self.history) - 1) % CHECKPOINT_EVERY == 0:
                self.checkpoints[len(self.history) - 1] = self.view.snapshot()
        if self.draw_viewport and not sim.headless:
            self._viewport(sim, recs, scenario, truth, t)
        if not (self.show or save_frame):
            return
        img = self.view.render() if (save_frame or self.cursor is None) else None
        if save_frame:
            d = Path(out) / "fleet_view"
            d.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(d / f"fleet_{int(round(t * 10)):05d}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
            self.frames_written += 1
        if self.show:
            try:
                self._ensure_window()
                if not self.replay:
                    cv2.imshow(self.window, img)
                    self._key(cv2.waitKey(1) & 0xFF)
                elif self.cursor is None:
                    cv2.imshow(self.window, self._with_bar(img))
                    self._key(cv2.waitKeyEx(1))
                else:
                    self._key(cv2.waitKeyEx(1))
                    self._show_replay()
                self._check_window_open()
            except cv2.error:
                self.show = False

    def _key(self, key: int) -> None:
        if key is None or key < 0:
            return
        if ord("0") <= key <= ord("9"):
            if self.view.select_drone(key - ord("0")):
                self._dirty = True
            return
        if not self.replay or not self.history:
            return
        last = len(self.history) - 1
        cur = last if self.cursor is None else self.cursor
        if key == KEY_SPACE:
            self.cursor = cur if self.cursor is None else None
            self._dirty = True
            return
        if key in KEY_STEPS:
            cur = int(np.clip(cur + KEY_STEPS[key], 0, last))
        elif key in KEY_HOME:
            cur = 0
        elif key in KEY_END:
            cur = last
        else:
            return
        self.cursor = None if (key in KEY_END and not self.finished) else cur
        self._dirty = True

    def frame_at(self, i: int) -> np.ndarray:
        """The fleet view as it was after step i of the run: the nearest checkpoint, then the logged steps after it."""
        if self._rview is None:
            import copy

            self._rview = copy.copy(self.view)
        ck = max(c for c in self.checkpoints if c <= i)
        rv = self._rview
        rv.restore(self.checkpoints[ck])
        for j in range(ck + 1, i + 1):
            t, recs, P, live, events = self.history[j]
            rv.step(t, recs, P, live, events)
        rv.sel = self.view.sel
        return rv.render()

    def _with_bar(self, img: np.ndarray) -> np.ndarray:
        """The fleet view with a status bar below it (below the image: clicks keep their coordinates)."""
        bar = np.full((BAR_H, img.shape[1], 3), 32, np.uint8)
        last = len(self.history) - 1
        i = last if self.cursor is None else self.cursor
        state = "REPLAY" if self.cursor is not None else ("END" if self.finished else "LIVE")
        t_now = self.history[i][0] if self.history else 0.0
        t_end = self.history[last][0] if self.history else 0.0
        txt = (f"{state}  t = {t_now:6.1f} / {t_end:6.1f} s{'  (run finished)' if self.finished else ''}    "
               "<- -> 0.1 s   up/down 1 s   PgUp/PgDn 10 s   Home/End   Space live/pause   0-9 drone   q/Esc/X close")
        cv2.putText(bar, txt, (10, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (230, 230, 230), 1, cv2.LINE_AA)
        if self.history:
            x = int(round(10 + (img.shape[1] - 20) * (i / max(1, last))))
            cv2.line(bar, (10, BAR_H - 3), (img.shape[1] - 10, BAR_H - 3), (90, 90, 90), 2)
            cv2.line(bar, (10, BAR_H - 3), (x, BAR_H - 3), (80, 170, 250), 2)
        return np.vstack([img, bar])

    def _show_replay(self) -> None:
        if not self._dirty or not self.history:
            return
        self._dirty = False
        i = len(self.history) - 1 if self.cursor is None else self.cursor
        img = self.frame_at(i) if (self.cursor is not None or self.finished) else self.view.render()
        cv2.imshow(self.window, self._with_bar(img))

    def hold(self) -> None:
        """After the run: keep the window open and the view movable back and forth until it is closed (X, q, Esc)."""
        self.finished = True
        if not (self.replay and self.show and self._window_created and self.history):
            self.close(force=True)
            return
        self._dirty = True
        try:
            while self.show:
                self._show_replay()
                key = cv2.waitKeyEx(30)
                if key in KEY_QUIT:
                    break
                self._key(key)
                self._check_window_open()
        except (cv2.error, KeyboardInterrupt):
            pass
        self.close(force=True)

    def _ensure_window(self) -> None:
        """A plain fixed-size window (1:1 pixels, so a click maps directly to the fleet-view image)."""
        if self._window_created or self._closed:
            return
        cv2.namedWindow(self.window, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.window, self._mouse)
        self._window_created = True
        _arrow_cursor(self.window)

    def _mouse(self, event, x, y, flags, _param=None):
        if event == cv2.EVENT_LBUTTONDOWN and self.show:
            shown = self._rview if (self.cursor is not None and self._rview is not None) else self.view
            k = shown.select_at(x, y)
            if k is not None:
                self.view.select_drone(k)
                self._dirty = True

    def _check_window_open(self):
        if not self.show or not self._window_created:
            return
        try:
            if cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1:
                self.show = False
        except cv2.error:
            self.show = False

    def close(self, force: bool = False) -> None:
        """Stop drawing and close this window; with replay the end of the run keeps it open for hold() (unless
        force).  Safe to call more than once."""
        if self.replay and not force and self.show and self._window_created and not self._closed:
            self.finished = True
            return
        if self._closed:
            return
        self._closed = True
        self.show = False
        if self._window_created:
            try:
                cv2.destroyWindow(self.window)
                cv2.waitKey(1)
            except cv2.error:
                pass
            self._window_created = False

    def _viewport(self, sim, recs, scenario, truth, t) -> None:
        """Onboard belief drawn at the drone's ESTIMATED pose: nearest echo per sector, escape arrow."""
        for rec in recs:
            p = np.asarray(rec.get("nav_p") or [0, 0, 0], float)
            yaw = math.radians(rec.get("nav_yaw_deg", 0.0))
            R = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0, 0, 1.0]])
            for s, sec in (rec.get("sectors") or {}).items():
                ech = [e for e in sec.get("echoes", []) if e[1] in VP_CLS]
                if not ech:
                    continue
                r, cls = min(ech, key=lambda e: e[0])
                sim.draw_line(p, p + R @ AXES[s] * r, VP_CLS[cls], 6.0, 0.12)
            esc = rec.get("escape") or {}
            v = np.asarray(rec.get("v_cmd") or [0, 0, 0], float)
            if esc.get("dir") and np.linalg.norm(v) > 0.05:
                sim.draw_arrow(p, p + 3.0 * v / max(np.linalg.norm(v), 1e-6), VP_ESCAPE, 10.0, 0.12)
        G = sim.cfg.gate
        for g in scenario.judged_gates:
            sim.draw_box(g.center, [G.cr_half_len, G.cr_half_width, G.cr_half_height], [60, 130, 230], 4.0, 0.12)
        if self._last_camera_t is None or t - self._last_camera_t >= 0.2:
            dt = 0.0 if self._last_camera_t is None else t - self._last_camera_t
            location, rotation = self._camera.update(truth.positions, dt, [g.center for g in scenario.judged_gates],
                                                     truth.intruders)
            sim.set_viewport_camera(location, rotation)
            self._last_camera_t = t
