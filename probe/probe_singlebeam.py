"""Phase-1 probe of a realistic directional sonar: ONE HoloOcean ``SinglebeamSonar`` on a BlueROV2.

Questions (v2 brief, section 2): does it see (A) another BlueROV2, (B) an arena gate, (C) a runtime-spawned
prop; what does it return (D); effective range (E); opening angle (F); behaviour near the surface (G) and the
seabed (H); noise (I); update rate and cost (J); octree pitfalls (K).

HoloOcean 2.3.0 sonars run on an octree of the static scene (engine source ``Octree.cpp``,
``HolodeckSonar.cpp``): the static octree is built by overlap tests the first time each 7.68 m cell is
needed and is cached on disk under ``Octrees/<map>/min<OctreeMin>_max<OctreeMax>``; agents get their own
moving octrees.  A runtime prop is therefore visible only if it already existed when the cell covering it
was first computed.  The sessions below test exactly that:

  k1  sonar declared in the scenario (octree built inside ``reset()``), gates + box spawned afterwards
      (the v1 order)                                    -> reproduces "runtime props are invisible"
  k2  fresh project cache, gates + box spawned FIRST, sonar attached afterwards with ``add_sensors``
                                                        -> props, vehicles, range, FOV, noise, surface,
                                                           seabed, timing
  k3  cache left by k2, props NOT spawned               -> stale-cache "ghosts" (why the cache must be
                                                           keyed by the static scene)

The probe uses ground truth freely (it is a calibration bench, not a controller).  Raw output goes to
``probe/out/sonar_probe/<session>/``; ``probe/analyze_sonar_probe.py`` turns it into figures and numbers.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates, make_spawner, visual_gates  # noqa: E402

OUT = ROOT / "probe" / "out" / "sonar_probe"
TPS = 30
SONAR_HZ = 10
OCTREE_MIN, OCTREE_MAX = 0.06, 5.0           # project-specific cache folder (min6_max768), never shared
OCTREE_DIR = (Path(os.environ.get("LOCALAPPDATA", "")) / "holoocean" / "2.3.0" / "worlds" / "Ocean" / "Windows"
              / "Holodeck" / "Octrees" / "OpenWater" / "min6_max768")
SONAR_X = 0.24                                # sonar origin: bow of the hull, on the body x axis [m]
OPEN_WATER = np.array([0.0, -40.0, -5.0])     # >= 25 m from every arena bar
BOX_POS = np.array([-5.0, 30.0, -4.3])        # >= 14 m from every arena bar
BOX_SIZE = 0.5
FAR_AWAY = [0.0, -200.0, -5.0]


def sonar_config(opening: float, noise: bool, view: bool = False) -> dict:
    cfg = {"OpeningAngle": opening, "RangeMin": 0.3, "RangeMax": 12.0, "RangeBins": 234,   # 5 cm bins
           "AddSigma": 0.0, "MultSigma": 0.0, "RangeSigma": 0.0, "ShowWarning": False,
           "InitOctreeRange": 50, "ViewRegion": view, "ViewOctree": -10}
    if noise:
        cfg.update({"AddSigma": 0.05, "MultSigma": 0.1, "RangeSigma": 0.05})
    return cfg


def rot_matrix(rpy_deg) -> np.ndarray:
    r, p, y = [math.radians(v) for v in rpy_deg]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class Bench:
    def __init__(self, session: str, opening: float, sonar_in_scenario: bool, camera: bool = True):
        import holoocean

        self.session, self.opening = session, opening
        self.dir = OUT / session
        self.dir.mkdir(parents=True, exist_ok=True)
        self.samples = open(self.dir / "samples.jsonl", "w", encoding="utf-8")
        self.events = []
        obs_sensors = [
            {"sensor_type": "PoseSensor", "sensor_name": "PoseSensor", "socket": "IMUSocket", "Hz": TPS},
            {"sensor_type": "RangeFinderSensor", "sensor_name": "AltimeterGT", "socket": "IMUSocket", "Hz": TPS,
             "configuration": {"LaserMaxDistance": 400, "LaserCount": 1, "LaserAngle": 90}},
        ]
        if camera:
            obs_sensors.append({"sensor_type": "RGBCamera", "sensor_name": "ProbeCam", "socket": "IMUSocket",
                                "location": [-3.2, 1.6, 1.1], "rotation": [0.0, 12.0, -22.0], "Hz": 2,
                                "configuration": {"CaptureWidth": 960, "CaptureHeight": 540, "FovAngle": 85}})
        if sonar_in_scenario:
            obs_sensors.append({"sensor_type": "SinglebeamSonar", "sensor_name": "Sonar", "location": [SONAR_X, 0, 0],
                                "Hz": SONAR_HZ, "configuration": sonar_config(opening, False, view=True)})
        agents = [
            {"agent_name": "observer", "agent_type": "BlueROV2", "location": list(OPEN_WATER), "rotation": [0, 0, 0],
             "control_scheme": 0, "sensors": obs_sensors},
            {"agent_name": "target", "agent_type": "BlueROV2", "location": FAR_AWAY, "rotation": [0, 0, 0],
             "control_scheme": 0, "sensors": [{"sensor_type": "PoseSensor", "sensor_name": "PoseSensor",
                                               "socket": "IMUSocket", "Hz": TPS}]},
        ]
        scen = {"name": f"sonar_probe_{session}", "world": "OpenWater", "package_name": "Ocean",
                "main_agent": "observer", "ticks_per_sec": TPS, "frames_per_sec": False,
                "octree_min": OCTREE_MIN, "octree_max": OCTREE_MAX, "agents": agents}
        t0 = time.time()
        self.env = holoocean.make(scenario_cfg=scen, show_viewport=False, ticks_per_sec=TPS, frames_per_sec=False)
        self.env.reset()
        self.log("env_ready", wall_s=round(time.time() - t0, 1))
        self.latest = {"observer": {}, "target": {}}
        self.tick_count = 0
        self.obs_pose = (list(OPEN_WATER), [0.0, 0.0, 0.0])
        self.tgt_pose = (FAR_AWAY, [0.0, 0.0, 0.0])

    def log(self, what, **kw):
        rec = {"event": what, **kw}
        self.events.append(rec)
        print(json.dumps(rec), flush=True)

    # --------------------------------------------------------------- scene / sensors
    def spawn_props(self, gates: bool = True, box: bool = True):
        if gates:
            spawner = make_spawner(self.env)
            spawner.spawn_gate_bars([bar for vg in visual_gates(HORSESHOE_TRACK) for bar in vg.bars])
        if box:
            self.env.spawn_prop("box", location=[float(v) for v in BOX_POS], rotation=[0, 0, 0], scale=BOX_SIZE,
                                sim_physics=False, material="steel")
        self.log("props_spawned", gates=gates, box=box, wall_time=time.time())

    def add_sonar(self, noise: bool, view: bool = True):
        from holoocean.sensors import SensorDefinition

        self.env._has_sonar = True            # no engine-handshake timeout while the octree is being built
        sd = SensorDefinition("observer", "BlueROV2", "Sonar", "SinglebeamSonar", socket="", location=[SONAR_X, 0, 0],
                              rotation=[0, 0, 0], config=sonar_config(self.opening, noise, view),
                              tick_every=TPS // SONAR_HZ)
        self.env.agents["observer"].add_sensors(sd)
        self.sonar_def = sd
        t0 = time.time()
        n = 0
        while True:
            self.tick()
            n += 1
            if "Sonar" in self._last_raw.get("observer", {}):
                break
            if n > 3000:
                raise RuntimeError("sonar never produced data")
        self.log("sonar_attached", noise=noise, first_data_after_ticks=n, wall_s=round(time.time() - t0, 1))

    def remove_sonar(self):
        self.env.agents["observer"].remove_sensors(self.sonar_def)
        self.tick()

    # --------------------------------------------------------------------- ticking
    def tick(self):
        for name, (loc, rot) in (("observer", self.obs_pose), ("target", self.tgt_pose)):
            self.env.agents[name].set_physics_state(loc, rot, [0, 0, 0], [0, 0, 0])
            self.env.act(name, np.zeros(8))
        raw = self.env.tick()
        self._last_raw = raw
        for name in ("observer", "target"):
            self.latest[name].update(raw.get(name, {}))
        self.tick_count += 1
        return raw

    def place(self, obs_loc, obs_rpy, tgt_loc=None, tgt_rpy=(0.0, 0.0, 0.0)):
        self.obs_pose = ([float(v) for v in obs_loc], [float(v) for v in obs_rpy])
        self.tgt_pose = ([float(v) for v in (tgt_loc if tgt_loc is not None else FAR_AWAY)], [float(v) for v in tgt_rpy])

    def record(self, exp: str, case: dict, captures: int = 6, settle_captures: int = 2, screenshot: str = ""):
        """Hold the current placement and record ``captures`` sonar outputs (after ``settle_captures``)."""
        got, wall = 0, []
        need = captures + settle_captures
        while got < need:
            t0 = time.perf_counter()
            raw = self.tick()
            wall.append(time.perf_counter() - t0)
            if "Sonar" not in raw.get("observer", {}):
                continue
            got += 1
            if got <= settle_captures:
                continue
            o, t = raw["observer"], self.latest["target"]
            rec = {"exp": exp, "case": case, "tick": self.tick_count,
                   "obs_pose": np.asarray(o["PoseSensor"]).tolist(),
                   "tgt_pose": np.asarray(t["PoseSensor"]).tolist(),
                   "altimeter": float(np.asarray(o["AltimeterGT"]).ravel()[0]),
                   "sonar": np.round(np.asarray(o["Sonar"], dtype=float), 5).tolist()}
            self.samples.write(json.dumps(rec) + "\n")
        if screenshot and "ProbeCam" in self.latest["observer"]:
            self.save_image(screenshot)
        return float(np.mean(wall))

    def save_image(self, name: str):
        import cv2

        img = np.asarray(self.latest["observer"]["ProbeCam"])[:, :, :3]
        cv2.imwrite(str(self.dir / f"{name}.png"), img)

    def timing(self, label: str, ticks: int = 90):
        wall = []
        for _ in range(ticks):
            t0 = time.perf_counter()
            self.tick()
            wall.append(time.perf_counter() - t0)
        w = np.asarray(wall)
        self.log("timing", label=label, ticks=ticks, mean_tick_ms=round(1000 * w.mean(), 2),
                 p95_tick_ms=round(1000 * np.percentile(w, 95), 2), real_time_factor=round((1 / TPS) / w.mean(), 2))

    def close(self):
        self.samples.close()
        self.env.__exit__(None, None, None)
        self.write_events()

    def write_events(self):
        listing = []
        if OCTREE_DIR.exists():
            for f in sorted(OCTREE_DIR.rglob("*.json")):
                st = f.stat()
                listing.append({"file": f.name, "kb": round(st.st_size / 1e3, 1), "mtime": st.st_mtime})
        (self.dir / "cache_listing.json").write_text(json.dumps(listing, indent=0), encoding="utf-8")
        (self.dir / "events.json").write_text(json.dumps(self.events, indent=1), encoding="utf-8")


# ---------------------------------------------------------------------------------------- experiments
def gate_g06():
    return {g.gate_id: g for g in load_arena_gates(HORSESHOE_TRACK)}["G06"]


def face_gate_poses(g, d: float, lateral: float = 0.0, dz: float = 0.0):
    """Observer on the approach side of gate g, d metres before the gate plane, looking along the axis."""
    left = np.array([-g.axis[1], g.axis[0], 0.0])
    sonar = g.center - d * g.axis + lateral * left + np.array([0, 0, dz])
    loc = sonar - SONAR_X * g.axis                  # the hull origin sits behind the sonar
    yaw = math.degrees(math.atan2(g.axis[1], g.axis[0]))
    return loc, (0.0, 0.0, yaw)


def exp_gate(b: Bench, distances=(1.5, 2.0, 3.0, 4.0, 6.0, 8.0)):
    g = gate_g06()
    for aim, lat, dz in (("left_pillar", 0.84, 0.0), ("centre", 0.0, 0.0), ("top_bar", 0.0, 0.84)):
        for d in distances:
            loc, rpy = face_gate_poses(g, d, lat, dz)
            b.place(loc, rpy)
            shot = f"gate_{aim}_{d:.0f}m" if d in (3.0,) else ""
            b.record("gate", {"aim": aim, "d_plane": d}, captures=5, screenshot=shot)


def exp_box(b: Bench, distances=(1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 10.0)):
    for d in distances:
        sonar = BOX_POS - np.array([d + BOX_SIZE / 2, 0, 0])
        b.place(sonar - np.array([SONAR_X, 0, 0]), (0, 0, 0))
        b.record("box", {"d_face": d}, captures=5, screenshot="box_3m" if d == 3.0 else "")


def exp_drone_range(b: Bench, label="drone", captures=6):
    obs = OPEN_WATER
    for aspect, yaw in (("head_on", 180.0), ("broadside", 90.0)):
        for d in (0.6, 0.8, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 11.0, 12.5):
            tgt = obs + np.array([SONAR_X + d, 0, 0])
            b.place(obs, (0, 0, 0), tgt, (0, 0, yaw))
            shot = f"{label}_{aspect}_3m" if d == 3.0 else ""
            b.record(label, {"aspect": aspect, "d_centre_from_sonar": d}, captures=captures, screenshot=shot)


def exp_fov(b: Bench, ranges=(3.0, 6.0)):
    obs = OPEN_WATER
    for R in ranges:
        for plane in ("horizontal", "vertical"):
            for ang in np.arange(0.0, 80.01, 2.5):
                off = R * math.tan(math.radians(ang))
                rel = np.array([SONAR_X + R, off, 0.0]) if plane == "horizontal" else np.array([SONAR_X + R, 0.0, off])
                b.place(obs, (0, 0, 0), obs + rel, (0, 0, 180.0))
                b.record("fov", {"R": R, "plane": plane, "angle_deg": float(ang)}, captures=3)


def exp_noise(b: Bench):
    obs = OPEN_WATER
    for d in (1.0, 2.0, 4.0, 8.0):
        b.place(obs, (0, 0, 0), obs + np.array([SONAR_X + d, 0, 0]), (0, 0, 180.0))
        b.record("noise", {"d_centre_from_sonar": d}, captures=40)
    b.place(obs, (0, 0, 0))                          # empty water: false alarms
    b.record("noise_empty", {}, captures=60)


def exp_surface(b: Bench):
    base = OPEN_WATER.copy()
    for depth in (0.6, 1.0, 2.0):
        loc = np.array([base[0], base[1], -depth])
        b.place(loc, (0, 0, 0))
        b.record("surface_horizontal", {"depth": depth}, captures=4)
        b.place(loc, (0, 0, 0), loc + np.array([SONAR_X + 4.0, 0, 0]), (0, 0, 180))
        b.record("surface_horizontal_target", {"depth": depth, "d": 4.0}, captures=4,
                 screenshot="surface_target" if depth == 0.6 else "")
    for pitch in (-90.0, 90.0):                      # find which sign points the bow up
        for depth in (2.0, 4.0, 8.0):
            b.place([base[0], base[1], -depth], (0, pitch, 0))
            b.record("vertical_look", {"pitch": pitch, "depth": depth}, captures=4)


def exp_seabed(b: Bench):
    base = OPEN_WATER.copy()
    b.place(base, (0, 0, 0))
    for _ in range(10):
        b.tick()
    alt = float(np.asarray(b.latest["observer"]["AltimeterGT"]).ravel()[0])
    seabed_z = base[2] - alt
    b.log("seabed", altimeter_at_open_water=alt, seabed_z=round(seabed_z, 2))
    if not (0 < alt < 390):
        return
    for pitch in (-90.0, 90.0):
        for h in (1.0, 2.0, 4.0, 8.0):
            b.place([base[0], base[1], seabed_z + h], (0, pitch, 0))
            b.record("seabed_vertical", {"pitch": pitch, "altitude": h}, captures=4)
    for h in (0.5, 1.0, 2.0):
        loc = np.array([base[0], base[1], seabed_z + h])
        b.place(loc, (0, 0, 0))
        b.record("seabed_horizontal", {"altitude": h}, captures=4, screenshot="seabed_horizontal" if h == 1.0 else "")
        b.place(loc, (0, 0, 0), loc + np.array([SONAR_X + 4.0, 0, 0]), (0, 0, 180))
        b.record("seabed_horizontal_target", {"altitude": h, "d": 4.0}, captures=4)


def clear_project_cache():
    if OCTREE_DIR.exists():
        shutil.rmtree(OCTREE_DIR)
    return not OCTREE_DIR.exists()


def cache_size_mb() -> float:
    if not OCTREE_DIR.exists():
        return 0.0
    return round(sum(p.stat().st_size for p in OCTREE_DIR.rglob("*") if p.is_file()) / 1e6, 1)


# --------------------------------------------------------------------------------------------- sessions
def session_k1(opening: float):
    clear_project_cache()
    b = Bench("k1", opening, sonar_in_scenario=True)
    g = gate_g06()
    b.place(*face_gate_poses(g, 3.0))
    for _ in range(40):                              # octree built inside reset(); let the init cells finish
        b.tick()
    b.spawn_props()
    for _ in range(10):
        b.tick()
    b.log("cache_after_init", size_mb=cache_size_mb())
    exp_gate(b, distances=(2.0, 3.0, 6.0))
    exp_box(b, distances=(2.0, 3.0, 6.0))
    obs = OPEN_WATER
    b.place(obs, (0, 0, 0), obs + np.array([SONAR_X + 3.0, 0, 0]), (0, 0, 180))
    b.record("drone", {"aspect": "head_on", "d_centre_from_sonar": 3.0}, captures=5, screenshot="k1_drone_3m")
    b.close()


def session_k2(opening: float, short: bool = False):
    clear_project_cache()
    b = Bench("k2" if not short else f"k2_oa{int(opening)}", opening, sonar_in_scenario=False)
    g = gate_g06()
    b.place(*face_gate_poses(g, 3.0))
    b.timing("no_sonar_open_scene")
    b.spawn_props()
    for _ in range(10):
        b.tick()
    b.add_sonar(noise=False)
    b.log("cache_after_attach", size_mb=cache_size_mb())
    if short:
        for _ in range(150):                         # let the octree finish before timing
            b.tick()
        b.timing("one_sonar_facing_gate_warm", ticks=150)
        b.place(OPEN_WATER, (0, 0, 0))
        for _ in range(30):
            b.tick()
        b.timing("one_sonar_open_water_warm", ticks=150)
        exp_fov(b, ranges=(4.0,))
        exp_drone_range(b, captures=3)
        b.close()
        return
    b.timing("one_sonar_facing_gate")
    exp_gate(b)
    exp_box(b)
    exp_drone_range(b)
    b.place(OPEN_WATER, (0, 0, 0))
    b.timing("one_sonar_open_water")
    exp_fov(b)
    exp_surface(b)
    exp_seabed(b)
    b.remove_sonar()
    b.add_sonar(noise=True, view=False)
    exp_noise(b)
    b.log("cache_final", size_mb=cache_size_mb())
    b.close()


def session_k3(opening: float):
    b = Bench("k3", opening, sonar_in_scenario=True)
    for _ in range(40):
        b.tick()
    exp_gate(b, distances=(2.0, 3.0))
    exp_box(b, distances=(2.0, 3.0))
    b.close()
    b.log("cache_cleanup", removed=clear_project_cache())
    b.write_events()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("session", choices=["k1", "k2", "k2_short", "k3"])
    ap.add_argument("--opening", type=float, default=120.0, help="sonar opening angle [deg]")
    a = ap.parse_args()
    if a.session == "k1":
        session_k1(a.opening)
    elif a.session == "k2":
        session_k2(a.opening)
    elif a.session == "k2_short":
        session_k2(a.opening, short=True)
    else:
        session_k3(a.opening)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
