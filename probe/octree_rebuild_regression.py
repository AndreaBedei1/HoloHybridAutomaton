"""Regression test of the HoloOcean sonar-octree patch (RebuildSonarOctree).

Hard case on purpose: the sonar exists from the start (declared in the scenario, octree built
inside ``reset()``) and the static scene changes afterwards.

  A  sonar created                         (scenario)
  B  spawn a runtime box and arena gate G06 after reset
  C  before any rebuild they are invisible (the octree predates them)
  D  env.rebuild_sonar_octree()            (full invalidation)
  E  box and gate become visible; a pre-existing BlueROV2 stays visible
  +  a second box is made visible with a *region* rebuild (local cell invalidation)
  +  a BlueROV2 spawned at runtime is invisible until a rebuild, visible after
  F  env.reset(): props removed (and the cache still contains them -> ghosts are expected here)
  G  env.rebuild_sonar_octree()
  H  no ghost echo from the removed props; a box spawned at a new place is visible, the old place silent

Also measures the cost of full and region rebuilds and checks that the official HoloOcean
installation's octree cache is not touched.  Writes results/v2/octree_patch/regression.json and
figures/v2/octree_patch/regression.png.  Exit code 0 iff every expectation holds.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.sim.holoocean_setup import ENV_MAX, ENV_MIN, OCTREE_MAX, OCTREE_MIN, use_patched_holoocean  # noqa: E402

PATCH = use_patched_holoocean()

from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates, make_spawner, visual_gates  # noqa: E402
from holo_fleet.sim.engine_watchdog import EngineWatchdog  # noqa: E402

OUT_JSON = ROOT / "results" / "v2" / "octree_patch" / "regression.json"
OUT_FIG = ROOT / "figures" / "v2" / "octree_patch" / "regression.png"
TPS, HZ = 30, 10
SONAR_X = 0.24
R_MIN, R_MAX, BINS = 0.3, 12.0, 234
RANGES = R_MIN + (np.arange(BINS) + 0.5) * (R_MAX - R_MIN) / BINS
BASE = np.array([0.0, -20.0, -5.0])
FAR = [0.0, 60.0, -5.0]


def first_return(p, thr=1e-3):
    idx = np.flatnonzero(np.asarray(p) > thr)
    return None if idx.size == 0 else float(RANGES[idx[0]])


def echo_near(p, r, tol=0.2, thr=1e-3):
    p = np.asarray(p)
    sel = (np.abs(RANGES - r) <= tol) & (p > thr)
    return bool(sel.any())


class Bench:
    def __init__(self):
        import holoocean

        sonar = {"sensor_type": "SinglebeamSonar", "sensor_name": "Sonar", "location": [SONAR_X, 0, 0], "Hz": HZ,
                 "configuration": {"OpeningAngle": 120, "RangeMin": R_MIN, "RangeMax": R_MAX, "RangeBins": BINS,
                                   "ShowWarning": False, "InitOctreeRange": 20}}
        agents = [
            {"agent_name": "observer", "agent_type": "BlueROV2", "location": list(BASE), "rotation": [0, 0, 0],
             "control_scheme": 0, "sensors": [sonar]},
            {"agent_name": "target", "agent_type": "BlueROV2", "location": FAR, "rotation": [0, 0, 0],
             "control_scheme": 0, "sensors": []},
        ]
        scen = {"name": "octree_regression", "world": "OpenWater", "package_name": "Ocean", "main_agent": "observer",
                "ticks_per_sec": TPS, "frames_per_sec": False, "octree_min": OCTREE_MIN, "octree_max": OCTREE_MAX,
                "env_min": ENV_MIN, "env_max": ENV_MAX, "agents": agents}
        self.env = holoocean.make(scenario_cfg=scen, show_viewport=False, ticks_per_sec=TPS, frames_per_sec=False)
        self.watchdog = EngineWatchdog(self.env).start()
        self.poses = {"observer": (list(BASE), [0, 0, 0]), "target": (FAR, [0, 0, 0])}
        self.last = None
        self.tick_ms = []
        self.looks = []

    def tick(self):
        for name, (loc, rot) in self.poses.items():
            if name in self.env.agents:
                self.env.agents[name].set_physics_state(loc, rot, [0, 0, 0], [0, 0, 0])
                self.env.act(name, np.zeros(8))
        t0 = time.perf_counter()
        raw = self.env.tick()
        self.tick_ms.append(1000 * (time.perf_counter() - t0))
        if "Sonar" in raw.get("observer", {}):
            self.last = np.asarray(raw["observer"]["Sonar"], dtype=float)
        return raw

    def settle(self, ticks=12):
        for _ in range(ticks):
            self.tick()

    def look(self, obs_loc, obs_rot=(0, 0, 0), label="", **others):
        self.poses["observer"] = ([float(v) for v in obs_loc], [float(v) for v in obs_rot])
        for name, loc in others.items():
            self.poses[name] = ([float(v) for v in loc], [0, 0, 180.0])
        self.settle(9)
        p = self.last.copy()
        echoes = [round(float(RANGES[i]), 3) for i in np.flatnonzero(p > 1e-3)]
        self.looks.append({"label": label, "first": first_return(p), "n_echo_bins": len(echoes),
                           "echo_ranges": echoes[:6] + (["..."] if len(echoes) > 6 else [])})
        return p

    def rebuild(self, region=None):
        t0 = time.time()
        n = self.env.rebuild_sonar_octree(region)
        self.settle(n)
        return {"ticks": n, "wall_s": round(time.time() - t0, 2)}


def g06_view(d=3.0):
    g = {x.gate_id: x for x in load_arena_gates(HORSESHOE_TRACK)}["G06"]
    left = np.array([-g.axis[1], g.axis[0], 0.0])
    sonar = g.center - d * g.axis + 0.84 * left
    yaw = math.degrees(math.atan2(g.axis[1], g.axis[0]))
    return sonar - SONAR_X * g.axis, (0, 0, yaw)


def official_cache_listing():
    d = Path(os.environ.get("LOCALAPPDATA", "")) / "holoocean" / "2.3.0" / "worlds" / "Ocean" / "Windows" / "Holodeck" / "Octrees"
    if not d.exists():
        return []
    return sorted((str(p.relative_to(d)), p.stat().st_size, int(p.stat().st_mtime)) for p in d.rglob("*") if p.is_file())


def main() -> int:
    from holoocean.agents import AgentDefinition

    before_official = official_cache_listing()
    res = {"patch": PATCH, "checks": {}, "costs": {}}
    chk = res["checks"]
    b = Bench()
    b.settle(40)
    # clean starting point: the private cache may hold cells of an earlier scene
    res["costs"]["initial_rebuild"] = b.rebuild(None)

    P_BOX1 = BASE + np.array([0.0, 6.0, 0.0])
    BOX1 = P_BOX1 + np.array([SONAR_X + 3.0 + 0.25, 0.0, 0.0])           # face 3.0 m from the sonar
    P_BOX2 = BASE + np.array([0.0, 12.0, 0.0])
    BOX2 = P_BOX2 + np.array([SONAR_X + 4.0 + 0.25, 0.0, 0.0])           # face 4.0 m
    P_AGENT = BASE + np.array([0.0, -6.0, 0.0])
    P_LATE = BASE + np.array([0.0, -12.0, 0.0])
    gate_loc, gate_rot = g06_view(3.0)

    # pre-existing BlueROV2 (agents always have their own octree)
    p = b.look(P_AGENT, label='agent initial', target=P_AGENT + np.array([SONAR_X + 4.0, 0, 0]))
    chk["agent_visible_initially"] = echo_near(p, 4.0 - 0.229)

    # B: runtime scene change after the sonar exists
    b.env.spawn_prop("box", location=[float(v) for v in BOX1], rotation=[0, 0, 0], scale=0.5, sim_physics=False, material="steel")
    make_spawner(b.env).spawn_gate_bars([bar for vg in visual_gates(HORSESHOE_TRACK, ["G06"]) for bar in vg.bars])
    b.settle(6)
    # C: invisible before the rebuild
    chk["C_box_invisible_before_rebuild"] = not echo_near(b.look(P_BOX1, label='box1 before rebuild'), 3.0, tol=0.5)
    chk["C_gate_invisible_before_rebuild"] = first_return(b.look(gate_loc, gate_rot, label='gate before rebuild')) is None
    # D/E: full rebuild
    res["costs"]["full_rebuild"] = b.rebuild(None)
    chk["E_box_visible_after_full_rebuild"] = echo_near(b.look(P_BOX1, label='box1 after full rebuild'), 3.0)
    gp = b.look(gate_loc, gate_rot, label='gate after full rebuild')
    chk["E_gate_visible_after_full_rebuild"] = echo_near(gp, 2.89, tol=0.2)
    chk["E_agent_still_visible"] = echo_near(b.look(P_AGENT, label='agent after rebuild', target=P_AGENT + np.array([SONAR_X + 4.0, 0, 0])), 4.0 - 0.229)
    b.poses["target"] = (FAR, [0, 0, 0])

    # region rebuild: second box, only the cells around it are recomputed
    b.env.spawn_prop("box", location=[float(v) for v in BOX2], rotation=[0, 0, 0], scale=0.5, sim_physics=False, material="steel")
    b.settle(6)
    chk["box2_invisible_before_region_rebuild"] = not echo_near(b.look(P_BOX2, label='box2 before region rebuild'), 4.0, tol=0.5)
    res["costs"]["region_rebuild"] = b.rebuild((list(BOX2 - 1.0), list(BOX2 + 1.0)))
    chk["box2_visible_after_region_rebuild"] = echo_near(b.look(P_BOX2, label='box2 after region rebuild'), 4.0)
    chk["box1_still_visible_after_region_rebuild"] = echo_near(b.look(P_BOX1, label='box1 after region rebuild'), 3.0)

    # BlueROV2 spawned at runtime
    late_loc = P_LATE + np.array([SONAR_X + 5.0, 0.0, 0.0])
    b.env.add_agent(AgentDefinition("late", "BlueROV2", sensors=[], starting_loc=[float(v) for v in late_loc],
                                    starting_rot=[0, 0, 180]))
    b.poses["late"] = ([float(v) for v in late_loc], [0, 0, 180.0])
    b.settle(6)
    chk["late_agent_invisible_before_rebuild"] = not echo_near(b.look(P_LATE, label='late agent before rebuild'), 5.0 - 0.229, tol=0.5)
    res["costs"]["agent_rebuild"] = b.rebuild(None)
    chk["late_agent_visible_after_rebuild"] = echo_near(b.look(P_LATE, label='late agent after rebuild'), 5.0 - 0.229)

    # F/G/H: scene change by reset (props removed) -> ghosts until rebuilt
    b.env.reset()
    b.poses.pop("late", None)
    b.settle(40)
    ghost_box = echo_near(b.look(P_BOX1, label='box1 place after reset, before rebuild'), 3.0)
    ghost_gate = first_return(b.look(gate_loc, gate_rot, label='gate place after reset, before rebuild')) is not None
    res["ghosts_after_reset_before_rebuild"] = {"box": ghost_box, "gate": ghost_gate}
    res["costs"]["rebuild_after_reset"] = b.rebuild(None)
    chk["H_no_ghost_box_after_rebuild"] = not echo_near(b.look(P_BOX1, label='box1 place after reset+rebuild'), 3.0, tol=0.5)
    chk["H_no_ghost_gate_after_rebuild"] = first_return(b.look(gate_loc, gate_rot, label='gate place after reset+rebuild')) is None
    chk["H_no_ghost_box2_after_rebuild"] = not echo_near(b.look(P_BOX2, label='box2 place after reset+rebuild'), 4.0, tol=0.5)
    # changed object: a box at a new place (2 m farther than box1)
    BOX3 = BOX1 + np.array([2.0, 0.0, 0.0])
    b.env.spawn_prop("box", location=[float(v) for v in BOX3], rotation=[0, 0, 0], scale=0.5, sim_physics=False, material="steel")
    b.settle(6)
    res["costs"]["region_rebuild_moved_box"] = b.rebuild((list(BOX3 - 1.0), list(BOX3 + 1.0)))
    p = b.look(P_BOX1, label='moved box (5 m) after region rebuild')
    chk["H_moved_box_visible_at_new_place"] = echo_near(p, 5.0)
    chk["H_old_place_silent"] = not echo_near(p, 3.0, tol=0.3)
    b.watchdog.stop()
    b.env.__exit__(None, None, None)

    chk["official_installation_cache_untouched"] = official_cache_listing() == before_official
    tm = np.asarray(b.tick_ms)
    res["costs"]["tick_ms_median"] = round(float(np.median(tm)), 1)
    res["costs"]["tick_ms_max"] = round(float(tm.max()), 1)
    res["looks"] = b.looks
    res["all_passed"] = all(chk.values())
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JSON.write_text(json.dumps(res, indent=1, default=str), encoding="utf-8")
    print(json.dumps(res, indent=1, default=str))
    return 0 if res["all_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
