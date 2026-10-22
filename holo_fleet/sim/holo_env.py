"""HoloOcean multi-agent wrapper (v2): owns the simulator and the ground truth.

Controllers only ever receive a :class:`SensorFrame` per drone built from the whitelisted onboard
sensors (``perception.sensor_suite.is_onboard``): six sonar echo profiles, DVL, IMU, compass and
depth.  PoseSensor/VelocitySensor/CollisionSensor and the visualisation cameras go to the referee
and to the UI only.

Static scene and sonar octree: the arena gates (and any prop) are spawned right after start, then
``env.rebuild_sonar_octree()`` (patched HoloOcean, docs/HOLOOCEAN_OCTREE_PATCH.md) makes every sonar
rebuild its octree from the current scene, so runtime props are visible and no cached cell of
another scene can produce a ghost echo.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.perception.frame import SensorFrame
from holo_fleet.perception.sensor_suite import SONAR_NAMES, is_onboard, onboard_sensor_configs
from holo_fleet.sim import holoocean_setup as hs
from holo_fleet.sim.sensor_noise import heading_biases, plan_windows

LOG = logging.getLogger(__name__)

TICKS_PER_SEC = 30
IMU_OFFSET = np.array([0.11, 0.0, -0.065])     # PoseSensor (IMUSocket) relative to the agent origin, body frame


@dataclass
class Truth:
    t: float
    positions: np.ndarray                 # (n,3) agent origins (hull centres)
    rotations: np.ndarray                 # (n,3,3) body->world
    yaw_deg: np.ndarray                   # (n,)
    velocities: np.ndarray                # (n,3)
    collision: np.ndarray                 # (n,) bool, CollisionSensor
    released: np.ndarray                  # (n,) bool
    current_drift: np.ndarray             # (n,3) effective drift applied at each drone
    current_cmd: np.ndarray               # (n,3) HoloOcean command actually sent
    intruders: Optional[np.ndarray] = None  # (m,3) scripted non-fleet vehicles (referee only)


def agent_origin_from_pose(pose: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose, dtype=float)
    return pose[:3, 3] - pose[:3, :3] @ IMU_OFFSET


class HoloFleetSim:
    """``spec`` needs: name, names, spawn_positions, spawn_yaw_deg, gate_ids, props, current,
    stress (beam_dropout, blackout_every_s, blackout_len_s), release_s, duration_s, seed,
    camera_drone (index or None), env_box (or None); optionally ``noise`` (sim.sensor_noise.NoiseProfile:
    HoloOcean noise parameters + seeded wrapper degradation of the onboard streams, None = deployed sensors).
    ``cfg.beacon`` enables the optional one-way presence beacon (sim/beacon.py)."""

    def __init__(self, spec, cfg: FleetConfig = DEFAULT, headless: bool = True, view_region: bool = False):
        self.spec = spec
        self.cfg = cfg
        self.headless = headless
        self.view_region = view_region
        self.names: List[str] = list(spec.names)
        self.intruders: List[Dict[str, Any]] = [dict(d) for d in (getattr(spec, "intruders", ()) or ())]
        self.intruder_names: List[str] = [d["name"] for d in self.intruders]
        self.env = None
        self.t = 0.0
        self.latest: Dict[str, Dict[str, Any]] = {n: {} for n in self.names + self.intruder_names}
        self.stamp: Dict[str, Dict[str, float]] = {n: {} for n in self.names + self.intruder_names}
        self.rng = np.random.default_rng(spec.seed + 12345)
        self.blackouts: Dict[str, List[tuple]] = {}
        self.pins: Dict[str, tuple] = {}
        self.setup_report: Dict[str, Any] = {}
        self.tick_wall_ms: List[float] = []
        self.sonar_captures = 0
        self.watchdog = None
        # sensor degradation (sim/sensor_noise.py): its own generator, so the stress stream above is unchanged
        self.noise = getattr(spec, "noise", None)
        self.nrng = np.random.default_rng([int(spec.seed), 7919])
        self.heading_bias = (heading_biases(self.noise, len(self.names), spec.seed) if self.noise is not None
                             else np.zeros(len(self.names)))
        self.noise_windows: Dict[str, Dict[str, List[tuple]]] = {}
        self.delayed: Dict[str, List[tuple]] = {n: [] for n in self.names}      # (t_due, key, value, t_capture)
        self.noise_stats = {"sonar_dropped": 0, "sonar_blackout": 0, "sonar_faded": 0, "sonar_shifted": 0,
                            "dvl_dropped": 0, "dvl_outage": 0}
        self.beacon = None
        self.beacon_inbox: Dict[str, List[tuple]] = {n: [] for n in self.names}
        if cfg.beacon.enabled:
            from holo_fleet.sim.beacon import BeaconTransport

            self.beacon = BeaconTransport(self.names, cfg.beacon, spec.seed,
                                          dropout=0.0 if self.noise is None else self.noise.beacon_dropout,
                                          ticks_per_sec=TICKS_PER_SEC)

    # ------------------------------------------------------------------ setup
    def _agent_cfg(self, k: int) -> Dict[str, Any]:
        name = self.names[k]
        sensors = onboard_sensor_configs(self.cfg, view_region=self.view_region, noise=self.noise,
                                         heading_bias_deg=float(self.heading_bias[k]))
        if self.beacon is not None:
            sensors.append(self.beacon.sensor_config(k))
        sensors += [
            {"sensor_type": "PoseSensor", "sensor_name": "PoseSensor", "socket": "IMUSocket", "Hz": 30},
            {"sensor_type": "VelocitySensor", "sensor_name": "VelocitySensor", "socket": "IMUSocket", "Hz": 30},
            {"sensor_type": "CollisionSensor", "sensor_name": "CollisionSensor", "Hz": 30},
        ]
        if self.spec.camera_drone is not None and k == self.spec.camera_drone:
            hz = self.cfg.perc.camera_hz
            sensors.append({"sensor_type": "RGBCamera", "sensor_name": "ChaseCamera", "socket": "IMUSocket",
                            "location": list(getattr(self.spec, "chase_offset", (-5.5, 0.0, 1.6))),
                            "rotation": [0.0, 12.0, 0.0], "Hz": hz,
                            "configuration": {"CaptureWidth": 800, "CaptureHeight": 450, "FovAngle": 80}})
            sensors.append({"sensor_type": "RGBCamera", "sensor_name": "SideCamera", "socket": "IMUSocket",
                            "location": list(getattr(self.spec, "side_offset", (1.0, 8.5, 1.2))),
                            "rotation": [0.0, 6.0, -90.0], "Hz": hz,
                            "configuration": {"CaptureWidth": 800, "CaptureHeight": 450, "FovAngle": 80}})
        pos = self.spec.spawn_positions[k]
        return {"agent_name": name, "agent_type": "BlueROV2", "location": [float(v) for v in pos],
                "rotation": [0.0, 0.0, float(self.spec.spawn_yaw_deg[k])], "control_scheme": 0, "sensors": sensors}

    def _intruder_cfg(self, d: Dict[str, Any]) -> Dict[str, Any]:
        return {"agent_name": d["name"], "agent_type": "BlueROV2", "location": [float(v) for v in d["start"]],
                "rotation": [0.0, 0.0, float(d.get("yaw_deg", 0.0))], "control_scheme": 0,
                "sensors": [{"sensor_type": "PoseSensor", "sensor_name": "PoseSensor", "socket": "IMUSocket", "Hz": 30}]}

    @staticmethod
    def intruder_position(d: Dict[str, Any], t: float) -> np.ndarray:
        """Scripted position: piecewise-linear ``waypoints`` [[t, x, y, z], ...] or start + velocity * t."""
        if d.get("waypoints"):
            W = np.asarray(d["waypoints"], float)
            return np.array([np.interp(t, W[:, 0], W[:, k]) for k in (1, 2, 3)])
        return np.asarray(d["start"], float) + np.asarray(d["velocity"], float) * max(0.0, t - float(d.get("t_start", 0.0)))

    def start(self) -> None:
        self.setup_report["holoocean"] = hs.use_patched_holoocean()
        import holoocean

        hs.wait_for_engine_semaphores()           # the client no longer loses the engine's start-up race

        from holo_fleet.sim.engine_watchdog import EngineWatchdog

        env_min, env_max = (self.spec.env_box if getattr(self.spec, "env_box", None) else (hs.ENV_MIN, hs.ENV_MAX))
        scenario = {
            "name": f"holo_fleet_{self.spec.name}", "world": "OpenWater", "package_name": "Ocean",
            "main_agent": self.names[0], "ticks_per_sec": TICKS_PER_SEC, "frames_per_sec": False,
            "window_width": 1280, "window_height": 720, "octree_min": hs.OCTREE_MIN, "octree_max": hs.OCTREE_MAX,
            "env_min": list(env_min), "env_max": list(env_max),
            "agents": [self._agent_cfg(k) for k in range(len(self.names))] + [self._intruder_cfg(d) for d in self.intruders],
        }
        t0 = time.time()
        last_exc = None
        for attempt, delay in enumerate((0.0, 3.0, 6.0, 12.0, 20.0), start=1):
            if delay:
                time.sleep(delay)
            try:
                self.env = holoocean.make(scenario_cfg=scenario, show_viewport=not self.headless,
                                          ticks_per_sec=TICKS_PER_SEC, frames_per_sec=False)
                break
            except Exception as exc:  # pragma: no cover - fallback: engine did not come up (restart it)
                last_exc = exc
                text = f"{type(exc).__name__}: {exc}".lower()
                LOG.warning("HoloOcean start attempt %d failed: %s", attempt, exc)
                self._kill_own_engines()
                if not any(s in text for s in ("semaphore", "timed out", "file not found", "impossibile trovare")):
                    raise
        else:
            raise RuntimeError(f"HoloOcean could not start after retries: {last_exc}")
        self.watchdog = EngineWatchdog(self.env).start()
        self.setup_report["engine_start_s"] = round(time.time() - t0, 1)
        self._spawn_static_scene()
        self._plan_blackouts()
        self.t = 0.0

    def _spawn_static_scene(self) -> None:
        from holo_fleet.arena_bridge import HORSESHOE_TRACK, make_spawner, visual_gates

        t0 = time.time()
        bars = []
        if self.spec.gate_ids:
            bars = [bar for vg in visual_gates(HORSESHOE_TRACK, list(self.spec.gate_ids)) for bar in vg.bars]
            spawner = make_spawner(self.env)
            spawner.spawn_gate_bars(bars)
            self.setup_report["gate_spawn"] = getattr(spawner.report, "__dict__", {})
        for prop in getattr(self.spec, "props", ()) or ():
            self.env.spawn_prop(prop.get("type", "box"), location=list(prop["location"]),
                                rotation=list(prop.get("rotation", (0, 0, 0))), scale=prop.get("scale", 1.0),
                                sim_physics=False, material=prop.get("material", "steel"))
        self._settle(4)
        if hs.client_supports_rebuild() and self.setup_report["holoocean"].get("patched"):
            wait = self.env.rebuild_sonar_octree()        # octree <- the scene that is really there
            self._settle(wait + 3)
            self.setup_report["octree"] = {"mode": "patched_rebuild", "ticks": wait + 3}
        else:  # pragma: no cover - unpatched engine: documented fallback only
            self.setup_report["octree"] = {"mode": "UNPATCHED: runtime props may be missing from the sonar octree"}
            LOG.warning("HoloOcean is not patched: runtime props may be invisible to the sonars")
            self._settle(12)
        self.setup_report["static_scene_s"] = round(time.time() - t0, 1)
        self.setup_report["gate_ids"] = list(self.spec.gate_ids)

    def _settle(self, ticks: int) -> None:
        for _ in range(ticks):
            for n in self.names + self.intruder_names:
                self.env.act(n, np.zeros(8))
            self._tick(record=False)

    @staticmethod
    def _kill_own_engines() -> None:
        try:
            import os

            import psutil

            me = os.getpid()
            for p in psutil.process_iter(["pid", "ppid", "name"]):
                if str(p.info.get("name") or "").lower() == "holodeck.exe" and p.info.get("ppid") == me:
                    p.kill()
        except Exception:  # pragma: no cover
            pass

    def _plan_blackouts(self) -> None:
        st = self.spec.stress
        for n in self.names:
            wins = []
            if st.blackout_every_s > 0:
                t = float(self.rng.uniform(4.0, st.blackout_every_s))
                while t < self.spec.duration_s:
                    wins.append((t, t + st.blackout_len_s))
                    t += float(self.rng.exponential(st.blackout_every_s)) + st.blackout_len_s
            self.blackouts[n] = wins
        nz = self.noise
        if nz is not None:
            for n in self.names:
                self.noise_windows[n] = {
                    "sonar": plan_windows(self.nrng, nz.sonar_blackout_every_s, nz.sonar_blackout_len_s,
                                          self.spec.duration_s),
                    "dvl": plan_windows(self.nrng, nz.dvl_outage_every_s, nz.dvl_outage_len_s, self.spec.duration_s)}

    def _blackout(self, name: str, t: float) -> bool:
        return any(a <= t <= b for a, b in self.blackouts.get(name, []))

    def _in_window(self, name: str, kind: str, t: float) -> bool:
        return any(a <= t <= b for a, b in self.noise_windows.get(name, {}).get(kind, []))

    def _degrade(self, name: str, key: str, val, t: float):
        """Seeded wrapper degradation of one onboard capture (None: not delivered)."""
        nz = self.noise
        if key in SONAR_NAMES:
            if self._in_window(name, "sonar", t):
                self.noise_stats["sonar_blackout"] += 1
                return None
            if nz.sonar_dropout > 0 and self.nrng.random() < nz.sonar_dropout:
                self.noise_stats["sonar_dropped"] += 1
                return None
            prof = None
            if nz.sonar_fade_prob > 0 and self.nrng.random() < nz.sonar_fade_prob:
                prof = np.asarray(val, dtype=float) * nz.sonar_fade_gain
                self.noise_stats["sonar_faded"] += 1
            if nz.sonar_range_jitter_m > 0:
                k = int(round(float(self.nrng.normal(0.0, nz.sonar_range_jitter_m)) / self.cfg.perc.sonar.bin_width))
                if k:
                    src = np.asarray(val if prof is None else prof, dtype=float).ravel()
                    prof = np.zeros_like(src)
                    if k > 0:
                        prof[k:] = src[:-k]
                    else:
                        prof[:k] = src[-k:]
                    self.noise_stats["sonar_shifted"] += 1
            return val if prof is None else prof
        if key == "DVLSensor":
            if self._in_window(name, "dvl", t):
                self.noise_stats["dvl_outage"] += 1
                return None
            if nz.dvl_dropout > 0 and self.nrng.random() < nz.dvl_dropout:
                self.noise_stats["dvl_dropped"] += 1
                return None
        return val

    # ------------------------------------------------------------------ stepping
    def _tick(self, record: bool = True) -> None:
        for name, (loc, rot) in self.pins.items():
            self.env.agents[name].set_physics_state(list(loc), list(rot), [0, 0, 0], [0, 0, 0])
        for d in self.intruders:                     # scripted kinematic motion, no reaction to anything
            loc = self.intruder_position(d, self.t)
            vel = (self.intruder_position(d, self.t + 0.1) - loc) / 0.1
            self.env.agents[d["name"]].set_physics_state([float(v) for v in loc], [0.0, 0.0, float(d.get("yaw_deg", 0.0))],
                                                         [float(v) for v in vel], [0, 0, 0])
        if self.beacon is not None:
            self.beacon.before_tick(self.env, self.t)
        w0 = time.perf_counter()
        raw = self.env.tick()
        if record:
            self.tick_wall_ms.append(1000.0 * (time.perf_counter() - w0))
        tick_t = self.t
        st = self.spec.stress
        for n in self.names + self.intruder_names:
            data = raw.get(n, raw if len(self.names) + len(self.intruder_names) == 1 else {})
            for key, val in data.items():
                if key in SONAR_NAMES:
                    self.sonar_captures += 1
                    # simulator-side degradation, identical for the six sonars: a dropped capture is
                    # simply not delivered (its age grows); a blackout drops every sonar of the drone
                    if self._blackout(n, tick_t) or (st.beam_dropout > 0 and self.rng.random() < st.beam_dropout):
                        continue
                if record and self.noise is not None and n in self.delayed and is_onboard(key):
                    val = self._degrade(n, key, val, tick_t)
                    if val is None:
                        continue
                    if self.noise.latency_s > 0:              # delivered late, stamped with its capture time
                        self.delayed[n].append((tick_t + self.noise.latency_s, key, val, tick_t))
                        continue
                self.latest[n][key] = val
                self.stamp[n][key] = tick_t
            if self.delayed.get(n):
                keep = []
                for item in self.delayed[n]:
                    if item[0] <= tick_t + 1e-6:
                        self.latest[n][item[1]], self.stamp[n][item[1]] = item[2], item[3]
                    else:
                        keep.append(item)
                self.delayed[n] = keep
        if self.beacon is not None:
            self.beacon.after_tick(self.env, tick_t)
            for r, rx in self.beacon.deliver(tick_t).items():
                self.beacon_inbox[r].extend(rx)

    def pin(self, name: str, location: Sequence[float], rotation: Sequence[float] = (0.0, 0.0, 0.0)) -> None:
        """Bench mode: hold an agent at a pose (perception and coverage experiments only)."""
        self.pins[name] = (tuple(float(v) for v in location), tuple(float(v) for v in rotation))

    def unpin(self, name: str) -> None:
        self.pins.pop(name, None)

    def truth(self) -> Truth:
        n = len(self.names)
        P, R = np.zeros((n, 3)), np.zeros((n, 3, 3))
        Y, V, C = np.zeros(n), np.zeros((n, 3)), np.zeros(n, dtype=bool)
        for k, name in enumerate(self.names):
            pose = np.asarray(self.latest[name].get("PoseSensor"), dtype=float)
            R[k] = pose[:3, :3]
            P[k] = agent_origin_from_pose(pose)
            Y[k] = math.degrees(math.atan2(pose[1, 0], pose[0, 0]))
            V[k] = np.asarray(self.latest[name].get("VelocitySensor", np.zeros(3))).ravel()[:3]
            C[k] = bool(np.asarray(self.latest[name].get("CollisionSensor", False)).any())
        released = np.array([self.t >= r for r in self.spec.release_s])
        drift = np.array([self.spec.current.drift_at(P[k], self.t) for k in range(n)])
        cmd = np.array([self.spec.current.command_at(P[k], self.t) for k in range(n)])
        intr = None
        if self.intruder_names:
            intr = np.array([agent_origin_from_pose(np.asarray(self.latest[nm]["PoseSensor"], float))
                             if self.latest[nm].get("PoseSensor") is not None else self.intruder_position(d, self.t)
                             for nm, d in zip(self.intruder_names, self.intruders)])
        return Truth(self.t, P, R, Y, V, C, released, drift, cmd, intr)

    def frames(self) -> Dict[str, SensorFrame]:
        out = {}
        for name in self.names:
            data = {k: v for k, v in self.latest[name].items() if is_onboard(k)}
            stamp = {k: v for k, v in self.stamp[name].items() if is_onboard(k)}
            if self.beacon is not None:                # receptions since the previous frame (identities only)
                rx = self.beacon_inbox[name]
                data["BeaconRx"] = tuple(rx)
                stamp["BeaconRx"] = max((x[1] for x in rx), default=-1e9)
                self.beacon_inbox[name] = []
            out[name] = SensorFrame(t=self.t, data=data, stamp=stamp)
        return out

    def step(self, commands: Dict[str, Optional[np.ndarray]], dt: float) -> None:
        """``commands[name]`` = 8 thruster values, or None to hold the drone at its launch pose (cage)."""
        n_ticks = max(1, int(round(dt * TICKS_PER_SEC)))
        truth_now = self.truth() if self.latest[self.names[0]].get("PoseSensor") is not None else None
        for k, name in enumerate(self.names):
            if truth_now is not None:
                self.env.set_ocean_currents(name, [float(v) for v in truth_now.current_cmd[k]])
            cmd = commands.get(name)
            if cmd is None and name not in self.pins:
                self.env.agents[name].teleport([float(v) for v in self.spec.spawn_positions[k]],
                                               [0.0, 0.0, float(self.spec.spawn_yaw_deg[k])])
                self.env.act(name, np.zeros(8))
            elif self.spec.fault_active(name, self.t):
                self.env.act(name, np.zeros(8))          # injected thruster failure: commands are not executed
            else:
                self.env.act(name, np.zeros(8) if cmd is None else np.asarray(cmd, dtype=float))
        for name in self.intruder_names:
            self.env.act(name, np.zeros(8))
        for _ in range(n_ticks):
            self.t = round(self.t + 1.0 / TICKS_PER_SEC, 6)
            self._tick()

    def image(self, key: str):
        if self.spec.camera_drone is None:
            return None
        return self.latest[self.names[self.spec.camera_drone]].get(key)

    def draw_line(self, a, b, color=(0, 255, 0), thickness=8.0, lifetime=0.12) -> None:
        if self.env is not None and not self.headless:
            self.env.draw_line([float(v) for v in a], [float(v) for v in b], list(color), thickness, lifetime)

    def draw_arrow(self, a, b, color=(255, 0, 0), thickness=10.0, lifetime=0.12) -> None:
        if self.env is not None and not self.headless:
            self.env.draw_arrow([float(v) for v in a], [float(v) for v in b], list(color), thickness, lifetime)

    def draw_box(self, center, extent, color=(255, 255, 0), thickness=6.0, lifetime=0.12) -> None:
        if self.env is not None and not self.headless:
            self.env.draw_box([float(v) for v in center], [float(v) for v in extent], list(color), thickness, lifetime)

    def set_viewport_camera(self, location, rotation) -> None:
        """Move HoloOcean's spectator viewport; camera data is consumed by display only."""
        if self.env is not None and not self.headless:
            self.env.move_viewport([float(v) for v in location], [float(v) for v in rotation])

    def perf(self) -> Dict[str, float]:
        w = np.asarray(self.tick_wall_ms) if self.tick_wall_ms else np.array([np.nan])
        return {"ticks": int(len(self.tick_wall_ms)), "mean_tick_ms": round(float(np.nanmean(w)), 2),
                "p95_tick_ms": round(float(np.nanpercentile(w, 95)), 2),
                "real_time_factor": round(float((1000.0 / TICKS_PER_SEC) / np.nanmean(w)), 3)}

    def close(self) -> None:
        if self.watchdog is not None:
            self.watchdog.stop()
        if self.env is not None:
            try:
                self.env.__exit__(None, None, None)
            except Exception as exc:  # pragma: no cover
                LOG.warning("HoloOcean close: %s", exc)
            self.env = None
