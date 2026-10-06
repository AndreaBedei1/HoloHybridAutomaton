"""HoloOcean multi-agent wrapper.

Owns the simulator and the ground truth.  Controllers only ever receive a
:class:`SensorFrame` per drone built from the whitelisted onboard sensors (see
``perception.sensor_suite.is_onboard``); PoseSensor/VelocitySensor/
CollisionSensor/ChaseCamera are routed to the referee/visualisation only.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from holo_fleet.arena_bridge import HORSESHOE_TRACK, make_spawner, thruster_command, visual_gates
from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.perception.perception import SensorFrame
from holo_fleet.perception.sensor_suite import is_onboard, onboard_sensor_configs, ring_name
from holo_fleet.sim.scenarios import ScenarioSpec

LOG = logging.getLogger(__name__)

TICKS_PER_SEC = 30


@dataclass
class Truth:
    t: float
    positions: np.ndarray                 # (n,3)
    yaw_deg: np.ndarray                   # (n,)
    velocities: np.ndarray                # (n,3)
    collision: np.ndarray                 # (n,) bool, CollisionSensor
    released: np.ndarray                  # (n,) bool
    current_drift: np.ndarray             # (n,3) effective drift applied at each drone
    current_cmd: np.ndarray               # (n,3) HoloOcean command actually sent


class HoloFleetSim:
    def __init__(self, spec: ScenarioSpec, cfg: FleetConfig = DEFAULT, headless: bool = True,
                 chase_camera: bool = True, top_camera: bool = True):
        self.spec = spec
        self.cfg = cfg
        self.headless = headless
        self.chase_camera = chase_camera
        self.top_camera = top_camera
        self.names = [p.drone_id for p in spec.plans]
        self.env = None
        self.t = 0.0
        self.latest: Dict[str, Dict[str, Any]] = {n: {} for n in self.names}
        self.stamp: Dict[str, Dict[str, float]] = {n: {} for n in self.names}
        self.rng = np.random.default_rng(spec.seed + 12345)
        self.blackouts: Dict[str, List[tuple]] = {}
        self.gate_spawn_report = None
        self.debug_frames: Dict[str, Any] = {}
        self.wall_start = None

    # ------------------------------------------------------------------ setup
    def _agent_cfg(self, k: int) -> Dict[str, Any]:
        name = self.names[k]
        sensors = onboard_sensor_configs(self.cfg, with_fls=(k not in self.spec.stress.disable_fls))
        sensors += [
            {"sensor_type": "PoseSensor", "sensor_name": "PoseSensor", "socket": "IMUSocket", "Hz": 30},
            {"sensor_type": "VelocitySensor", "sensor_name": "VelocitySensor", "socket": "IMUSocket", "Hz": 30},
            {"sensor_type": "CollisionSensor", "sensor_name": "CollisionSensor", "Hz": 30},
        ]
        # Visualisation cameras (never delivered to a controller) on the rear drone of the formation.
        # Conventions measured with probe/probe_cameras.py: pitch is positive DOWNWARD, yaw and
        # lateral offsets are standard (CCW / +y left).  Looking straight down into deep water is black,
        # so both cameras look almost horizontally against the bright mid-water background.
        if k == self.spec.n - 1 and self.chase_camera:
            sensors.append({"sensor_type": "RGBCamera", "sensor_name": "ChaseCamera", "socket": "IMUSocket",
                            "location": [-4.5, 0.0, 0.9], "rotation": [0.0, 8.0, 0.0], "Hz": 5,
                            "configuration": {"CaptureWidth": 640, "CaptureHeight": 400, "FovAngle": 75}})
        if k == self.spec.n - 1 and self.top_camera:
            sensors.append({"sensor_type": "RGBCamera", "sensor_name": "SideCamera", "socket": "IMUSocket",
                            "location": [1.0, 7.0, 0.7], "rotation": [0.0, 5.0, -90.0], "Hz": 5,
                            "configuration": {"CaptureWidth": 640, "CaptureHeight": 400, "FovAngle": 80}})
        pos = self.spec.spawn_positions[k]
        return {"agent_name": name, "agent_type": "BlueROV2", "location": [float(v) for v in pos],
                "rotation": [0.0, 0.0, float(self.spec.spawn_yaw_deg[k])], "control_scheme": 0, "sensors": sensors}

    def start(self) -> None:
        import holoocean

        scenario = {
            "name": f"holo_fleet_{self.spec.name}", "world": "OpenWater", "package_name": "Ocean",
            "main_agent": self.names[0], "ticks_per_sec": TICKS_PER_SEC, "frames_per_sec": False,
            "window_width": 1280, "window_height": 720,
            "agents": [self._agent_cfg(k) for k in range(self.spec.n)],
        }
        t0 = time.time()
        self.env = holoocean.make(scenario_cfg=scenario, show_viewport=not self.headless,
                                  ticks_per_sec=TICKS_PER_SEC, frames_per_sec=False)
        self.env.reset()
        LOG.info("HoloOcean ready in %.1fs", time.time() - t0)
        if self.spec.use_arena_gates:
            spawner = make_spawner(self.env)
            spawner.spawn_gate_bars([bar for vg in visual_gates(HORSESHOE_TRACK) for bar in vg.bars])
            self.gate_spawn_report = spawner.report.__dict__
        self._plan_blackouts()
        # settle a few ticks
        for _ in range(6):
            for k, n in enumerate(self.names):
                self.env.act(n, np.zeros(8))
            self._tick()
        self.t = 0.0
        self.wall_start = time.time()

    def _plan_blackouts(self) -> None:
        st = self.spec.stress
        for n in self.names:
            wins = []
            if st.blackout_every_s > 0:
                t = float(self.rng.uniform(5.0, st.blackout_every_s))
                while t < self.spec.duration_s:
                    wins.append((t, t + st.blackout_len_s))
                    t += float(self.rng.exponential(st.blackout_every_s)) + st.blackout_len_s
            self.blackouts[n] = wins

    def _blackout(self, name: str, t: float) -> bool:
        return any(a <= t <= b for a, b in self.blackouts.get(name, []))

    # ------------------------------------------------------------------ stepping
    def _tick(self) -> None:
        raw = self.env.tick()
        tick_t = self.t
        for n in self.names:
            st = raw.get(n, raw if len(self.names) == 1 else {})
            for key, val in st.items():
                if key.startswith("ProxSonar_") or key == "FrontSonar":
                    if self._blackout(n, tick_t):
                        continue                          # sensor outage: no new data, staleness grows
                    if key.startswith("ProxSonar_"):
                        val = self._ring_model(np.asarray(val, dtype=float))
                self.latest[n][key] = val
                self.stamp[n][key] = tick_t

    def _ring_model(self, r: np.ndarray) -> np.ndarray:
        r = r.copy().ravel()
        valid = r > 0
        r[valid] += self.rng.normal(0.0, self.cfg.perc.range_noise_std, size=int(valid.sum()))
        if self.spec.stress.beam_dropout > 0:
            drop = self.rng.random(r.shape) < self.spec.stress.beam_dropout
            r[drop] = -1.0
        return r

    def truth(self) -> Truth:
        n = len(self.names)
        P, Y, V, C = np.zeros((n, 3)), np.zeros(n), np.zeros((n, 3)), np.zeros(n, dtype=bool)
        for k, name in enumerate(self.names):
            pose = np.asarray(self.latest[name].get("PoseSensor"))
            P[k] = pose[:3, 3]
            Y[k] = math.degrees(math.atan2(pose[1, 0], pose[0, 0]))
            V[k] = np.asarray(self.latest[name].get("VelocitySensor", np.zeros(3))).ravel()[:3]
            C[k] = bool(np.asarray(self.latest[name].get("CollisionSensor", False)).any())
        released = np.array([self.t >= r for r in self.spec.release_s])
        drift = np.array([self.spec.current.drift_at(P[k], self.t) for k in range(n)])
        cmd = np.array([self.spec.current.command_at(P[k], self.t) for k in range(n)])
        return Truth(self.t, P, Y, V, C, released, drift, cmd)

    def frames(self) -> Dict[str, SensorFrame]:
        out = {}
        for name in self.names:
            data = {k: v for k, v in self.latest[name].items() if is_onboard(k)}
            stamp = {k: v for k, v in self.stamp[name].items() if is_onboard(k)}
            out[name] = SensorFrame(t=self.t, data=data, stamp=stamp)
        return out

    def step(self, commands: Dict[str, Optional[Dict[str, float]]], dt: float) -> None:
        n_ticks = max(1, int(round(dt * TICKS_PER_SEC)))
        truth_now = self.truth()
        for k, name in enumerate(self.names):
            self.env.set_ocean_currents(name, [float(v) for v in truth_now.current_cmd[k]])
            cmd = commands.get(name)
            if cmd is None:
                # launch cage: vehicle held at its launch pose until released (simulator side)
                self.env.agents[name].teleport([float(v) for v in self.spec.spawn_positions[k]],
                                               [0.0, 0.0, float(self.spec.spawn_yaw_deg[k])])
                self.env.act(name, np.zeros(8))
            else:
                self.env.act(name, thruster_command(cmd["surge"], cmd["sway"], cmd["heave"], cmd["yaw"],
                                                    self.cfg.plant.thruster_limit))
        for _ in range(n_ticks):
            self.t = round(self.t + 1.0 / TICKS_PER_SEC, 6)
            self._tick()

    def debug_image(self, key: str):
        return self.latest[self.names[-1]].get(key)

    def close(self) -> None:
        if self.env is not None:
            try:
                self.env.__exit__(None, None, None)
            except Exception as exc:  # pragma: no cover
                LOG.warning("HoloOcean close: %s", exc)
            self.env = None
