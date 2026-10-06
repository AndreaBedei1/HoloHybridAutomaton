"""Run one scenario in HoloOcean and write every artifact of a run directory."""

from __future__ import annotations

import csv
import datetime as _dt
import json
import logging
import math
import os
import platform
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.control.controller import DroneController
from holo_fleet.referee.referee import Referee
from holo_fleet.sim.scenarios import SCENARIOS, ScenarioSpec

LOG = logging.getLogger("holo_fleet.runner")

PERTURBING_MODES = {"SEPARATION_WARNING", "COLLISION_AVOIDANCE", "GATE_APPROACH", "GATE_YIELD", "GATE_PASS",
                    "FAILSAFE_HOLD_OR_RETREAT"}


class JsonlWriter:
    def __init__(self, path: Path):
        self.f = open(path, "w", encoding="utf-8")

    def write(self, obj: Dict[str, Any]) -> None:
        self.f.write(json.dumps(obj, separators=(",", ":")) + "\n")

    def close(self) -> None:
        self.f.close()


def _save_png(img, path: Path) -> None:
    """Save a camera frame (JPEG for the periodic frames to keep run folders small)."""
    import cv2

    a = np.asarray(img)
    if a.ndim == 3 and a.shape[2] == 4:
        a = a[:, :, :3]
    if path.suffix.lower() in (".jpg", ".jpeg"):
        cv2.imwrite(str(path), a, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    else:
        cv2.imwrite(str(path), a)


def _save_sonar(img, path: Path) -> None:
    import cv2

    a = np.asarray(img, dtype=float)
    a = np.clip(a / max(a.max(), 1e-6), 0, 1)
    a = (255 * a).astype(np.uint8)[::-1, :]          # far range at the top
    a = cv2.applyColorMap(a, cv2.COLORMAP_INFERNO)
    a = cv2.resize(a, (384, 384), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(str(path), a)


def localized_disturbance(spec: ScenarioSpec, P: np.ndarray, t: float) -> bool:
    for c in spec.current.components:
        if c.kind == "uniform":
            continue
        for k in range(len(P)):
            if np.linalg.norm(c.at(P[k], t)) > 0.05:
                return True
    return False


def run(scenario: str, out_root: str = "results", seed: int = 0, headless: bool = True, cfg: FleetConfig = DEFAULT,
        run_id: Optional[str] = None, duration: Optional[float] = None, frame_every_s: float = 1.0,
        early_stop: bool = True, **scenario_kwargs) -> Path:
    from holo_fleet.sim.holo_env import HoloFleetSim

    spec: ScenarioSpec = SCENARIOS[scenario](seed=seed, cfg=cfg, **scenario_kwargs)
    if duration is not None:
        spec.duration_s = duration
    run_id = run_id or f"{spec.name}_s{seed}_{_dt.datetime.now().strftime('%Y%m%d_%H%M%S')}"
    run_dir = Path(out_root) / run_id
    (run_dir / "frames").mkdir(parents=True, exist_ok=True)
    (run_dir / "sensors").mkdir(parents=True, exist_ok=True)
    dt = cfg.env.dt

    rng = np.random.default_rng(seed + 777)
    nav_offsets = []
    for _ in spec.plans:
        e = rng.normal(size=3) * np.array([1, 1, 0])
        n = np.linalg.norm(e)
        nav_offsets.append(e / n * spec.stress.nav_init_error_m if n > 0 and spec.stress.nav_init_error_m > 0 else np.zeros(3))

    run_config = {
        "run_id": run_id, "scenario": scenario, "spec_name": spec.name, "description": spec.description,
        "seed": seed, "n_drones": spec.n, "duration_s": spec.duration_s, "dt": dt,
        "comms_enabled": bool(spec.comms_enabled), "release_s": spec.release_s,
        "current_components": spec.current.describe(), "stress": spec.stress.__dict__,
        "mission_gates": spec.mission_gate_ids, "params": spec.params,
        "drones": [{"id": p.drone_id, "slot": p.slot_index, "slot_offset": p.my_slot.__dict__,
                    "launch_position": p.launch_position.tolist(), "launch_yaw_deg": p.launch_yaw_deg,
                    "static_rank": p.static_rank, "failsafe_layer_dz": p.failsafe_layer_dz,
                    "nav_init_offset": nav_offsets[k].tolist()} for k, p in enumerate(spec.plans)],
        "path_waypoints": spec.plans[0].path.waypoints.tolist(), "path_depth_z": spec.plans[0].path.depth_z,
        "fleet_config": cfg.to_dict(),
        "host": platform.node(), "started": _dt.datetime.now().isoformat(),
        "holoocean_world": "OpenWater (package Ocean)", "arena": "marine_race_arena / Horseshoe Bay (reused)",
    }
    (run_dir / "run_config.json").write_text(json.dumps(run_config, indent=2))

    sim = HoloFleetSim(spec, cfg, headless=headless)
    mission_gates = [g for g in spec.plans[0].structures if g.gate_id in spec.mission_gate_ids]
    referee = Referee(spec.plans, mission_gates, cfg)
    controllers: Dict[str, DroneController] = {}
    w_state = {p.drone_id: JsonlWriter(run_dir / f"{p.drone_id}_state.jsonl") for p in spec.plans}
    w_obs = {p.drone_id: JsonlWriter(run_dir / f"{p.drone_id}_observations.jsonl") for p in spec.plans}
    w_act = {p.drone_id: JsonlWriter(run_dir / f"{p.drone_id}_actions.jsonl") for p in spec.plans}
    w_ev = JsonlWriter(run_dir / "events.jsonl")
    wall0 = time.time()
    last_frame_t = -1e9
    last_sensor_t = -1e9
    calm_done_since: Optional[float] = None
    n_steps = 0
    status = "completed"
    try:
        sim.start()
        w_ev.write({"t": 0.0, "type": "sim_started", "gate_spawn": sim.gate_spawn_report,
                    "wall_setup_s": round(time.time() - wall0, 1)})
        while sim.t < spec.duration_s:
            frames = sim.frames()
            commands: Dict[str, Any] = {}
            modes = []
            for k, plan in enumerate(spec.plans):
                name = plan.drone_id
                if sim.t + 1e-9 < spec.release_s[k]:
                    commands[name] = None
                    modes.append("NOT_RELEASED")
                    continue
                if name not in controllers:
                    controllers[name] = DroneController(plan, cfg, nav_init_offset=nav_offsets[k])
                    w_ev.write({"t": sim.t, "type": "released", "drone": name})
                out = controllers[name].step(frames[name], dt)
                commands[name] = out["command"]
                modes.append(out["mode"])
                obs = out["observation"]
                w_state[name].write({"t": round(sim.t, 3), "mode": out["mode"], "committed": out["committed"],
                                     "gate_index": out["gate_index"], "nav_p": obs["nav"]["p"],
                                     "nav_yaw_deg": obs["nav"]["yaw_deg"], "edge": out["transition"]["edge"],
                                     "enabled_edges": out["transition"]["enabled_edges"],
                                     "determinism_violations": controllers[name].ha.determinism_violations})
                w_obs[name].write({"t": round(sim.t, 3), "abstract": out["abstract"], "local": obs})
                w_act[name].write({"t": round(sim.t, 3), "mode": out["mode"], "flow": out["flow"],
                                   "command": {k2: round(v, 4) for k2, v in out["command"].items()}})
                for ev in out["events"]:
                    w_ev.write(ev)
            sim.step(commands, dt)
            n_steps += 1
            truth = sim.truth()
            pert = any(m in PERTURBING_MODES or m == "NOT_RELEASED" for m in modes) or \
                localized_disturbance(spec, truth.positions, truth.t)
            referee.update(truth, modes, perturbation=pert)
            # imagery
            if sim.t - last_frame_t >= frame_every_s - 1e-9:
                last_frame_t = sim.t
                for key in ("ChaseCamera", "TopCamera"):
                    img = sim.debug_image(key)
                    if img is not None:
                        _save_png(img, run_dir / "frames" / f"{key}_{int(round(sim.t * 10)):05d}.jpg")
            if sim.t - last_sensor_t >= 10.0:
                last_sensor_t = sim.t
                for name in sim.names:
                    cam = sim.latest[name].get("FrontCamera")
                    if cam is not None:
                        _save_png(cam, run_dir / "sensors" / f"{name}_FrontCamera_{int(sim.t):04d}.png")
                    son = sim.latest[name].get("FrontSonar")
                    if son is not None:
                        _save_sonar(son, run_dir / "sensors" / f"{name}_FrontSonar_{int(sim.t):04d}.png")
                        np.save(run_dir / "sensors" / f"{name}_FrontSonar_{int(sim.t):04d}.npy", np.asarray(son))
            if n_steps % 100 == 0:
                LOG.info("t=%.1f wall=%.0fs modes=%s", sim.t, time.time() - wall0, modes)
            # early stop: everybody done and (formation recovered or no formation) for 8 s
            if early_stop and controllers and len(controllers) == spec.n and \
                    all(c.mission_complete for c in controllers.values()) and \
                    all(m in ("FORMATION_FOLLOW",) for m in modes):
                calm_done_since = sim.t if calm_done_since is None else calm_done_since
                if sim.t - calm_done_since >= 8.0:
                    status = "completed_early_stop"
                    break
            else:
                calm_done_since = None
    except Exception as exc:
        status = f"error: {type(exc).__name__}: {exc}"
        LOG.exception("run failed")
        raise
    finally:
        sim.close()
        for w in list(w_state.values()) + list(w_obs.values()) + list(w_act.values()):
            w.close()
        metrics = referee.metrics()
        det = {n: c.ha.determinism_violations for n, c in controllers.items()}
        metrics["run"] = {"status": status, "sim_time_s": round(sim.t, 2), "steps": n_steps,
                          "wall_time_s": round(time.time() - wall0, 1),
                          "determinism_monitor_violations": det,
                          "mission_complete": {n: c.mission_complete for n, c in controllers.items()},
                          "comms_enabled": bool(spec.comms_enabled),
                          "inter_agent_messages_sent": 0 if not spec.comms_enabled else None}
        (run_dir / "referee_metrics.json").write_text(json.dumps(metrics, indent=2, default=float))
        if referee.rows:
            keys = list(dict.fromkeys(k for r in referee.rows for k in r.keys()))
            with open(run_dir / "referee_timeseries.csv", "w", newline="") as f:
                wr = csv.DictWriter(f, fieldnames=keys)
                wr.writeheader()
                wr.writerows(referee.rows)
        decisions = {}
        for c in controllers.values():
            for e in c.events:
                if e.get("type") == "decision":
                    decisions[e["decision"]] = decisions.get(e["decision"], 0) + 1
        summary = {
            "run_id": run_id, "scenario": spec.name, "seed": seed, "status": status, "sim_time_s": round(sim.t, 1),
            "n_drones": spec.n, "comms": int(bool(spec.comms_enabled)),
            "min_pair_distance_m": metrics["P1_separation"]["min_distance_overall"],
            "P1_holds": metrics["P1_separation"]["holds"],
            "P1_violations": metrics["P1_separation"]["violations_d_lt_d_safe"],
            "P2_holds": metrics["P2_mutual_exclusion"]["holds"],
            "P2_max_occupancy": max(metrics["P2_mutual_exclusion"]["max_occupancy"].values(), default=None),
            "P3_holds": metrics["P3_formation_recovery"]["holds"],
            "P3_episodes": metrics["P3_formation_recovery"]["n_episodes"],
            "P3_max_recovery_s": metrics["P3_formation_recovery"]["max_recovery_time_s"],
            "final_form_err_m": metrics["P3_formation_recovery"]["final_form_err"],
            "max_drift_m_s": metrics["envelope"]["max_effective_drift_applied"],
            "inside_envelope": metrics["envelope"]["inside_envelope"],
            "determinism_violations": sum(det.values()),
            "decisions": json.dumps(decisions),
        }
        with open(run_dir / "summary.csv", "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=list(summary.keys()))
            wr.writeheader()
            wr.writerow(summary)
        w_ev.write({"t": sim.t, "type": "run_finished", "status": status})
        w_ev.close()
        LOG.info("run %s finished: %s", run_id, json.dumps(summary))
    return run_dir
