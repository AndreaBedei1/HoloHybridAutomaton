"""Run one scenario in HoloOcean (v2): controllers (onboard only) + referee (ground truth only).

Writes results/<run_id>/: run_config.json, events.jsonl, drone_<k>_state.jsonl,
referee_timeseries.csv, referee_metrics.json, sensor_audit.json, summary.csv, perf.json and (optionally)
camera / fleet-view frames for GIFs.  A run that does not reach its end writes ``status: INCOMPLETE``.
``noise`` (sim.sensor_noise.NoiseProfile) degrades the onboard sensors, ``seed`` overrides the scenario seed
(every seeded simulator-side effect); ``cfg.beacon`` enables the B2 one-way acoustic beacon (identity pings only).
"""

from __future__ import annotations

import csv
import dataclasses
import json
import time
from pathlib import Path
from typing import Callable, Dict, Optional

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.control.controller import DroneController
from holo_fleet.referee.referee import Referee
from holo_fleet.referee.sensor_audit import SensorAudit
from holo_fleet.sim.holo_env import HoloFleetSim
from holo_fleet.sim.scenarios import SCENARIOS, Scenario

DT = 0.1


def _jsonable(x):
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (set, frozenset)):
        return sorted(x)
    return str(x)


def _error_status(exc: BaseException, t: float) -> Dict:
    """Run status of an exception: SUT when a frame of the drones' own code (control, perception, automaton) is on
    the traceback, INFRA otherwise (simulator, transport)."""
    import traceback

    frames = traceback.extract_tb(exc.__traceback__)
    sut = any(any(part in f.filename.replace("\\", "/") for part in ("holo_fleet/control/", "holo_fleet/perception/",
                                                                     "holo_fleet/ha/")) for f in frames)
    return {"status": "ERROR", "error_kind": "SUT" if sut else "INFRA", "error": repr(exc), "t": round(float(t), 2),
            "traceback": "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))}


def beacon_identities(plans, names, plan) -> dict:
    """Pinger code (drone index) -> slot, for the receiver flying ``plan``: only the drones of its own formation (the
    plans built for one formation share its path).  Slot indices restart in every formation, so a code of another
    formation is not a neighbour and is ignored by the receiver."""
    return {names.index(q.drone_id): q.slot_index for q in plans if q.path is plan.path}


def run(name: str, out_root: Path, headless: bool = True, run_id: Optional[str] = None,
        duration: Optional[float] = None, ui: Optional[Callable] = None, frame_every_s: float = 0.5,
        cfg: FleetConfig = DEFAULT, noise=None, seed: Optional[int] = None, noise_drones=None,
        terminal: Optional[Dict] = None) -> Dict:
    """``noise_drones``: indices of the drones that receive ``noise`` (None: all; the others keep the deployed
    sensors).  ``terminal``: {"profile": sensor-profile name, "deadline_s": optional override} ends the run on SUCCESS,
    FAILURE or DEADLINE (referee/terminal.py, criteria v3) instead of at the scenario duration; the horizon is then the
    deadline, by default referee/criteria.deadline_v3 of the scenario and profile."""
    sc: Scenario = SCENARIOS[name](cfg)
    if sc.cfg_patch:
        cfg = dataclasses.replace(cfg, **sc.cfg_patch)
    if seed is not None:
        sc.sim.seed = int(seed)
    sc.sim.noise = noise
    sc.sim.noise_drones = None if noise_drones is None else tuple(int(k) for k in noise_drones)
    monitor = None
    if terminal is not None:
        from holo_fleet.referee.criteria import deadline_v3, nominal_end_s
        from holo_fleet.referee.terminal import TerminalMonitor

        t_nom = nominal_end_s(sc)                   # the planned end: the same T_nom as the offline replay
        duration = float(terminal.get("deadline_s") or deadline_v3(sc, cfg, terminal.get("profile")))
        sc.sim.duration_s = duration                # noise windows, stress and logs cover the whole horizon
    run_id = run_id or name
    out = Path(out_root) / run_id
    out.mkdir(parents=True, exist_ok=True)
    if frame_every_s != float("inf"):
        (out / "frames").mkdir(parents=True, exist_ok=True)
    T = float(duration or sc.duration_s)
    status = {"status": "INCOMPLETE", "reason": "started"}
    (out / "run_status.json").write_text(json.dumps(status), encoding="utf-8")
    sim = HoloFleetSim(sc.sim, cfg, headless=headless)
    wall0 = time.time()
    try:
        sim.start()
    except BaseException:
        try:
            if ui is not None and hasattr(ui, "close"):
                ui.close()
        finally:
            sim.close()
        raise
    try:                              # the drones' own construction: an error here is the system under test's
        ctrls = {p.drone_id: DroneController(p, cfg) for p in sc.plans}
        if cfg.beacon.enabled:        # pre-mission knowledge: the static beacon identity of every slot's drone
            for p in sc.plans:
                ctrls[p.drone_id].perception.acoustic.set_identities(beacon_identities(sc.plans, sim.names, p))
            if getattr(sim, "beacon", None) is not None:   # H6 payload: each drone's ONBOARD estimate and bound
                sim.beacon.payload_source = lambda k: ctrls[sim.names[k]].perception.payload()
    except Exception as exc:  # noqa: BLE001 - recorded, then propagated
        (out / "run_status.json").write_text(json.dumps(_error_status(exc, 0.0)), encoding="utf-8")
        try:
            if ui is not None and hasattr(ui, "close"):
                ui.close()
        finally:
            sim.close()
        raise
    audit = SensorAudit(sim.names, cfg, DT)
    truth_prev = None
    tmpl = sc.template.offsets() if sc.template is not None else None
    ref = Referee(sim.names, tmpl, sc.path, sc.judged_gates, cfg, sc.formation_enabled,
                  clock=sc.plans[0].clock if sc.formation_enabled else None, faults=getattr(sc.sim, "faults", ()))
    if terminal is not None:
        monitor = TerminalMonitor(sc, cfg, sim.names, duration, t_nom)
    cfg_dump = {"scenario": name, "title": sc.title, "description": sc.description, "focus": sc.focus,
                "duration_s": T, "n_drones": len(sim.names), "template": None if sc.template is None else sc.template.name,
                "gates": [g.gate_id for g in sc.judged_gates], "current": sc.sim.current.describe(),
                "intruders": list(getattr(sc.sim, "intruders", ())), "cfg_patch": sc.cfg_patch,
                "faults": list(getattr(sc.sim, "faults", ())),
                "seed": sc.sim.seed, "noise": None if noise is None else noise.as_dict(),
                "noise_drones": None if noise is None else sorted(getattr(sim, "noisy_idx", range(len(sim.names)))),
                "terminal": None if monitor is None else {"criteria": "v3", "t_nom": monitor.t_nom,
                                                           "deadline_s": monitor.deadline,
                                                           "profile": terminal.get("profile")},
                "heading_bias_deg": [round(float(b), 2) for b in getattr(sim, "heading_bias", ())],
                "disturbance_windows": sc.disturbance_windows, "stress": sc.sim.stress.__dict__,
                "setup": sim.setup_report, "comms_enabled": cfg.comms_enabled, "fleet_config": cfg.to_dict(),
                "plans": [{"drone": p.drone_id, "slot": p.slot_index, "queue_lateral": p.queue_lateral,
                           "queue_dz": p.queue_dz, "queue_s": round(p.queue_s, 3),
                           "static_rank": p.static_rank, "launch": np.round(p.launch_position, 3).tolist()}
                          for p in sc.plans],
                "slots": None if sc.template is None else [[sl.along, sl.lateral, sl.dz] for sl in sc.template.slots],
                "path": np.round(sc.path.waypoints, 3).tolist(), "path_depth": sc.path.depth_z,
                "clock": None if sc.plans[0].clock is None else dataclasses.asdict(sc.plans[0].clock)}
    (out / "run_config.json").write_text(json.dumps(cfg_dump, indent=1, default=_jsonable), encoding="utf-8")
    ev_f = open(out / "events.jsonl", "w", encoding="utf-8")
    st_f = {n: open(out / f"{n}_state.jsonl", "w", encoding="utf-8") for n in sim.names}
    ev_f.write(json.dumps({"t": 0.0, "type": "sim_started", "setup": sim.setup_report}, default=_jsonable) + "\n")
    rows = []
    n_steps = int(round(T / DT))
    last_frame_t = -1e9
    ctrl_ms = []
    fault_on = {i: False for i in range(len(getattr(sc.sim, "faults", ())))}
    try:
        for step in range(n_steps):
            step_events = []
            for i, f in enumerate(getattr(sc.sim, "faults", ())):         # ground truth, never seen by a controller
                on = f["t_on"] <= sim.t and (f.get("t_off") is None or sim.t < f["t_off"])
                if on != fault_on[i]:
                    fault_on[i] = on
                    fe = {"t": round(sim.t, 2), "drone": f["drone"], "source": "simulator",
                          "type": "FAULT_INJECTED" if on else "FAULT_CLEARED", "fault": f["type"],
                          "permanent": f.get("t_off") is None}
                    ev_f.write(json.dumps(fe) + "\n")
                    step_events.append(fe)
            frames = sim.frames()
            cmds = {}
            c0 = time.perf_counter()
            for nm, c in ctrls.items():
                cmds[nm] = c.step(frames[nm], DT)
            ctrl_ms.append(1000.0 * (time.perf_counter() - c0))
            if truth_prev is not None:               # referee side: frames vs the truth at the frame time
                audit.update(truth_prev, frames, ctrls)
            sim.step(cmds, DT)
            truth = sim.truth()
            truth_prev = truth
            modes = [ctrls[nm].ha.mode.value for nm in sim.names]
            row = ref.update(truth, modes, sc.disturbance_active(sim.t))
            for nm, c in ctrls.items():
                rec = dict(c.last_record)
                st_f[nm].write(json.dumps(rec, default=_jsonable) + "\n")
                for e in c.events:
                    ev_f.write(json.dumps({"drone": nm, **e}, default=_jsonable) + "\n")
                    step_events.append({"drone": nm, **e})
                c.events.clear()
            row["modes"] = "|".join(modes)
            live = ref.live()
            row["p1_ok"], row["p2_ok"] = int(live["p1_ok"]), int(live["p2_ok"])
            row["formation_state"], row["episodes"] = live["formation"] or "", live["episodes"]
            row["cur_x"], row["cur_y"], row["cur_z"] = (round(float(v), 3) for v in truth.current_drift[0])
            rows.append(row)
            want_frame = sim.t - last_frame_t >= frame_every_s
            if ui is not None:
                ui(sim=sim, scenario=sc, ctrls=ctrls, referee=ref, truth=truth, t=sim.t, out=out,
                   save_frame=want_frame, events=step_events)
            if want_frame:
                last_frame_t = sim.t
                img = sim.image("ChaseCamera")
                if img is not None:
                    import cv2

                    cv2.imwrite(str(out / "frames" / f"chase_{int(round(sim.t * 10)):05d}.jpg"),
                                np.asarray(img)[:, :, :3], [cv2.IMWRITE_JPEG_QUALITY, 85])
            if monitor is not None and monitor.update(sim.t, truth.positions, ref,
                                                      [ctrls[nm].last_record for nm in sim.names]):
                ev_f.write(json.dumps({"t": sim.t, "type": "TERMINAL", **monitor.summary()}, default=_jsonable) + "\n")
                break
        if monitor is not None and monitor.outcome is None:
            monitor.finalize(sim.t, ref, [ctrls[nm].last_record for nm in sim.names])
            ev_f.write(json.dumps({"t": sim.t, "type": "TERMINAL", **monitor.summary()}, default=_jsonable) + "\n")
        status = {"status": "COMPLETE"}
    except KeyboardInterrupt:
        status = {"status": "INCOMPLETE", "reason": "interrupted by the user", "t": sim.t}
    except Exception as exc:  # noqa: BLE001 - classified, recorded and not hidden as an infrastructure failure
        status = _error_status(exc, sim.t)
        # recorded before the exception propagates: an error of the drones' own code is a failure of the system
        # under test (robustness_metrics: SUT_ERROR), not an infrastructure problem to retry
        (out / "run_status.json").write_text(json.dumps(status), encoding="utf-8")
        raise
    finally:
        try:
            ev_f.write(json.dumps({"t": sim.t, "type": "run_finished", **status}) + "\n")
            ev_f.close()
            for f in st_f.values():
                f.close()
            perf = {**sim.perf(), "controller_ms_mean_all_drones": round(float(np.mean(ctrl_ms)), 2) if ctrl_ms else None,
                    "controller_hz": round(1.0 / DT, 1), "sonar_hz": cfg.perc.sonar.hz,
                    "wall_s": round(time.time() - wall0, 1), "sim_s": round(sim.t, 2)}
        finally:
            try:
                if ui is not None and hasattr(ui, "close"):
                    ui.close()
            finally:
                sim.close()
    if rows:
        keys = sorted({k for r in rows for k in r})
        with open(out / "referee_timeseries.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
    metrics = ref.metrics()
    det = {nm: c.ha.determinism_violations for nm, c in ctrls.items()}
    obs_inc = {nm: c.observation_violations for nm, c in ctrls.items()}
    rank_uses = sum(c.perception.gp.rank_uses for c in ctrls.values())
    occ_timeouts = sum(c.perception.gp.occ_timeouts for c in ctrls.values())
    env_violations = 0
    with open(out / "events.jsonl", encoding="utf-8") as f:
        for line in f:
            if '"ENVELOPE_VIOLATION"' in line:
                env_violations += 1
    beacon = None if getattr(sim, "beacon", None) is None else sim.beacon.summary()
    metrics["run"] = {**status, "scenario": name, "sim_time_s": round(sim.t, 2), "determinism_violations": det,
                      "observation_consistency_violations": obs_inc,
                      # B0: no message at all; B2: every one-way identity ping counts (no payload, no reply)
                      "comms_enabled": cfg.comms_enabled, "inter_agent_messages": 0 if beacon is None else beacon["transmissions"],
                      "message_content": "none" if beacon is None else
                          "static identity only (one-way ping; receiver-side OWTT range + array bearing)",
                      "acoustic_exit_witnesses": sum(c.perception.gp.exit_kind["ACOUSTIC"] for c in ctrls.values()),
                      "sonar_exit_witnesses": sum(c.perception.gp.exit_kind["SONAR"] for c in ctrls.values()),
                      "acoustic_tracker": ({k: sum(c.perception.acoustic.stats[k] for c in ctrls.values())
                                            for k in next(iter(ctrls.values())).perception.acoustic.stats}
                                           if cfg.beacon.enabled else None),
                      "ground_truth_used_by_controllers": any(c.uses_ground_truth for c in ctrls.values()),
                      "static_rank_uses": rank_uses, "occupancy_timeouts": occ_timeouts,
                      "self_declared_envelope_violations": env_violations, "perf": perf,
                      "seed": sc.sim.seed, "noise_profile": None if noise is None else noise.name,
                      "noise_wrapper_stats": dict(getattr(sim, "noise_stats", {})),
                      "beacon": beacon, "terminal": None if monitor is None else monitor.summary()}
    metrics["sensor_audit"] = audit.summary()
    metrics["envelope"] = envelope_block(out, metrics["envelope"], cfg)
    (out / "referee_metrics.json").write_text(json.dumps(metrics, indent=1, default=_jsonable), encoding="utf-8")
    (out / "perf.json").write_text(json.dumps(perf, indent=1), encoding="utf-8")
    (out / "run_status.json").write_text(json.dumps(status), encoding="utf-8")
    summ = summary_row(name, metrics)
    with open(out / "summary.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(summ))
        w.writeheader()
        w.writerow(summ)
    return metrics


def summary_row(name: str, m: Dict) -> Dict:
    p3 = m["P3_formation_recovery"]
    rec = [e["recovery_time_s"] for e in p3["episodes"] if e["recovery_time_s"] is not None]
    deg = m.get("P3_degraded") or {}
    return {"scenario": name, "status": m["run"]["status"], "sim_time_s": m["run"]["sim_time_s"],
            "P1_holds": m["P1_separation"]["holds"], "min_distance": m["P1_separation"]["min_distance"],
            "P2_holds": m["P2_mutual_exclusion"]["holds"],
            "max_occupancy": max(m["P2_mutual_exclusion"]["max_occupancy"].values(), default=None),
            "P3_episodes": p3["n_episodes"], "P3_all_recovered": p3["all_recovered_within_run"],
            "max_recovery_time_s": max(rec) if rec else None,
            "P3_degraded_holds": deg.get("holds"), "absent_at_end": "+".join(deg.get("absent_at_end") or []),
            "collisions": m["P1_separation"]["physical_contacts"] + len(m["P1_separation"]["collision_sensor_edges"]),
            "messages": m["run"]["inter_agent_messages"], "inside_envelope": m["envelope"]["inside_envelope"],
            "static_rank_uses": m["run"]["static_rank_uses"]}


def envelope_block(run_dir: Path, raw: Dict, cfg: FleetConfig = DEFAULT) -> Dict:
    """referee_metrics['envelope']: raw current statistics + the control-feasible verdict."""
    from holo_fleet.referee.envelope import evaluate_run

    ev = evaluate_run(Path(run_dir), cfg)
    return {"max_horizontal_drift": raw.get("max_horizontal_drift"), "max_vertical_drift": raw.get("max_vertical_drift"),
            **{k: v for k, v in ev.items() if k != "drones"}, "drones": ev["drones"]}


def _envelope_line(m: Dict) -> str:
    """Envelope verdict (control-feasible) next to the vehicles' own view (self-declared
    ENVELOPE_VIOLATION: persistent thrust saturation or a steady command beyond the authority)."""
    e = m["envelope"]
    n_self = m["run"]["self_declared_envelope_violations"]
    n_fault = e.get("self_declared_by_faulty_drones", 0)
    own = f"; vehicles: {n_self} self-declared ENVELOPE_VIOLATION" if n_self else "; vehicles: ENVELOPE_OK"
    if n_fault:
        own += f" ({n_fault} by the drone with the injected thruster fault)"
    if "verdict" not in e:
        return "envelope not evaluated (run without logs)"
    if e["verdict"] == "OUTSIDE":
        return (f"OUTSIDE THE CONTROL-FEASIBLE ENVELOPE (required command up to {e['max_required_over_authority']:.2f} "
                f"x authority, current up to {e['max_current_m_s']:.2f} m/s){own}")
    if e["verdict"] == "LIMIT":
        return f"PASS at the limit (within the plant-model tolerance){own}"
    return f"PASS (inside the control-feasible envelope){own}"


def print_summary(name: str, m: Dict, assumptions: Optional[str] = None) -> str:
    p1, p2, p3 = m["P1_separation"], m["P2_mutual_exclusion"], m["P3_formation_recovery"]
    bar = "=" * 49
    lines = [bar, "SCENARIO COMPLETE" if m["run"]["status"] == "COMPLETE" else f"SCENARIO {m['run']['status']}", bar, "",
             "P1 INTER-VEHICLE SEPARATION", "PASS" if p1["holds"] else "FAIL",
             f"minimum distance: {p1['min_distance']} m", f"required: {p1['d_safe']} m", ""]
    if p2["gates"]:
        lines += ["P2 CRITICAL REGION", "PASS" if p2["holds"] else "FAIL",
                  f"maximum occupancy: {max(p2['max_occupancy'].values())}",
                  f"entry order: {p2['entry_order']}", f"static rank used: {m['run']['static_rank_uses']} times", ""]
    else:
        lines += ["P2 CRITICAL REGION", "n/a (no gate in this scenario)", ""]
    if p3["enabled"]:
        lines += ["P3 FORMATION RECOVERY",
                  ("PASS" if p3["all_recovered_within_run"] else "NOT RECOVERED WITHIN THE RUN") +
                  ("" if p3["n_episodes"] else " (formation never lost)")]
        for e in p3["episodes"]:
            lines += [f"formation lost at: {e['t_lost']} s", f"formation recovered at: {e['t_recovered']} s",
                      f"recovery time: {e['recovery_time_s']} s"]
        lines.append("")
        deg = m.get("P3_degraded") or {}
        if deg.get("exercised"):
            lines += ["P3-deg DEGRADED FORMATION (remaining drones on their original slots)",
                      "PASS" if deg["holds"] else "NOT RE-FORMED WITHIN THE RUN"]
            for e in deg["absence_log"]:
                lines.append(f"{e['drone']} {'absent (slot vacant)' if e['event'] == 'absent' else 'back'} at {e['t']} s")
            lines += [f"final formation error of the present drones: {deg['final_form_err_present']} m", ""]
    else:
        lines += ["P3 FORMATION RECOVERY", "n/a (no formation judged in this scenario)", ""]
    coll = p1["physical_contacts"] + len(p1["collision_sensor_edges"])
    det = sum((m["run"].get("determinism_violations") or {}).values())
    inc = m["run"].get("observation_consistency_violations")
    comm = (f"{m['run']['inter_agent_messages']} messages" if not m["run"].get("beacon") else
            f"{m['run']['inter_agent_messages']} one-way identity pings (no payload, no reply)")
    lines += ["COLLISIONS", str(coll), "", "COMMUNICATION", comm, "",
              f"AUTOMATON DETERMINISM VIOLATIONS: {det}",
              f"OBSERVATION CONSISTENCY VIOLATIONS: {'n/a' if inc is None else sum(inc.values())}", "",
              "GROUND TRUTH USED BY CONTROLLERS", "YES" if m["run"]["ground_truth_used_by_controllers"] else "NO", "",
              "FORMAL ASSUMPTIONS", assumptions or _envelope_line(m), "", bar]
    text = "\n".join(lines)
    print(text)
    return text
