"""Envelope verdict of a run (DI-27), from its logs and the true current.

For every drone and control step:

* the true current at the drone's true position (the logged current field);
* if the step is judged (not an avoidance manoeuvre): the control-feasibility of the velocity the drone was
  asked to deliver (logged ``v_cmd``), on its body axes (logged heading), with the authority of its mode
  (control/current_envelope.check_current, plant curve);
* the exercised range of the current (|w_h| <= current_validated_max, |w_z| <= current_vertical_max).

The persistence rule is the onboard one (leaky, 4 s).  Verdict:

* OUTSIDE - the current leaves the exercised range, or some drone needs more than the authority plus the
  plant-model tolerance persistently;
* LIMIT   - inside the range, and the requirement exceeds the authority persistently but only within the
  plant-model tolerance (the run is at the edge of the envelope: the model cannot decide);
* INSIDE  - otherwise.

The vehicles' own verdict (self-declared ENVELOPE_VIOLATION, persistent saturation) is reported next to it:
OUTSIDE and a self-declared violation are expected to go together.  Ground truth is used here for judging
only (offline); nothing is fed back to a controller.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Dict

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.control.current_envelope import Persistence, check_current
from holo_fleet.control.lowlevel import AUTHORITY
from holo_fleet.sim.currents import CurrentComponent, CurrentField

AVOIDANCE = ("SEPARATION_WARNING", "COLLISION_AVOIDANCE")
DT = 0.1


def _authority(mode: str) -> float:
    return AUTHORITY["brake"] if mode == "FAILSAFE_HOLD_OR_RETREAT" else AUTHORITY["nominal"]


def evaluate_run(run: Path, cfg: FleetConfig = DEFAULT) -> Dict:
    run = Path(run)
    rc = json.loads((run / "run_config.json").read_text(encoding="utf-8"))
    n = rc["n_drones"]
    rows = list(csv.DictReader(open(run / "referee_timeseries.csv", encoding="utf-8")))
    field = CurrentField([CurrentComponent(**c) for c in rc.get("current", [])])
    tol = cfg.plant.cmd_model_tolerance
    # a drone with an injected actuator fault (simulator side, DI-29) declares a violation because its thrusters do
    # not respond, not because of the current: those declarations do not count for the current envelope
    faulty = {f["drone"] for f in rc.get("faults", []) or []}
    declared = [json.loads(line).get("drone") for line in open(run / "events.jsonl", encoding="utf-8")
                if '"ENVELOPE_VIOLATION"' in line]
    self_declared = len(declared)
    by_faulty = sum(1 for d in declared if d in faulty)
    drones, any_out_range, any_out, any_over = {}, False, False, False
    for k in range(n):
        st = [json.loads(line) for line in open(run / f"drone_{k}_state.jsonl", encoding="utf-8")]
        m = min(len(st), len(rows))
        p_out, p_over = Persistence(), Persistence()
        first_out = first_over = None
        mx = {"magnitude": 0.0, "vertical": 0.0, "ratio": 0.0}
        worst = {"t": None, "head": 0.0, "lateral": 0.0, "requested_speed": 0.0}
        t_over = t_out = 0.0
        out_range = False
        for i in range(m):
            r, row = st[i], rows[i]
            p = np.array([float(row[f"x{k}"]), float(row[f"y{k}"]), float(row[f"z{k}"])])
            w = field.drift_at(p, float(row["t"]))
            mag, wz = float(np.linalg.norm(w[:2])), float(w[2])
            mx["magnitude"], mx["vertical"] = max(mx["magnitude"], mag), max(mx["vertical"], abs(wz))
            if mag > cfg.env.current_validated_max + 1e-9 or abs(wz) > cfg.env.current_vertical_max + 1e-9:
                out_range = True
            judged = r["mode"] not in AVOIDANCE
            over = out = False
            if judged:
                c = check_current(w, r["v_cmd"], cfg, heading=math.radians(r["nav_yaw_deg"]), authority=_authority(r["mode"]))
                ratio = c.required_cmd / c.authority
                if ratio > mx["ratio"]:          # the binding step: components where the requirement peaks
                    mx["ratio"] = ratio
                    worst = {"t": round(r["t"], 1), "head": c.head, "lateral": c.lateral,
                             "requested_speed": float(np.linalg.norm(np.asarray(r["v_cmd"], float)[:2]))}
                over, out = ratio > 1.0, ratio > 1.0 + tol
                t_over += DT * over
                t_out += DT * out
            if not p_over.update(over, DT) and first_over is None:
                first_over = r["t"]
            if not p_out.update(out, DT) and first_out is None:
                first_out = r["t"]
        drones[f"drone_{k}"] = {
            "max_current_m_s": round(mx["magnitude"], 3), "max_vertical_m_s": round(mx["vertical"], 3),
            "max_required_over_authority": round(mx["ratio"], 3),
            "worst_step": {"t": worst["t"], "head_m_s": round(worst["head"], 3), "lateral_m_s": round(worst["lateral"], 3),
                           "requested_speed_m_s": round(worst["requested_speed"], 3)},
            "time_over_authority_s": round(t_over, 1), "time_over_authority_plus_tolerance_s": round(t_out, 1),
            "persistent_infeasibility_at_s": first_out, "persistent_limit_at_s": first_over,
            "outside_exercised_range": out_range}
        any_out_range |= out_range
        any_out |= first_out is not None
        any_over |= first_over is not None
    verdict = "OUTSIDE" if (any_out_range or any_out) else ("LIMIT" if any_over else "INSIDE")
    wname, wd = max(drones.items(), key=lambda kv: kv[1]["max_required_over_authority"])
    return {
        "definition": "control-feasible current envelope (DI-27): steady command for the requested velocity within "
                      "the authority (plant model, tolerance %.2f), current within the exercised range" % tol,
        "current_validated_max_m_s": cfg.env.current_validated_max,
        "current_vertical_max_m_s": cfg.env.current_vertical_max,
        "max_current_m_s": round(max(d["max_current_m_s"] for d in drones.values()), 3),
        "max_vertical_m_s": round(max(d["max_vertical_m_s"] for d in drones.values()), 3),
        "max_required_over_authority": wd["max_required_over_authority"],
        "worst_step": {"drone": wname, **wd["worst_step"]},
        "verdict": verdict, "inside_envelope": verdict != "OUTSIDE",
        "self_declared_envelope_violations": self_declared,
        "self_declared_by_faulty_drones": by_faulty,
        "coherent_with_vehicles": (verdict == "OUTSIDE") == (self_declared - by_faulty > 0),
        "drones": drones,
    }
