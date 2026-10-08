"""Measure, on the demonstration logs, the assumptions the formal proofs rest on (offline, ground truth allowed).

    python scripts/validate_assumptions_v2.py

Reads results/v2/demos/<scenario>/ (controller logs = onboard knowledge, referee time series = ground
truth) and writes results/v2/ASSUMPTIONS.{json,md}.  Nothing here feeds back into a controller.
"""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.config import DEFAULT  # noqa: E402
from holo_fleet.sim.scenarios import SCENARIOS  # noqa: E402

DEMOS = ROOT / "results" / "v2" / "demos"
OUT = ROOT / "results" / "v2"
ORDER = ["p1_head_on", "p1_vertical_escape", "p1_two_lines", "p1_close_encounter", "formation_triangle",
         "formation_square", "formation_six", "formation_recovery_head_current", "formation_gust", "gate_single",
         "integrated_short"]


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def load(name):
    d = DEMOS / name
    if not (d / "run_status.json").exists() or json.loads((d / "run_status.json").read_text())["status"] != "COMPLETE":
        return None
    cfg = json.loads((d / "run_config.json").read_text(encoding="utf-8"))
    n = cfg["n_drones"]
    rows = list(csv.DictReader(open(d / "referee_timeseries.csv", encoding="utf-8")))
    t = np.array([_f(r["t"]) for r in rows])
    P = np.stack([np.array([[_f(r[f"x{k}"]), _f(r[f"y{k}"]), _f(r[f"z{k}"])] for r in rows]) for k in range(n)], axis=1)
    I = None
    if "ix0" in rows[0]:
        I = np.array([[_f(r["ix0"]), _f(r["iy0"]), _f(r["iz0"])] for r in rows])
    states = [[json.loads(l) for l in open(d / f"drone_{k}_state.jsonl", encoding="utf-8")] for k in range(n)]
    met = json.loads((d / "referee_metrics.json").read_text(encoding="utf-8"))
    return {"name": name, "n": n, "t": t, "P": P, "I": I, "states": states, "cfg": cfg, "met": met}


# stress tests that are OUT of the claimed envelope by design (declared in the scenario description)
DECLARED_OUT = {"formation_gust"}
T_EXIT = 4.0       # EnvelopeMonitor.t_exit: a drone leaves a self-declared violation after 4 s unsaturated


R_SAFETY = 5.0     # the guards can only be affected below d_warning_exit + max looseness of the bound (2.7 + 1.2 m)
T_START = 0.3      # first two captures: an echo needs 2 of 3 captures to be confirmed


def check_run(run) -> dict:
    n, P, t, I = run["n"], run["P"], run["t"], run["I"]
    env, G = DEFAULT.env, DEFAULT.gate
    res = {}
    # A1 onboard distance <= true distance to the nearest other hull (soundness + completeness within R_SAFETY);
    # misses farther away (weak returns near 8 m, hulls masked by the gate bars) are counted separately
    worst, viol, samples, missed, far_missed = 9.0, 0, 0, 0, 0
    max_age = 0.0
    nav_err = 0.0
    for k in range(n):
        st = run["states"][k]
        m = min(len(st), len(t))
        for i in range(m):
            r = st[i]
            others = [np.linalg.norm(P[i, j] - P[i, k]) for j in range(n) if j != k]
            if I is not None:
                others.append(np.linalg.norm(I[i] - P[i, k]))
            d_true = min(others) if others else math.inf
            if t[i] < T_START:
                continue
            if d_true > R_SAFETY:
                if d_true <= 8.0 and (r.get("d_min") is None or r["d_min"] > d_true):
                    far_missed += 1
                continue
            samples += 1
            d_on = r.get("d_min")
            if d_on is None:
                missed += 1
                continue
            worst = min(worst, d_true - d_on)
            viol += int(d_on > d_true + 1e-6)
            ages = [v["age"] for v in r["sectors"].values() if v.get("healthy")]
            if ages:
                max_age = max(max_age, max(ages))
            nav_err = max(nav_err, float(np.linalg.norm(np.asarray(r["nav_p"]) - P[i, k])))
    res["A1_onboard_distance_le_true"] = {"samples_within_5m": samples, "violations": viol, "no_target": missed,
                                          "min_margin_m": round(float(worst), 3) if samples else None,
                                          "missed_or_late_5_to_8m": far_missed}
    res["A2_sonar_age_max_s"] = round(max_age, 3)
    res["A7_nav_error_max_m"] = round(nav_err, 3)
    # A3 closing speed of every pair <= c_max
    dt = np.diff(t)
    cmax = 0.0
    for a in range(n):
        for b in range(a + 1, n):
            d = np.linalg.norm(P[:, a] - P[:, b], axis=1)
            c = -np.diff(d) / np.maximum(dt, 1e-6)
            cmax = max(cmax, float(np.nanmax(c)))
    res["A3_closing_speed_max_m_s"] = round(cmax, 3)
    # gate assumptions
    sc = SCENARIOS[run["name"]](DEFAULT)
    if sc.judged_gates:
        g = sc.judged_gates[0]
        v_in, queued_min, sw_queued = [], math.inf, 0
        m = min(min(len(st) for st in run["states"]), len(t)) - 1
        for i in range(m):
            q = [k for k in range(n) if run["states"][k][i]["mode"] == "GATE_YIELD" and run["states"][k][i]["gate"].get("at_queue")]
            for a in range(len(q)):
                for b in range(a + 1, len(q)):
                    queued_min = min(queued_min, float(np.linalg.norm(P[i, q[a]] - P[i, q[b]])))
            for k in range(n):
                gf = g.to_gate_frame(P[i, k])
                if abs(gf[0]) <= G.cr_half_len and abs(gf[1]) <= G.cr_half_width:
                    gf2 = g.to_gate_frame(P[i + 1, k])
                    v_in.append((gf2[0] - gf[0]) / max(t[i + 1] - t[i], 1e-6))
        res["A4_queued_pair_min_distance_m"] = None if queued_min == math.inf else round(queued_min, 3)
        res["A5_speed_inside_CR_min_m_s"] = round(float(np.min(v_in)), 3) if v_in else None
    # P3 A4: onboard formation error in the formation scenarios (no gate: the gate procedure breaks it on purpose)
    if sc.formation_enabled and sc.template is not None and not sc.judged_gates:
        fe = [r["form"]["form_err"] for k in range(n) for r in run["states"][k] if r.get("form")]
        res["formation_err_onboard_p95_m"] = round(float(np.percentile(fe, 95)), 3) if fe else None
    # envelope: E1 the current (referee), E2 the vehicle's own view (persistent actuator saturation, i.e. the
    # current is stronger than the drone: onboard proxy of the unrejected-drift bound w_drift_max of P1)
    m, r = run["met"], run["met"]["run"]
    res["E1_drift"] = {"max_horizontal_m_s": m["envelope"]["max_horizontal_drift"],
                       "max_vertical_m_s": m["envelope"]["max_vertical_drift"],
                       "inside": m["envelope"]["inside_envelope"], "declared_out_of_envelope": run["name"] in DECLARED_OUT}
    res["E2_self_declared_envelope_violations"] = r["self_declared_envelope_violations"]
    res["saturated_time_max_s"] = round(max(0.1 * sum(bool(x.get("saturated")) for x in st) for st in run["states"]), 1)
    res["determinism_violations"] = sum(r["determinism_violations"].values())
    inc = r.get("observation_consistency_violations")
    res["observation_consistency_violations"] = None if inc is None else sum(inc.values())
    fs = [x["t"] for st in run["states"] for x in st if x["mode"] == "FAILSAFE_HOLD_OR_RETREAT"]
    res["failsafe_steps"] = len(fs)
    # P3 A1 (every perturbation ends) and A3 (no FAILSAFE once it has ended, after the monitor's t_exit)
    if sc.formation_enabled and sc.template is not None:
        t_end = max((b for _a, b in run["cfg"].get("disturbance_windows") or []), default=0.0)
        res["P3_A1_perturbations_end_s"] = t_end if run["cfg"].get("disturbance_windows") else None
        res["P3_A3_failsafe_steps_after_perturbation"] = sum(1 for x in fs if x > t_end + T_EXIT)
    return res


def main() -> int:
    env = DEFAULT.env
    out = {}
    for name in ORDER:
        run = load(name)
        if run is None:
            continue
        out[name] = check_run(run)
        print(name, out[name])
    claims = {
        "A1 onboard distance <= true distance, every hull within 5 m detected (S0 on real data)": all(
            v["A1_onboard_distance_le_true"]["violations"] == 0 and v["A1_onboard_distance_le_true"]["no_target"] == 0 for v in out.values()),
        f"A2 sonar data age <= tau_max = {env.tau_max} s": all(v["A2_sonar_age_max_s"] <= env.tau_max + 1e-9 for v in out.values()),
        f"A3 pair closing speed <= c_max = {env.c_max:.2f} m/s": all(v["A3_closing_speed_max_m_s"] <= env.c_max + 1e-9 for v in out.values()),
        "A4 queued neighbours >= spacing - 2 x 0.10 m apart (formal M4c)": all(
            (v.get("A4_queued_pair_min_distance_m") or 9.0) >= DEFAULT.gate.queue_spacing - 0.2 - 1e-9 for v in out.values()),
        "A5 a committed drone crosses the CR at >= 0.12 m/s (formal M3)": all((v.get("A5_speed_inside_CR_min_m_s") or 1.0) >= 0.12 for v in out.values()),
        f"E1 current drift <= {env.current_drift_max} m/s horizontal, <= {env.current_vertical_max} m/s vertical "
        f"(every run except the declared stress test {', '.join(sorted(DECLARED_OUT))})": all(
            v["E1_drift"]["inside"] for v in out.values() if not v["E1_drift"]["declared_out_of_envelope"]),
        "E2 no persistent actuator saturation: no self-declared ENVELOPE_VIOLATION (the vehicle rejects the current; "
        "onboard proxy of w_drift_max)": all(v["E2_self_declared_envelope_violations"] == 0 for v in out.values()
                                             if not v["E1_drift"]["declared_out_of_envelope"]),
        "P3 A3 no FAILSAFE once the perturbation has ended (formation runs)": all(
            v.get("P3_A3_failsafe_steps_after_perturbation", 0) == 0 for v in out.values()),
        "automaton determinism violations = 0": all(v["determinism_violations"] == 0 for v in out.values()),
        "observation consistency violations = 0": all(v["observation_consistency_violations"] == 0 for v in out.values()),
    }
    failing_e2 = [n for n, v in out.items() if v["E2_self_declared_envelope_violations"] and not v["E1_drift"]["declared_out_of_envelope"]]
    (OUT / "ASSUMPTIONS.json").write_text(json.dumps({"claims": claims, "runs": out}, indent=1), encoding="utf-8")
    lines = ["# Assumptions of the formal proofs, measured on the demonstration runs (v2)", "",
             "Generated by `scripts/validate_assumptions_v2.py` from `results/v2/demos/` (one seed per scenario).", "",
             "| assumption | holds on every run |", "|---|---|"]
    lines += [f"| {k} | {'yes' if v else '**no**' + (' (' + ', '.join(failing_e2) + ')' if k.startswith('E2') else '')} |"
              for k, v in claims.items()]
    lines += ["", "| scenario | A1 samples (<= 5 m) / violations / no target / min margin [m] | misses 5-8 m | A2 max age [s] | "
              "A3 max closing [m/s] | A4 queued pair min [m] | A5 min speed in CR [m/s] | nav error max [m] | onboard form err p95 [m] |",
              "|" + "---|" * 9]
    for name, v in out.items():
        a1 = v["A1_onboard_distance_le_true"]
        lines.append(f"| {name} | {a1['samples_within_5m']} / {a1['violations']} / {a1['no_target']} / {a1['min_margin_m']} | "
                     f"{a1['missed_or_late_5_to_8m']} | {v['A2_sonar_age_max_s']} | "
                     f"{v['A3_closing_speed_max_m_s']} | {v.get('A4_queued_pair_min_distance_m', '-')} | "
                     f"{v.get('A5_speed_inside_CR_min_m_s', '-')} | {v['A7_nav_error_max_m']} | {v.get('formation_err_onboard_p95_m', '-')} |")
    lines += ["", "A1 counts every control step (after the first two captures) at which another hull, drone or scripted vehicle, "
              "is within 5 m of a drone: the guards can be affected only below d_warning_exit plus the largest looseness of the bound "
              "(2.7 + 1.2 m).  Between 5 and 8 m a hull may be missed for a capture (weak returns) or masked by the gate bars; "
              "those cases are counted separately and do not concern P1.", "",
              "A2: the age of the sonar data used by the guards (0 when every capture arrives; tau_max covers one dropped capture). "
              "A4: the closest pair of drones that are both holding their queue points. "
              "A5: the along-axis speed of the drone inside the critical region (formal M3 needs >= 0.12 m/s).", "",
              "## Envelope and monitors", "",
              "| scenario | max drift horizontal / vertical [m/s] | drift envelope (E1) | self-declared envelope violations (E2) | "
              "longest-saturated drone: time saturated [s] | FAILSAFE steps (all drones) | P3: perturbation ends at [s] / "
              "FAILSAFE steps after it (A3) | determinism viol. | observation consistency viol. |", "|" + "---|" * 9]
    for name, v in out.items():
        e1 = v["E1_drift"]
        lab = ("inside" if e1["inside"] else "OUT") + (" (declared stress test)" if e1["declared_out_of_envelope"] else "")
        p3 = "-" if "P3_A3_failsafe_steps_after_perturbation" not in v else \
            f"{v['P3_A1_perturbations_end_s'] if v['P3_A1_perturbations_end_s'] is not None else 'no current window'} / " \
            f"{v['P3_A3_failsafe_steps_after_perturbation']}"
        lines.append(f"| {name} | {e1['max_horizontal_m_s']} / {e1['max_vertical_m_s']} | {lab} | "
                     f"{v['E2_self_declared_envelope_violations']} | {v['saturated_time_max_s']} | {v['failsafe_steps']} | "
                     f"{p3} | {v['determinism_violations']} | {v['observation_consistency_violations']} |")
    lines += ["", "E1 is the current measured by the referee at the true drone positions against the claimed drift bound. "
              "E2 is the drone's own view: its EnvelopeMonitor declares ENVELOPE_VIOLATION after 4 s of persistent "
              "thrust saturation (the current is stronger than the drone at the commanded speed) and the automaton "
              "takes the fault edge to FAILSAFE; a drift inside E1 is not enough when the current opposes the motion "
              "(see DESIGN_ITERATIONS.md, DI-24). FAILSAFE steps include the ones caused by such a declaration."]
    (OUT / "ASSUMPTIONS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(claims, indent=1))
    return 0 if all(claims.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
