"""Analysis of the validation experiments (communication sweep, emergence): tables and figures.

    python scripts/analyze_validation.py <out dir> <raw dir> [<raw dir> ...]
"""

import csv
import gzip
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

PAYLOAD_BITS = 64
COMPLETE = ("PASS", "P3_LATE")
GATE_MODES = ("MUTEX_APPROACH", "MUTEX_YIELD", "MUTEX_PASS")


def factor(proto: str):
    if proto == "B2_LOW":
        return "baseline", 0.0
    if proto == "B2_BLACKOUT":
        return "blackout", 1.0
    if proto.startswith("B2_SW-"):
        from holo_fleet.sim.acoustic_channel import SWEEP_KEYS
        part = proto[6:]
        key = next(k for k in sorted(SWEEP_KEYS, key=len, reverse=True) if part.startswith(k))
        return key, float(part[len(key):])
    return proto, 0.0


def states_of(d: Path):
    files = sorted(d.glob("drone_*_state.jsonl.gz"), key=lambda f: int(f.name.split("_")[1]))
    return [[json.loads(x) for x in gzip.open(f, "rt", encoding="utf-8")] for f in files]


def premise_and_packets(d: Path) -> dict:
    """Premise C classes and CR entries (scripts/audit_premise_c.py, by the order of events) and the acoustic packets of the
    run: received, accepted, rejected, discarded as stale (older than acoustic.latency_max_s on delivery)."""
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from audit_premise_c import one

    a = one(str(d))
    at = (json.loads((d / "referee_metrics.json").read_text(encoding="utf-8")).get("run") or {}).get("acoustic_tracker") or {}
    return {"pc_satisfied": a["counts"]["SATISFIED"], "pc_concurrent": a["counts"]["CONCURRENT"],
            "pc_violated": a["counts"]["VIOLATED"], "pc_inconclusive": a["counts"]["INCONCLUSIVE"],
            "cr_not_admitted": len(a["entries_not_admitted"]), "rx_received": at.get("received", 0),
            "rx_accepted": at.get("accepted", 0), "rx_rejected": at.get("rejected", 0), "rx_stale": at.get("stale", 0)}


def run_row(d: Path) -> dict:
    m = json.loads((d / "run_metrics.json").read_text(encoding="utf-8"))
    st = states_of(d)
    n = len(st)
    dt = 0.1
    gate_wait = [sum(1 for r in s if r.get("mode") in ("MUTEX_APPROACH", "MUTEX_YIELD")) * dt for s in st]
    live_sil = 0.0
    for k, s in enumerate(st):
        for r in s:
            if r.get("mode") == "DEAD":
                continue
            sil = (r.get("gate") or {}).get("silence") or []
            if sil:
                live_sil = max(live_sil, max(sil))
    bc = m.get("beacon") or {}
    dur = (m.get("terminal") or {}).get("t_end") or m.get("horizon_s")
    q = m.get("p3") or {}
    term = m.get("terminal") or {}
    reach = [v for v in (term.get("reached_plan_target_t") or {}).values() if v is not None]
    passed = [v for v in (term.get("gate_passed_t") or {}).values() if v is not None]
    f, lvl = factor(m.get("protocol") or "")
    return {"protocol": m.get("protocol"), "factor": f, "level": lvl, "profile": m.get("profile"), "scenario": m.get("scenario"),
            "seed": m.get("seed"), "class": m.get("class"), "complete": m.get("class") in COMPLETE,
            "P1_min_m": (m.get("P1") or {}).get("min_true_separation_m"), "P2_holds": (m.get("P2") or {}).get("holds"),
            "P3b": (q.get("P3b") or {}).get("verdict"), "P3c": (q.get("P3c") or {}).get("verdict"),
            "duration_s": dur, "completion_s": round(max(reach), 1) if reach else None,
            "last_gate_s": round(max(passed), 1) if passed else None, "false_vacancy": (m.get("P3") or {}).get("false_vacancy"),
            "fairness_holds": (m.get("fairness") or {}).get("holds"), "on_time": m.get("on_time"), "deadline_s": m.get("deadline_a_priori_s"),
            "gate_wait_mean_s": round(float(np.mean(gate_wait)), 1) if n else None, "gate_wait_max_s": round(float(max(gate_wait)), 1) if n else None,
            "reception_rate": bc.get("reception_rate"), "latency_mean_ms": bc.get("latency_mean_ms"),
            "longest_absence_s": bc.get("longest_absence_s"), "max_live_silence_s": round(live_sil, 1),
            "overhead_bps_per_drone": round(PAYLOAD_BITS / bc["period_s"], 1) if bc.get("period_s") else None,
            "premise_v_live_dead": (m.get("premise_v") or {}).get("live_declared_dead"),
            "fairness_gap_s": (m.get("fairness") or {}).get("longest_gap_s"),
            **premise_and_packets(d),
            "gate_nav_beyond": (m.get("gate_nav") or {}).get("committed_beyond"),
            "tags": "+".join(m.get("cause_tags") or []), "run_dir": d.as_posix()}


def emergence(d: Path) -> dict:
    """Reaction of the other drones to the displaced one (drone_2 in lost_drone_rejoin and em_delay_head)."""
    from holo_fleet.config import DEFAULT
    from holo_fleet.sim.scenarios import SCENARIOS

    m = json.loads((d / "run_metrics.json").read_text(encoding="utf-8"))
    sc = SCENARIOS[m["scenario"]](DEFAULT)
    jet = [c for c in sc.sim.current.components if c.kind == "jet"][0]
    st = states_of(d)
    R = list(csv.DictReader(open(d / "referee_timeseries.csv", encoding="utf-8")))
    t = np.array([float(r["t"]) for r in R])
    fe = np.array([float(r["form_err"]) if r.get("form_err") not in (None, "") else np.nan for r in R])
    displaced = 2
    v_nom0 = DEFAULT.form.v_nominal
    w_end = jet.t_off + float(DEFAULT.form.t_rejoin)

    def adapting(r):
        f = r.get("form") or {}
        return bool(f.get("reduced")) or r.get("mode") == "FORMATION_WAIT_REJOIN" or (f.get("v_ref") is not None and f["v_ref"] < v_nom0 - 1e-3)
    react = None
    vref = {}
    v_nom = DEFAULT.form.v_nominal if hasattr(DEFAULT.form, "v_nominal") else None
    adapt_s, yielded_m, vmin = 0.0, 0.0, None
    for k in range(len(st)):
        if k == displaced:
            continue
        for a, b in zip(st[k], st[k][1:]):
            if a["t"] < jet.t_on or a["t"] > w_end:
                continue
            f = a.get("form") or {}
            if adapting(a):
                adapt_s += b["t"] - a["t"]
            v = f.get("v_ref")
            if v is not None:
                vmin = v if vmin is None else min(vmin, v)
                if v_nom is not None:
                    yielded_m += max(v_nom - v, 0.0) * (b["t"] - a["t"])
    for k in range(len(st)):
        if k == displaced:
            continue
        for r in st[k]:
            if jet.t_on <= r["t"] <= w_end and adapting(r):
                react = r["t"] if react is None else min(react, r["t"])
                break
        vref[k] = [(r["t"], (r.get("form") or {}).get("v_ref")) for r in st[k]]
    after = t >= jet.t_off
    rec = None
    hold = 4.0 / DEFAULT.form.k_slot
    i0 = int(np.searchsorted(t, jet.t_off))
    run = None
    for i in range(i0, len(t)):
        if fe[i] <= 1.0:
            run = t[i] if run is None else run
            if t[i] - run >= hold:
                rec = run
                break
        else:
            run = None
    return {"protocol": m["protocol"], "scenario": m["scenario"], "seed": m["seed"], "class": m["class"],
            "jet_on": jet.t_on, "jet_off": jet.t_off, "reaction_s": None if react is None else round(react - jet.t_on, 1),
            "max_form_err_m": round(float(np.nanmax(fe[t >= jet.t_on])), 2), "form_err_at_jet_off_m": round(float(fe[i0]), 2),
            "recovery_after_jet_s": None if rec is None else round(rec - jet.t_off, 1),
            "mean_form_err_after_m": round(float(np.nanmean(fe[after])), 2),
            "adapt_s": round(adapt_s, 1), "v_ref_min": None if vmin is None else round(vmin, 3), "yielded_m": round(yielded_m, 2),
            "P1_min_m": (m.get("P1") or {}).get("min_true_separation_m"), "duration_s": (m.get("terminal") or {}).get("t_end"),
            "_t": t.tolist(), "_fe": fe.tolist(), "_vref": {str(k): v for k, v in vref.items()}}


def main(out: Path, raws) -> None:
    out.mkdir(parents=True, exist_ok=True)
    dirs = [p.parent for r in raws for p in Path(r).rglob("run_metrics.json")]
    comm = [run_row(d) for d in dirs if json.loads((d / "run_metrics.json").read_text())["scenario"] in ("gate_single", "lost_drone_mutex", "formation_square")]
    emer = [emergence(d) for d in dirs if json.loads((d / "run_metrics.json").read_text())["scenario"] in ("lost_drone_rejoin", "em_delay_head")]
    if comm:
        with open(out / "comm_runs.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(comm[0])); w.writeheader(); w.writerows(comm)
    if emer:
        with open(out / "emergence_runs.csv", "w", newline="", encoding="utf-8") as f:
            keys = [k for k in emer[0] if not k.startswith("_")]
            w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows([{k: e[k] for k in keys} for e in emer])
    L = ["# Validation experiments", ""]
    if comm:
        L += ["## Communication sweep", "", "| factor | level | scenario | runs | complete | P3 failures | P1/P2 ok | mean completion s | mean last gate s | mean gate wait s | max live silence s | fairness violated | false vacancies | premise C sat / conc / viol / inc | CR entries not admitted | packets received | accepted | rejected | stale |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        g = defaultdict(list)
        for r in comm:
            g[(r["factor"], r["level"], r["scenario"])].append(r)
        order = {"baseline": 0, "per": 1, "lat": 2, "jit": 3, "loss": 4, "burst": 5, "asym": 6, "blackout": 7}
        for key in sorted(g, key=lambda k: (order.get(k[0], 9), k[1], k[2])):
            rs = g[key]
            ok = sum(1 for r in rs if r["P2_holds"] and (r["P1_min_m"] or 0) >= 1.0)
            mean = lambda f: round(float(np.mean([r[f] for r in rs if r[f] is not None])), 1) if any(r[f] is not None for r in rs) else None
            L.append(f"| {key[0]} | {key[1]:g} | {key[2]} | {len(rs)} | {sum(r['complete'] for r in rs)} | {sum(r['class'] == 'P3_FAILURE' for r in rs)} | {ok} | "
                     f"{mean('completion_s')} | {mean('last_gate_s')} | "
                     f"{mean('gate_wait_mean_s')} | {max(r['max_live_silence_s'] for r in rs)} | {sum(r['fairness_holds'] is False for r in rs)} | "
                     f"{sum(r['false_vacancy'] or 0 for r in rs)} | "
                     f"{sum(r['pc_satisfied'] for r in rs)} / {sum(r['pc_concurrent'] for r in rs)} / {sum(r['pc_violated'] for r in rs)} / "
                     f"{sum(r['pc_inconclusive'] for r in rs)} | {sum(r['cr_not_admitted'] for r in rs)} | "
                     f"{sum(r['rx_received'] for r in rs)} | {sum(r['rx_accepted'] for r in rs)} | {sum(r['rx_rejected'] for r in rs)} | "
                     f"{sum(r['rx_stale'] for r in rs)} |")
        plot_comm(out, comm)
    if emer:
        L += ["", "## Emergence", "", "| protocol | scenario | seed | class | reaction s | max form err m | err at jet end m | recovery after jet s | mean err after m | P1 min m | duration s |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
        for e in sorted(emer, key=lambda e: (e["scenario"], e["protocol"], e["seed"])):
            L.append(f"| {e['protocol']} | {e['scenario']} | {e['seed']} | {e['class']} | {e['reaction_s']} | {e['max_form_err_m']} | {e['form_err_at_jet_off_m']} | "
                     f"{e['recovery_after_jet_s']} | {e['mean_form_err_after_m']} | {e['P1_min_m']} | {e['duration_s']} |")
        L += ["", "### Emergence by configuration (mean, min-max over seeds)", "",
              "| protocol | scenario | runs | complete | reaction s | adaptation time s | progress yielded m | min v_ref m/s | max form err m | recovery after jet s | mean err after m | P1 min m |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|"]
        g = defaultdict(list)
        for e in emer:
            g[(e["protocol"], e["scenario"])].append(e)

        def agg(rs, f):
            v = [r[f] for r in rs if r[f] is not None]
            return f"{np.mean(v):.2f} ({min(v):.2f}-{max(v):.2f}) n={len(v)}" if v else "none"
        for key in sorted(g, key=lambda k: (k[1], k[0])):
            rs = g[key]
            L.append(f"| {key[0]} | {key[1]} | {len(rs)} | {sum(r['class'] in COMPLETE for r in rs)} | {agg(rs, 'reaction_s')} | {agg(rs, 'adapt_s')} | "
                     f"{agg(rs, 'yielded_m')} | {agg(rs, 'v_ref_min')} | {agg(rs, 'max_form_err_m')} | {agg(rs, 'recovery_after_jet_s')} | "
                     f"{agg(rs, 'mean_form_err_after_m')} | {min(r['P1_min_m'] for r in rs)} |")
        plot_emergence(out, emer)
    (out / "VALIDATION_TABLES.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print("\n".join(L))


def plot_comm(out: Path, comm) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    facs = ["per", "lat", "jit", "loss", "burst", "asym"]
    labels = {"per": "beacon period (s)", "lat": "latency (s)", "jit": "jitter (s)", "loss": "random loss", "burst": "burst length (s)", "asym": "asymmetric loss"}
    base = {"per": 1.0, "lat": 0.15, "jit": 0.0, "loss": 0.08, "burst": 0.0, "asym": 0.0}
    fig, axes = plt.subplots(2, 6, figsize=(18, 6), sharey="row")
    for j, fa in enumerate(facs):
        for sc, mk in (("gate_single", "o"), ("lost_drone_mutex", "s"), ("formation_square", "^")):
            pts = defaultdict(list)
            for r in comm:
                if r["scenario"] != sc:
                    continue
                if r["factor"] == fa:
                    pts[r["level"]].append(r)
                elif r["factor"] == "baseline":
                    pts[base[fa]].append(r)
            xs = sorted(pts)
            ys = [[r["last_gate_s"] for r in pts[x] if r["last_gate_s"] is not None] for x in xs]
            ok = [i for i, y in enumerate(ys) if y]
            if ok:
                mu = np.array([np.mean(ys[i]) for i in ok])
                axes[0, j].errorbar([xs[i] for i in ok], mu, yerr=[mu - np.array([min(ys[i]) for i in ok]), np.array([max(ys[i]) for i in ok]) - mu],
                                    marker=mk, capsize=3, label=sc)
            axes[1, j].plot(xs, [np.mean([r["complete"] for r in pts[x]]) for x in xs], marker=mk, label=sc)
        axes[1, j].set_xlabel(labels[fa])
    axes[0, 0].set_ylabel("last gate passage (s), mean and range")
    axes[1, 0].set_ylabel("fraction complete")
    axes[0, 0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out / "comm_sweep.png", dpi=130)
    plt.close(fig)


def plot_emergence(out: Path, emer) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    scs = sorted({e["scenario"] for e in emer})
    fig, axes = plt.subplots(2, len(scs), figsize=(6 * len(scs), 6), squeeze=False)
    for j, sc in enumerate(scs):
        for e in emer:
            if e["scenario"] != sc:
                continue
            c = "tab:blue" if e["protocol"].startswith("B2_") else "tab:red"
            axes[0, j].plot(e["_t"], e["_fe"], color=c, alpha=0.6, lw=1)
            for k, v in e["_vref"].items():
                axes[1, j].plot([a for a, b in v], [b if b is not None else np.nan for a, b in v], color=c, alpha=0.4, lw=0.8)
            axes[0, j].axvspan(e["jet_on"], e["jet_off"], color="grey", alpha=0.15)
        axes[0, j].set_title(f"{sc} (blue B2, red no adaptation)")
        axes[0, j].set_ylabel("formation error (m)")
        axes[0, j].axhline(1.0, color="k", lw=0.5, ls="--")
        axes[1, j].set_ylabel("reference speed of the other drones (m/s)")
        axes[1, j].set_xlabel("t (s)")
    fig.tight_layout()
    fig.savefig(out / "emergence.png", dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    main(Path(sys.argv[1]), sys.argv[2:])
