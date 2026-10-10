"""Automaton diagram of the local hybrid automaton, generated from the spec (holo_fleet/ha/spec.py).

    python scripts/make_automaton_diagram.py

Writes figures/v2/automaton/automaton_full.{png,svg} (every mode, every transition, the exact readable guards)
and automaton_presentation.{png,svg} (the same automaton, fewer words, large text).

Nothing is drawn by hand: the states, the transitions and their guards come from ``build_edges`` (the edge table
executed at runtime and encoded in Z3).  The readable guards are the edges' ``label`` fields, exact under the
priority order (formal/check_determinism.py, L1).  :func:`diagram_model` groups the 120 edges (12 per mode, self-
loops included) the way the automaton is built - the hazard and mutex edges are the same from every mode, the
formation-level edges depend on whether the drone keeps its slot - and :func:`coverage` checks that every edge of
the spec is represented (tests/test_automaton_diagram.py).
"""

from __future__ import annotations

import sys
from collections import Counter
from pathlib import Path
from typing import Dict, List, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402

from holo_fleet.config import DEFAULT, FleetConfig  # noqa: E402
from holo_fleet.ha.guard_text import pretty, thresholds, UNITS  # noqa: E402
from holo_fleet.ha.spec import IN_FORMATION, MODES, PRIORITY_ORDER, SELF_RECOVERED, Mode, build_edges  # noqa: E402

OUT = ROOT / "figures" / "v2" / "automaton"
# the state colours of the fleet view and of the v2 figures (holo_fleet/ui/fleet_view.py, make_figures_v2.py)
COLOR = {Mode.FORMATION_FOLLOW: "#2f9e5b", Mode.FORMATION_WAIT_REJOIN: "#e0a21c", Mode.DEGRADED_FORMATION: "#8a63c9",
         Mode.FORMATION_RECOVERY: "#ef7d32", Mode.MUTEX_APPROACH: "#7fb2e5", Mode.MUTEX_YIELD: "#3d85c6",
         Mode.MUTEX_PASS: "#1c4e9c", Mode.SEPARATION_WARNING: "#e8577a", Mode.COLLISION_AVOIDANCE: "#c62828",
         Mode.FAILSAFE_HOLD_OR_RETREAT: "#4b4a46"}
INK, INK2, MUTED, LINE, BAND, SURF = "#1f1e1c", "#55534d", "#8e8c84", "#6f6d66", "#f1f0eb", "#fcfcfb"

GLOBAL_EDGES = ("fault", "collision_risk", "separation_warning", "pass_continue", "pass_done", "commit", "yield",
                "approach")
KEEPING_EDGES = ("formation_lost", "neighbour_missing", "slot_vacant", "follow")
RECOVERED_EDGES = ("formation_recovered", "recovered_wait_rejoin", "recovered_degraded", "recover")
# priority levels of the brief (PRIORITY_ORDER grouped): FAILSAFE > COLLISION > WARNING > MUTEX > RECOVERY > keeping
LEVELS = [("FAILSAFE", (Mode.FAILSAFE_HOLD_OR_RETREAT,)), ("COLLISION AVOIDANCE", (Mode.COLLISION_AVOIDANCE,)),
          ("SEPARATION WARNING", (Mode.SEPARATION_WARNING,)),
          ("MUTEX", (Mode.MUTEX_PASS, Mode.MUTEX_YIELD, Mode.MUTEX_APPROACH)),
          ("FORMATION RECOVERY", (Mode.FORMATION_RECOVERY,)),
          ("FORMATION KEEPING", (Mode.FORMATION_WAIT_REJOIN, Mode.DEGRADED_FORMATION, Mode.FORMATION_FOLLOW))]


# ---------------------------------------------------------------------------------------------- model
def guard(label: str, cfg: FleetConfig = DEFAULT, values: bool = False) -> str:
    """Readable guard of a spec label; the own-slot-recovered conjunction is named ``self_recovered``."""
    txt = pretty(label, cfg, values=values)
    sr = pretty(SELF_RECOVERED, cfg, values=values)
    return txt.replace(sr, "self_recovered").replace("¬(self_recovered)", "¬self_recovered")


def diagram_model(cfg: FleetConfig = DEFAULT) -> Dict:
    """The automaton as drawn: priority rows (edges present in every mode), formation-keeping arrows, the
    recovered group (edges of every mode outside formation keeping) and the self-loops, each with the spec
    edges it represents."""
    E = build_edges(cfg)
    assert [m for _name, ms in LEVELS for m in ms] == PRIORITY_ORDER, "priority bands out of date"
    rows = []
    for name in GLOBAL_EDGES:
        es = {m: next(e for e in E[m] if e.name == name) for m in MODES}
        targets = {e.target for e in es.values()}
        assert len(targets) == 1, (name, targets)
        common = Counter(e.label for e in es.values()).most_common(1)[0][0]
        rows.append({"edge": name, "target": targets.pop(), "label": common,
                     "variants": {m: e.label for m, e in es.items() if e.label != common},
                     "set_committed": es[MODES[0]].set_committed, "covers": [(m, name) for m in MODES]})
    keeping = []
    for src in IN_FORMATION:
        for e in E[src]:
            if e.name in KEEPING_EDGES and e.target != src:
                keeping.append({"source": src, "target": e.target, "edge": e.name, "label": e.label,
                                "reason": e.reason, "covers": [(src, e.name)]})
    recovered = []
    outside = [m for m in MODES if m not in IN_FORMATION]
    for name in RECOVERED_EDGES:
        es = {m: next(e for e in E[m] if e.name == name) for m in outside}
        tgt = {e.target for e in es.values()}
        assert len(tgt) == 1 and len({e.label for e in es.values()}) == 1, name
        recovered.append({"edge": name, "target": tgt.pop(), "label": es[outside[0]].label, "sources": outside,
                          "reason": es[Mode.FORMATION_RECOVERY].reason, "covers": [(m, name) for m in outside]})
    loops = {m: [e.name for e in E[m] if e.target == m] for m in MODES}
    return {"modes": list(MODES), "rows": rows, "keeping": keeping, "recovered": recovered, "loops": loops,
            "n_edges": sum(len(v) for v in E.values()), "t_rejoin": cfg.form.t_rejoin, "v_wait": cfg.form.v_wait,
            "edges": {(m, e.name): e for m in MODES for e in E[m]}}


def coverage(model: Dict) -> Tuple[List, List]:
    """(missing, extra): spec edges not represented in the drawing, drawn elements that are not spec edges."""
    drawn = set()
    for r in model["rows"] + model["keeping"] + model["recovered"]:
        drawn |= set(r["covers"])
    for m, names in model["loops"].items():
        drawn |= {(m, n) for n in names}
    spec = set(model["edges"])
    return sorted(spec - drawn, key=str), sorted(drawn - spec, key=str)


# ---------------------------------------------------------------------------------------------- drawing helpers
SHORT = {m: m.value for m in Mode}
NICK = {Mode.FORMATION_FOLLOW: "FOLLOW", Mode.FORMATION_WAIT_REJOIN: "WAIT_REJOIN",
        Mode.DEGRADED_FORMATION: "DEGRADED", Mode.FORMATION_RECOVERY: "RECOVERY", Mode.MUTEX_APPROACH: "APPROACH",
        Mode.MUTEX_YIELD: "YIELD", Mode.MUTEX_PASS: "PASS", Mode.SEPARATION_WARNING: "WARNING",
        Mode.COLLISION_AVOIDANCE: "AVOID", Mode.FAILSAFE_HOLD_OR_RETREAT: "FAILSAFE"}


def _style():
    plt.rcParams.update({"font.family": ["Segoe UI", "DejaVu Sans", "sans-serif"], "svg.fonttype": "none",
                         "figure.facecolor": SURF, "savefig.facecolor": SURF})


def _box(ax, xy, w, h, mode, title, sub="", fs=11, sub_fs=8.5, loop=True):
    x, y = xy
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0.02,rounding_size=0.9",
                                fc=COLOR[mode], ec="white", lw=1.5, zorder=3))
    ax.text(x, y + (0.42 * h / 3.0 if sub else 0.0), title, ha="center", va="center", color="white", fontsize=fs,
            fontweight="bold", zorder=4)
    if sub:
        ax.text(x, y - 0.32 * h, sub, ha="center", va="center", color="white", fontsize=sub_fs, zorder=4)
    if loop:                                            # compact self-loop: otherwise remain
        ax.text(x + w / 2 - 0.35, y + h / 2 - 0.25, "↻", ha="right", va="top", color="white", fontsize=fs,
                zorder=4)


def _arrow(ax, p0, p1, rad=0.0, color=LINE, lw=1.4, ls="-"):
    ax.add_patch(FancyArrowPatch(p0, p1, connectionstyle=f"arc3,rad={rad}", arrowstyle="-|>", mutation_scale=13,
                                 color=color, lw=lw, ls=ls, zorder=2, shrinkA=0, shrinkB=0))


def _path(ax, pts, color=LINE, lw=1.4, ls="-"):
    """Orthogonal connector: straight segments through ``pts``, arrow head on the last one."""
    xs, ys = zip(*pts)
    ax.plot(xs[:-1] + (xs[-1],), ys[:-1] + (ys[-1],), color=color, lw=lw, ls=ls, zorder=2, solid_capstyle="butt")
    _arrow(ax, pts[-2], pts[-1], color=color, lw=lw)


def _label(ax, xy, text, fs=8.5, color=INK, ha="center", bg=True, weight="normal", va="center"):
    ax.text(xy[0], xy[1], text, ha=ha, va=va, fontsize=fs, color=color, zorder=5, fontweight=weight,
            bbox=dict(boxstyle="round,pad=0.15", fc=SURF, ec="none", alpha=0.9) if bg else None)


def _chip(ax, x, y, mode, fs=8.5, w=None):
    txt = NICK[mode]
    w = w or 0.62 * len(txt) + 1.6
    ax.add_patch(FancyBboxPatch((x, y - 0.75), w, 1.5, boxstyle="round,pad=0.02,rounding_size=0.7",
                                fc=COLOR[mode], ec="none", zorder=3))
    ax.text(x + w / 2, y, txt, ha="center", va="center", color="white", fontsize=fs, fontweight="bold", zorder=4)
    return x + w


def _kl(model, s, t, cfg):
    k = next(k for k in model["keeping"] if k["source"] == s and k["target"] == t)
    return guard(k["label"], cfg)


def _rl(model, t, cfg):
    return guard(next(r for r in model["recovered"] if r["target"] == t)["label"], cfg)


# ---------------------------------------------------------------------------------------------- full version
def render_full(model: Dict, cfg: FleetConfig, path: Path) -> None:
    _style()
    fig = plt.figure(figsize=(20, 12.6))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 63)
    ax.axis("off")
    rows = {r["edge"]: r for r in model["rows"]}
    ax.text(1.5, 61.2, "Local hybrid automaton of every drone", fontsize=19, fontweight="bold", color=INK, va="center")
    ax.text(1.5, 59.4, f"Generated from holo_fleet/ha/spec.py (build_edges): {len(model['modes'])} modes, "
                       f"{model['n_edges']} edges ({model['n_edges'] // len(model['modes'])} per mode in priority "
                       "order, self-loops included).  Every guard is the edge's readable label, proved equal to the "
                       "executed guard under the priority order (formal/check_determinism.py, L1).",
            fontsize=9.5, color=INK2, va="center")
    # ------------------------------------------------------------------ left: priority rows (every mode)
    ax.add_patch(FancyBboxPatch((1.0, 2.0), 40.5, 55.6, boxstyle="round,pad=0.0,rounding_size=1.0", fc="white",
                                ec="#dedcd4", lw=1.0, zorder=0))
    ax.text(2.2, 56.2, "From every mode: guards evaluated top-down, the first that holds wins",
            fontsize=11.5, fontweight="bold", color=INK, va="center")
    y = 53.4
    for i, r in enumerate(model["rows"], 1):
        ax.text(2.4, y, f"{i}", fontsize=10, fontweight="bold", color=MUTED, va="center")
        xe = _chip(ax, 3.6, y, r["target"])
        txt = guard(r["label"], cfg)
        if r["set_committed"] is not None:
            txt += f"     [committed := {'true' if r['set_committed'] else 'false'}]"
        ax.text(xe + 0.8, y, "←  " + txt, fontsize=10, color=INK, va="center")
        y -= 1.75
        notes: Dict[str, List[str]] = {}
        for m, lab in r["variants"].items():
            notes.setdefault(lab, []).append(NICK[m])
        for lab, ms in notes.items():
            ax.text(xe + 2.3, y + 0.35, f"in {', '.join(ms)}: {guard(lab, cfg)}   (hysteresis)", fontsize=8.6,
                    color=INK2, va="center")
            y -= 1.35
        if i == 3:
            ax.plot([2.2, 40.5], [y + 0.55, y + 0.55], color="#e6e4dc", lw=1.0)
            ax.text(2.4, y - 0.2, "no hazard: mutual-exclusion zone (committed latch, gate perception)", fontsize=8.6,
                    color=MUTED, va="center", style="italic")
            y -= 1.7
        y -= 0.55
    ax.plot([2.2, 40.5], [y + 0.85, y + 0.85], color="#e6e4dc", lw=1.0)
    ax.text(2.4, y, "9", fontsize=10, fontweight="bold", color=MUTED, va="center")
    ax.text(3.6, y + 0.55, "otherwise (no hazard, not committed, outside the mutex zone): the formation level of the\n"
                    "current mode decides - lower bands of the graph on the right.  From a mode outside formation\n"
                    "keeping (FAILSAFE, AVOID, WARNING, APPROACH, YIELD, PASS, RECOVERY): FORMATION_RECOVERY\n"
                    "until self_recovered, then FOLLOW / WAIT_REJOIN / DEGRADED by neighbors_ok and degraded",
            fontsize=9.3, color=INK, va="top")
    y -= 5.0
    th = thresholds(cfg)
    ax.text(2.4, y, "Thresholds (holo_fleet/config.py)", fontsize=10, fontweight="bold", color=INK, va="center")
    y -= 1.5
    items = [f"{k} = {v:g} {UNITS[k]}" for k, v in th.items()] + [f"t_rejoin = {model['t_rejoin']:.1f} s",
                                                                   f"v_wait = {model['v_wait']:.2f} m/s"]
    for j in range(0, len(items), 3):
        ax.text(2.6, y, "     ".join(items[j:j + 3]), fontsize=9.0, color=INK2, va="center")
        y -= 1.3
    y -= 0.4
    ax.text(2.4, y, "Observation (onboard perception only)", fontsize=10, fontweight="bold", color=INK, va="center")
    y -= 1.45
    for line in ("d_min: conservative distance to the nearest echo (6 sonars)   form_err: own slot error   t_ok: time "
                 "below e_ok",
                 "sense_ok / env_ok: sensors fresh and consistent / commanded motion deliverable (envelope monitor)",
                 "mutex_zone, at_queue, occ_busy, has_prio, passed: gate perception (sector patterns, own navigation)",
                 "neighbors_ok: every expected neighbour not declared vacant is seen   committed: latch of the passage",
                 f"degraded: latch - a missing neighbour's slot declared vacant after t_rejoin = {model['t_rejoin']:.0f} s",
                 "self_recovered = " + pretty(SELF_RECOVERED, cfg)):
        ax.text(2.6, y, line, fontsize=8.6, color=INK2, va="center")
        y -= 1.25
    # ------------------------------------------------------------------ right: the state graph
    bands = {"FAILSAFE": (51.7, 56.6), "COLLISION AVOIDANCE": (45.9, 50.9), "SEPARATION WARNING": (40.1, 45.1),
             "MUTEX": (26.0, 39.3), "FORMATION RECOVERY": (17.6, 25.2), "FORMATION KEEPING": (0.6, 16.8)}
    for k, (name, _modes) in enumerate(LEVELS):
        y0, y1 = bands[name]
        ax.add_patch(FancyBboxPatch((43.0, y0), 56.4, y1 - y0, boxstyle="round,pad=0.0,rounding_size=0.8",
                                    fc=BAND if k % 2 == 0 else "#f7f6f2", ec="none", zorder=0))
        ax.text(99.0, y1 - 0.45, f"priority {k + 1}\n{name}", fontsize=8.2, color=MUTED, ha="right", va="top",
                fontweight="bold", linespacing=1.15)
    ax.text(43.4, 58.0, "State graph: one band per priority level;  ↻ = otherwise remain in the mode (self-loop)",
            fontsize=11.5, fontweight="bold", color=INK, va="center")
    W, H = 12.0, 3.0
    pos = {Mode.FAILSAFE_HOLD_OR_RETREAT: (63.0, 54.0), Mode.COLLISION_AVOIDANCE: (63.0, 48.3),
           Mode.SEPARATION_WARNING: (63.0, 42.6), Mode.MUTEX_APPROACH: (57.0, 31.5), Mode.MUTEX_YIELD: (72.5, 31.5),
           Mode.MUTEX_PASS: (88.0, 31.5), Mode.FORMATION_RECOVERY: (72.0, 21.0),
           Mode.FORMATION_WAIT_REJOIN: (53.0, 9.0), Mode.FORMATION_FOLLOW: (72.0, 9.0),
           Mode.DEGRADED_FORMATION: (91.0, 9.0)}
    subs = {Mode.FAILSAFE_HOLD_OR_RETREAT: "hold the failsafe layer (+ heave probe)",
            Mode.COLLISION_AVOIDANCE: "certified escape", Mode.SEPARATION_WARNING: "brake / open the distance",
            Mode.MUTEX_APPROACH: "to the own queue point", Mode.MUTEX_YIELD: "hold the queue point",
            Mode.MUTEX_PASS: "through the critical region", Mode.FORMATION_RECOVERY: "to the slot, rejoin from behind",
            Mode.FORMATION_WAIT_REJOIN: f"keep slot, progress {model['v_wait']:.2f} m/s",
            Mode.FORMATION_FOLLOW: "keep slot, planned speed", Mode.DEGRADED_FORMATION: "original slots, hole stays"}
    for m, p in pos.items():
        _box(ax, p, W, H, m, SHORT[m], subs[m], fs=9.0, sub_fs=7.8)
    L, R = (lambda m: pos[m][0] - W / 2), (lambda m: pos[m][0] + W / 2)
    T, B = (lambda m: pos[m][1] + H / 2), (lambda m: pos[m][1] - H / 2)
    # rows 1-3 and 8: from every mode (bus)
    bx = 45.0
    ax.plot([bx, bx], [pos[Mode.MUTEX_APPROACH][1], 55.6], color="#b9b7ae", lw=2.4, zorder=1, solid_capstyle="round")
    _label(ax, (bx + 0.2, 56.6), "from every mode", fs=8.4, color=INK2, ha="left", bg=False)
    for m, edge, k in ((Mode.FAILSAFE_HOLD_OR_RETREAT, "fault", 1), (Mode.COLLISION_AVOIDANCE, "collision_risk", 2),
                       (Mode.SEPARATION_WARNING, "separation_warning", 3)):
        _arrow(ax, (bx, pos[m][1]), (L(m), pos[m][1]), color="#8e8c84")
        _label(ax, (bx + 0.5, pos[m][1] + 0.85), f"row {k}: " + guard(rows[edge]["label"], cfg), fs=8.0, ha="left",
               bg=False, color=INK2)
    ap, yi, pa = Mode.MUTEX_APPROACH, Mode.MUTEX_YIELD, Mode.MUTEX_PASS
    _arrow(ax, (bx, pos[ap][1]), (L(ap), pos[ap][1]), color="#8e8c84")
    _label(ax, (bx + 0.3, B(ap) - 0.75), "row 8: " + guard(rows["approach"]["label"], cfg), fs=8.0, ha="left",
           bg=False, color=INK2)
    # hysteresis self-loops and the hazards clearing
    ca, sw = Mode.COLLISION_AVOIDANCE, Mode.SEPARATION_WARNING
    for m, edge in ((Mode.FAILSAFE_HOLD_OR_RETREAT, "fault"), (ca, "collision_risk"), (sw, "separation_warning")):
        lab = rows[edge]["variants"].get(m, rows[edge]["label"])
        _label(ax, (R(m) + 0.6, pos[m][1] + 0.75), "↻ " + guard(lab, cfg), fs=7.8, ha="left", bg=False,
               color=INK2)
    _arrow(ax, (pos[ca][0], B(ca)), (pos[sw][0], T(sw)))
    _label(ax, (pos[ca][0] + 0.6, (B(ca) + T(sw)) / 2), guard(rows["separation_warning"]["variants"][ca], cfg),
           fs=7.8, ha="left", bg=False)
    ax.text(80.0, 46.9, "leaving FAILSAFE / AVOID / WARNING:\nwhen the hazard clears, the lower rows\ndecide again "
                        "(4-8 in the mutex zone, else\nFORMATION_RECOVERY until the own slot\nis recovered, then the "
                        "formation level)", fontsize=7.8, color=MUTED, va="center", ha="left")
    # mutex chain: rows 4-8 as they apply between the mutex modes
    _arrow(ax, (R(ap), pos[ap][1] + 0.6), (L(yi), pos[yi][1] + 0.6))
    _label(ax, ((R(ap) + L(yi)) / 2, T(ap) + 0.85), guard(rows["yield"]["label"], cfg).replace("mutex_zone ∧ ", ""),
           fs=7.8)
    _arrow(ax, (L(yi), pos[yi][1] - 0.6), (R(ap), pos[ap][1] - 0.6))
    _label(ax, ((R(ap) + L(yi)) / 2, B(ap) - 0.85), "¬at_queue", fs=7.8)
    _arrow(ax, (R(yi), pos[yi][1]), (L(pa), pos[pa][1]))
    _label(ax, ((R(yi) + L(pa)) / 2, T(yi) + 0.85), guard(rows["commit"]["label"], cfg).replace("mutex_zone ∧ ", ""),
           fs=7.8)
    _arrow(ax, (L(ap) + 1.2, T(ap)), (R(pa) - 1.2, T(pa)), rad=-0.23)
    _label(ax, (72.5, 38.2), "commit from APPROACH as well  [committed := true]", fs=7.4, color=INK2)
    _label(ax, (R(pa), B(pa) - 0.75), "↻ " + guard(rows["pass_continue"]["label"], cfg), fs=7.6,
           color=INK2, bg=False, ha="right")
    rc = Mode.FORMATION_RECOVERY
    _arrow(ax, (L(pa) + 2.0, B(pa)), (R(rc) - 2.0, T(rc)))
    _label(ax, (82.0, 25.3), guard(rows["pass_done"]["label"], cfg) + "\n[committed := false]", fs=7.8, ha="left")
    _label(ax, (L(rc) - 0.4, T(rc) + 0.75), "↻ " + _rl(model, rc, cfg), fs=7.6, color=INK2, bg=False,
           ha="right")
    # formation level: recovery <-> keeping (nested orthogonal connectors, no crossing)
    wa, fo, de = Mode.FORMATION_WAIT_REJOIN, Mode.FORMATION_FOLLOW, Mode.DEGRADED_FORMATION
    cr = COLOR[rc]
    yo, yi_ = pos[rc][1] + 0.6, pos[rc][1] - 0.6               # outer / inner horizontal of the L connectors
    _path(ax, [(L(rc), yo), (pos[wa][0] - 1.2, yo), (pos[wa][0] - 1.2, T(wa))], color=COLOR[wa])
    _label(ax, (59.6, yo + 0.75), _rl(model, wa, cfg), fs=7.8)
    _path(ax, [(pos[wa][0] + 1.2, T(wa)), (pos[wa][0] + 1.2, yi_), (L(rc), yi_)], color=cr)
    _label(ax, (60.2, yi_ - 0.8), _kl(model, wa, rc, cfg), fs=7.8)
    _path(ax, [(R(rc), yo), (pos[de][0] + 1.2, yo), (pos[de][0] + 1.2, T(de))], color=COLOR[de])
    _label(ax, (85.0, yo + 0.75), _rl(model, de, cfg), fs=7.8)
    _path(ax, [(pos[de][0] - 1.2, T(de)), (pos[de][0] - 1.2, yi_), (R(rc), yi_)], color=cr)
    _label(ax, (83.4, yi_ - 0.8), _kl(model, de, rc, cfg), fs=7.8)
    ym = (B(rc) + T(fo)) / 2
    _arrow(ax, (pos[rc][0] + 1.2, B(rc)), (pos[fo][0] + 1.2, T(fo)), color=COLOR[fo])
    _label(ax, (pos[fo][0] + 1.9, ym), _rl(model, fo, cfg), fs=7.8, ha="left")
    _arrow(ax, (pos[fo][0] - 1.2, T(fo)), (pos[rc][0] - 1.2, B(rc)), color=cr)
    _label(ax, (pos[fo][0] - 1.9, ym), _kl(model, fo, rc, cfg) + "\n(own_slot_lost)", fs=7.8, ha="right")
    # among the formation-keeping modes
    _arrow(ax, (L(fo), pos[fo][1] + 0.6), (R(wa), pos[wa][1] + 0.6), color=COLOR[wa])
    _label(ax, ((L(fo) + R(wa)) / 2, T(fo) + 1.0), _kl(model, fo, wa, cfg) + "\n(neighbor_missing)", fs=7.8)
    _arrow(ax, (R(wa), pos[wa][1] - 0.6), (L(fo), pos[fo][1] - 0.6), color=COLOR[fo])
    _label(ax, ((L(fo) + R(wa)) / 2, B(fo) - 1.0), _kl(model, wa, fo, cfg) + "\n(neighbor_rejoined)", fs=7.8)
    _arrow(ax, (R(fo), pos[fo][1] + 0.6), (L(de), pos[de][1] + 0.6), color=COLOR[de])
    _label(ax, ((R(fo) + L(de)) / 2, T(fo) + 1.0), _kl(model, fo, de, cfg), fs=7.8)
    _arrow(ax, (L(de), pos[de][1] - 0.6), (R(fo), pos[fo][1] - 0.6), color=COLOR[fo])
    _label(ax, ((R(fo) + L(de)) / 2, B(fo) - 1.0), _kl(model, de, fo, cfg) + "\n(slot_reoccupied)", fs=7.8)
    _path(ax, [(pos[wa][0] + 1.2, B(wa)), (pos[wa][0] + 1.2, 3.9), (pos[de][0] - 1.2, 3.9), (pos[de][0] - 1.2, B(de))],
          color=COLOR[de])
    _label(ax, (72.0, 3.9), _kl(model, wa, de, cfg) + f"   (rejoin_timeout: slot vacant after t_rejoin = "
                            f"{model['t_rejoin']:.0f} s)", fs=7.8)
    _path(ax, [(pos[de][0] + 1.2, B(de)), (pos[de][0] + 1.2, 1.6), (pos[wa][0] - 1.2, 1.6), (pos[wa][0] - 1.2, B(wa))],
          color=COLOR[wa])
    _label(ax, (72.0, 1.6), _kl(model, de, wa, cfg) + "   (another neighbour missing)", fs=7.8)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path.with_suffix(".png"), dpi=110)
    fig.savefig(path.with_suffix(".svg"))
    plt.close(fig)


# ---------------------------------------------------------------------------------------------- presentation
def render_presentation(model: Dict, cfg: FleetConfig, path: Path) -> None:
    _style()
    fig = plt.figure(figsize=(16, 9))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 56.25)
    ax.axis("off")
    th = thresholds(cfg)
    rows = {r["edge"]: r for r in model["rows"]}
    ax.text(3.0, 53.4, "What each drone decides, on its own", fontsize=24, fontweight="bold", color=INK, va="center")
    ax.text(3.0, 50.6, "One automaton per drone, 10 states, no communication.  Higher rows win: a hazard always "
                       "overrides the mission.", fontsize=13, color=INK2, va="center")
    # priority axis
    ax.add_patch(FancyArrowPatch((6.0, 4.5), (6.0, 45.5), arrowstyle="-|>", mutation_scale=22, color="#b9b7ae", lw=3))
    ax.text(4.2, 25.0, "priority", rotation=90, fontsize=13, color=MUTED, va="center", ha="center")
    W, H = 15.0, 4.4
    pos = {Mode.FAILSAFE_HOLD_OR_RETREAT: (20.0, 39.5), Mode.COLLISION_AVOIDANCE: (41.0, 39.5),
           Mode.SEPARATION_WARNING: (62.0, 39.5), Mode.MUTEX_APPROACH: (20.0, 28.5), Mode.MUTEX_YIELD: (41.0, 28.5),
           Mode.MUTEX_PASS: (62.0, 28.5), Mode.FORMATION_RECOVERY: (41.0, 18.5),
           Mode.FORMATION_WAIT_REJOIN: (20.0, 9.0), Mode.FORMATION_FOLLOW: (41.0, 9.0),
           Mode.DEGRADED_FORMATION: (62.0, 9.0)}
    subs = {Mode.FAILSAFE_HOLD_OR_RETREAT: "hold position", Mode.COLLISION_AVOIDANCE: "escape",
            Mode.SEPARATION_WARNING: "brake, open distance", Mode.MUTEX_APPROACH: "go to queue point",
            Mode.MUTEX_YIELD: "wait in the queue", Mode.MUTEX_PASS: "cross the gate alone",
            Mode.FORMATION_RECOVERY: "back to own slot, from behind",
            Mode.FORMATION_WAIT_REJOIN: f"slot kept, slower ({model['v_wait']:.2f} m/s)",
            Mode.FORMATION_FOLLOW: f"slot kept, survey speed", Mode.DEGRADED_FORMATION: "slot kept, hole stays"}
    for m, p in pos.items():
        _box(ax, p, W, H, m, NICK[m], subs[m], fs=15, sub_fs=11, loop=False)
    L, R = (lambda m: pos[m][0] - W / 2), (lambda m: pos[m][0] + W / 2)
    T, B = (lambda m: pos[m][1] + H / 2), (lambda m: pos[m][1] - H / 2)
    # hazards: from any state (a bar above the row)
    ax.plot([12.5, 69.5], [46.4, 46.4], color="#b9b7ae", lw=3, solid_capstyle="round", zorder=1)
    ax.text(70.5, 46.4, "from ANY state", fontsize=12.5, color=INK2, fontweight="bold", va="center")
    for m, txt in ((Mode.FAILSAFE_HOLD_OR_RETREAT, "sensor or envelope fault"),
                   (Mode.COLLISION_AVOIDANCE, guard(rows["collision_risk"]["label"], cfg, True)),
                   (Mode.SEPARATION_WARNING, guard(rows["separation_warning"]["label"], cfg, True))):
        _arrow(ax, (pos[m][0], 46.4), (pos[m][0], T(m)), lw=1.8)
        _label(ax, (pos[m][0] + 0.8, 44.6), txt, fs=11.5, ha="left", bg=False)
    _label(ax, (41.0, 34.4), "↓  hazard cleared: back to the mission level", fs=11.5, color=INK2, bg=False)
    # mutex
    ap, yi, pa, rc = Mode.MUTEX_APPROACH, Mode.MUTEX_YIELD, Mode.MUTEX_PASS, Mode.FORMATION_RECOVERY
    _arrow(ax, (R(ap), pos[ap][1]), (L(yi), pos[yi][1]), lw=1.8)
    _label(ax, ((R(ap) + L(yi)) / 2, T(ap) + 1.6), "no priority\nor occupied", fs=11)
    _arrow(ax, (R(yi), pos[yi][1]), (L(pa), pos[pa][1]), lw=1.8)
    _label(ax, ((R(yi) + L(pa)) / 2, T(yi) + 1.6), "priority and free\n(left first, then top)", fs=11)
    _path(ax, [(pos[pa][0], B(pa)), (pos[pa][0], pos[rc][1]), (R(rc), pos[rc][1])], lw=1.8)
    _label(ax, (pos[pa][0] + 0.8, 22.4), "passed", fs=11.5, ha="left", bg=False)
    wa, fo, de = Mode.FORMATION_WAIT_REJOIN, Mode.FORMATION_FOLLOW, Mode.DEGRADED_FORMATION
    _arrow(ax, (pos[wa][0], T(wa)), (pos[ap][0], B(ap)), lw=1.8)
    _label(ax, (pos[wa][0] + 0.8, 18.6), "mutex zone ahead\n(from every\nformation state)", fs=11, ha="left",
           bg=False)
    # formation
    _arrow(ax, (pos[fo][0] - 1.5, T(fo)), (pos[rc][0] - 1.5, B(rc)), lw=1.8, color=COLOR[rc])
    _label(ax, (pos[fo][0] - 2.3, 14.4), "own slot lost\n(error > " + f"{th['e_lost']:g} m)", fs=11, ha="right",
           bg=False)
    _arrow(ax, (pos[rc][0] + 1.5, B(rc)), (pos[fo][0] + 1.5, T(fo)), lw=1.8, color=COLOR[fo])
    _label(ax, (pos[fo][0] + 2.3, 14.4), "slot recovered\n(error < " + f"{th['e_ok']:g} m for {th['t_ok_hold']:g} s)",
           fs=11, ha="left", bg=False)
    _arrow(ax, (L(fo), pos[fo][1] + 0.8), (R(wa), pos[wa][1] + 0.8), lw=1.8, color=COLOR[wa])
    _label(ax, ((L(fo) + R(wa)) / 2, T(fo) + 0.7), "neighbour missing", fs=11)
    _arrow(ax, (R(wa), pos[wa][1] - 0.8), (L(fo), pos[fo][1] - 0.8), lw=1.8, color=COLOR[fo])
    _label(ax, ((L(fo) + R(wa)) / 2, B(fo) - 1.3), "neighbour rejoined", fs=11)
    _arrow(ax, (L(de), pos[de][1]), (R(fo), pos[fo][1]), lw=1.8, color=COLOR[fo])
    _label(ax, ((R(fo) + L(de)) / 2, T(fo) + 0.7), "drone back in its slot", fs=11)
    _path(ax, [(pos[wa][0], B(wa)), (pos[wa][0], 3.0), (pos[de][0], 3.0), (pos[de][0], B(de))], lw=1.8,
          color=COLOR[de])
    _label(ax, (41.0, 3.0), f"not back within t_rejoin = {model['t_rejoin']:.0f} s → slot declared vacant", fs=11)
    ax.text(73.0, 30.5, "WAIT_REJOIN: the drones that miss a\nneighbour slow the formation to\n"
                        f"{model['v_wait']:.2f} m/s, so the lost drone catches\nup at "
                        f"{cfg.form.v_recovery_max - model['v_wait']:.2f} m/s instead of "
                        f"{cfg.form.v_recovery_max - cfg.form.v_nominal:.2f} m/s.\n\n"
                        "DEGRADED: no reconfiguration, the\nother drones keep their own slots\nat the survey speed; "
                        "the drone can\nstill come back.\n\nEvery state stays in itself when\nnothing above applies.  "
                        "Exact guards\nand all 120 edges: automaton_full.", fontsize=11.5, color=INK2, va="center")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path.with_suffix(".png"), dpi=120)
    fig.savefig(path.with_suffix(".svg"))
    plt.close(fig)


def main() -> int:
    model = diagram_model(DEFAULT)
    missing, extra = coverage(model)
    if missing or extra:
        print("diagram does not match the spec:", missing, extra)
        return 1
    render_full(model, DEFAULT, OUT / "automaton_full")
    render_presentation(model, DEFAULT, OUT / "automaton_presentation")
    print(f"automaton diagrams: {len(model['modes'])} modes, {model['n_edges']} edges covered -> {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
