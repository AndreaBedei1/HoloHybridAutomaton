"""The automaton diagram is generated from the spec (scripts/make_automaton_diagram.py): every edge of
build_edges is represented, nothing else; the guards drawn are the spec labels; the committed figures name every
state and show the guards; the readable labels only use observation variables and configured thresholds."""

import html
import sys
from pathlib import Path

import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.ha import guard_text
from holo_fleet.ha.spec import BOOL_VARS, IN_FORMATION, MODES, REAL_VARS, Mode, build_edges

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
FIG = ROOT / "figures" / "v2" / "automaton"


@pytest.fixture(scope="module")
def mad():
    import make_automaton_diagram

    return make_automaton_diagram


def test_diagram_represents_exactly_the_spec_edges(mad):
    model = mad.diagram_model(DEFAULT)
    missing, extra = mad.coverage(model)
    assert not missing and not extra
    E = build_edges(DEFAULT)
    assert model["n_edges"] == sum(len(v) for v in E.values()) == 120
    assert set(model["modes"]) == set(Mode) and len(model["modes"]) == 10
    # the formation-keeping arrows are exactly the non-self-loop formation edges of the keeping modes
    drawn = {(k["source"], k["target"]) for k in model["keeping"]}
    assert drawn == {(m, e.target) for m in IN_FORMATION for e in E[m]
                     if e.name in mad.KEEPING_EDGES and e.target != m}
    # self-loops: one per mode at least ("otherwise: remain")
    assert all(model["loops"][m] for m in MODES)


def test_drawn_guards_are_the_spec_labels(mad):
    model = mad.diagram_model(DEFAULT)
    E = build_edges(DEFAULT)
    for r in model["rows"]:
        for m in MODES:
            assert next(e for e in E[m] if e.name == r["edge"]).label == r["variants"].get(m, r["label"])
    for k in model["keeping"]:
        assert next(e for e in E[k["source"]] if e.name == k["edge"]).label == k["label"]
    allowed = set(REAL_VARS) | set(BOOL_VARS) | set(guard_text.thresholds(DEFAULT))
    for m, es in E.items():
        for e in es:
            assert e.label and guard_text.names(e.label) <= allowed, (e.name, e.label)
            # a mode change always logs a cause, a self-loop none
            assert bool(e.reason) == (e.target != m), (m, e.name, e.reason)


def test_committed_figures_show_every_state_and_the_guards(mad):
    model = mad.diagram_model(DEFAULT)
    for name in ("automaton_full", "automaton_presentation"):
        assert (FIG / f"{name}.png").stat().st_size > 50_000
    full = html.unescape((FIG / "automaton_full.svg").read_text(encoding="utf-8"))
    for m in Mode:
        assert m.value in full, m
    for r in model["rows"]:
        assert mad.guard(r["label"], DEFAULT) in full, r["edge"]
    for k in model["keeping"]:
        assert mad.guard(k["label"], DEFAULT) in full, (k["source"], k["target"])
    assert f"t_rejoin = {DEFAULT.form.t_rejoin:.0f} s" in full
    pres = html.unescape((FIG / "automaton_presentation.svg").read_text(encoding="utf-8"))
    for nick in mad.NICK.values():
        assert nick in pres, nick
