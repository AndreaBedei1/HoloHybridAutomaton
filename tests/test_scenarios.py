"""Scenario catalogue: short, one seed, generic templates, honest envelope labels."""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.sim.scenarios import SCENARIOS

EXPECTED = {"p1_head_on": 2, "p1_vertical_escape": 4, "p1_two_lines": 6, "formation_triangle": 3,
            "formation_square": 4, "formation_six": 6, "formation_gust": 4, "gate_single": 3, "integrated_short": 3}


def test_catalogue_is_small_and_short():
    assert set(SCENARIOS) == set(EXPECTED)
    for name, n in EXPECTED.items():
        sc = SCENARIOS[name](DEFAULT)
        assert len(sc.plans) == n == len(sc.sim.names), name
        assert sc.duration_s <= 95.0, name
        assert sc.sim.seed == 0, name


def test_same_controller_three_templates():
    names = {SCENARIOS[n](DEFAULT).template.name for n in ("formation_triangle", "formation_square", "formation_six")}
    assert names == {"triangle", "square", "line6"}


def test_gate_scenarios_use_only_arena_gate_g06():
    for name in ("gate_single", "integrated_short"):
        sc = SCENARIOS[name](DEFAULT)
        assert [g.gate_id for g in sc.judged_gates] == ["G06"] and sc.sim.gate_ids == ("G06",)
        assert {round(p.queue_s, 2) for p in sc.plans} == {round(DEFAULT.gate.queue_s(len(sc.plans)), 2)}
        assert sorted(p.queue_lateral for p in sc.plans) == sorted(DEFAULT.gate.queue_laterals(len(sc.plans)))


def test_currents_inside_the_claimed_envelope_except_declared_disturbances():
    env = DEFAULT.env
    for name in EXPECTED:
        sc = SCENARIOS[name](DEFAULT)
        for t in np.arange(0.0, sc.duration_s, 0.5):
            if sc.disturbance_active(t):
                continue
            for p in sc.sim.spawn_positions:
                w = sc.sim.current.drift_at(np.asarray(p, float), t)
                assert np.linalg.norm(w[:2]) <= env.current_drift_max + 1e-9 and abs(w[2]) <= env.current_vertical_max


def test_only_the_safety_layer_demo_switches_the_traffic_rule_off():
    for name in EXPECTED:
        sc = SCENARIOS[name](DEFAULT)
        assert sc.cfg_patch == ({"traffic_rule": False} if name == "p1_vertical_escape" else {}), name
        assert bool(getattr(sc.sim, "intruders", ())) == (name == "p1_vertical_escape")
