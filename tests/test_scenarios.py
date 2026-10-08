"""Scenario catalogue: short, one seed, generic templates, honest envelope labels."""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.sim.scenarios import SCENARIOS

EXPECTED = {"p1_head_on": 2, "p1_vertical_escape": 4, "p1_two_lines": 6, "p1_close_encounter": 2,
            "formation_triangle": 3, "formation_square": 4, "formation_six": 6, "formation_recovery_head_current": 4,
            "formation_gust": 4, "gate_single": 3, "integrated_short": 3}


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


def test_close_encounter_is_a_simultaneous_right_angle_crossing():
    sc = SCENARIOS["p1_close_encounter"](DEFAULT)
    (a0, b0), (a1, b1) = [(p.path.waypoints[0], p.path.waypoints[-1]) for p in sc.plans]
    d0, d1 = (b0 - a0) / np.linalg.norm(b0 - a0), (b1 - a1) / np.linalg.norm(b1 - a1)
    assert abs(float(d0 @ d1)) < 1e-9                                  # perpendicular legs
    cross = np.array([0.0, -31.0])
    assert np.isclose(np.linalg.norm(cross - a0), np.linalg.norm(cross - a1))     # same lead: same arrival
    assert all(p.clock.v == DEFAULT.form.v_nominal for p in sc.plans)  # survey speed on both clocks
    assert not sc.sim.current.components and not getattr(sc.sim, "intruders", ())


def test_head_current_stays_at_the_claimed_drift_limit():
    sc = SCENARIOS["formation_recovery_head_current"](DEFAULT)
    jet, = sc.sim.current.components
    assert np.isclose(np.linalg.norm(jet.drift), DEFAULT.env.current_drift_max) and jet.drift[0] < 0.0
    worst = max(np.linalg.norm(sc.sim.current.drift_at(np.array([x, y, -5.0]), t)[:2])
                for t in np.arange(10.0, 26.0, 0.5) for x in np.arange(-20.0, -8.0, 0.5) for y in (-32.25, -35.75))
    assert worst <= DEFAULT.env.current_drift_max + 1e-9
