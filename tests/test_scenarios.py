"""Scenario catalogue: short, one seed, generic templates, honest envelope labels."""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.mission import queue_assignment, queue_line
from holo_fleet.sim.scenarios import SCENARIOS

EXPECTED = {"p1_head_on": 2, "p1_vertical_escape": 4, "p1_two_lines": 6, "p1_close_encounter": 2,
            "formation_triangle": 3, "formation_square": 4, "formation_six": 6, "formation_recovery_head_current": 4,
            "formation_gust": 4, "gate_single": 3, "integrated_short": 3, "lost_drone_rejoin": 4,
            "lost_drone_timeout": 4, "mutex_deadlock_resolution": 3, "line_parallel_mutex": 3, "lost_drone_mutex": 3}
GATE_SCENARIOS = ("gate_single", "integrated_short", "mutex_deadlock_resolution", "line_parallel_mutex", "lost_drone_mutex")


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
    G = DEFAULT.gate
    for name in GATE_SCENARIOS:
        sc = SCENARIOS[name](DEFAULT)
        assert [g.gate_id for g in sc.judged_gates] == ["G06"] and sc.sim.gate_ids == ("G06",)
        assert {round(p.queue_s, 2) for p in sc.plans} == {round(queue_line(sc.template.slots, G), 2)}
        qa = queue_assignment(sc.template.slots, G)
        assert [(p.queue_lateral, p.queue_dz) for p in sc.plans] == qa
        assert sorted(p.static_rank for p in sc.plans) == list(range(len(sc.plans)))
    # abreast templates keep the v2 abreast queue; the stacked pair queues one drone above the other
    sc = SCENARIOS["gate_single"](DEFAULT)
    assert sorted(p.queue_lateral for p in sc.plans) == sorted(G.queue_laterals(3))
    assert {p.queue_s for p in sc.plans} == {G.queue_s(3)}                 # abreast: the v2 queue line
    sc = SCENARIOS["mutex_deadlock_resolution"](DEFAULT)
    assert [(p.queue_lateral, p.queue_dz) for p in sc.plans] == [(2.5, 1.75), (2.5, -1.75), (-2.5, 0.0)]
    assert [p.static_rank for p in sc.plans] == [0, 1, 2]


def test_lost_drone_scenarios_inject_one_simulator_fault_on_a_rear_drone():
    for name, permanent in (("lost_drone_rejoin", False), ("lost_drone_timeout", True), ("lost_drone_mutex", True)):
        sc = SCENARIOS[name](DEFAULT)
        assert len(sc.sim.faults) == 1
        f = sc.sim.faults[0]
        k = int(f["drone"].split("_")[1])
        assert f["type"] == "THRUSTER_FAILURE" and (f["t_off"] is None) == permanent
        assert sc.template.slots[k].along == min(sl.along for sl in sc.template.slots)     # nobody runs into it
        if name == "lost_drone_mutex":                       # the left drone of the line: the leftmost present goes
            assert sc.template.name == "line3" and k == 0
        assert sc.disturbance_active(f["t_on"] + 0.1)
    for name in set(EXPECTED) - {"lost_drone_rejoin", "lost_drone_timeout", "lost_drone_mutex"}:
        assert not SCENARIOS[name](DEFAULT).sim.faults, name


def test_currents_inside_the_envelope_except_declared_disturbances():
    """Outside the declared disturbance windows every current is control-feasible for the survey velocity
    (DI-27, along the mission path at the planned clock speed) and inside the exercised range."""
    from holo_fleet.control.current_envelope import check_current

    for name in EXPECTED:
        sc = SCENARIOS[name](DEFAULT)
        for plan, p in zip(sc.plans, sc.sim.spawn_positions):
            _q, d, _n = plan.path.frame_at(0.0)
            v = np.array([d[0], d[1], 0.0]) * (plan.clock.v if plan.clock is not None else DEFAULT.form.v_nominal)
            for t in np.arange(0.0, sc.duration_s, 0.5):
                if sc.disturbance_active(t):
                    continue
                c = check_current(sc.sim.current.drift_at(np.asarray(p, float), t), v, DEFAULT)
                assert c.ok, (name, t, c.reason)


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


def test_head_current_is_in_range_but_not_control_feasible():
    """The jet stays within the exercised range (0.6 m/s) but, against the 0.30 m/s survey velocity, it is
    beyond the control-feasible head limit (about 0.41 m/s): outside the corrected envelope (DI-27)."""
    from holo_fleet.control.current_envelope import check_current, head_limit

    sc = SCENARIOS["formation_recovery_head_current"](DEFAULT)
    jet, = sc.sim.current.components
    assert np.isclose(np.linalg.norm(jet.drift), DEFAULT.env.current_validated_max) and jet.drift[0] < 0.0
    worst = max(np.linalg.norm(sc.sim.current.drift_at(np.array([x, y, -5.0]), t)[:2])
                for t in np.arange(10.0, 26.0, 0.5) for x in np.arange(-20.0, -8.0, 0.5) for y in (-32.25, -35.75))
    assert worst <= DEFAULT.env.current_validated_max + 1e-9
    c = check_current(jet.drift, (DEFAULT.form.v_nominal, 0.0, 0.0), DEFAULT)
    assert c.in_range and not c.feasible and norm_head(jet) > head_limit(DEFAULT.form.v_nominal, DEFAULT)


def norm_head(jet):
    return -float(jet.drift[0])
