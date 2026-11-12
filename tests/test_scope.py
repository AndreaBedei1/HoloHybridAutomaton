"""The operating scope: the exclusion zone of a gate (holo_fleet/scope.py), and every failure scenario outside it."""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.scope import failure_excluded, obstruction_half
from holo_fleet.sim.scenarios import SCENARIOS


def _gate():
    return SCENARIOS["lost_drone_mutex"](DEFAULT).judged_gates[0]


def test_the_critical_region_and_the_space_beside_the_frame_are_excluded():
    g = _gate()
    h = obstruction_half(g)
    assert failure_excluded(g, g.from_gate_frame([0.0, 0.0, 0.0]), 0.0)
    assert failure_excluded(g, g.from_gate_frame([-(h[0] + 0.9), 0.0, 0.0]), 0.0)          # within the coast
    assert not failure_excluded(g, g.from_gate_frame([-(h[0] + 1.1), 0.0, 0.0]), 0.0)


def test_the_zone_grows_with_the_current_and_covers_everything_when_the_hull_cannot_rise():
    g = _gate()
    p = g.from_gate_frame([-8.0, 0.0, 0.0])
    assert not failure_excluded(g, p, 0.0) and failure_excluded(g, p, 0.3)
    assert failure_excluded(g, g.from_gate_frame([-60.0, 30.0, 0.0]), 0.0, w_down=DEFAULT.failure.v_rise_min_mps)


def test_above_the_frame_nothing_is_excluded():
    g = _gate()
    h = obstruction_half(g)
    assert not failure_excluded(g, g.from_gate_frame([0.0, 0.0, h[2] + 0.01]), 0.5)


def _max_current(sc):
    """Upper bound of the horizontal and downward water speed of the scenario (sum over its components)."""
    w_h = w_dn = 0.0
    for c in sc.sim.current.describe():
        d = np.asarray(c.get("drift", (0.0, 0.0, 0.0)), float)
        w_h += float(np.hypot(d[0], d[1]))
        w_dn += max(-float(d[2]), 0.0)
    return w_h, w_dn


def test_every_failure_scenario_fails_outside_the_exclusion_zone():
    """The drone fails at t_on: it has moved at most v_max_nominal * t_on from its spawn, towards the gate in the worst case."""
    v = DEFAULT.env.v_max_nominal
    n = 0
    for name, make in SCENARIOS.items():
        sc = make(DEFAULT)
        for f in sc.sim.faults:
            if f["type"] != "TOTAL_FAILURE" or f.get("t_off") is not None:
                continue
            p0 = np.asarray(sc.sim.spawn_positions[sc.sim.names.index(f["drone"])], float)
            w_h, w_dn = _max_current(sc)
            for g in sc.judged_gates:
                u = np.asarray(g.center, float) - p0
                u[2] = 0.0
                p = p0 + min(v * float(f["t_on"]), float(np.linalg.norm(u))) * u / max(float(np.linalg.norm(u)), 1e-9)
                assert not failure_excluded(g, p, w_h, w_dn), name
                n += 1
    assert n >= 3                                         # lost_drone_mutex and its two rise variants
