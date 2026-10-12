"""3-D escape on sector patterns (deployed planner): guarantees, vertical escapes, no chattering."""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.control.escape import EscapePlanner
from holo_fleet.perception import sonar_geometry as sg

ENV = DEFAULT.env
F = frozenset


def test_single_threat_escape_guarantee_for_every_pattern():
    pl = EscapePlanner()
    for P in sg.regions():
        pl.reset()
        ch = pl.escape_velocity([(P, ENV.v_open)], [], 0.0, ENV.v_escape)
        assert ch.feasible and ch.guarantee >= ENV.g_min, (sorted(P), ch.label, ch.guarantee)


def test_warning_filter_opens_at_v_open_for_every_pattern():
    pl = EscapePlanner()
    for P in sg.regions():
        pl.reset()
        ch = pl.warning_velocity(np.array([0.3, 0.0, 0.0]), [(P, ENV.v_open)], [], 0.0, ENV.v_max_nominal)
        opening = float((-(sg.regions()[P] @ ch.v_body)).min()) - sg.SAMPLING_ERR * float(np.linalg.norm(ch.v_body))
        assert ch.feasible and opening >= ENV.v_open - 1e-9, sorted(P)


def test_threat_ahead_below_is_escaped_upwards_and_vice_versa():
    pl = EscapePlanner()
    assert pl.escape_velocity([(F({"FRONT", "DOWN"}), ENV.v_open)], [], 0.0, ENV.v_escape).direction[2] > 0.35
    pl.reset()
    assert pl.escape_velocity([(F({"FRONT", "UP"}), ENV.v_open)], [], 0.0, ENV.v_escape).direction[2] < -0.35


def test_boxed_in_drone_leaves_through_a_free_vertical_sector():
    pl = EscapePlanner()
    threats = [(F({"FRONT", "DOWN"}), ENV.v_open)]
    ch = pl.escape_velocity(threats, [], 0.0, ENV.v_escape, occupied={"LEFT", "RIGHT", "REAR", "FRONT"},
                            depth_room=(4.0, 4.0))
    assert ch.direction[2] > 0.35 and ch.guarantee > 0


def test_escape_respects_the_depth_band():
    pl = EscapePlanner()
    ch = pl.escape_velocity([(F({"FRONT", "DOWN"}), ENV.v_open)], [], 0.0, ENV.v_escape, depth_room=(4.0, 0.3))
    assert ch.direction[2] <= 0.3


def test_hysteresis_keeps_a_valid_previous_direction():
    pl = EscapePlanner()
    first = pl.escape_velocity([(F({"FRONT"}), ENV.v_open)], [], 0.0, ENV.v_escape)
    again = [pl.escape_velocity([(F({"FRONT"}), ENV.v_open)], [], 0.0, ENV.v_escape,
                                mission_body=np.array([0.0, 0.0, s])).label for s in (1.0, -1.0, 1.0, -1.0)]
    assert set(again) == {first.label}


def test_head_on_pair_does_not_escape_the_same_way():
    a, b = EscapePlanner(), EscapePlanner()
    ea = a.escape_velocity([(F({"FRONT"}), ENV.v_open)], [], 0.0, ENV.v_escape).direction        # heading east
    eb = b.escape_velocity([(F({"FRONT"}), ENV.v_open)], [], np.pi, ENV.v_escape).direction      # heading west
    rot = np.diag([-1.0, -1.0, 1.0])                                                              # B's body -> world
    assert float(ea @ (rot @ eb)) < 0.5
