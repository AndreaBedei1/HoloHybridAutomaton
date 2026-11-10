"""A silent drone that may be a passive hull in the CR holds the gate (AcousticExit.adrift)."""

import dataclasses
import math

import numpy as np

from holo_fleet.config import DEFAULT, BeaconConfig
from holo_fleet.mission import GateSpec, MissionPlan, Path, abreast_slots, queue_line
from holo_fleet.perception import payload as pl
from holo_fleet.perception.acoustic import AcousticTracker
from holo_fleet.perception.gate_perception import GatePerception

B2 = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_RB"))
GATE = GateSpec("G", np.array([0., 0., -5.]), np.array([1., 0., 0.]), np.array([0., -1., 0.]), 2., 2.)
P = np.array([-5.4, 0., -5.])


def _observer():
    tracker = AcousticTracker(B2)
    tracker.set_identities({k: k for k in (0, 1, 2)})
    slots = abreast_slots(3)
    path = Path(np.array([[-30., 0.], [30., 0.]]), -5.)
    plan = MissionPlan(drone_id="d0", slot_index=0, slots=slots, path=path, gates=[GATE], structures=[GATE], launch_position=P,
                       launch_yaw_deg=0., static_rank=0, s_end=40., queue_lateral=0., queue_s=queue_line(slots, B2.gate))
    gp = GatePerception(plan, B2, acoustic=tracker)
    gp.acoustic_exit.others = [1]
    return tracker, gp


def _rx(tracker, t, q, phase=pl.GP_NONE):
    d = np.asarray(q, float) - P
    r = float(np.linalg.norm(d))
    az, el = math.atan2(d[1], d[0]), math.asin(d[2] / r)
    t_arr = t - 0.05
    fr = pl.frame_index(t_arr - r / B2.beacon.sound_speed, B2.beacon.period_s)
    tracker.update(t, [(1, t_arr, r, az, el, pl.encode(fr, q, 0.5, 0, phase))], P, 0., 0.5)


def _water(ae, v):
    for _ in range(50):
        ae.note_water([v, 0.0, 0.0])


def test_a_hull_that_can_reach_the_cr_holds_latch_and_admission_until_it_is_heard_again():
    tr, gp = _observer()
    ae = gp.acoustic_exit
    _water(ae, 0.3)
    _rx(tr, 1.0, [-5.4, 3.5, -5.])
    tr.heard[2] = 61.0                                  # the receiver hears someone: it is not deaf
    o = gp.update(61.0, P, 0.0, [], {})
    assert ae.adrift(61.0, GATE) == {1} and o.occ_busy and not o.entry_ok
    # t_occ_max does not release it
    tr.heard[2] = 200.0
    o = gp.update(200.0, P, 0.0, [], {})
    assert o.occ_busy and not o.entry_ok
    _rx(tr, 201.0, [-5.4, 3.5, -5.])
    assert ae.adrift(201.0, GATE) == set()


def test_the_reach_grows_with_the_silence_at_the_water_speed():
    tr, _gp = _observer()
    ae = _gp.acoustic_exit
    _water(ae, 0.3)
    _rx(tr, 1.0, [-9.0, 0.0, -5.])
    tr.heard[2] = 200.0
    assert ae.adrift(1.0 + 4.0, GATE) == set()
    assert ae.adrift(1.0 + 25.0, GATE) == {1}


def test_a_hull_too_far_to_reach_the_cr_before_rising_over_it_holds_nothing():
    """The dead drone of lost_drone_mutex: far from the gate in still water; the sonar rules apply."""
    tr, gp = _observer()
    ae = gp.acoustic_exit
    _water(ae, 0.0)
    _rx(tr, 1.0, [-15.0, 3.5, -5.])
    tr.heard[2] = 500.0
    assert ae.adrift(500.0, GATE) == set()
    assert not gp.update(500.0, P, 0.0, [], {}).occ_busy


def test_a_drone_above_the_top_of_the_cr_holds_nothing():
    tr, gp = _observer()
    ae = gp.acoustic_exit
    _water(ae, 0.3)
    _rx(tr, 1.0, [-3.0, 0.0, -2.0])
    tr.heard[2] = 100.0
    assert ae.adrift(100.0, GATE) == set()


def test_a_deaf_receiver_learns_nothing_from_silence():
    tr, gp = _observer()
    ae = gp.acoustic_exit
    _water(ae, 0.3)
    _rx(tr, 1.0, [-5.4, 3.5, -5.])
    assert tr.deaf(1.0 + B2.form.t_silent_s + 1.0) and ae.adrift(1.0 + B2.form.t_silent_s + 1.0, GATE) == set()


def test_a_fresh_report_of_not_engaged_excludes_it():
    tr, gp = _observer()
    ae = gp.acoustic_exit
    _water(ae, 0.3)
    _rx(tr, 1.0, [-5.4, 3.5, -5.])
    tr.heard[2] = 2.0
    assert ae.adrift(2.0, GATE) == set()


def test_a_passed_report_excludes_it_for_good_a_declared_limit():
    """A hull that reported having passed and is then carried back upstream is not covered (docs/GUARANTEES.md)."""
    tr, gp = _observer()
    ae = gp.acoustic_exit
    _water(ae, 0.3)
    _rx(tr, 1.0, [3.0, 0.0, -5.], phase=pl.GP_PASSED)
    tr.heard[2] = 100.0
    assert ae.adrift(100.0, GATE) == set()


def test_the_hold_rests_on_the_fix_and_not_on_what_the_water_does_afterwards():
    tr, gp = _observer()
    ae = gp.acoustic_exit
    _water(ae, 0.3)
    _rx(tr, 1.0, [-5.4, 3.5, -5.])
    tr.heard[2] = 61.0
    assert ae.adrift(61.0, GATE) == {1}
    ae.water_speeds.clear()
    _water(ae, 0.0)
    tr.heard[2] = 91.0
    o = gp.update(91.0, P, 0.0, [], {})
    assert ae.adrift(91.0, GATE) == {1} and o.occ_busy and not o.entry_ok


def test_a_receiver_that_goes_deaf_keeps_the_holds_it_has_and_starts_no_new_one():
    tr, gp = _observer()
    ae = gp.acoustic_exit
    _water(ae, 0.3)
    _rx(tr, 1.0, [-5.4, 3.5, -5.])
    tr.heard[2] = 30.0
    assert ae.adrift(30.0, GATE) == {1}
    t = 30.0 + B2.form.t_silent_s + 1.0
    assert tr.deaf(t) and ae.adrift(t, GATE) == {1}
    assert not gp.update(t, P, 0.0, [], {}).entry_ok
    tr2, gp2 = _observer()
    ae2 = gp2.acoustic_exit
    _water(ae2, 0.3)
    _rx(tr2, 1.0, [-5.4, 3.5, -5.])
    assert tr2.deaf(t) and ae2.adrift(t, GATE) == set()
