"""Occupancy evidence must protect commitment as well as admission and pending releases."""

import dataclasses

import numpy as np
import pytest

from holo_fleet.config import DEFAULT, BeaconConfig
from holo_fleet.mission import GateSpec, MissionPlan, Path, abreast_slots, queue_line
from holo_fleet.perception.acoustic import AcousticTracker
from holo_fleet.perception.acoustic_gate import AcousticExit
from holo_fleet.perception.gate_perception import GatePerception
from holo_fleet.perception.targets import Target

B2 = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_RB"))
GATE = GateSpec("G", np.array([0., 0., -5.]), np.array([1., 0., 0.]), np.array([0., -1., 0.]), 2., 2.)


def observer(p=None):
    qs = queue_line(abreast_slots(3), B2.gate)
    p = np.array([qs, 0., -5.]) if p is None else np.asarray(p)
    tracker = AcousticTracker(B2)
    tracker.set_identities({1: 1, 2: 2})
    tracker.update(0., (), p, 0.)
    plan = MissionPlan(drone_id="d", slot_index=0, slots=abreast_slots(3),
                       path=Path(np.array([[-20., 0.], [20., 0.]]), -5.), gates=[GATE], structures=[GATE],
                       launch_position=p, launch_yaw_deg=0., static_rank=0, s_end=40., queue_lateral=0., queue_s=qs)
    return p, tracker, GatePerception(plan, B2, acoustic=tracker)


def heard_pair(tracker, p, r):
    for t in (1., 2.):
        tracker.update(t, [(1, t - .25, r, 0., 0.)], p, 0.)


def test_deployed_multipath_envelope_protects_an_inside_occupant():
    p, tracker, gp = observer([-1.179, 0., -5.])
    # True observer -2.179, occupant +0.8: range 2.979 plus the declared +3 m excess.
    heard_pair(tracker, p, 5.979)
    assert all(f.accepted for f in tracker.all_fixes(1))
    assert not gp.acoustic_exit.clear_of_cr(GATE, tracker.latest(1, 2., 5.), 2.)
    assert gp.acoustic_exit.heard_inside(2., GATE)


@pytest.mark.parametrize("initial", ["FREE", "BUSY", "EXITING"])
def test_positive_occupancy_keeps_every_latch_state_busy(initial):
    p, tracker, gp = observer()
    heard_pair(tracker, p, -p[0])
    gp.occ_state, gp.exit_seen_t = initial, 0.
    sonar_time = gp.last_corridor_t
    o = gp.update(2., p, 0., [], {})
    assert o.occ_state == "BUSY" and o.occ_busy and not o.entry_ok
    assert gp.last_corridor_t == sonar_time  # acoustic evidence must not change the sonar witness's clock


def test_positive_occupancy_vetoes_a_sonar_witness_and_discards_its_confirmation():
    p, tracker, gp = observer()
    heard_pair(tracker, p, -p[0])
    gp.occ_state, gp.last_corridor_t, gp.busy_since = "BUSY", 0., 0.
    echo = Target(frozenset({"FRONT"}), 7.5, 7.5, 0., 0., "DYNAMIC")
    for t in (2., 2.1, 2.2):
        o = gp.update(t, p, 0., [echo], {})
        assert o.occ_state == "BUSY" and o.exit_witness is None
    # Once the receptions expire, suppressed captures cannot immediately become a release.
    for t in (5., 5.1):
        o = gp.update(t, p, 0., [echo], {})
        assert o.occ_state == "BUSY"
    assert gp.update(5.2, p, 0., [echo], {}).occ_state == "EXITING"


def test_acoustic_only_episode_never_times_out_without_positive_clearance():
    # an occupant heard inside and then silent is unknown, not gone: no timeout release (positive evidence only)
    p, tracker, gp = observer()
    heard_pair(tracker, p, -p[0])
    assert gp.update(2., p, 0., [], {}).occ_state == "BUSY"
    assert gp.update(2. + B2.acoustic.fresh_s + .1, p, 0., [], {}).occ_state == "BUSY"
    assert gp.update(2. + B2.gate.t_occ_max + .1, p, 0., [], {}).occ_state == "BUSY"
    assert gp.occ_timeouts == 0


def test_acoustic_only_episode_releases_on_positive_clearance_of_every_other_drone():
    p, tracker, gp = observer()
    heard_pair(tracker, p, -p[0])
    assert gp.update(2., p, 0., [], {}).occ_state == "BUSY"
    # 7.5 m outward motion in 15 s; all three outside confirmations follow the acoustic episode.
    for t in (17., 18., 19.):
        tracker.update(t, [(1, t, -p[0] + 7.5, 0., 0.), (2, t, 30., 0., 0.)], p, 0.)
        o = gp.update(t, p, 0., [], {})
    assert o.occ_state == "EXITING" and o.exit_witness["rule"] == "A2"
    assert gp.update(20.1, p, 0., [], {}).occ_state == "FREE"
    assert gp.last_corridor_t == -1e9 and gp.occ_timeouts == 0


def test_acoustic_only_episode_needs_clearance_of_every_drone_heard_possibly_inside():
    # an acoustic-only episode started from FREE: a drone that was never heard inside would have crossed the FRONT
    # corridor visibly to enter (the sonar latch premise) and started a sonar episode; every drone HEARD possibly
    # inside during the episode must be positively cleared (A2)
    p, tracker, gp = observer()
    for t in (1., 2.):
        tracker.update(t, [(1, t - .25, -p[0], 0., 0.), (2, t - .2, -p[0], 0., 0.)], p, 0.)
    assert gp.update(2., p, 0., [], {}).occ_state == "BUSY"
    assert gp.suspects == {1, 2}
    for t in (17., 18., 19.):                       # only drone 1 is heard outside
        tracker.update(t, [(1, t, -p[0] + 7.5, 0., 0.)], p, 0.)
        assert gp.update(t, p, 0., [], {}).occ_state == "BUSY"
    for t in (20., 21., 22.):                       # drone 2 too: released
        tracker.update(t, [(1, t, -p[0] + 7.5, 0., 0.), (2, t - .1, -p[0] + 7.5, 0., 0.)], p, 0.)
        o = gp.update(t, p, 0., [], {})
    assert o.occ_state == "EXITING" and o.exit_witness["rule"] == "A2"
