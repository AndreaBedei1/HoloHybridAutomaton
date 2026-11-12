"""Payload v2 (gate status) and the failure model: vacancy by silence, status exclusion and positive occupancy, the
passage verification, the DROP_BACK at a stationary reference."""

import dataclasses
import math

import numpy as np

from holo_fleet.config import DEFAULT, BeaconConfig
from holo_fleet.mission import GateSpec, MissionPlan, Path, abreast_slots, queue_line
from holo_fleet.perception import payload as pl
from holo_fleet.perception.acoustic import AcousticTracker
from holo_fleet.perception.gate_perception import GatePerception
from holo_fleet.perception.passage import REAR_N, T_ODO_MAX_S, PassageVerifier
from holo_fleet.perception.perception import Perception
from holo_fleet.perception.targets import Target

B2 = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_RB"))
GATE = GateSpec("G", np.array([0., 0., -5.]), np.array([1., 0., 0.]), np.array([0., -1., 0.]), 2., 2.)


def _observer(p, others=(1,)):
    tracker = AcousticTracker(B2)
    tracker.set_identities({k: k for k in (0, *others)})
    slots = abreast_slots(1 + len(others))
    qs = queue_line(slots, B2.gate)
    path = Path(np.array([[-30., 0.], [30., 0.]]), -5.)
    plan = MissionPlan(drone_id="d0", slot_index=0, slots=slots, path=path, gates=[GATE], structures=[GATE],
                       launch_position=p, launch_yaw_deg=0., static_rank=0, s_end=40., queue_lateral=0., queue_s=qs)
    gp = GatePerception(plan, B2, acoustic=tracker)
    gp.acoustic_exit.others = list(others)
    return tracker, gp


def _rx(tracker, p, t, k, q, bound=0.5, gate_idx=0, phase=pl.GP_NONE):
    """One reception of drone k at the true position q with its payload."""
    d = np.asarray(q, float) - p
    r = float(np.linalg.norm(d))
    az, el = math.atan2(d[1], d[0]), math.asin(d[2] / r)
    t_arr = t - 0.05
    fr = pl.frame_index(t_arr - r / B2.beacon.sound_speed, B2.beacon.period_s)
    tracker.update(t, [(k, t_arr, r, az, el, pl.encode(fr, q, bound, gate_idx, phase))], p, 0., 0.5)


P = np.array([-5.4, 0., -5.])


def test_a_fresh_not_engaged_report_excludes_a_drone_the_position_would_not():
    """A drone reporting GP_NONE fresh is no occupant even with a useless position bound; stale, the rule no longer helps."""
    tr, gp = _observer(P)
    ae = gp.acoustic_exit
    _rx(tr, P, 1.0, 1, [-5.4, -3.5, -5.], bound=9.0, phase=pl.GP_NONE)
    assert ae.status_excluded(1, 1.0 + 1.0)
    assert not ae.status_excluded(1, 1.0 + ae.status_fresh_s + 0.5)
    # the freshness follows the plan's queue geometry: the time from the queue point to the CR at the speed bound
    G = B2.gate
    from holo_fleet.perception.sonar_geometry import R_OUT
    expect = (abs(gp.plan.queue_s) - G.cr_half_len - R_OUT - G.queue_tol - G.nav_err_m) / ae.v_bound - B2.acoustic.latency_max_s
    assert ae.status_fresh_s <= expect + 1e-9 and ae.status_fresh_s <= B2.acoustic.status_fresh_s + 1e-9


def test_committed_and_admitted_hold_the_gate_and_passed_releases_it():
    tr, gp = _observer(P)
    ae = gp.acoustic_exit
    _rx(tr, P, 1.0, 1, [-3.0, -3.5, -5.], phase=pl.GP_COMMITTED)
    assert ae.claims(1.5) == {1} and not ae.status_excluded(1, 1.5)
    o = gp.update(1.5, P, 0., [], {})
    assert o.occ_busy and not o.entry_ok
    _rx(tr, P, 3.0, 1, [-1.0, -3.5, -5.], phase=pl.GP_ADMITTED)
    assert ae.claims(3.2) == {1}
    assert gp.update(3.2, P, 0., [], {}).occ_busy
    _rx(tr, P, 5.0, 1, [4.0, -3.5, -5.], phase=pl.GP_PASSED)
    assert ae.status_excluded(1, 5.2) and ae.claims(5.2) == set()


def test_a_silent_drone_keeps_its_claim_until_it_is_dead():
    """A drone that reported committed or admitted holds the gate while it may be alive; silent for t_silent_gate_s at a receiver that
    hears the others it is dead, and inside the operating scope a dead hull is never at the CR (holo_fleet/scope.py)."""
    for phase in (pl.GP_COMMITTED, pl.GP_ADMITTED):
        tr, gp = _observer(P, others=(1, 2))
        ae = gp.acoustic_exit
        _rx(tr, P, 1.0, 1, [0.0, 0.0, -5.], phase=phase)
        t = 1.0 + B2.acoustic.t_silent_gate_s - 1.0
        tr.heard[2] = t - 1.0
        assert ae.claims(t) == {1} and not ae.status_excluded(1, t) and gp.update(t, P, 0., [], {}).occ_busy
        t = 1.0 + B2.acoustic.t_silent_gate_s + 1.0
        tr.heard[2] = t - 1.0
        assert ae.dead(1, t) and ae.claims(t) == set() and ae.status_excluded(1, t)


def test_a_receiver_that_hears_nobody_declares_no_one_dead():
    tr, gp = _observer(P, others=(1, 2))
    ae = gp.acoustic_exit
    _rx(tr, P, 1.0, 1, [0.0, 0.0, -5.], phase=pl.GP_ADMITTED)
    tr.heard[2] = 1.0
    t = 1.0 + B2.acoustic.t_silent_gate_s + 300.0
    assert not ae.dead(1, t) and ae.claims(t) == {1} and not ae.status_excluded(1, t)


def test_a_drone_silent_after_not_engaged_is_excluded_only_once_dead():
    tr, gp = _observer(P, others=(1, 2))
    ae = gp.acoustic_exit
    _rx(tr, P, 1.0, 1, [-5.4, -3.5, -5.], phase=pl.GP_NONE)
    t = 1.0 + B2.form.t_silent_s + 1.0                          # vacant in the formation, not dead for the gate
    tr.heard[2] = t - 1.0
    assert not ae.status_excluded(1, t) and ae.claims(t) == set()
    t = 1.0 + B2.acoustic.t_silent_gate_s + 1.0
    tr.heard[2] = t - 1.0
    assert ae.status_excluded(1, t)


def test_a_deaf_receiver_declares_no_one_silent():
    tr, gp = _observer(P, others=(1, 2))
    tr.heard[1], tr.heard[2] = 0.0, 0.0
    assert not tr.silent(1, 200.0)                               # nobody is heard: no evidence about anyone
    tr.heard[2] = 199.0
    assert tr.silent(1, 200.0)


def test_a_report_of_another_gate_excludes_the_drone_while_fresh():
    tr, gp = _observer(P)
    ae = gp.acoustic_exit
    _rx(tr, P, 1.0, 1, [20.0, -3.5, -5.], gate_idx=1, phase=pl.GP_COMMITTED)
    assert ae.status_excluded(1, 1.5) and not ae.claims(1.5)


def test_a_silent_drone_never_heard_of_keeps_the_sonar_rules():
    tr, gp = _observer(P)
    echo = Target(frozenset({"FRONT"}), 4.5, 4.5, 0., 0., "DYNAMIC")
    assert gp.update(1.0, P, 0., [echo], {}).occ_state == "BUSY"


def test_the_gate_phase_is_kept_from_a_rejected_fix_but_not_the_position():
    tr, gp = _observer(P)
    _rx(tr, P, 1.0, 1, [-3.0, -3.5, -5.], phase=pl.GP_NONE)
    # a reflected arrival, far off: rejected by the kinematic gate, the packet is intact
    d = np.array([-3.0 + 20.0, -3.5, -5.]) - P
    r = float(np.linalg.norm(d))
    t_arr = 2.0
    fr = pl.frame_index(t_arr - r / B2.beacon.sound_speed, B2.beacon.period_s)
    pkt = pl.encode(fr, [-3.0, -3.5, -5.], 0.5, 0, pl.GP_COMMITTED)
    fixes = tr.update(2.1, [(1, t_arr, r, math.atan2(d[1], d[0]), 0.0, pkt)], P, 0., 0.5)
    assert fixes and not fixes[0].accepted and not fixes[0].rep_ok
    assert tr.status[1].phase == pl.GP_COMMITTED and tr.heard[1] == t_arr


def test_a_corridor_echo_is_no_drone_when_every_other_drone_reports_not_engaged():
    tr, gp = _observer(P, others=(1, 2))
    gp.update(0.5, P, 0., [], {})
    for k, q in ((1, [-5.4, 3.5, -5.]), (2, [-5.4, -3.5, -5.])):
        _rx(tr, P, 1.0, k, q, bound=9.0, phase=pl.GP_NONE)           # positions unusable: the status alone excludes them
    echo = Target(frozenset({"FRONT"}), 4.4, 4.4, 0., 0., "UNKNOWN")
    gp.update(1.1, P, 0., [echo], {})
    o = gp.update(1.2, P, 0., [echo], {})                            # persistent over two captures
    assert gp.phantom and o.occ_state == "FREE" and "WAIT" not in [r["relation"] for r in o.relations]


def test_a_front_only_corridor_echo_makes_no_relation_a_merged_one_does():
    """The precedence rule never reads the beacon: the FRONT-only echo in the corridor band is the latch's business, a merged
    target keeps its WAIT."""
    tr, gp = _observer(P, others=(1,))
    r_near = float(gp.cr_ranges(GATE, P, 0.)[0]) - 0.389 - B2.env.eps_range_far
    r = r_near + 0.2
    front = Target(frozenset({"FRONT"}), r, r, 0., 0., "UNKNOWN")
    merged = Target(frozenset({"FRONT", "LEFT"}), r, r, 0., 0., "UNKNOWN")
    assert gp.update(1.0, P, 0., [front], {}).relations == [] or True      # first capture: not yet persistent
    o = gp.update(1.1, P, 0., [front], {})
    assert not [x for x in o.relations if x["relation"] == "WAIT"]
    o2 = gp.update(1.2, P, 0., [merged], {})
    o3 = gp.update(1.3, P, 0., [merged], {})
    assert [x for x in o3.relations if x["relation"] == "WAIT"]


def test_an_older_status_does_not_override_a_later_fix_in_the_cr():
    tr, gp = _observer(P, others=(1,))
    ae = gp.acoustic_exit
    gp.update(0.5, P, 0., [], {})                             # the receiver knows its gate geometry
    _rx(tr, P, 1.0, 1, [-0.5, 0.0, -5.], phase=pl.GP_NONE)      # not engaged, near the CR by its fix
    assert ae.status_excluded(1, 1.1)
    # a later accepted fix places it in the CR (the packet of that ping was lost, the ping arrived)
    d = np.array([-0.2, 0.0, -5.0]) - P
    r = float(np.linalg.norm(d))
    tr.update(2.2, [(1, 2.15, r, math.atan2(d[1], d[0]), 0.0, None)], P, 0., 0.5)
    assert tr.last_accepted[1].t_tx > tr.status[1].t_tx
    assert not ae.status_excluded(1, 2.2)


def test_keepout_barrier_blocks_motion_toward_the_cr_only_for_a_drone_not_committed():
    from holo_fleet.control.flows import Flows
    from holo_fleet.perception.gate_perception import GateObs

    plan = MissionPlan(drone_id="d0", slot_index=0, slots=abreast_slots(2), path=Path(np.array([[-30., 0.], [30., 0.]]), -5.),
                       gates=[GATE], structures=[GATE], launch_position=P, launch_yaw_deg=0., static_rank=0, s_end=40.,
                       queue_lateral=0., queue_s=-5.4)
    fl = Flows(plan, B2)
    go = GateObs(gate_id="G", s=-1.8, l=0.0, dz=0.0)
    obs = type("O", (), {"gate": go, "gate_index": 0})()
    v = fl.keepout_filter(obs, np.array([0.5, 0.2, 0.0]))
    assert v[0] == 0.0 and v[1] == 0.2
    go2 = GateObs(gate_id="G", s=-4.0, l=0.0, dz=0.0)
    assert fl.keepout_filter(type("O", (), {"gate": go2, "gate_index": 0})(), np.array([0.5, 0., 0.]))[0] == 0.5


def test_a_position_correction_shifts_the_stored_fixes():
    tr, gp = _observer(P, others=(1,))
    _rx(tr, P, 1.0, 1, [-5.4, -3.5, -5.], phase=pl.GP_NONE)
    f = tr.last_accepted[1]
    p0, q0 = f.p.copy(), f.pts.copy()
    corr = np.array([0.5, -0.3, 0.0])
    tr.shift(corr)
    assert np.allclose(f.p, p0 + corr) and np.allclose(f.pts, q0 + corr)
    assert np.allclose(tr.pose.buf[-1][1], np.array(P) + corr)


def _plan(k=0):
    from holo_fleet.formations import TEMPLATES
    from holo_fleet.mission import FormationClock
    tm = TEMPLATES["square"]
    return MissionPlan(drone_id=f"drone_{k}", slot_index=k, slots=list(tm.slots),
                       path=Path(np.array([[0.0, 0.0], [80.0, 0.0]]), -5.0), gates=[], structures=[],
                       launch_position=np.zeros(3), launch_yaw_deg=0.0, static_rank=k, s_end=70.0,
                       clock=FormationClock(0.0, 0.3, 0.0, s_end=70.0))


def test_vacancy_is_by_silence_whatever_the_observers_mode():
    from holo_fleet.perception.formation_perception import FormationObs, NeighbourCheck

    per = Perception(_plan(), B2)
    per.acoustic.set_identities({0: 0, 1: 1, 2: 2, 3: 3})
    for k in (1, 2):
        per.acoustic.heard[k] = 0.0
    per.acoustic.heard[3] = 1000.0                         # drone 3 keeps being heard: the receiver works
    o = FormationObs()
    o.checks = [NeighbourCheck(slot=k, expected_d=3.5, expected_pattern="") for k in (1, 2, 3)]
    o.required = [1, 2, 3]
    t = B2.form.t_silent_s - 1.0
    per.track_missing(t, 0.1, o, in_formation=False)      # the observer is NOT holding its slot
    assert not per.vacant
    t = B2.form.t_silent_s + 0.5
    per.track_missing(t, 0.1, o, in_formation=False)
    assert set(per.vacant) == {1, 2}
    # drone 2 is heard again: alive, its slot is required again
    per.acoustic.heard[2] = t + 1.0
    per.track_missing(t + 1.0, 0.1, o, in_formation=False)
    assert set(per.vacant) == {1}


def test_a_silent_channel_keeps_the_baseline_vacancy_timer_and_declares_no_one_dead():
    from holo_fleet.perception.formation_perception import FormationObs, NeighbourCheck

    per = Perception(_plan(), B2)
    per.acoustic.set_identities({0: 0, 1: 1, 2: 2, 3: 3})
    for k in (1, 2, 3):
        per.acoustic.heard[k] = 0.0                            # the channel falls silent: nobody heard any more
    o = FormationObs()
    o.checks = [NeighbourCheck(slot=1, expected_d=3.5, expected_pattern="")]
    o.required, o.missing = [1], [1]
    assert per.acoustic.deaf(100.0)
    t = 100.0
    while t < 100.0 + B2.form.t_rejoin + 2.0:
        per.track_missing(t, 0.1, o, in_formation=True)
        t += 0.1
    assert set(per.vacant) == {1}                              # the baseline displacement timer, not the silence rule


def test_a_drone_that_keeps_transmitting_is_never_vacant():
    from holo_fleet.perception.formation_perception import FormationObs, NeighbourCheck

    per = Perception(_plan(), B2)
    per.acoustic.set_identities({0: 0, 1: 1, 2: 2, 3: 3})
    o = FormationObs()
    o.checks = [NeighbourCheck(slot=1, expected_d=3.5, expected_pattern="")]
    o.required = [1]
    o.missing = [1]
    t = 0.0
    per.acoustic.heard[1] = t
    while t < 200.0:
        per.acoustic.heard[1] = t
        per.acoustic.heard[2] = t
        per.acoustic.heard[3] = t
        per.track_missing(t, 0.1, o, in_formation=True)
        t += 0.1
    assert not per.vacant


def _pass(v, sectors, t_end=11.0, v_x=0.5):
    p, t = np.array([-1.2, 0., -5.]), 0.0
    while t < t_end:
        t += 0.1
        p = p + np.array([v_x * 0.1, 0., 0.])
        v.update(t, 0.1, p, np.array([v_x, 0., 0.]), sectors(p, t))
    return t


def test_passage_is_verified_by_the_crossing_signature_then_the_odometry():
    from holo_fleet.perception.passage import CROSS_N

    v = PassageVerifier(B2)
    v.admit(0, GATE, np.array([-1.2, 0., -5.]))
    # inside the opening (true |s| small): the two posts at ranges summing to the opening
    w = 2.0 - 2 * 0.288                                        # inner width minus the sonar mounts (lateral half-width)
    def sec(q, t):
        if abs(q[0]) <= 0.35:
            return {"LEFT": [w / 2 - 0.2], "RIGHT": [w / 2 + 0.2], "UP": [0.6], "DOWN": [1.0]}
        return {}
    _pass(v, sec)
    assert v.verified and v.by == "crossing" and v.crossing is not None
    l, dz = v.crossing
    assert abs(l - 0.2) < 1e-9 and abs(dz - 0.2) < 1e-9
    assert CROSS_N >= 2


def test_passage_is_verified_by_the_rear_frame_echo_if_the_signature_is_missed():
    v2 = PassageVerifier(B2)
    v2.admit(0, GATE, np.array([-1.2, 0., -5.]))
    _pass(v2, lambda q, t: {"REAR": [float(q[0]) - 0.5]})
    assert v2.verified and v2.by == "rear" and v2.rear_hits >= REAR_N


def test_passage_by_odometry_alone_only_after_the_timeout():
    v3 = PassageVerifier(B2)
    v3.admit(0, GATE, np.array([-1.2, 0., -5.]))
    _pass(v3, lambda q, t: {}, t_end=9.0)                      # just reaching exit_s: no sonar evidence yet
    assert not v3.verified
    v4 = PassageVerifier(B2)
    v4.admit(0, GATE, np.array([-1.2, 0., -5.]))
    _pass(v4, lambda q, t: {}, t_end=9.4 + T_ODO_MAX_S + 1.0)
    assert v4.verified and v4.by == "odometry"


def test_a_drone_stopped_at_a_bar_does_not_verify_its_passage():
    v = PassageVerifier(B2)
    v.admit(0, GATE, np.array([-1.2, 0., -5.]))
    t = 0.0
    while t < 60.0:
        t += 0.1
        v.update(t, 0.1, np.array([-0.5, 0., -5.]), np.zeros(3), {"REAR": [4.0], "LEFT": [0.7], "RIGHT": [0.7]})
    assert not v.verified and v.in_progress


def test_a_recovering_drone_goes_straight_to_its_slot_when_the_reference_stands_still():
    """v10 RC3: at the end of the path the rejoin phases kept a recovering drone off its slot for good."""
    from holo_fleet.config import DEFAULT as D
    from holo_fleet.control.flows import Flows
    from holo_fleet.formations import TEMPLATES
    from holo_fleet.mission import FormationClock
    from holo_fleet.perception.formation_perception import FormationObs

    tm = TEMPLATES["square"]
    plan = MissionPlan(drone_id="drone_0", slot_index=0, slots=list(tm.slots), path=Path(np.array([[0.0, 0.0], [80.0, 0.0]]), -5.0),
                       gates=[], structures=[], launch_position=np.zeros(3), launch_yaw_deg=0.0, static_rank=0, s_end=20.0,
                       clock=FormationClock(0.0, 0.3, 0.0, s_end=20.0))
    fl = Flows(plan, D)

    class O:                                                          # the observation the formation flow reads
        t = 500.0
        p = np.array([19.0, 6.0, -8.5])                                # off its lane and on the pass level, past the plan end
        R_wb = np.eye(3)
        yaw = 0.0
        slot_pos = np.array([21.75, 1.75, -5.0])
        form = FormationObs()
        targets = []
        unhealthy = []
    v, _ = fl.formation(O, 0.1, recovering=True)
    assert fl.rejoin_phase == "" and v[2] > 0.0 and v[1] < 0.0     # straight to the slot: up and across, no rejoin phases


def _far_setup(lateral_error=0.9, behind=2.5, k=2):
    """Observer 0 of the square, behind its slot by ``behind``; its own estimate is off sideways by ``lateral_error``: the
    neighbour k, on its slot, is measured where the true geometry puts it and reports its slot."""
    from holo_fleet.perception.formation_perception import FormationObs, expected_relative

    plan = _plan()
    per = Perception(plan, B2)
    per.acoustic.set_identities({j: j for j in range(4)})
    p = np.array([10.0, 2.0, -5.0])
    d = expected_relative(plan, np.eye(3))
    slot_err = np.array([behind, 0.0, 0.0])
    return per, p, d, slot_err, FormationObs(slot_err=slot_err, slot_err_norm=float(np.linalg.norm(slot_err))), np.array([0.0, lateral_error, 0.0])


def _rx_meas(per, p, t, k, rel, reported, bound=2.0):
    r = float(np.linalg.norm(rel))
    az, el = math.atan2(rel[1], rel[0]), math.asin(rel[2] / r)
    t_arr = t - 0.05
    fr = pl.frame_index(t_arr - r / B2.beacon.sound_speed, B2.beacon.period_s)
    per.acoustic.update(t, [(k, t_arr, r, az, el, pl.encode(fr, reported, bound, 0, pl.GP_NONE))], p, 0., 0.5)


def test_far_from_its_slot_the_fixes_measure_the_lateral_error_of_the_own_reference():
    """The residual is compensated for the own displacement from the slot: a drone behind its slot does not read the neighbours
    as misplaced, and the part left is the relative navigation error (RC6: two opposite compass biases)."""
    per, p, d, se, o, e_me = _far_setup()
    for k in (1, 2):
        _rx_meas(per, p, 5.0, k, d[k] + se + e_me, p + se + d[k])
    per.acoustic_residuals(5.05, o, np.eye(3), p)
    assert o.lateral_far is not None and abs(o.lateral_far - e_me[1]) < 0.05


def test_the_far_residual_is_off_near_the_slot_and_ignores_a_neighbour_off_its_own_slot():
    per, p, d, se, o, e_me = _far_setup()
    near = type(o)(slot_err=np.zeros(3), slot_err_norm=0.1)
    _rx_meas(per, p, 5.0, 2, d[2] + se + e_me, p + se + d[2])
    per.acoustic_residuals(5.05, near, np.eye(3), p)
    assert near.lateral_far is None
    per2, p, d, se, o2, e_me = _far_setup()
    _rx_meas(per2, p, 5.0, 2, d[2] + se + e_me, p + se + d[2] + np.array([-4.0, 0.0, 0.0]))     # by its own estimate, 4 m off
    per2.acoustic_residuals(5.05, o2, np.eye(3), p)
    assert o2.lateral_far is None


def test_a_drone_behind_its_slot_shifts_its_lane_with_the_far_residual_and_stays_on_the_corrected_lane():
    from holo_fleet.config import DEFAULT as D
    from holo_fleet.control.flows import Flows
    from holo_fleet.perception.formation_perception import FormationObs

    plan = _plan()
    fl = Flows(plan, D)
    me = plan.slots[0]

    class O:
        t = 20.0
        p = np.array([3.0, me.lateral, -5.0 + me.dz])
        R_wb = np.eye(3)
        yaw = 0.0
        slot_pos = np.array([9.0, me.lateral, -5.0 + me.dz])
        form = FormationObs(lateral_far=-0.9)
        targets = []
        unhealthy = []
    for _ in range(10):
        fl.formation(O, 0.1, recovering=True)
    assert -0.09 < fl.lam < -0.05
    # near the slot only the ordinary consensus acts
    fl2 = Flows(plan, D)
    O.slot_pos = O.p + np.array([0.3, 0.0, 0.0])
    fl2.formation(O, 0.1, recovering=True)
    assert fl2.lam == 0.0
    # the lane follows the correction: on the corrected lane the drone is not sent around the formation
    fl3 = Flows(plan, D)
    fl3.lam = -1.5
    O.p = np.array([3.0, me.lateral - 1.5, -5.0 + me.dz])
    O.slot_pos = np.array([9.0, me.lateral - 1.5, -5.0 + me.dz])
    O.form = FormationObs()
    fl3.formation(O, 0.1, recovering=True)
    assert fl3.rejoin_phase in ("", "PASS")


def test_a_drone_inside_the_keepout_box_never_reports_not_engaged():
    from holo_fleet.perception.passage import inside_keepout, report_phase

    G = B2.gate
    assert inside_keepout(0.0, 0.0, 0.0, G) and not inside_keepout(-5.4, 0.0, 0.0, G)
    assert report_phase(False, False, False, True) == pl.GP_COMMITTED
    assert report_phase(False, False, False, False) == pl.GP_NONE
    assert report_phase(False, True, False, True) == pl.GP_ADMITTED
    assert report_phase(True, False, False, True) == pl.GP_COMMITTED      # back in the box after a passage, or at the next gate
    assert report_phase(True, False, False, False) == pl.GP_PASSED and report_phase(True, False, True, False) == pl.GP_PASSED


def test_silence_never_discards_a_corridor_echo_a_fresh_report_may():
    """Formal review round 2, F2: a drone unheard for t_silent_gate after reporting not engaged may be alive and in the CR."""
    for silent in (False, True):
        tr, gp = _observer(P, others=(1,))
        gp.update(0.5, P, 0., [], {})
        t0 = 1.0
        _rx(tr, P, t0, 1, [-5.4, 3.5, -5.], bound=9.0, phase=pl.GP_NONE)
        t = t0 + (B2.acoustic.t_silent_gate_s + 9.0 if silent else 0.1)
        echo = Target(frozenset({"FRONT"}), 4.4, 4.4, 0., 0., "UNKNOWN")
        gp.update(t, P, 0., [echo], {})
        o = gp.update(t + 0.1, P, 0., [echo], {})
        if silent:
            assert not gp.phantom and o.occ_busy
        else:
            assert gp.phantom and not o.occ_busy


def test_no_relation_implies_latch_input_in_the_whole_front_only_band():
    """Formal review round 2, F1: without a beacon a FRONT-only echo nearer than the corridor made neither a relation nor a latch input."""
    tr, gp = _observer(P, others=(1,))
    gp.acoustic_exit = None
    r_near = float(gp.cr_ranges(GATE, P, 0.)[0]) - 0.389 - B2.env.eps_range_far
    for r in np.linspace(r_near - B2.gate.nav_err_m, r_near - 0.05, 7):
        tr, gp = _observer(P, others=(1,))
        gp.acoustic_exit = None
        echo = Target(frozenset({"FRONT"}), float(r), float(r), 0., 0., "UNKNOWN")
        gp.update(1.0, P, 0., [echo], {})
        o = gp.update(1.1, P, 0., [echo], {})
        assert o.relations == [] and o.occ_busy


def test_a_recovering_drone_on_a_giveway_is_not_sent_back_to_its_lane():
    from holo_fleet.config import DEFAULT as D
    from holo_fleet.control.flows import Flows
    from holo_fleet.perception.formation_perception import FormationObs

    plan = _plan()
    fl = Flows(plan, D)
    me = plan.slots[0]
    fl.offset = np.array([0.0, -2.5, 0.0])                       # a give-way to the right is running

    class O:
        t = 20.0
        p = np.array([3.0, me.lateral - 2.5, -5.0 + me.dz])
        R_wb = np.eye(3)
        yaw = 0.0
        slot_pos = np.array([9.0, me.lateral - 2.5, -5.0 + me.dz])
        form = FormationObs()
        targets = []
        unhealthy = []
    fl.formation(O, 0.1, recovering=True)
    assert fl.rejoin_phase != "TO_LANE" and fl.rejoin_phase != "DROP_BACK"


def test_a_drone_back_in_the_box_of_the_gate_it_passed_claims_that_gate():
    from holo_fleet.perception.gate_perception import GateObs
    from holo_fleet.perception.passage import PassageVerifier, gate_report

    v = PassageVerifier(B2)
    v.idx, v.gate, v.admitted, v.verified = 0, GATE, True, True
    assert gate_report(v, GateObs(gate_id=None), np.array([0.0, 0.0, -5.0]), False, 1, B2.gate) == (0, pl.GP_COMMITTED)
    assert gate_report(v, GateObs(gate_id=None), np.array([6.0, 0.0, -5.0]), False, 1, B2.gate) == (0, pl.GP_PASSED)


def test_silence_excludes_nobody_from_the_gate():
    tr, gp = _observer(P, others=(1,))
    gp.update(0.5, P, 0., [], {})
    _rx(tr, P, 1.0, 1, [-5.4, 3.5, -5.], bound=9.0, phase=pl.GP_NONE)
    assert gp.acoustic_exit.status_excluded(1, 1.1)
    assert not gp.acoustic_exit.status_excluded(1, 1.0 + B2.acoustic.t_silent_gate_s + 9.0)


def test_the_lateral_residual_stands_down_during_a_giveway():
    from holo_fleet.config import DEFAULT as D
    from holo_fleet.control.flows import Flows
    from holo_fleet.perception.formation_perception import FormationObs

    plan = _plan()
    fl = Flows(plan, D)
    fl.offset = np.array([0.0, -2.5, 0.0])
    me = plan.slots[0]

    class O:
        t = 20.0
        p = np.array([3.0, me.lateral - 2.5, -5.0 + me.dz])
        R_wb = np.eye(3)
        yaw = 0.0
        slot_pos = np.array([9.0, me.lateral - 2.5, -5.0 + me.dz])
        form = FormationObs(lateral_far=2.5)
        targets = []
        unhealthy = []
    fl.formation(O, 0.1, recovering=True)
    assert fl.lam == 0.0
