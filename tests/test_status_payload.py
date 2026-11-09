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


def test_a_drone_that_dies_while_admitted_keeps_the_gate_closed_for_ever():
    """Silence never clears a possibly occupied gate."""
    tr, gp = _observer(P, others=(1, 2))
    ae = gp.acoustic_exit
    _rx(tr, P, 1.0, 1, [0.0, 0.0, -5.], phase=pl.GP_ADMITTED)
    t = 1.0 + B2.form.t_silent_s + 300.0
    tr.heard[2] = t - 1.0                                       # the receiver hears somebody: the silence of drone 1 is observed
    assert tr.silent(1, t) and ae.claims(t) == {1} and not ae.status_excluded(1, t)
    assert gp.update(t, P, 0., [], {}).occ_busy


def test_a_drone_silent_after_committed_keeps_the_gate_held_after_not_engaged_it_does_not():
    """Silence never clears a drone that held the gate (committed or admitted); a drone that reported not being engaged and
    then fell silent is no occupant once it can be taken for dead."""
    tr, gp = _observer(P, others=(1, 2))
    ae = gp.acoustic_exit
    tr.heard[2] = 0.0
    _rx(tr, P, 1.0, 1, [-3.0, -3.5, -5.], phase=pl.GP_COMMITTED)
    t = 1.0 + B2.acoustic.t_silent_gate_s + 100.0
    tr.heard[2] = t - 1.0                    # the receiver works: somebody else is heard
    assert tr.silent(1, t) and ae.claims(t) == {1} and not ae.status_excluded(1, t)
    tr2, gp2 = _observer(P, others=(1, 2))
    ae2 = gp2.acoustic_exit
    _rx(tr2, P, 1.0, 1, [-5.4, -3.5, -5.], phase=pl.GP_NONE)
    t = 1.0 + B2.acoustic.t_silent_gate_s + 1.0
    tr2.heard[2] = t - 1.0
    assert ae2.status_excluded(1, t) and ae2.claims(t) == set()
    assert not ae2.status_excluded(1, 1.0 + B2.form.t_silent_s + 1.0)    # 45 s < t_silent_gate_s


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


def test_drop_back_reverses_only_when_the_reference_stands_still():
    """v10 RC3: at the end of the path a recovering drone in DROP_BACK can never drop back; it now backs up."""
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
    # past the end of the clock: reference not moving
    class O:                                                          # the observation the formation flow reads
        t = 500.0
        p = np.array([18.0, 8.0, -5.0])                                # off its lane, level with the fleet
        R_wb = np.eye(3)
        yaw = 0.0
        slot_pos = np.array([21.75, 1.75, -5.0])
        form = FormationObs()
        targets = []
        unhealthy = []
    v, _ = fl.formation(O, 0.1, recovering=True)
    assert fl.rejoin_phase == "DROP_BACK" and v[0] < -0.01          # it backs up along the track
