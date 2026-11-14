"""B2 payload (H6): packet format, slot budget, onboard navigation bound and its use as independent evidence."""

import dataclasses
import math

import numpy as np
import pytest

from holo_fleet.config import DEFAULT, BeaconConfig
from holo_fleet.mission import GateSpec, MissionPlan, Path, abreast_slots, queue_line
from holo_fleet.perception import payload as pl
from holo_fleet.perception.acoustic import AcousticTracker
from holo_fleet.perception.gate_perception import GatePerception
from holo_fleet.perception.nav import NavBound, NavState

B2 = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_RB"))
GATE = GateSpec("G", np.array([0., 0., -5.]), np.array([1., 0., 0.]), np.array([0., -1., 0.]), 2., 2.)


def test_packet_round_trip_quantisation_and_bound_rounded_up():
    p = np.array([-20.1234, 17.276, -4.2031])
    pkt = pl.encode(7, p, 1.234)
    rep = pl.decode(pkt)
    assert rep.frame == 7 % pl.FRAMES and len(pkt) * 8 == pl.PAYLOAD_BITS == 64
    assert np.linalg.norm(rep.p - p) <= pl.QUANT_ERR + 1e-9
    assert 1.234 + pl.QUANT_ERR <= rep.bound < 1.234 + pl.QUANT_ERR + pl.BOUND_FINE_Q
    assert rep.gate_idx == 0 and rep.phase == pl.GP_NONE
    rep2 = pl.decode(pl.encode(1, p, 7.0, gate_idx=2, phase=pl.GP_ADMITTED))
    assert rep2.gate_idx == 2 and rep2.phase == pl.GP_ADMITTED


def test_the_bound_field_is_rounded_up_and_never_saturates_to_a_smaller_value():
    for b in (0.0, 0.31, 1.0, 3.89, 3.9, 3.91, 6.2, 7.1, 12.34, 15.4):
        code = pl.encode_bound(b)
        assert code != pl.BOUND_INVALID and pl.decode_bound(code) >= b - 1e-9
        assert pl.decode_bound(code) - b <= (pl.BOUND_FINE_Q if b <= pl.BOUND_FINE_MAX else pl.BOUND_COARSE_Q) + 1e-9
    assert pl.encode_bound(15.5) == pl.BOUND_INVALID and pl.encode_bound(math.inf) == pl.BOUND_INVALID


def test_corrupted_or_unrepresentable_packets_are_not_used():
    pkt = bytearray(pl.encode(1, [1., 2., -3.], 0.5))
    pkt[3] ^= 0x10
    assert pl.decode(bytes(pkt)) is None
    for rep in (pl.decode(pl.encode(1, [1., 2., -3.], float("inf"))),
                pl.decode(pl.encode(1, [150., 2., -3.], 0.5)),
                pl.decode(pl.encode(1, [1., 2., -3.], 99.0))):
        assert rep is not None and not rep.valid_position


def test_frame_index_is_robust_to_millisecond_errors_at_every_slot():
    P = 1.0
    for n in (2, 3, 6, 7):
        for m in (0, 5, 255):
            for k in range(n):
                t = m * P + k * P / n
                assert pl.frame_index(t - 0.003, P) == pl.frame_index(t + 0.003, P) == m % pl.FRAMES


def test_payload_fits_the_tdma_slot_for_six_drones_and_is_refused_when_it_cannot():
    from holo_fleet.sim.beacon import BeaconTransport

    cfg = BeaconConfig(mode="ACOUSTIC_RB")
    BeaconTransport([f"d{k}" for k in range(6)], cfg, 0)
    with pytest.raises(ValueError):
        BeaconTransport([f"d{k}" for k in range(8)], cfg, 0)


def test_nav_bound_grows_with_distance_noise_and_outage_and_never_shrinks():
    nb = NavBound(B2)
    s = NavState(p=np.zeros(3), yaw=0.0)
    s.v_world = np.array([0.3, 0.0, 0.0])
    prev = nb.value
    for i in range(1, 201):
        s.t_dvl, s.t_depth = i * 0.1, i * 0.1
        nb.update(i * 0.1, 0.1, s)
        assert nb.value >= prev
        prev = nb.value
    lin = B2.acoustic.nav_b0_m + nb.k_lin * 6.0 + nb.const
    assert nb.horizontal == pytest.approx(lin + nb.k_rw * math.sqrt(200 * 0.01))
    before = nb.horizontal
    for i in range(201, 211):
        nb.update(i * 0.1, 0.1, s, model=True)
    q = float(np.interp(nb.out_len, nb.q_len, nb.q_max))
    assert nb.outage == pytest.approx(q) and 0.3 < q < 0.5
    nb2 = NavBound(B2)
    s2 = NavState(p=np.zeros(3), yaw=0.0)
    s2.v_world, s2.t_dvl, s2.t_depth = np.array([0.3, 0.0, 0.0]), 20.0, 20.0
    for i in range(201, 211):
        nb2.update(i * 0.1, 0.1, s2, model=False)
    assert nb2.outage > nb.outage
    nb.corrected(np.array([0.3, 0.4, 0.0]))
    assert nb.aid == pytest.approx(0.5) and not nb.invalid


def test_the_bound_is_a_ball_over_horizontal_and_vertical_errors():
    nb = NavBound(B2)
    nb.z = 1.0
    assert nb.value == pytest.approx(math.hypot(nb.horizontal, 1.0))
    assert nb.value > max(nb.horizontal, 1.0)


def test_a_correction_larger_than_the_bound_invalidates_it():
    nb = NavBound(B2)
    nb.corrected(np.array([nb.horizontal + 0.5, 0.0, 0.0]))
    assert nb.invalid and nb.value == math.inf
    assert not pl.decode(pl.encode(0, [0., 0., -3.], nb.value)).valid_position


def test_held_dvl_samples_count_as_an_outage_through_the_frame_stamps():
    """Integration with the real frame: a frame holds the latest DVL value with its OLD stamp."""
    from holo_fleet.perception.perception import Perception
    from holo_fleet.perception.frame import SensorFrame
    from holo_fleet.sim.scenarios import SCENARIOS

    plan = SCENARIOS["gate_single"](B2).plans[0]
    per = Perception(plan, B2)
    dvl = np.array([0.3, 0.0, 0.0])
    for i in range(1, 31):
        t = i * 0.1
        stamp_dvl = t if t <= 1.0 else 1.0
        fr = SensorFrame(t=t, data={"DVLSensor": dvl, "Compass": np.array([1.0, 0.0, 0.0]), "DepthSensor": [-5.0]},
                         stamp={"DVLSensor": stamp_dvl, "Compass": t, "DepthSensor": t})
        per.nav.update(t, 0.1, fr.data.get("Compass"), fr.data.get("DVLSensor"), fr.data.get("DepthSensor"), None,
                       stamps=fr.stamp)
        per.nav_bound.update(t, 0.1, per.nav.state)
    assert per.nav.state.t_dvl == pytest.approx(1.0)
    assert per.nav_bound.outage > 0.5


def _observer(p):
    tracker = AcousticTracker(B2)
    tracker.set_identities({1: 1, 2: 2})
    tracker.update(0., (), p, 0., 0.5)
    qs = queue_line(abreast_slots(3), B2.gate)
    plan = MissionPlan(drone_id="d", slot_index=0, slots=abreast_slots(3),
                       path=Path(np.array([[-20., 0.], [20., 0.]]), -5.), gates=[GATE], structures=[GATE],
                       launch_position=p, launch_yaw_deg=0., static_rank=0, s_end=40., queue_lateral=0., queue_s=qs)
    return tracker, GatePerception(plan, B2, acoustic=tracker)


def _hear(tracker, p, r, rep_p, bound, times=(1., 2.)):
    for t in times:
        t_arr = t - .25
        frame = pl.frame_index(t_arr - r / B2.beacon.sound_speed, B2.beacon.period_s)
        tracker.update(t, [(1, t_arr, r, 0., 0., pl.encode(frame, rep_p, bound))], p, 0., 0.5)
        tracker.status.clear()


def test_payload_separates_the_indistinguishable_multipath_histories():
    """Observer at s=-4.6, measured 8.4 m: inside with +3 m excess (s=0.8) or outside (s=3.8).  The range-bearing
    measurement alone must veto both; the independent onboard estimate of the sender separates them."""
    p = np.array([-4.6, 0., -5.])
    tracker, gp = _observer(p)
    _hear(tracker, p, 8.4, [3.8, 0., -5.], 1.0)
    f = tracker.latest(1, 2., 5.)
    assert f.rep_ok
    assert not gp.acoustic_exit.heard_inside(2., GATE)
    tracker2, gp2 = _observer(p)
    _hear(tracker2, p, 8.4, [0.8, 0., -5.], 1.0)
    assert tracker2.latest(1, 2., 5.).rep_ok
    assert gp2.acoustic_exit.heard_inside(2., GATE)


def test_payload_inconsistent_with_the_measurement_is_rejected_and_the_veto_kept():
    p = np.array([-4.6, 0., -5.])
    tracker, gp = _observer(p)
    _hear(tracker, p, 5.4, [20.0, 0., -5.], 0.5)
    f = tracker.latest(1, 2., 5.)
    assert not f.rep_ok and f.rep_reason == "inconsistent"
    assert gp.acoustic_exit.heard_inside(2., GATE)


def test_a_large_bound_gives_no_clearance():
    p = np.array([-4.6, 0., -5.])
    tracker, gp = _observer(p)
    _hear(tracker, p, 8.4, [3.8, 0., -5.], 4.0)
    assert gp.acoustic_exit.heard_inside(2., GATE)


def test_unknown_occupant_never_times_out_to_free():
    """A drone last heard possibly inside, then silent: the t_occ_max fallback does not release the CR."""
    p = np.array([-4.6, 0., -5.])
    tracker, gp = _observer(p)
    _hear(tracker, p, 4.6, [0.0, 0., -5.], 0.5)
    gp.occ_state, gp.last_corridor_t, gp.busy_since = "BUSY", 2., 2.
    for t in (5., 15., 2. + B2.gate.t_occ_max + 5.):
        assert gp.update(t, p, 0., [], {}).occ_state == "BUSY"


def test_a_packet_of_another_frame_is_not_used():
    p = np.array([-4.6, 0., -5.])
    tracker, gp = _observer(p)
    for t in (1., 2.):
        tracker.update(t, [(1, t - .25, 8.4, 0., 0., pl.encode(3, [3.8, 0., -5.], 1.0))], p, 0., 0.5)
    f = tracker.latest(1, 2., 5.)
    assert not f.rep_ok and f.rep_reason == "frame"


def test_the_veto_needs_two_cleared_fixes_to_drop():
    p = np.array([-4.6, 0., -5.])
    tracker, gp = _observer(p)
    _hear(tracker, p, 8.4, [0.8, 0., -5.], 0.5)
    assert gp.acoustic_exit.heard_inside(2., GATE)
    _hear(tracker, p, 8.4, [3.8, 0., -5.], 0.5, times=(3.,))
    assert gp.acoustic_exit.heard_inside(3., GATE)
    _hear(tracker, p, 8.4, [3.8, 0., -5.], 0.5, times=(4.,))
    assert not gp.acoustic_exit.heard_inside(4., GATE)


def test_stage_two_entry_is_held_while_an_occupant_is_merely_unheard():
    p = np.array([-4.6, 0., -5.])
    tracker, gp = _observer(p)
    _hear(tracker, p, 4.6, [0.0, 0., -5.], 0.5)
    assert not gp.update(2., p, 0., [], {}).entry_ok
    o = gp.update(2. + B2.acoustic.fresh_s + 1.0, p, 0., [], {})
    assert not gp.acoustic_exit.heard_inside(2. + B2.acoustic.fresh_s + 1.0, GATE) and not o.entry_ok


def test_a_corridor_echo_no_fleet_drone_can_have_produced_does_not_set_the_latch():
    from holo_fleet.perception.targets import Target

    p = np.array([-4.6, 0., -5.])
    tracker, gp = _observer(p)
    gp.acoustic_exit.others = [1]
    echo = Target(frozenset({"FRONT"}), 4.0, 4.0, 0., 0., "DYNAMIC")
    _hear(tracker, p, 20.0, [-4.6, 20.0, -5.], 0.5)
    o = gp.update(2.1, p, 0., [echo], {})
    assert gp.phantom and o.occ_state == "FREE"
    tracker2, gp2 = _observer(p)
    gp2.acoustic_exit.others = [1]
    _hear(tracker2, p, 4.6, [0.0, 0., -5.], 0.5)
    o2 = gp2.update(2.1, p, 0., [echo], {})
    assert not gp2.phantom and o2.occ_state == "BUSY"
    tracker3, gp3 = _observer(p)
    gp3.acoustic_exit.others = [1]
    assert gp3.update(2.1, p, 0., [echo], {}).occ_state == "BUSY"


def test_silence_never_clears_a_drone_last_known_possibly_inside():
    """v11: the buoyancy-rise release of v8 is gone: a silent drone last known possibly in the CR stays a possible
    occupant for ever (silence alone is no proof that the gate is physically clear)."""
    p = np.array([-4.6, 0., -5.])
    tracker, gp = _observer(p)
    _hear(tracker, p, 4.6, [0.0, 0., -5.], 0.5)
    for t in (2. + 60., 2. + 3600.):
        assert gp.acoustic_exit.last_known_inside(t, GATE)
