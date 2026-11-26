"""Fixes of the explicit-communication review (formal/check_b2_comm.py): the B2 admission wait, no admission in FAILSAFE or with
an unhealthy FRONT sonar, silence never excludes, claims only delay admissions."""

import dataclasses
from types import SimpleNamespace

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.control.controller import DroneController
from holo_fleet.control.flows import RELOC_ZONE_M, Flows
from holo_fleet.ha.spec import Mode
from holo_fleet.perception import payload as pl
from holo_fleet.perception.frame import SensorFrame
from holo_fleet.perception.perception import sonar_sensor_name
from holo_fleet.perception.sonar_processing import SECTORS, SectorReading
from holo_fleet.sim.scenarios import SCENARIOS

from test_status_payload import B2, P, _observer, _rx

G = DEFAULT.gate
A = B2.acoustic


def _obs(plan, s, t, wait=0.0, entry_ok=True, loc_ok=True):
    g = plan.gates[0]
    gate = SimpleNamespace(s=s, l=0.0, dz=0.0, queue_s=plan.queue_s, queue_l=plan.queue_lateral, queue_dz=plan.queue_dz,
                           loc_ok=loc_ok, entry_ok=entry_ok, admit_wait_s=wait)
    return SimpleNamespace(t=t, p=np.asarray(g.from_gate_frame([s, 0.0, 0.0]), float), gate=gate, gate_index=0, targets=[])


def _plan():
    return SCENARIOS["lost_drone_mutex"](DEFAULT).plans[0]


def test_the_derived_parameters_match_their_derivation():
    assert G.admit_zone_m == RELOC_ZONE_M
    assert G.t_cross_max == pytest.approx((G.passage_clear_s - G.stage2_hold_s + RELOC_ZONE_M + G.nav_err_m) / G.v_cross_min)
    assert A.commit_admit_min_s > A.admit_fresh_s + 2 * A.clock_skew_s + 0.1
    assert A.admit_fresh_s >= 3 * B2.beacon.period_s + A.latency_max_s


def test_no_admission_before_the_commitment_is_old_enough():
    """With admit_wait_s the hold keeps a localized, quiet drone until its commitment has that age (own clock)."""
    plan = _plan()
    fl = Flows(plan, DEFAULT)
    s_hold, w = G.stage2_hold_s, A.commit_admit_min_s
    fl.gate(_obs(plan, s_hold + 0.02, 10.0, wait=w), "MUTEX_PASS")
    fl.gate(_obs(plan, s_hold + 0.02, 10.0 + w - 0.2, wait=w), "MUTEX_PASS")
    assert not fl.cr_cleared and fl.entry_state == "HOLD"
    fl.gate(_obs(plan, s_hold + 0.02, 10.0 + w + 0.1, wait=w), "MUTEX_PASS")
    assert fl.cr_cleared and fl.entry_state == "ADMIT"


def test_the_degraded_wait_is_not_given_up_before_it_is_possible():
    """A degraded admission (some drone not freshly heard) waits T_c + t_cross_max after the commitment; the give-up timer of the
    hold runs only from then, so the drone is admitted, not retreating."""
    plan = _plan()
    fl = Flows(plan, DEFAULT)
    s_hold, w = G.stage2_hold_s, A.commit_admit_min_s + G.t_cross_max
    t = 0.0
    while t < w - 0.1:
        fl.gate(_obs(plan, s_hold + 0.02, t, wait=w), "MUTEX_PASS")
        assert not fl.cr_cleared and not fl.hold_timeout
        t += 0.5
    fl.gate(_obs(plan, s_hold + 0.02, w + 0.1, wait=w), "MUTEX_PASS")
    assert fl.cr_cleared and not fl.hold_timeout


def test_the_wait_restarts_with_a_new_commitment():
    plan = _plan()
    fl = Flows(plan, DEFAULT)
    w = A.commit_admit_min_s
    fl.gate(_obs(plan, G.stage2_hold_s + 0.02, 0.0, wait=w), "MUTEX_PASS")
    fl.gate(_obs(plan, -6.0, 1.0, wait=w), "MUTEX_APPROACH")
    fl.gate(_obs(plan, G.stage2_hold_s + 0.02, 5.0, wait=w), "MUTEX_PASS")
    assert not fl.cr_cleared
    fl.gate(_obs(plan, G.stage2_hold_s + 0.02, 9.1, wait=w), "MUTEX_PASS")
    assert fl.cr_cleared


def test_admit_wait_is_the_short_one_only_with_a_fresh_report_of_every_other_drone():
    tr, gp = _observer(P, others=(1, 2))
    o = gp.update(1.0, P, 0., [], {})
    assert not o.comm_fresh and o.admit_wait_s == pytest.approx(A.commit_admit_min_s + G.t_cross_max)
    _rx(tr, P, 1.0, 1, [-5.4, -3.5, -5.], phase=pl.GP_NONE)
    assert gp.update(1.2, P, 0., [], {}).admit_wait_s == pytest.approx(A.commit_admit_min_s + G.t_cross_max)
    _rx(tr, P, 1.5, 2, [-5.4, 3.5, -5.], phase=pl.GP_NONE)
    o = gp.update(1.6, P, 0., [], {})
    assert o.comm_fresh and o.admit_wait_s == pytest.approx(A.commit_admit_min_s)
    o = gp.update(1.0 + A.admit_fresh_s + 0.1, P, 0., [], {})
    assert not o.comm_fresh and o.admit_wait_s == pytest.approx(A.commit_admit_min_s + G.t_cross_max)


def test_b0_has_no_admission_wait():
    from holo_fleet.perception.gate_perception import GatePerception

    plan = _plan()
    o = GatePerception(plan, DEFAULT).update(1.0, np.asarray(plan.gates[0].from_gate_frame([-2.5, 0., 0.]), float), 0., [], {})
    assert o.admit_wait_s == 0.0


def _healthy(ok):
    return {s: SectorReading(sector=s, age=0.0 if ok else 9.0, healthy=ok) for s in SECTORS}


def test_an_unhealthy_front_sonar_is_never_quiet():
    """entry_ok: no echo is evidence only from a working FRONT sonar - quiet needs it healthy for the whole quiet window."""
    tr, gp = _observer(P, others=(1,))
    assert gp.update(1.0, P, 0., [], _healthy(True)).entry_ok
    assert not gp.update(1.1, P, 0., [], _healthy(False)).entry_ok
    assert not gp.update(1.3, P, 0., [], _healthy(True)).entry_ok
    assert gp.update(1.45, P, 0., [], _healthy(True)).entry_ok


def test_no_admission_when_not_admissible():
    plan = _plan()
    fl = Flows(plan, DEFAULT)
    fl.gate(_obs(plan, G.stage2_hold_s + 0.02, 1.0), "MUTEX_PASS", admissible=False)
    assert not fl.cr_cleared
    fl.gate(_obs(plan, G.stage2_hold_s + 0.02, 1.1), "MUTEX_PASS", admissible=True)
    assert fl.cr_cleared


def _frame(t):
    data = {"DVLSensor": np.zeros(7), "Compass": np.array([1.0, 0.0, 0.0]), "DepthSensor": np.array([-5.0]),
            "IMUSensor": np.zeros((4, 3))}
    for s in SECTORS:
        data[sonar_sensor_name(s)] = np.zeros(234)
    return SensorFrame(t, data, {k: t for k in data})


def _committed_line_pair(cfg=B2, reverse_slots=False):
    plans = SCENARIOS["line_parallel_mutex"](cfg).plans
    if reverse_slots:
        permutation = (1, 0, 2)
        slots = [plans[0].slots[k] for k in permutation]
        plans = [dataclasses.replace(p, slots=slots, slot_index=permutation.index(p.slot_index)) for p in plans]
    pair = [DroneController(p, cfg) for p in plans[:2]]
    for ctl in pair:
        ctl.perception.acoustic.set_identities({k: k for k in range(len(plans))})
        ctl.step(_frame(0.0), 0.1)
        ctl.ha.committed, ctl.ha.mode = True, Mode.MUTEX_PASS
    return pair


def _hear_gate_phase(receiver, sender, t, phase, gate_idx=0):
    _rx(receiver.perception.acoustic, receiver.perception.nav.state.p, t, sender.plan.slot_index,
        sender.perception.nav.state.p, gate_idx=gate_idx, phase=phase)


@pytest.mark.parametrize("reverse_slots", [False, True])
def test_simultaneous_commitment_concedes_to_the_left_then_requires_its_withdrawal(reverse_slots):
    first, second = _committed_line_pair(reverse_slots=reverse_slots)
    _hear_gate_phase(first, second, 1.0, pl.GP_COMMITTED)
    _hear_gate_phase(second, first, 1.0, pl.GP_COMMITTED)
    first.step(_frame(1.0), 0.1)
    second.step(_frame(1.0), 0.1)
    assert first.last_obs.gate.decision == second.last_obs.gate.decision == "PRIORITY"
    assert first.ha.committed and not first.flows.cr_cleared and not first.last_obs.gate.entry_ok
    assert second.ha.mode == Mode.MUTEX_APPROACH and not second.ha.committed
    assert second.perception.tx_phase == pl.GP_NONE and not second.flows.cr_cleared
    event = next(e for e in second.events if e["type"] == "COMMITMENT_CONCEDED")
    assert event["peer_slot"] == first.plan.slot_index

    t = 1.0 + A.admit_fresh_s + 1.0
    first.step(_frame(t), 0.1)
    assert not first.last_obs.gate.entry_ok and first.last_obs.gate.occ_busy
    _hear_gate_phase(first, second, t + 0.1, second.perception.tx_phase)
    first.step(_frame(t + 0.1), 0.1)
    assert first.last_obs.gate.entry_ok and first.last_obs.gate.occ_state == "EXITING"


@pytest.mark.parametrize("case", ["stale", "another_gate", "admitted_peer", "stage2", "admitted", "nav_invalid", "fault", "disabled"])
def test_concession_requires_current_competition_and_a_safe_unadmitted_state(case):
    from holo_fleet.config import communication_disabled

    first, second = _committed_line_pair(communication_disabled(B2) if case == "disabled" else B2)
    t = 1.0 + A.admit_fresh_s + 1.0 if case == "stale" else 1.0
    phase = pl.GP_ADMITTED if case == "admitted_peer" else pl.GP_COMMITTED
    _hear_gate_phase(second, first, 1.0, phase, gate_idx=int(case == "another_gate"))
    if case in ("stage2", "admitted"):
        g = second.plan.gates[0]
        p = np.asarray(g.from_gate_frame([G.stage2_hold_s, 0.0, 0.0]), float)
        second.perception.nav.state.p = p
        if case == "admitted":
            second.flows.cr_cleared = True
            second.passage.admit(0, g, p)
            second.perception.nav.state.p = second.plan.launch_position.copy()
    if case == "nav_invalid":
        second.perception.nav_bound.invalid = True
    frame = _frame(t)
    if case == "fault":
        del frame.data[sonar_sensor_name("FRONT")]
    second.step(frame, 0.1)
    assert second.ha.committed
    if case == "fault":
        assert second.ha.mode == Mode.FAILSAFE_HOLD_OR_RETREAT


def test_a_committed_drone_in_failsafe_is_never_admitted():
    """The latent defect: a committed drone runs the gate flow as MUTEX_PASS in every mode; in FAILSAFE (a stale sensor) its stage-2
    check would see no echo.  The controller passes admissible = MUTEX_PASS and sense_ok."""
    ctl = DroneController(_plan(), DEFAULT)
    ctl.step(_frame(0.0), 0.1)
    real_update = ctl.perception.update
    calls = []

    def stale(*a, **k):
        obs = real_update(*a, **k)
        g = ctl.plan.gates[0]
        obs.p = np.asarray(g.from_gate_frame([G.stage2_hold_s + 0.02, 0.0, 0.0]), float)
        obs.gate.s, obs.gate.l, obs.gate.dz, obs.gate.loc_ok, obs.gate.entry_ok = G.stage2_hold_s + 0.02, 0.0, 0.0, True, True
        obs.sense_ok = obs.ab.sense_ok = False
        return obs

    real_gate = ctl.flows.gate

    def spy(obs, mode, admissible=True):
        calls.append((mode, admissible))
        return real_gate(obs, mode, admissible=admissible)

    ctl.perception.update, ctl.flows.gate = stale, spy
    ctl.ha.committed = True
    for k in range(1, 6):
        ctl.step(_frame(0.1 * k), 0.1)
    assert ctl.ha.mode == Mode.FAILSAFE_HOLD_OR_RETREAT
    assert calls and all(m == "MUTEX_PASS" and not adm for m, adm in calls) and not ctl.flows.cr_cleared


def test_silence_never_excludes_nor_releases_a_claim_nor_discards_an_echo():
    """Rule D is a diagnostic: a claimant silent far beyond t_silent_gate_s, last heard far outside the exclusion zone, with another
    drone heard, keeps its claim, stays a possible occupant, and its corridor echo is never a phantom."""
    tr, gp = _observer(P, others=(1, 2))
    ae = gp.acoustic_exit
    ae.g = gp.plan.gates[0]
    _rx(tr, P, 1.0, 1, [-14.0, 0.0, -5.], phase=pl.GP_COMMITTED)
    t = 1.0 + A.t_silent_gate_s + 30.0
    for tt in np.arange(2.0, t, 1.0):
        _rx(tr, P, float(tt), 2, [-20.0, 6.0, -5.], phase=pl.GP_NONE)
    assert ae.dead(1, t)
    assert ae.claims(t) == {1} and not ae.status_excluded(1, t) and not ae.status_excluded(1, t, fresh_only=True)
    assert not ae.no_fleet_drone(t, ae.g, P, 0.0, [3.0])
    o = gp.update(t, P, 0., [], _healthy(True))
    assert o.occ_busy and not o.entry_ok and o.dead == [] and o.silent == [1]


def test_claims_and_silence_only_delay_an_admission():
    """Monotonicity on the deployed class (randomized): whatever drone 1 reported last, its silence afterwards (while drone 2 is heard)
    never releases its claim and never shortens the admission wait (it lengthens it, except after a verified passage of this gate,
    which excludes the drone whatever its age); a received claim always holds the entry."""
    rng = np.random.default_rng(7)
    for _ in range(40):
        tr, gp = _observer(P, others=(1, 2))
        ae = gp.acoustic_exit
        ae.g = gp.plan.gates[0]
        ph = int(rng.choice([pl.GP_NONE, pl.GP_COMMITTED, pl.GP_ADMITTED, pl.GP_PASSED]))
        _rx(tr, P, 1.0, 1, [rng.uniform(-30, -8), rng.uniform(-6, 6), -5.], phase=ph)
        t_end = 1.0 + float(rng.uniform(0.5, 400.0))
        for tt in np.arange(1.5, t_end, 1.0):
            _rx(tr, P, float(tt), 2, [-20.0, 6.0, -5.], phase=pl.GP_NONE)
        o = gp.update(t_end, P, 0., [], _healthy(True))
        if ph in (pl.GP_COMMITTED, pl.GP_ADMITTED):
            assert ae.claims(t_end) == {1} and not o.entry_ok and o.occ_busy
        if ph == pl.GP_PASSED:
            # a verified passage of this gate excludes drone 1 for good (B2C-Ep): its age changes nothing
            assert o.comm_fresh and o.admit_wait_s == pytest.approx(A.commit_admit_min_s)
        elif t_end - 1.0 > A.admit_fresh_s + 0.1:
            assert not o.comm_fresh and o.admit_wait_s == pytest.approx(A.commit_admit_min_s + G.t_cross_max)
        _rx(tr, P, t_end + 0.2, 1, [-8.0, 0.0, -5.], phase=pl.GP_COMMITTED)
        o2 = gp.update(t_end + 0.3, P, 0., [], _healthy(True))
        assert not o2.entry_ok and o2.occ_busy


def test_other_regions_never_leave_out_a_drone_for_its_silence():
    """The map-aiding premise (Perception._other_regions): a drone not heard for far longer than t_silent_gate_s while another
    drone is heard is unknown (None), not left out; the former rule D left it out and an echo of it could be taken for a bar."""
    from holo_fleet.perception.perception import Perception
    from test_status_payload import _plan, _rx_meas

    plan = _plan()
    per = Perception(plan, B2)
    per.acoustic.set_identities({j: j for j in range(4)})
    p = np.array([10.0, 2.0, -5.0])
    rel = {k: np.array([4.0 * k, 0.0, 0.0]) for k in (1, 2, 3)}
    for k in (1, 2, 3):
        _rx_meas(per, p, 1.0, k, rel[k], p + rel[k])
    assert per._other_regions(1.1) is not None
    t = 1.0 + B2.acoustic.t_silent_gate_s + 30.0
    _rx_meas(per, p, t - 0.5, 3, rel[3], p + rel[3])
    assert per.acoustic.silent(1, t, B2.acoustic.t_silent_gate_s) and per.acoustic.silent(2, t, B2.acoustic.t_silent_gate_s)
    assert per._other_regions(t) is None


def test_a_drone_that_reported_passing_this_gate_does_not_force_the_degraded_wait():
    """all_fresh: a drone whose latest report is GP_PASSED for this gate is excluded for good (as in status_excluded, formal
    Pi4): its stale report does not switch the admission to the degraded wait."""
    tr, gp = _observer(P, others=(1, 2))
    _rx(tr, P, 1.0, 1, [6.0, 0.0, -5.], phase=pl.GP_PASSED)
    _rx(tr, P, 1.5, 2, [-5.4, 3.5, -5.], phase=pl.GP_NONE)
    o = gp.update(1.6, P, 0., [], {})
    assert o.comm_fresh and o.admit_wait_s == pytest.approx(A.commit_admit_min_s)
    _rx(tr, P, 1.0 + A.admit_fresh_s + 0.5, 2, [-5.4, 3.5, -5.], phase=pl.GP_NONE)
    o = gp.update(1.0 + A.admit_fresh_s + 0.6, P, 0., [], {})
    assert o.comm_fresh and o.admit_wait_s == pytest.approx(A.commit_admit_min_s)


def test_a_stale_report_of_another_kind_still_forces_the_degraded_wait():
    tr, gp = _observer(P, others=(1, 2))
    _rx(tr, P, 1.0, 1, [-3.0, 0.0, -5.], phase=pl.GP_COMMITTED)
    _rx(tr, P, 1.0 + A.admit_fresh_s + 0.5, 2, [-5.4, 3.5, -5.], phase=pl.GP_NONE)
    o = gp.update(1.0 + A.admit_fresh_s + 0.6, P, 0., [], {})
    assert not o.comm_fresh and o.admit_wait_s == pytest.approx(A.commit_admit_min_s + G.t_cross_max)


def test_a_passed_report_of_another_gate_index_is_no_exemption():
    tr, gp = _observer(P, others=(1,))
    _rx(tr, P, 1.0, 1, [6.0, 0.0, -5.], gate_idx=1, phase=pl.GP_PASSED)
    o = gp.update(1.0 + A.admit_fresh_s + 0.6, P, 0., [], {})
    assert not o.comm_fresh



def test_b2_with_every_other_drone_fresh_needs_no_timed_bound_and_b0_always_does():
    """The timed admission bound (formal ST1) is the B0 argument; B2 with fresh reports of every other drone rests on B2C-E1."""
    tr, gp = _observer(P, others=(1,))
    _rx(tr, P, 1.0, 1, [-5.4, -3.5, -5.], phase=pl.GP_NONE)
    assert not gp.update(1.1, P, 0., [], {}).timed_admission
    assert gp.update(1.0 + A.admit_fresh_s + 0.5, P, 0., [], {}).timed_admission
    from holo_fleet.perception.gate_perception import GatePerception

    plan = _plan()
    b0 = GatePerception(plan, DEFAULT).update(1.0, np.asarray(plan.gates[0].from_gate_frame([-2.5, 0., 0.]), float), 0., [], {})
    assert b0.timed_admission
