"""A correction of the estimate accepted without a validated localization is a hypothesis.  It stands until a fit puts the hull inside the opening; a hold that ends with no valid localization undoes it
(med9 lost_drone_mutex s995: two fits moved the estimate 2.2 m the wrong way, beyond the reach of any later fit, and the drone looped at the gate for the rest of the mission).  The step waits for the give-up
edge: while committed, a jump of the estimate could feed the position-gated transitions of the hold (formal review, N13)."""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.control.controller import DroneController
from holo_fleet.control.flows import Flows
from holo_fleet.ha.spec import Mode
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.perception.sonar_processing import EchoClassifier
from holo_fleet.perception.structure_aiding import IDENTITY_STEP_M, StructureAiding
from holo_fleet.sim.scenarios import SCENARIOS

from test_reloc_hold import _obs
from test_structure_aiding import BARS, GATE, _R, _axial_run, _profiles, _world


def _aid():
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    return clf, StructureAiding(clf, DEFAULT, gates=[GATE], acq_min=6.0)


def _update(aid, clf, p_true, p, R, t, others):
    return aid.update(p, R, _profiles(clf, p_true, R), {s: 0.0 for s in sg.SECTORS}, 0.25, t=t, others=others)


def test_an_along_axis_correction_without_a_validated_fit_can_be_undone():
    clf, aid = _aid()
    p_est = _world(-3.3, 0.0)
    p = _axial_run(aid, clf, _world(-2.7, 0.0), p_est, _R(), others=[])
    assert not np.allclose(p, p_est) and aid.loc_t < 0.0
    c = aid.take_unconfirmed()
    assert np.allclose(p + c, p_est, atol=1e-9)
    assert aid.take_unconfirmed() is None and aid.axial_total == 0.0
    assert not aid.recent and not aid.recent_ax


def test_a_correction_under_the_identity_rule_can_be_undone_also_when_it_went_the_wrong_way():
    # no noise at all: beside the bar the fit lands on the far side of it, the estimate moves away from the truth
    clf, aid = _aid()
    R, p_true, p_est = _R(), _world(-1.6, 1.8), _world(-1.6, 1.4)
    p, applied = p_est.copy(), np.zeros(3)
    for k in range(12):
        c = _update(aid, clf, p_true, p, R, float(k), [])
        if c is not None:
            p, applied = p + c, applied + c
    assert np.linalg.norm(applied) > 0.3 and aid.loc_t < 0.0 and aid.last["identified"]
    assert np.allclose(aid.unconfirmed, applied[:2])
    assert abs(GATE.to_gate_frame(p)[1] - 1.8) > abs(GATE.to_gate_frame(p_est)[1] - 1.8)          # it did go the wrong way
    assert np.allclose(p + aid.take_unconfirmed(), p_est, atol=1e-9)


def _validated(aid, clf, offset):
    R, p_true = _R(), _world(-1.6, 0.05)                    # in the opening, the side sectors see the bars
    est = p_true + np.array([0.0, offset, 0.0])
    applied = False
    for k in range(12):
        c = _update(aid, clf, p_true, est, R, float(k), None)
        if c is not None:
            est, applied = est + c, True
    return applied


def test_a_validated_fit_in_the_consistency_band_confirms_the_corrections_before_it():
    clf, aid = _aid()
    aid.unconfirmed = np.array([0.1, -0.7])
    assert not _validated(aid, clf, 0.05)
    assert aid.valid(11.0) and aid.take_unconfirmed() is None


def test_a_validated_correction_confirms_the_corrections_before_it():
    clf, aid = _aid()
    aid.unconfirmed = np.array([0.1, -0.7])
    assert _validated(aid, clf, 0.45)
    assert aid.valid(11.0) and aid.take_unconfirmed() is None


def _controller():
    return DroneController(SCENARIOS["lost_drone_mutex"](DEFAULT).plans[2], DEFAULT)


def _timeout(ctl, loc_ok=False, entry_ok=True):
    ctl.flows.events.append({"t": 100.0, "type": "ENTRY_HOLD_TIMEOUT", "loc_ok": loc_ok, "entry_ok": entry_ok})


def test_the_undo_waits_for_the_give_up_edge_and_then_moves_the_estimate_and_the_stored_fixes():
    ctl = _controller()
    shifted = []
    ctl.perception.acoustic.shift = shifted.append
    ctl.perception.nav_bound.aid = 5.0                      # a bound the step fits in: its growth is measurable
    ctl.perception.aiding.unconfirmed = np.array([0.25, -2.2])
    p0 = ctl.perception.nav.state.p.copy()
    b0 = ctl.perception.nav_bound.horizontal
    ctl.ha.committed = True
    ctl.was_committed = True
    _timeout(ctl)
    ctl._drain_flow_events(100.0, Mode.MUTEX_PASS)
    assert np.allclose(ctl.perception.nav.state.p, p0) and not shifted and ctl.undo_pending      # still committed: nothing moves
    ctl._drain_flow_events(100.1, Mode.MUTEX_PASS)
    assert np.allclose(ctl.perception.nav.state.p, p0)
    ctl.ha.committed = False
    ctl._drain_flow_events(101.0, Mode.MUTEX_APPROACH)
    step = np.array([-0.25, 2.2, 0.0])
    assert np.allclose(ctl.perception.nav.state.p - p0, step) and not ctl.undo_pending
    assert len(shifted) == 1 and np.allclose(shifted[0], step)
    assert abs(ctl.perception.nav_bound.horizontal - (b0 + float(np.hypot(0.25, 2.2)))) < 1e-9
    assert not ctl.perception.nav_bound.invalid
    assert ctl.perception.aiding.take_unconfirmed() is None
    assert [e["type"] for e in ctl.events] == ["ENTRY_HOLD_TIMEOUT", "UNCONFIRMED_CORRECTIONS_UNDONE"]
    assert ctl.events[1]["t"] == 101.0 and not ctl.flows.events


def test_a_hold_that_ended_blocked_and_without_localization_also_undoes():
    ctl = _controller()
    ctl.perception.aiding.unconfirmed = np.array([0.25, -0.5])
    p0 = ctl.perception.nav.state.p.copy()
    _timeout(ctl, loc_ok=False, entry_ok=False)
    ctl._drain_flow_events(100.0, Mode.MUTEX_APPROACH)
    assert np.allclose(ctl.perception.nav.state.p - p0, [-0.25, 0.5, 0.0])


def test_a_hold_that_ends_with_a_valid_localization_undoes_nothing():
    ctl = _controller()
    ctl.perception.aiding.unconfirmed = np.array([0.25, -2.2])
    p0 = ctl.perception.nav.state.p.copy()
    _timeout(ctl, loc_ok=True, entry_ok=False)
    ctl._drain_flow_events(100.0, Mode.MUTEX_APPROACH)
    assert np.allclose(ctl.perception.nav.state.p, p0) and not ctl.undo_pending
    assert [e["type"] for e in ctl.events] == ["ENTRY_HOLD_TIMEOUT"]


def test_after_the_give_up_a_jump_of_the_estimate_feeds_no_transition_of_the_hold():
    plan = SCENARIOS["lost_drone_mutex"](DEFAULT).plans[0]
    fl = Flows(plan, DEFAULT)
    for s in (0.34, -2.36):
        fl.gate(_obs(plan, s, False, 1.0), "MUTEX_APPROACH")
        assert not fl.past_centre and not fl.abort_request and not fl.cr_cleared and not fl.reloc_hold


def test_a_measured_crossing_ends_what_there_was_to_undo():
    ctl = _controller()
    gate = ctl.plan.gates[0]
    ctl.perception.aiding.unconfirmed = np.array([0.25, -0.5])
    ctl.undo_pending = True
    ctl.passage.crossing = (0.0, 0.0)
    ctl._apply_crossing(50.0, gate, gate.from_gate_frame([0.0, 0.2, 0.0]))
    assert ctl.perception.aiding.take_unconfirmed() is None and not ctl.undo_pending


def test_only_the_corrections_of_the_current_commitment_can_be_undone():
    ctl = _controller()
    ctl.perception.aiding.unconfirmed = np.array([0.4, 0.4])                # real drift corrected at the queue
    ctl.ha.committed = True
    ctl._drain_flow_events(10.0, Mode.MUTEX_PASS)
    assert np.allclose(ctl.perception.aiding.unconfirmed, 0.0)
    ctl.perception.aiding.unconfirmed = np.array([0.0, -0.5])               # within the commitment
    ctl._drain_flow_events(10.1, Mode.MUTEX_PASS)
    assert np.allclose(ctl.perception.aiding.unconfirmed, [0.0, -0.5])


def test_a_correction_under_the_identity_rule_is_capped_and_a_right_one_arrives_in_steps():
    clf, aid = _aid()
    R, p_true, p = _R(), _world(-1.3, 2.2), _world(-1.3, 1.3)
    e0, steps = float(np.linalg.norm(p[:2] - p_true[:2])), []
    for k in range(40):
        c = _update(aid, clf, p_true, p, R, float(k), [])
        if c is not None:
            steps.append(float(np.linalg.norm(c)))
            p = p + c
    assert steps and max(steps) <= IDENTITY_STEP_M + 1e-9 and len(steps) >= 2
    assert float(np.linalg.norm(p[:2] - p_true[:2])) < e0 - 0.5
