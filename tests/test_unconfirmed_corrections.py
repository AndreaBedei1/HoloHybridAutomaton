"""A correction of the estimate accepted without a validated localization is a hypothesis.  It stands until a fit puts the hull inside the opening; a hold that ends with no valid localization undoes it
(med9 lost_drone_mutex s995: two fits mirrored about the gate axis moved the estimate 2.2 m the wrong way, beyond the reach of any later fit, and the drone looped at the gate for the rest of the mission)."""

import math

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.control.controller import DroneController
from holo_fleet.ha.spec import Mode
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.perception.sonar_processing import EchoClassifier
from holo_fleet.perception.structure_aiding import StructureAiding
from holo_fleet.sim.scenarios import SCENARIOS

from test_structure_aiding import BARS, GATE, _R, _axial_run, _profiles, _world


def _aid():
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    return clf, StructureAiding(clf, DEFAULT, gates=[GATE], acq_min=6.0)


def test_a_correction_without_a_validated_fit_can_be_undone():
    clf, aid = _aid()
    p_est = _world(-3.3, 0.0)
    p = _axial_run(aid, clf, _world(-2.7, 0.0), p_est, _R(), others=[])
    assert not np.allclose(p, p_est) and aid.loc_t < 0.0
    c = aid.take_unconfirmed()
    assert np.allclose(p + c, p_est, atol=1e-9)
    assert aid.take_unconfirmed() is None and aid.axial_total == 0.0


def test_a_validated_fit_confirms_the_corrections_before_it():
    clf, aid = _aid()
    p = _world(-3.3, 0.0)
    p = _axial_run(aid, clf, _world(-2.7, 0.0), p, _R(), others=[])
    clf, aid = _aid()
    p = _axial_run(aid, clf, _world(-2.7, 0.0), _world(-3.3, 0.0), _R(), others=[])
    assert not np.allclose(aid.unconfirmed, 0.0)
    R = _R()
    p_true = _world(-1.6, 0.05)                        # in the opening, the side sectors see the bars: a validated fit
    est = p_true + np.array([0.0, 0.05, 0.0])
    for k in range(12):
        corr = aid.update(est, R, _profiles(clf, p_true, R), {s: 0.0 for s in sg.SECTORS}, 0.25, t=float(k))
        if corr is not None:
            est = est + corr
    assert aid.valid(11.0) and aid.take_unconfirmed() is None


def _controller():
    return DroneController(SCENARIOS["lost_drone_mutex"](DEFAULT).plans[2], DEFAULT)


def test_a_hold_that_ends_without_localization_undoes_the_unconfirmed_corrections():
    ctl = _controller()
    ctl.perception.aiding.unconfirmed = np.array([0.25, -2.2])
    p0 = ctl.perception.nav.state.p.copy()
    b0 = ctl.perception.nav_bound.value
    ctl.flows.events.append({"t": 100.0, "type": "ENTRY_HOLD_TIMEOUT", "loc_ok": False, "entry_ok": True})
    ctl._drain_flow_events(Mode.MUTEX_PASS)
    assert np.allclose(ctl.perception.nav.state.p - p0, [-0.25, 2.2, 0.0])
    assert ctl.perception.nav_bound.value >= b0 + math.hypot(0.25, 2.2) - 1e-9
    assert ctl.perception.aiding.take_unconfirmed() is None
    assert [e["type"] for e in ctl.events] == ["ENTRY_HOLD_TIMEOUT", "UNCONFIRMED_CORRECTIONS_UNDONE"]
    assert not ctl.flows.events


def test_a_hold_that_ends_with_a_valid_localization_undoes_nothing():
    ctl = _controller()
    ctl.perception.aiding.unconfirmed = np.array([0.25, -2.2])
    p0 = ctl.perception.nav.state.p.copy()
    ctl.flows.events.append({"t": 100.0, "type": "ENTRY_HOLD_TIMEOUT", "loc_ok": True, "entry_ok": False})
    ctl._drain_flow_events(Mode.MUTEX_PASS)
    assert np.allclose(ctl.perception.nav.state.p, p0)
    assert [e["type"] for e in ctl.events] == ["ENTRY_HOLD_TIMEOUT"]
