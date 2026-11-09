"""Lateral sonar consensus of the formation (control/flows.py): the lateral counterpart of the along-track progress
consensus (sigma).  A drone whose dead reckoning drifts sideways (a compass bias: about 1 m over the validation
missions at the MEDIUM level) tracks its slot in its own estimate and is really off the formation; the neighbours'
range residuals measure that relative displacement and the integrated lateral offset moves the slot reference to
remove it."""

import dataclasses
from types import SimpleNamespace

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.control.flows import Flows
from holo_fleet.formations import TEMPLATES
from holo_fleet.mission import FormationClock, MissionPlan, Path

FR = DEFAULT.form
DT = 0.1


def _plan(k=2):
    tm = TEMPLATES["square"]
    return MissionPlan(drone_id=f"drone_{k}", slot_index=k, slots=list(tm.slots),
                       path=Path(np.array([[0.0, 0.0], [200.0, 0.0]]), -5.0), gates=[], structures=[],
                       launch_position=np.zeros(3), launch_yaw_deg=0.0, static_rank=k, s_end=190.0,
                       clock=FormationClock(0.0, 0.3, 0.0, s_end=190.0))


def _slot(plan, t):
    return np.array([plan.clock.s(t) + plan.my_slot.along, plan.my_slot.lateral, plan.path.depth_z + plan.my_slot.dz])


def _obs(p, t, plan, fl, lateral_corr):
    return SimpleNamespace(t=t, p=np.asarray(p, float), slot_pos=_slot(plan, t) + fl.lat_offset, targets=[],
                           R_wb=np.eye(3), yaw=0.0, form=SimpleNamespace(along_corr=0.0, lateral_corr=lateral_corr, lateral_far=None))


def _fly(cfg, bias, t_end=120.0):
    """Closed loop: the true position is the estimate + ``bias`` (left normal); the neighbours hold their true
    slots, so the lateral residual is the true lateral displacement from the own slot, sign to correct."""
    plan = _plan()
    fl = Flows(plan, cfg)
    p = _slot(plan, 0.0)
    t = 0.0
    while t < t_end:
        true_l = p[1] + bias
        v, _ = fl.formation(_obs(p, t, plan, fl, plan.my_slot.lateral - true_l), DT)
        p = p + v * DT
        t += DT
    return fl, p[1] + bias - plan.my_slot.lateral


def test_a_lateral_drift_is_removed_relative_to_the_neighbours():
    for bias in (1.0, -1.2, 0.5):
        fl, dev = _fly(DEFAULT, bias)
        assert abs(dev) < FR.e_ok * 0.5, (bias, dev)               # well inside the recovery tolerance
        assert abs(fl.lam) <= FR.progress_max


def test_without_the_lateral_consensus_the_drift_stays_beyond_the_tolerance():
    cfg = dataclasses.replace(DEFAULT, form=dataclasses.replace(FR, k_lateral=0.0))
    _fl, dev = _fly(cfg, 1.0)
    assert abs(dev) > FR.e_ok


def test_the_lateral_consensus_runs_only_near_the_own_slot_and_is_bounded():
    plan = _plan()
    fl = Flows(plan, DEFAULT)
    far = _obs(_slot(plan, 20.0) + [-3.0, 0.0, 0.0], 20.0, plan, fl, 0.9)       # 3 m behind its slot
    for _ in range(50):
        fl.formation(far, DT, recovering=True)
    assert fl.lam == 0.0 and not fl.lat_offset.any()
    for _ in range(3000):                                                          # near: integrates, capped
        fl.formation(_obs(_slot(plan, 20.0), 20.0, plan, fl, 0.9), DT)
    assert 0.0 < fl.lam <= FR.progress_max
    assert fl.lat_offset[1] > 0.0 and abs(fl.lat_offset[0]) < 1e-9 and fl.lat_offset[2] == 0.0   # left normal
