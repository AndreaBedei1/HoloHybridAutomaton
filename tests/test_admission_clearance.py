"""Near-field clearance of the admission, the admission zone, the give-up of the hold and the queue standoff that leaves
the clearance to the lead drone (without them no B0 lead drone is admitted, or the hold is never given up)."""

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.control.flows import ACQ_OFFSETS, QUEUE_BACKOFF_MAX_M, RELOC_ZONE_M, Flows
from holo_fleet.mission import queue_line
from holo_fleet.sim.scenarios import SCENARIOS

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "formal"))
from check_stale import zone_distance  # noqa: E402

G = DEFAULT.gate


def test_the_acquisition_offsets_and_the_zone_constants_agree():
    assert RELOC_ZONE_M == G.admit_zone_m
    assert sorted(dz for _dl, dz in ACQ_OFFSETS) == [-G.acq_dz_m, G.acq_dz_m]
    assert all(G.in_admission_zone(G.stage2_hold_s, dl, dz) for dl, dz in ACQ_OFFSETS)
    assert G.in_admission_zone(G.stage2_hold_s, 0.0, 0.0)


def test_the_clearance_exceeds_the_zone_distance_under_premise_z_and_not_under_nav_err_alone():
    """ST6 under premise Z; with the committed navigation premise alone the zone distance exceeds the clearance (ST6l)."""
    d = zone_distance(DEFAULT, G.zone_err_h_m, G.zone_err_h_m, G.zone_err_v_m)
    assert d < G.admit_clear_m < G.queue_standoff_m
    worst = float(np.sqrt((G.admit_zone_m + G.admit_zone_fwd_m + 2 * G.nav_err_m) ** 2
                          + (2 * G.admit_zone_lat_m + 2 * G.nav_err_m) ** 2
                          + (2 * (G.acq_dz_m + G.admit_zone_dz_m) + 2 * DEFAULT.acoustic.nav_depth_err_m) ** 2))
    assert worst > G.admit_clear_m


def test_the_queue_stays_abreast_at_the_line():
    plans = SCENARIOS["line_parallel_mutex"](DEFAULT).plans
    assert {round(p.queue_s, 6) for p in plans} == {round(queue_line(plans[0].slots, G), 6)}


def _qobs(plan, t, s, d_front):
    g = plan.gates[0]
    gate = SimpleNamespace(s=s, l=plan.queue_lateral, dz=plan.queue_dz, queue_s=plan.queue_s, queue_l=plan.queue_lateral,
                           queue_dz=plan.queue_dz)
    tg = [SimpleNamespace(cls="DYNAMIC", pattern=frozenset({"FRONT"}), d_lower=d_front)] if d_front is not None else []
    return SimpleNamespace(t=t, p=np.asarray(g.from_gate_frame([s, plan.queue_lateral, plan.queue_dz]), float), gate=gate,
                           gate_index=0, targets=tg)


def test_a_queued_drone_backs_off_from_a_hull_ahead_and_returns_when_it_moves_on():
    """The queue standoff (a follower 3.2 m behind the hold would keep the lead from its clearance): a FRONT hull nearer than
    queue_standoff_m moves the target back along the axis, the hysteresis band holds it, beyond the band the queue point."""
    plan = SCENARIOS["line_parallel_mutex"](DEFAULT).plans[1]
    assert abs(plan.queue_lateral) < 1e-9
    fl = Flows(plan, DEFAULT)
    qs = plan.queue_s
    fl.gate(_qobs(plan, 0.0, qs, 2.9), "MUTEX_YIELD")
    assert fl.queue_backoff_s is not None and fl.queue_backoff_s < qs - (G.queue_standoff_m - 2.9) + 1e-9
    held = fl.queue_backoff_s
    fl.gate(_qobs(plan, 0.1, qs - 0.9, G.queue_standoff_m + 0.1), "MUTEX_APPROACH")
    assert fl.queue_backoff_s == held
    fl.gate(_qobs(plan, 0.2, qs - 0.9, G.queue_standoff_m + 0.4), "MUTEX_APPROACH")
    assert fl.queue_backoff_s is None
    fl.gate(_qobs(plan, 0.3, qs, None), "MUTEX_YIELD")
    assert fl.queue_backoff_s is None
    fl.gate(_qobs(plan, 0.4, qs, 0.5), "MUTEX_YIELD")
    assert fl.queue_backoff_s >= qs - QUEUE_BACKOFF_MAX_M - 1e-9


def _obs(plan, t, s=None, l=0.0, dz=0.0, timed=True, entry_after=None, admit_clear=True, comm=False):
    g = plan.gates[0]
    s = G.stage2_hold_s if s is None else s
    gate = SimpleNamespace(s=s, l=l, dz=dz, queue_s=plan.queue_s, queue_l=plan.queue_lateral, queue_dz=plan.queue_dz,
                           loc_ok=True, entry_ok=True, admit_wait_s=0.0,
                           entry_after=(-1e9 + G.t_admit_stale) if entry_after is None else entry_after,
                           timed_admission=timed and not comm, admit_clear=admit_clear)
    return SimpleNamespace(t=t, p=np.asarray(g.from_gate_frame([s, l, dz]), float), gate=gate, gate_index=0, targets=[])


def _plan():
    return SCENARIOS["line_parallel_mutex"](DEFAULT).plans[0]


@pytest.mark.parametrize("l,dz", [(G.admit_zone_lat_m + 0.01, 0.0), (0.0, G.acq_dz_m + G.admit_zone_dz_m + 0.02)])
def test_a_timed_drone_held_just_outside_the_zone_gives_up(l, dz):
    """Out of the zone the dwell term is t + T, so a give-up anchored on it would never fire (formal ST3o)."""
    plan = _plan()
    fl = Flows(plan, DEFAULT)
    t = 0.0
    while t < G.t_admit_stale + G.t_occ_max + G.t_clear + 5.0 and not fl.hold_timeout:
        fl.gate(_obs(plan, t, l=l, dz=dz), "MUTEX_PASS")
        assert not fl.cr_cleared
        t += 0.5
    assert fl.hold_timeout and t <= G.t_admit_stale + G.t_occ_max + G.t_clear + 1.0


def test_a_timed_drone_in_the_zone_is_admitted_before_its_give_up():
    plan = _plan()
    fl = Flows(plan, DEFAULT)
    t = 0.0
    while not fl.cr_cleared:
        fl.gate(_obs(plan, t), "MUTEX_PASS")
        assert not fl.hold_timeout
        t += 0.5
    assert G.t_admit_stale < t <= G.t_admit_stale + 1.0


def test_no_admission_without_the_clearance_on_every_path_and_the_hold_is_given_up():
    """Every admission needs the clearance: timed, B2 with fresh reports of every other drone (comm_clear: no timed bound)
    and an exit witness (no timed bound); a clearance that never comes ends in the give-up, not in an endless hold."""
    plan = _plan()
    for timed, comm in ((True, False), (False, True), (False, False)):
        fl = Flows(plan, DEFAULT)
        t = 0.0
        while t < 120.0 and not fl.hold_timeout:
            fl.gate(_obs(plan, t, timed=timed, comm=comm, admit_clear=False), "MUTEX_PASS")
            assert not fl.cr_cleared
            t += 0.5
        assert fl.hold_timeout, (timed, comm)
        fl2 = Flows(plan, DEFAULT)
        fl2.gate(_obs(plan, 0.0, timed=timed, comm=comm), "MUTEX_PASS")
        fl2.gate(_obs(plan, G.t_admit_stale + 0.6, timed=timed, comm=comm), "MUTEX_PASS")
        assert fl2.cr_cleared, (timed, comm)


def test_a_committed_follower_in_standoff_stays_beyond_the_admission_clearance():
    """The standoff limit is above admit_clear_m, so a follower holding in standoff does not keep the
    drone ahead from its clearance."""
    plan = _plan()
    fl = Flows(plan, DEFAULT)
    near = SimpleNamespace(cls="DYNAMIC", pattern=frozenset({"FRONT"}), d_lower=G.admit_clear_m + 0.1)
    far = SimpleNamespace(cls="DYNAMIC", pattern=frozenset({"FRONT"}), d_lower=G.admit_clear_m + 0.6)
    assert fl._ahead(SimpleNamespace(targets=[near])) and not fl._ahead(SimpleNamespace(targets=[far]))


def test_a_side_column_never_stands_off_and_the_middle_one_does_from_a_diagonal_hull():
    """The side columns see the middle queued drone in FRONT+LEFT / FRONT+RIGHT within queue_standoff_m and, standing
    off, would hold 2.5 m behind their queue points for the whole run; only a hull directly ahead counts."""
    plan = SCENARIOS["line_parallel_mutex"](DEFAULT).plans[0]
    fl = Flows(plan, DEFAULT)
    g = plan.gates[0]
    qs = plan.queue_s
    for pat in ({"FRONT", "RIGHT"}, {"FRONT", "LEFT"}):
        gate = SimpleNamespace(s=qs - 2.0, l=plan.queue_lateral, dz=plan.queue_dz, queue_s=qs, queue_l=plan.queue_lateral,
                               queue_dz=plan.queue_dz)
        tg = [SimpleNamespace(cls="DYNAMIC", pattern=frozenset(pat), d_lower=3.5)]
        o = SimpleNamespace(t=0.0, p=np.asarray(g.from_gate_frame([qs - 2.0, plan.queue_lateral, plan.queue_dz]), float),
                            gate=gate, gate_index=0, targets=tg)
        fl.gate(o, "MUTEX_APPROACH")
        assert fl.queue_backoff_s is None, pat
    mid = SCENARIOS["line_parallel_mutex"](DEFAULT).plans[1]
    assert abs(mid.queue_lateral) < 1e-9
    fm = Flows(mid, DEFAULT)
    gate = SimpleNamespace(s=mid.queue_s, l=0.0, dz=0.0, queue_s=mid.queue_s, queue_l=0.0, queue_dz=mid.queue_dz)
    tg = [SimpleNamespace(cls="DYNAMIC", pattern=frozenset({"FRONT", "RIGHT"}), d_lower=2.9)]
    fm.gate(SimpleNamespace(t=0.0, p=np.asarray(g.from_gate_frame([mid.queue_s, 0.0, 0.0]), float), gate=gate,
                            gate_index=0, targets=tg), "MUTEX_YIELD")
    assert fm.queue_backoff_s is not None and fm.queue_backoff_s >= mid.queue_s - QUEUE_BACKOFF_MAX_M - 1e-9
