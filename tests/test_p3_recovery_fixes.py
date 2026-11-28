"""P3 recovery (formal check_formation_coupled.py): the keeping-mode catch-up authority, the operating-domain precondition
of the clock speed, the re-synchronisation of the private formation times off the slot and at the rendezvous hold,
and the time base of the reported-position comparison.  Closed loops on the real Flows / FormationTime classes."""

import dataclasses
import math
from types import SimpleNamespace

import numpy as np
import pytest

from holo_fleet.config import DEFAULT, BeaconConfig
from holo_fleet.control.flows import Flows
from holo_fleet.control.formation_time import FormationTime
from holo_fleet.formations import TEMPLATES
from holo_fleet.mission import FormationClock, MissionPlan, Path, Slot

FR = DEFAULT.form
DT = 0.1
B2 = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_RB"))


def _plan(v, k=1, template="triangle", slots=None, clock=None):
    sl = list(slots) if slots is not None else list(TEMPLATES[template].slots)
    return MissionPlan(drone_id=f"drone_{k}", slot_index=k, slots=sl,
                       path=Path(np.array([[0.0, 0.0], [3000.0, 0.0]]), -5.0), gates=[], structures=[],
                       launch_position=np.zeros(3), launch_yaw_deg=0.0, static_rank=k, s_end=2990.0,
                       clock=clock or FormationClock(0.0, v, 0.0, s_end=2990.0))


def _slot(pl, fl, t):
    me = pl.my_slot
    return np.array([pl.clock.s(fl.ftime.now(t)) + fl.sigma + me.along, me.lateral + fl.lam, pl.path.depth_z + me.dz])


def _obs(t, p, slot, along=0.0, lateral=0.0):
    return SimpleNamespace(t=t, p=p.copy(), slot_pos=slot, targets=[], R_wb=np.eye(3), yaw=0.0,
                           form=SimpleNamespace(along_corr=along, lateral_corr=lateral, lateral_far=None))


def _follow(v_plan, e0, w_head=0.0, t_end=200.0):
    """One drone in a keeping mode (the automaton's hysteresis: RECOVERY above e_lost until e_ok for t_ok_hold),
    ``e0`` behind its slot, a head disturbance ``w_head``; returns the slot errors and the modes."""
    pl = _plan(v_plan)
    fl = Flows(pl, DEFAULT)
    p = _slot(pl, fl, 0.0) - np.array([e0, 0.0, 0.0])
    t, mode, ok, out = 0.0, "FOLLOW", None, []
    while t < t_end:
        sl = _slot(pl, fl, t)
        err = float(np.linalg.norm(sl - p))
        if mode == "FOLLOW" and err > FR.e_lost:
            mode = "RECOVERY"
        if mode == "RECOVERY":
            ok = (t if ok is None else ok) if err < FR.e_ok else None
            if ok is not None and t - ok >= FR.t_ok_hold:
                mode, ok = "FOLLOW", None
        v, _ = fl.formation(_obs(t, p, sl), DT, recovering=mode == "RECOVERY")
        p = p + (v - np.array([w_head, 0.0, 0.0])) * DT
        t = round(t + DT, 6)
        out.append((t, err, mode))
    return out


def test_the_keeping_cap_leaves_a_catch_up_authority_above_the_plan_speed():
    assert FR.keeping_cap(FR.v_nominal) == pytest.approx(FR.v_slot_max)
    for v in (0.0, 0.25, 0.30, 0.35, 0.40):
        cap = FR.keeping_cap(v)
        assert cap >= v + FR.v_catch_min - 1e-12 and cap <= FR.v_recovery_max <= DEFAULT.env.v_max_nominal


@pytest.mark.parametrize("w_head", [0.0, 0.02])
def test_a_drone_behind_its_slot_in_a_keeping_mode_catches_up_at_the_fastest_admitted_clock(w_head):
    """integrated_short runs the clock at 0.40 m/s: with the constant keeping cap 0.40 (v_slot_max = v_plan) the drone 0.8 m behind its slot stays
    0.8 m behind for ever (no head disturbance) or cycles FOLLOW / RECOVERY between 0.34 and 1.2 m (0.02 m/s)."""
    late = [(e, m) for t, e, m in _follow(0.40, 0.8, w_head) if t > 150.0]
    assert max(e for e, _m in late) < FR.e_ok
    assert {m for _e, m in late} == {"FOLLOW"}


def test_a_clock_faster_than_the_operating_domain_is_refused_at_plan_time():
    Flows(_plan(FR.v_recovery_max - FR.v_catch_min), DEFAULT)
    with pytest.raises(ValueError, match="operating domain"):
        Flows(_plan(FR.v_recovery_max - FR.v_catch_min + 0.01), DEFAULT)


ABREAST = (Slot(0.0, 1.75, 0.0), Slot(0.0, -1.75, 0.0))


def _pair(dtau, with_rep, gate=True, v=0.30, t0=100.0, t_end=1000.0, age=0.5):
    """An abreast B2 pair 3.5 m apart, both on their own slots in FORMATION_RECOVERY, private delays differing by
    ``dtau``: the along offset of the references is v * dtau minus what the progress consensus sigma absorbs (+-2 m at
    most).  The lag of the drone behind is measured on its reported position (band e_ok + (cap - v_wait) age).  With
    ``gate`` the fix feeds sigma only while the neighbour is 'present' (within e_lost plus a fix uncertainty of 0.3 m,
    rule A-P), else the abreast sonar residual has no along part (along weight 0)."""
    pls = [_plan(v, k, slots=ABREAST) for k in range(2)]
    fls = [Flows(pl, B2) for pl in pls]
    fls[1].ftime.tau = dtau
    ps = [_slot(pls[k], fls[k], t0) for k in range(2)]
    modes, ok, t = ["RECOVERY", "RECOVERY"], [None, None], t0
    while t < t_end:
        sl = [_slot(pls[k], fls[k], t) for k in range(2)]
        vs, errs = [], []
        for i in range(2):
            j = 1 - i
            off = ps[j] - (sl[i] + np.array([0.0, ABREAST[j].lateral - ABREAST[i].lateral, 0.0]))
            band = FR.e_ok + (FR.keeping_cap(v) - FR.v_wait) * age
            lag = (-float(off[0]) - band) if float(off[0]) < -band else None
            form_err = max(float(np.linalg.norm(sl[i] - ps[i])), abs(float(np.linalg.norm(ps[j] - ps[i])) - 3.5))
            if modes[i] != "RECOVERY" and form_err > FR.e_lost:
                modes[i] = "RECOVERY"
            if modes[i] == "RECOVERY":
                ok[i] = (t if ok[i] is None else ok[i]) if form_err < FR.e_ok else None
                if ok[i] is not None and t - ok[i] >= FR.t_ok_hold:
                    modes[i], ok[i] = "FOLLOW", None
            rec = modes[i] == "RECOVERY"
            present = not gate or float(np.linalg.norm(off)) <= FR.e_lost + 0.3
            obs = _obs(t, ps[i], sl[i], along=float(off[0]) if present else 0.0, lateral=float(off[1]))
            v_i, _ = fls[i].formation(obs, DT, recovering=rec)
            fls[i].ftime.update(t, DT, incomplete=False, keeping=not rec, behind=lag,
                                behind_rep=lag if with_rep else None)
            vs.append(v_i)
            errs.append(form_err)
        ps = [ps[k] + vs[k] * DT for k in range(2)]
        t = round(t + DT, 6)
    return modes, errs, fls


@pytest.mark.parametrize("dtau,gate", [(8.0, True), (22.0, True), (22.0, False)])
def test_a_pair_with_different_delays_leaves_formation_recovery(dtau, gate):
    """8 s of delay difference at 0.30 m/s (one asymmetric reduced-progress episode, C3e: up to 10.8 s) with the fix
    feeding sigma only while the neighbour is present, or 22 s with sigma always fed (it saturates at +-2 m): the residual
    stays above e_ok, so without the re-synchronisation in FORMATION_RECOVERY both drones recover for ever (formal C2m);
    with the lag of the reported positions the drone ahead slows its reference and both return to FORMATION_FOLLOW."""
    modes, errs, fls = _pair(dtau, with_rep=False, gate=gate)
    assert modes == ["RECOVERY", "RECOVERY"] and min(errs) > FR.e_ok
    modes, errs, fls = _pair(dtau, with_rep=True, gate=gate)
    assert modes == ["FOLLOW", "FOLLOW"] and max(errs) < FR.e_ok
    assert fls[0].ftime.tau > 0.25 * dtau and fls[1].ftime.tau == pytest.approx(dtau)


def test_off_slot_resynchronisation_uses_reported_lag_without_relative_echo():
    ft = FormationTime(FormationClock(0.0, FR.v_nominal, 0.0), DEFAULT)
    for i in range(300):
        ft.update(10.0 + DT * i, DT, incomplete=False, keeping=False, behind=5.0, behind_rep=None)
    assert ft.sync == 0.0 and ft.tau == pytest.approx(0.0)
    for i in range(300):
        ft.update(40.0 + DT * i, DT, incomplete=False, keeping=False, behind=None, behind_rep=2.0)
    assert ft.sync == pytest.approx(min(FR.k_progress * 2.0, FR.v_nominal - FR.v_wait)) and ft.tau > 0.0


def test_keeping_clock_uses_fresh_report_when_relative_echo_is_unavailable():
    ft = FormationTime(FormationClock(0.0, FR.v_nominal, 0.0), DEFAULT)
    ft.episode_t = FR.t_rejoin
    ft.update(30.0, DT, incomplete=True, behind=None, behind_rep=2.0)
    assert ft.sync > 0.0 and ft.rate < 1.0
    ft.update(30.0 + DT, DT, incomplete=True, behind=None, behind_rep=None)
    assert ft.sync == 0.0


def _hold_clock():
    return FormationClock(0.0, 0.30, 0.0, holds=((6.0, 30.0),), s_end=200.0)


def test_the_clock_reports_the_end_of_the_planned_hold():
    c = _hold_clock()
    assert c.hold_end(10.0) is None and c.hold_end(19.99) is None
    assert c.hold_end(20.0) == pytest.approx(50.0) and c.hold_end(49.0) == pytest.approx(50.0)
    assert c.hold_end(51.0) is None
    end = FormationClock(0.0, 0.30, 0.0, s_end=3.0)
    assert end.hold_end(100.0) is None


def test_the_rendezvous_hold_absorbs_the_private_delays_without_moving_the_slot():
    """Two drones reach the hold with delays 4 s and 12 s (dwell 30 s): both leave it on the planned schedule, the slot
    staying at the hold point meanwhile; a delay longer than the dwell keeps the excess (formal C3h)."""
    fts = []
    for tau in (4.0, 12.0, 40.0):
        ft = FormationTime(_hold_clock(), DEFAULT)
        ft.tau = tau
        fts.append(ft)
    s_seen = {k: [] for k in range(3)}
    t = 0.0
    while t < 120.0:
        for k, ft in enumerate(fts):
            s_before = ft.clock.s(ft.now(t))
            ft.update(t, DT, incomplete=False)
            s_after = ft.clock.s(ft.now(t))
            assert s_after == pytest.approx(s_before, abs=1e-9)
            s_seen[k].append(s_after)
        t = round(t + DT, 6)
    assert fts[0].tau == pytest.approx(0.0) and fts[1].tau == pytest.approx(0.0)
    assert fts[2].tau == pytest.approx(10.0, abs=DT + 1e-9)
    assert fts[0].resync == pytest.approx(4.0) and fts[1].resync == pytest.approx(12.0)


def test_the_reported_position_is_extrapolated_on_the_own_formation_time():
    """Mission time past the end of the hold, own formation time still inside it (delay 5 s): the neighbours' slots
    do not move, so the report must not be moved by v * age (0.75 m at 2.5 s), as the mission-time base did."""
    from holo_fleet.perception.perception import Perception

    clock = _hold_clock()
    per = Perception(_plan(0.30, k=0, template="square", clock=clock), B2)
    t, age = 52.0, 2.5
    f = SimpleNamespace(rep_ok=True, t_tx=t - age, rep=SimpleNamespace(p=np.array([10.0, 0.0, -5.0])))
    t_hat = np.array([1.0, 0.0, 0.0])
    per.t_form = t - 5.0
    off, band = per._rep_offset(f, t, np.array([10.0, 0.0, -5.0]), t_hat)
    assert np.allclose(off, 0.0)
    assert band == pytest.approx(FR.e_ok + (FR.keeping_cap(0.30) - FR.v_wait) * age)
    per.t_form = None
    off_mission, _b = per._rep_offset(f, t, np.array([10.0, 0.0, -5.0]), t_hat)
    assert off_mission[0] == pytest.approx(0.30 * age)


def _rx_meas(per, p, t, k, rel, reported, bound=0.5):
    from holo_fleet.perception import payload as pl

    r = float(np.linalg.norm(rel))
    az, el = math.atan2(rel[1], rel[0]), math.asin(rel[2] / r)
    t_arr = t - 0.05
    fr = pl.frame_index(t_arr - r / B2.beacon.sound_speed, B2.beacon.period_s)
    per.acoustic.update(t, [(k, t_arr, r, az, el, pl.encode(fr, reported, bound, 0, pl.GP_NONE))], p, 0., 0.5)


def test_the_lag_of_a_reported_position_is_kept_apart_from_the_relative_fix_lag():
    from holo_fleet.perception.formation_perception import expected_relative
    from holo_fleet.perception.perception import Perception

    plan = _plan(0.30, k=0, template="square")
    per = Perception(plan, B2)
    per.acoustic.set_identities({j: j for j in range(4)})
    p = np.array([10.0, 2.0, -5.0])
    d = expected_relative(plan, np.eye(3))
    back = np.array([-2.5, 0.0, 0.0])
    for t in (5.0, 6.0):
        _rx_meas(per, p, t, 1, d[1] + back, p + d[1] + back)
    per.acoustic_neighbours(6.05, np.zeros(3), np.eye(3), p)
    assert 1 in per.acoustic_lag_rep and per.acoustic_lag_rep[1] == pytest.approx(per.acoustic_lag[1])
    assert 0.0 < per.acoustic_lag_rep[1] < 2.5


def test_the_controller_passes_the_reported_lag_to_the_formation_time(monkeypatch):
    from holo_fleet.control.controller import DroneController
    from holo_fleet.perception.perception import Perception
    from holo_fleet.perception.frame import SensorFrame
    from holo_fleet.perception.sonar_geometry import SECTORS, sonar_sensor_name
    from holo_fleet.sim.scenarios import SCENARIOS

    data = {"DVLSensor": np.zeros(7), "Compass": np.array([1.0, 0.0, 0.0]), "DepthSensor": np.array([-5.0]),
            "IMUSensor": np.zeros((4, 3)), "AcousticRx": ()}
    for sec in SECTORS:
        data[sonar_sensor_name(sec)] = np.zeros(DEFAULT.perc.sonar.ranges().size)

    seen = {}
    orig_update = Perception.update

    def update(self, *a, **kw):
        obs = orig_update(self, *a, **kw)
        obs.form.behind_rep = 1.25
        return obs

    def ft_update(self, t, dt, **kw):
        seen.update(kw)

    monkeypatch.setattr(Perception, "update", update)
    monkeypatch.setattr(FormationTime, "update", ft_update)
    c = DroneController(SCENARIOS["lost_drone_rejoin"](B2).plans[0], B2)
    c.step(SensorFrame(0.0, data, {k: 0.0 for k in data}), DT)
    assert seen.get("behind_rep") == 1.25
