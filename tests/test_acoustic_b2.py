"""Variant B2 (range-bearing beacon): channel model, onboard fixes, acoustic exit evidence, formation rules."""

import dataclasses
import math

import numpy as np
import pytest

from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates
from holo_fleet.config import DEFAULT, BeaconConfig
from holo_fleet.mission import MissionPlan, Path, Slot, abreast_slots, queue_line
from holo_fleet.perception.acoustic import AcousticTracker, box_bounds, region_samples
from holo_fleet.perception.acoustic_gate import AcousticExit, gate_points
from holo_fleet.perception.gate_perception import GatePerception
from holo_fleet.perception.sonar_geometry import R_OUT
from holo_fleet.perception.targets import Target
from holo_fleet.sim import acoustic_channel as ac

B2 = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_RB"))
G = B2.gate
G06 = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
YAW = math.atan2(G06.axis[1], G06.axis[0])
N = 4
QS = queue_line(abreast_slots(N), G)
LATS = [s.lateral for s in abreast_slots(N)]


def measure(p_obs, yaw, p_tgt):
    """Exact (range, az, el) of p_tgt in the body frame of an observer at p_obs with heading yaw (level)."""
    d = np.asarray(p_tgt, float) - np.asarray(p_obs, float)
    c, s = math.cos(yaw), math.sin(yaw)
    b = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
    r = float(np.linalg.norm(b))
    return r, math.atan2(b[1], b[0]), math.asin(b[2] / r)


def gate_world(s, l, dz=0.0):
    return G06.from_gate_frame([s, l, dz])


class Bench:
    """One observer queued at lateral ``lat``; other drones are slots 1.. (identity = slot)."""

    def __init__(self, lat=LATS[0], n=N, cfg=B2):
        self.cfg = cfg
        self.p = gate_world(QS, lat)
        self.tr = AcousticTracker(cfg)
        self.tr.set_identities({k: k for k in range(n)})
        self.t = 0.0
        self.tr.update(self.t, (), self.p, YAW)

    def step(self, dt=0.1, rx=()):
        self.t = round(self.t + dt, 4)
        return self.tr.update(self.t, rx, self.p, YAW)

    def ping(self, slot, p_tgt, err=(0.0, 0.0, 0.0)):
        r, az, el = measure(self.p, YAW, p_tgt)
        return self.step(rx=[(slot, self.t + 0.1 - 0.05, r + err[0], az + math.radians(err[1]), el + math.radians(err[2]))])


# ---------------------------------------------------------------------------------------------- channel model
def test_channel_levels_are_ordered_and_blackout_delivers_nothing():
    lv = [ac.CLEAN, ac.LOW, ac.MEDIUM]
    for f in ("sigma_range_m", "sigma_az_deg", "dropout", "nlos_rx", "latency_s"):
        assert [getattr(c, f) for c in lv] == sorted(getattr(c, f) for c in lv)
    m = ac.ChannelModel(ac.BLACKOUT, 3, 0, 50.0)
    assert all(m.measure(1.0, 0, 1, np.array([0.1, 0.0, 5.0, -4.0])) is None for _ in range(50))


def test_clean_direct_path_errors_stay_inside_the_onboard_envelope():
    m = ac.ChannelModel(ac.MEDIUM, 6, 3, 50.0)
    br, ba, be = box_bounds(10.0, B2)
    out = 0
    for i in range(4000):
        x = m.measure(200.0 + i * 0.0, 1, 2, np.array([0.3, -0.1, 10.0, -4.0]))     # outside the outages
        if x is None:
            continue
        r, az, el, nlos = x
        assert not nlos
        out += (abs(r - 10.0) > br) or (abs(az - 0.3) > ba) or (abs(el + 0.1) > be)
    assert out == 0                                   # 3 sigma + bias bounds: none in 4000 draws at this seed


def test_blocked_path_is_lost_or_a_longer_reflection():
    m = ac.ChannelModel(ac.MEDIUM, 3, 1, 50.0)
    blocked = np.full(4, -1.0)
    assert m.measure(100.0, 0, 1, blocked) is None
    got = [m.reflected(100.0, 0, 1, (0.0, 0.0, 6.0)) for _ in range(400)]
    got = [g for g in got if g is not None]
    assert 0.2 < len(got) / 400 < 0.5 and all(g[3] for g in got)
    assert min(g[0] for g in got) > 6.0 * m.scale + 0.3 - 4 * ac.MEDIUM.sigma_range_m - 0.5


def test_tdma_slots_are_tick_aligned_and_disjoint():
    from holo_fleet.sim.beacon import BeaconTransport

    names = [f"d{k}" for k in range(6)]
    tr = BeaconTransport(names, B2.beacon, seed=0)
    assert tr.rb and tr.slot_ticks * len(names) <= tr.period_ticks and tr.slot_ticks >= 2
    ticks = {n: [t for t in range(90) if (t - k * tr.slot_ticks) % tr.period_ticks == 0] for k, n in enumerate(names)}
    all_t = sorted(x for v in ticks.values() for x in v)
    assert len(all_t) == len(set(all_t)) and all(len(v) == 3 for v in ticks.values())


# ---------------------------------------------------------------------------------------------- fixes
def test_region_covers_every_error_inside_the_envelope():
    rng = np.random.default_rng(0)
    for _ in range(300):
        r, az, el = rng.uniform(1.0, 20.0), rng.uniform(-math.pi, math.pi), rng.uniform(-0.6, 0.6)
        br, ba, be = box_bounds(r, B2)
        pts, slack = region_samples(r, az, el, B2)
        tr_ = r + rng.uniform(-br, br)
        ta, te = az + rng.uniform(-ba, ba), el + rng.uniform(-be, be)
        truth = tr_ * np.array([math.cos(te) * math.cos(ta), math.cos(te) * math.sin(ta), math.sin(te)])
        assert float(np.min(np.linalg.norm(pts - truth, axis=1))) <= slack + 1e-9


def test_noisy_fixes_are_accepted_and_an_outlier_jump_is_rejected_but_kept():
    b = Bench()
    tgt = gate_world(QS, LATS[1])
    for k in range(5):
        b.ping(1, tgt, err=(0.1 * (-1) ** k, 1.0, -1.0))
        b.step(0.9)
    f = b.ping(1, tgt, err=(4.0, 25.0, 0.0))[0]                    # a reflected arrival: 4 m longer, 25 deg off
    assert not f.accepted and f.reason == "kinematic"
    assert len([x for x in b.tr.all_fixes(1) if x.accepted]) == 5 and len(b.tr.all_fixes(1)) == 6


def test_stale_and_implausible_receptions_are_not_used():
    b = Bench()
    r, az, el = measure(b.p, YAW, gate_world(QS, LATS[1]))
    b.step(0.5)
    assert b.step(rx=[(1, b.t - 0.6, r, az, el)]) == []              # delivered 0.6 s after its arrival
    assert b.tr.stats["stale"] == 1
    up = gate_world(QS, LATS[1], dz=+4.3 + 2.0)                        # mirror image above the surface
    f = b.ping(1, up)[0]
    assert not f.accepted and f.reason == "depth"


def test_absence_of_receptions_gives_no_fix():
    b = Bench()
    for _ in range(50):
        b.step()
    assert b.tr.latest(1, b.t, 10.0) is None


def test_two_consistent_rejected_fixes_restart_a_track_with_a_new_id():
    b = Bench()
    b.ping(1, gate_world(QS, LATS[1]))
    b.step(3.5)
    far = gate_world(QS - 12.0, LATS[1])                               # the drone really is elsewhere now (re-heard)
    f1 = b.ping(1, far)[0]
    assert not f1.accepted
    b.step(0.9)
    f2 = b.ping(1, far)[0]
    assert f2.accepted and f2.reason == "restart" and f2.track == 1 and b.tr.stats["track_restarts"] == 1


# ---------------------------------------------------------------------------------------------- gate (P2)
def exit_bench(lat=LATS[0], n=N):
    b = Bench(lat, n)
    ae = AcousticExit(B2, b.tr, list(range(1, n)))
    return b, ae


def corridor(b, ae, occ_p):
    """The occupant seen in the FRONT corridor now (t_lc)."""
    r = float(np.linalg.norm(occ_p - b.p))
    ae.note_corridor(b.t, G06, b.p, YAW, [r - 0.3])


def queued_others(b, k_occ, n=N, lat_obs=LATS[0]):
    """Pings of the other queued drones at their queue points."""
    for k, lat in zip(range(1, n), [x for x in LATS if x != lat_obs]):
        if k != k_occ:
            b.ping(k, gate_world(QS, lat))


def test_occupant_without_fixes_never_releases():
    b, ae = exit_bench()
    corridor(b, ae, gate_world(-0.6, 0.0))
    for _ in range(40):
        queued_others(b, k_occ=1)
        b.step(0.5)
        assert ae.evaluate(b.t, G06) is None
    assert 1 in ae.last["possible"]


def _pass(b, ae, k_occ=1, s_end=9.0, steps=12, others=True):
    corridor(b, ae, gate_world(-0.6, 0.0))
    if others:
        queued_others(b, k_occ)
    for s in np.linspace(-0.6, s_end, steps):
        b.ping(k_occ, gate_world(float(s), 0.0))
        if others:
            queued_others(b, k_occ)
        b.step(0.7)
    return ae.evaluate(b.t, G06)


def test_confirmed_exit_of_the_occupant_with_everyone_accounted_releases():
    b, ae = exit_bench()
    w = _pass(b, ae)
    assert w is not None and w["sector"] == "ACOUSTIC" and w["slots"] == [1]


def test_a_single_fix_beyond_is_not_enough():
    b, ae = exit_bench()
    corridor(b, ae, gate_world(-0.6, 0.0))
    queued_others(b, 1)
    b.ping(1, gate_world(-0.5, 0.0))
    b.step(0.8)
    b.ping(1, gate_world(6.0, 0.0))                       # a jump: rejected anyway; then nothing more
    queued_others(b, 1)
    assert ae.evaluate(b.t, G06) is None


def test_fixes_that_leave_the_cr_reachable_do_not_release():
    b, ae = exit_bench()
    w = _pass(b, ae, s_end=1.6)                            # still close to the exit face: region reaches the CR
    assert w is None and 1 in ae.last["not_outside"]


def test_stale_exit_fixes_lose_their_evidence():
    b, ae = exit_bench()
    assert _pass(b, ae) is not None
    for _ in range(30):
        b.step(0.5)                                      # 15 s without receptions: the regions grow back over the CR
    assert ae.evaluate(b.t, G06) is None


def test_a_silent_drone_that_could_be_in_the_cr_blocks_the_release():
    b, ae = exit_bench()
    corridor(b, ae, gate_world(-0.6, 0.0))
    for s in np.linspace(-0.6, 9.0, 12):
        b.ping(1, gate_world(float(s), 0.0))
        b.ping(2, gate_world(QS, LATS[2]))               # drone 3 never heard
        b.step(0.7)
    assert ae.evaluate(b.t, G06) is None and 3 in ae.last["possible"]


def test_a_contradictory_fix_inside_the_cr_blocks_the_release():
    b, ae = exit_bench()
    assert _pass(b, ae) is not None
    b.ping(1, gate_world(0.0, 0.0))                        # inconsistent with the track (a jump back): rejected ...
    assert not b.tr.all_fixes(1)[-1].accepted
    queued_others(b, 1)
    assert ae.evaluate(b.t, G06) is None                  # ... but it could place drone 1 in the CR


def test_a_second_possible_occupant_near_the_cr_blocks_the_release():
    b, ae = exit_bench()
    corridor(b, ae, gate_world(-0.6, 0.0))
    for s in np.linspace(-0.6, 9.0, 12):
        b.ping(1, gate_world(float(s), 0.0))
        b.ping(2, gate_world(-1.2, 0.0))                 # drone 2 at the hold point, right at the CR
        b.ping(3, gate_world(QS, LATS[3]))
        b.step(0.7)
    assert ae.evaluate(b.t, G06) is None and 2 in ae.last["not_outside"]


def test_noisy_fixes_inside_the_envelope_never_place_a_cr_hull_beyond_the_exit():
    """Brute force on the decision geometry: an occupant anywhere in the CR, observed from every queue point with
    errors at the corners of the envelope, never gives a fix whose region lies beyond the exit face."""
    b, ae = exit_bench()
    br_f = lambda r: box_bounds(r, B2)
    half = np.array([G.cr_half_len, G.cr_half_width, G.cr_half_height])
    for lat in LATS:
        p = gate_world(QS, lat)
        for c in [np.array([a, bb, cc]) * half for a in (-1, 0, 1) for bb in (-1, 0, 1) for cc in (-1, 0, 1)]:
            r, az, el = measure(p, YAW, gate_world(*c))
            br, ba, be = br_f(r)
            for sr in (-1, 1):
                for sa in (-1, 1):
                    for se in (-1, 1):
                        from holo_fleet.perception.acoustic import Fix, _rot
                        body, slack = region_samples(r + sr * br, az + sa * ba, el + se * be, B2)
                        pts = p + body @ _rot(YAW).T
                        f = Fix(slot=1, t=0.0, r=r, az=az, el=el, p=pts.mean(0), pts=pts, slack=slack, rel=pts.mean(0) - p)
                        assert ae.s_lower(G06, f, 0.0) <= G.cr_half_len + R_OUT


def test_gate_latch_ignores_acoustics_while_the_corridor_is_occupied():
    plan = MissionPlan(drone_id="d", slot_index=0, slots=[Slot(0, l, 0) for l in LATS],
                       path=Path(np.array([[-20.0, 0], [20.0, 0]]), 0.0), gates=[G06], structures=[G06],
                       launch_position=gate_world(QS, LATS[0]), launch_yaw_deg=math.degrees(YAW), static_rank=0,
                       s_end=40.0, queue_lateral=LATS[0], queue_s=QS)
    b = Bench()
    gp = GatePerception(plan, B2, acoustic=b.tr)
    tg = lambda r: Target(pattern=frozenset({"FRONT"}), r_min=r, d_lower=r, age=0.0, closing_rate=0.0, cls="DYNAMIC", ranges={})
    occ = gate_world(-0.6, 0.0)
    r_occ = float(np.linalg.norm(occ - b.p)) - 0.3
    for _ in range(3):
        gp.update(b.t, b.p, YAW, [tg(r_occ)], {})
        b.step()
    for s in np.linspace(-0.6, 0.4, 4):                    # the occupant crosses the CR, seen in the corridor
        b.ping(1, gate_world(float(s), 0.0))
        queued_others(b, 1)
        o = gp.update(b.t, b.p, YAW, [tg(r_occ)], {})
        b.step(0.7)
        assert o.occ_state == "BUSY"
    for s in np.linspace(1.0, 10.0, 10):                   # corridor empty (masked / gone), the occupant moves on
        b.ping(1, gate_world(float(s), 0.0))
        queued_others(b, 1)
        echo = [tg(r_occ)] if s > 9.0 else []              # an echo in the corridor again: back to BUSY
        o = gp.update(b.t, b.p, YAW, echo, {})
        b.step(0.7)
        if o.occ_state != "BUSY":
            break
    assert o.occ_state == "EXITING" and o.exit_witness["sector"] == "ACOUSTIC" and o.exit_witness["slots"] == [1]
    assert gp.exit_kind["ACOUSTIC"] == 1


def test_b0_gate_perception_has_no_acoustic_path():
    plan = MissionPlan(drone_id="d", slot_index=0, slots=[Slot(0, 0, 0)], path=Path(np.array([[-20.0, 0], [20.0, 0]]), 0.0),
                       gates=[G06], structures=[G06], launch_position=gate_world(QS, 0.0),
                       launch_yaw_deg=math.degrees(YAW), static_rank=0, s_end=40.0, queue_lateral=0.0, queue_s=QS)
    assert GatePerception(plan, DEFAULT, acoustic=AcousticTracker(DEFAULT)).acoustic_exit is None


def test_gate_points_matches_the_gate_frame():
    q = np.array([[1.0, 2.0, -4.0], [-3.0, 0.5, -5.0]])
    assert np.allclose(gate_points(G06, q), np.array([G06.to_gate_frame(x) for x in q]))


# ---------------------------------------------------------------------------------------------- formation (P3)
from holo_fleet.control.controller import DroneController                         # noqa: E402
from holo_fleet.perception.formation_perception import NEIGHBOUR_MEMORY_S, expected_relative   # noqa: E402
from holo_fleet.perception.frame import SensorFrame                               # noqa: E402
from holo_fleet.perception.sonar_geometry import SECTORS, sonar_sensor_name        # noqa: E402
from holo_fleet.sim.scenarios import SCENARIOS                                    # noqa: E402

DT = 0.1
RANGES = DEFAULT.perc.sonar.ranges()


def _frame(t, rx=()):
    data = {"DVLSensor": np.zeros(7), "Compass": np.array([1.0, 0.0, 0.0]), "DepthSensor": np.array([-5.0]),
            "IMUSensor": np.zeros((4, 3))}
    for s in SECTORS:
        data[sonar_sensor_name(s)] = np.zeros(RANGES.size)          # the sonar sees nobody
    stamp = {k: t for k in data}
    data["AcousticRx"] = tuple(rx)
    return SensorFrame(t, data, stamp)


def _run_form(cfg, heard, steps=80, offset=None, scenario="lost_drone_rejoin"):
    """``heard(t)``: slots received at t; each heard where the perception expects it (plus ``offset[slot]``)."""
    plan = SCENARIOS[scenario](cfg).plans[0]
    c = DroneController(plan, cfg)
    c.perception.acoustic.set_identities({k: k for k in range(len(plan.slots))})
    first, recs = {}, []
    for i in range(steps):
        t = round(DT * i, 3)
        rx = []
        if i > 0:
            pc = c.perception
            slot, R_form, _ = pc.slot_reference(c.ftime.now(t) if hasattr(c, "ftime") else t, c.flows.sigma)
            exp = expected_relative(plan, R_form)
            for k in heard(t):
                rel = exp[k] + (slot - pc.nav.state.p) + (np.zeros(3) if not offset else np.asarray(offset.get(k, 0.0)))
                r = float(np.linalg.norm(rel))
                rx.append((k, t - 0.05, r, math.atan2(rel[1], rel[0]), math.asin(rel[2] / r)))
        c.step(_frame(t, rx), DT)
        recs.append(c.last_record)
        for k, v in c.perception.missing_state.items():
            if v:
                first.setdefault(k, t)
    return c, first, recs


def _every(period, slots=(1, 2, 3)):
    return lambda t: slots if abs(t / period - round(t / period)) < 1e-6 else ()


def test_heard_where_expected_a_sonar_occluded_neighbour_is_not_missing():
    _c, b0, _ = _run_form(DEFAULT, _every(1.0))
    _c, b2, _ = _run_form(B2, _every(1.0))
    assert set(b0) == {1, 2, 3} and all(v == pytest.approx(NEIGHBOUR_MEMORY_S + DT) for v in b0.values())
    assert b2 == {}                                                    # rule A-P: present by its fixes


def test_one_lost_ping_does_not_make_a_heard_neighbour_missing():
    # pings every 1 s, the one of t = 3 s lost: the latest fix stays fresh (fresh_s) and near its slot
    lossy = lambda t: (1, 2, 3) if abs(t - round(t)) < 1e-6 and round(t) != 3 else ()
    _c, b2, _ = _run_form(B2, lossy)
    assert b2 == {}


def test_a_silent_channel_is_the_baseline():
    assert _run_form(B2, lambda t: ())[1] == _run_form(DEFAULT, lambda t: ())[1]


def test_presence_never_confirms_a_missing_neighbour_back():
    late = lambda t: (1, 2, 3) if t >= 3.0 and abs(t - round(t)) < 1e-6 else ()
    c, first, _ = _run_form(B2, late)
    assert set(first) == {1, 2, 3}
    assert all(c.perception.missing_state[k] for k in (1, 2, 3))        # only a sonar confirmation brings it back


def test_a_neighbour_heard_far_from_its_slot_is_declared_missing_on_the_second_fix():
    off = {2: np.array([-6.0, 0.0, 0.0])}                              # 6 m behind its slot, the others in place
    c, first, recs = _run_form(B2, _every(1.0), offset=off)
    assert set(first) == {2} and first[2] <= 2.0 + 1e-9               # fixes at 1.0 and 2.0 s
    assert 2 in recs[-1]["form"]["acoustic_displaced"]


def test_the_along_track_offset_of_a_neighbour_feeds_the_progress_consensus():
    off = {k: np.array([0.6, 0.0, 0.0]) for k in (1, 2, 3)}            # everyone 0.6 m ahead of me
    c, _first, recs = _run_form(B2, _every(1.0), offset=off, steps=30)
    assert recs[-1]["form"]["acoustic_corr"] == [1, 2, 3]
    assert c.flows.sigma > 0.0                                         # my reference moves ahead
    c0, _f, _r = _run_form(DEFAULT, _every(1.0), offset=off, steps=30)
    assert c0.flows.sigma == 0.0                                       # B0: nothing seen, nothing corrected


def _square_rear():
    """Square, front-left drone (slot 0): its rear neighbour, 3.5 m straight behind, and the REAR echo of a drone."""
    from holo_fleet.perception.formation_perception import check_formation
    from holo_fleet.formations import TEMPLATES
    plan = MissionPlan(drone_id="drone_0", slot_index=0, slots=list(TEMPLATES["square"].slots),
                       path=Path(np.array([[0, 0], [50.0, 0]]), -5), gates=[], structures=[],
                       launch_position=np.zeros(3), launch_yaw_deg=0.0, static_rank=0, s_end=40)
    rear = next(k for k, d in expected_relative(plan, np.eye(3)).items() if d[0] < -1.0 and abs(d[1]) < 0.5)
    healthy = {s: True for s in SECTORS}
    echo = lambda d: Target(pattern=frozenset({"REAR"}), r_min=d - 0.48, d_lower=d - 0.48, age=0.0, closing_rate=0.0,
                            cls="DYNAMIC", ranges={"REAR": d - 0.48})
    return plan, rear, healthy, echo, check_formation


def test_an_identified_neighbour_far_from_its_slot_is_not_seen_at_it():
    plan, rear, healthy, echo, check_formation = _square_rear()
    far = np.array([-8.5, 0.0, 0.0])                                   # 5 m behind its slot, heard there
    b0 = check_formation(plan, DEFAULT, np.zeros(3), np.eye(3), np.eye(3), [echo(8.5)], healthy)
    b2 = check_formation(plan, B2, np.zeros(3), np.eye(3), np.eye(3), [echo(8.5)], healthy, acoustic={rear: far})
    c0, c2 = (next(c for c in o.checks if c.slot == rear) for o in (b0, b2))
    assert not c0.seen and not c2.seen                                 # outside the range gate of its slot: not there
    assert b2.form_err == pytest.approx(b0.form_err)                   # no formation residual from a displaced drone


def test_the_fix_picks_the_identified_echo_inside_the_slot_gate():
    plan, rear, healthy, echo, check_formation = _square_rear()
    near = np.array([-4.3, 0.0, 0.0])                                  # 0.8 m behind its slot, heard there
    o = check_formation(plan, B2, np.zeros(3), np.eye(3), np.eye(3), [echo(3.5), echo(4.3)], healthy,
                        acoustic={rear: near})
    c = next(c for c in o.checks if c.slot == rear)
    assert c.seen and c.measured_d == pytest.approx(4.3, abs=0.1)      # the identified drone, not the nearer echo


def test_blocked_geometry_uses_the_map_bars_and_hulls():
    """The transport's shadow test runs on the scene the simulator hands it (map bars: centre / axes / half)."""
    from holo_fleet.sim.beacon import BeaconTransport

    tr = BeaconTransport(["a", "b", "c"], B2.beacon, seed=0)
    bars = list(G06.bars)
    a, b = gate_world(-3.0, 1.1), gate_world(3.0, 1.1)            # a ray through the left side bar
    tr.scene = lambda: (np.array([a, b, gate_world(-8.0, 6.0)]), np.zeros(3), bars)
    geom, shadow = tr._blocked_geometry(0, 1)
    assert shadow and geom[2] == pytest.approx(float(np.linalg.norm(a - b)))
    tr.scene = lambda: (np.array([a, b, (a + b) / 2.0]), np.zeros(3), [])   # a hull halfway
    assert tr._blocked_geometry(0, 1)[1]
    tr.scene = lambda: (np.array([a, b, gate_world(-8.0, 6.0)]), np.zeros(3), [])
    assert not tr._blocked_geometry(0, 1)[1]
