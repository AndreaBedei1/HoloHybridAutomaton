"""Critical-region occupancy belief (latch) and priority persistence, on synthetic sonar targets."""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.mission import GateSpec, MissionPlan, Path, Slot
from holo_fleet.perception.gate_perception import GatePerception
from holo_fleet.perception.targets import Target

G = DEFAULT.gate
GATE = GateSpec("G", np.zeros(3), np.array([1.0, 0, 0]), np.array([0, -1.0, 0]), 1.5, 1.5)
QS = G.queue_s(3)


def perception(lateral=0.0):
    plan = MissionPlan(drone_id="d", slot_index=0, slots=[Slot(0, 0, 0)], path=Path(np.array([[-20.0, 0], [20.0, 0]]), 0.0),
                       gates=[GATE], structures=[GATE], launch_position=np.array([QS, lateral, 0.0]), launch_yaw_deg=0.0,
                       static_rank=0, s_end=40.0, queue_lateral=lateral, queue_s=QS)
    return GatePerception(plan, DEFAULT)


def tg(pattern, r, cls="DYNAMIC"):
    return Target(pattern=frozenset(pattern), r_min=r, d_lower=r, age=0.0, closing_rate=0.0, cls=cls)


def run(gp, seq, p=None, t0=0.0):
    p = np.array([QS, 0.0, 0.0]) if p is None else p
    o, t = None, t0
    for targets in seq:
        o = gp.update(t, p, 0.0, targets, {})
        t += 0.1
    return o, t


def test_free_without_echoes_and_priority_after_t_clear():
    gp = perception()
    o, _ = run(gp, [[]] * 5)
    assert o.occ_state == "FREE" and not o.has_prio and o.decision == "PRIORITY"
    o, _ = run(gp, [[]] * 8, t0=0.5)
    assert o.has_prio


def test_corridor_echo_latches_busy_and_masking_keeps_it():
    gp = perception()
    o, t = run(gp, [[tg({"FRONT"}, 4.6)]] * 3)
    assert o.occ_state == "BUSY" and o.decision == "WAIT"
    o, t = run(gp, [[]] * 60, t0=t)                          # 6 s masked by the gate bars
    assert o.occ_state == "BUSY" and o.occ_busy


def test_exit_seen_then_free_after_t_clear():
    gp = perception()
    _, t = run(gp, [[tg({"FRONT"}, 4.6)]] * 3)
    _, t = run(gp, [[]] * 20, t0=t)
    o, t = run(gp, [[tg({"FRONT"}, 7.5)]] * 2, t0=t)         # beyond the CR
    assert o.occ_state == "EXITING"
    o, t = run(gp, [[tg({"FRONT"}, 8.0)]] * int(G.t_clear / 0.1 + 2), t0=t)
    assert o.occ_state == "FREE"


def test_single_capture_echo_does_not_latch():
    gp = perception()
    o, t = run(gp, [[tg({"FRONT"}, 4.6, cls="UNKNOWN")]])
    o, t = run(gp, [[]] * 3, t0=t)
    assert o.occ_state == "FREE"


def test_queued_neighbour_never_counts_as_occupancy():
    gp = perception(lateral=0.0)
    o, _ = run(gp, [[tg({"RIGHT"}, 2.9), tg({"LEFT"}, 2.9)]] * 20)
    assert o.occ_state == "FREE" and o.decision == "WAIT"     # LEFT neighbour goes first
    gp = perception(lateral=3.5)
    o, _ = run(gp, [[tg({"RIGHT"}, 2.9)]] * 20, p=np.array([QS, 3.5, 0.0]))
    assert o.occ_state == "FREE" and o.has_prio


def test_drone_between_queue_and_cr_is_a_wait_not_an_occupancy():
    gp = perception()
    o, _ = run(gp, [[tg({"FRONT"}, 2.0)]] * 5)                # nearer than the CR
    assert o.decision == "WAIT" and o.occ_state == "FREE"


def test_timeout_is_counted():
    gp = perception()
    _, t = run(gp, [[tg({"FRONT"}, 4.6)]] * 3)
    o, t = run(gp, [[]] * int(G.t_occ_max / 0.1 + 5), t0=t)
    assert o.occ_state == "FREE" and o.occ_timeouts == 1
