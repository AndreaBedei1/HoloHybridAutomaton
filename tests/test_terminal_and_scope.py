"""Terminal criterion of a run (referee/terminal.py, criteria v3), the a-priori deadline (referee/criteria.py) and the
deterministic choice of the degraded drones."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.referee import criteria
from holo_fleet.referee.referee import Referee
from holo_fleet.referee.terminal import TerminalMonitor
from holo_fleet.sim.scenarios import SCENARIOS

ROOT = Path(__file__).resolve().parents[1]


class Ref:
    """Minimal referee stand-in: violation lists, the live formation state and per-drone deviations."""

    def __init__(self, n):
        self.p1_violations, self.p2_violations, self.contacts, self.collision_edges = [], [], [], []
        self.intruder_contacts, self.intruder_min_t, self.intruder_min_d = [], None, 1e9
        self.formation = "OK"
        self.tmpl = np.zeros((n, 3))
        self.dev = np.zeros(n)

    def live(self):
        return {"formation": self.formation, "formation_present": self.formation}

    def deviations(self, P, idx):
        return self.dev


def _monitor(name="formation_triangle", deadline=None):
    sc = SCENARIOS[name](DEFAULT)
    dl = criteria.deadline_v3(sc, DEFAULT, "CLEAN") if deadline is None else deadline
    mon = TerminalMonitor(sc, DEFAULT, sc.sim.names, dl)
    return mon, np.array(criteria.plan_targets(sc)), len(sc.sim.names)


def test_deadline_terms_come_from_the_plan_the_protocol_and_the_profile():
    G, F = DEFAULT.gate, DEFAULT.form
    sc = SCENARIOS["gate_single"](DEFAULT)
    terms = criteria.deadline_terms(sc, DEFAULT, "MEDIUM-SUBSET")
    serial = sum((abs(p.queue_lateral) + G.exit_s - p.queue_s) / G.v_pass + G.t_occ_max + G.t_clear for p in sc.plans)
    dwell = sum(d for _s, d in sc.plans[0].clock.holds)
    assert terms["T_nom"] == pytest.approx(criteria._clock_end(sc.plans[0].clock, sc.plans[0].s_end), abs=0.1)
    assert terms["T_nom"] > sc.duration_s                          # the planned end, not the simulation duration
    assert terms["gate"] == pytest.approx(max(0.0, serial - dwell), abs=0.1)     # only the excess over the dwell
    assert terms["formation"] == pytest.approx(F.t_rejoin + criteria.t_hold_s(DEFAULT), abs=0.1)
    assert terms["failsafe"] > 0.0 and criteria.deadline_terms(sc, DEFAULT, "CLEAN")["failsafe"] == 0.0
    lost = criteria.deadline_terms(SCENARIOS["lost_drone_timeout"](DEFAULT), DEFAULT, "CLEAN")
    assert lost["faults"] == pytest.approx(F.t_rejoin * (1.0 - F.v_wait / F.v_nominal), abs=0.1)
    p1 = criteria.deadline_terms(SCENARIOS["p1_head_on"](DEFAULT), DEFAULT, "CLEAN")
    assert p1["gate"] == 0.0 and p1["formation"] == 0.0 and p1["avoidance"] > 0.0   # single drones: give-way detour
    assert criteria.profile_family("LOW-ALL") == "LOW" and criteria.profile_family(None) == "CLEAN"


def test_constants_match_their_declared_sources():
    import inspect

    from holo_fleet.control.flows import Flows
    assert inspect.signature(Flows.traffic_offset).parameters["offset_max"].default == criteria.GIVEWAY_OFFSET_M
    assert DEFAULT.ref.t_ok_hold == pytest.approx(criteria.t_hold_s(DEFAULT), abs=0.05)      # 4 / k_slot
    assert criteria.progress_tol_m(DEFAULT) == pytest.approx(3.0)
    # the FAILSAFE allowance grows with the horizon at the 99 % Poisson quantile of the profile's outages
    assert criteria.failsafe_allowance_s("MEDIUM-ALL", 300.0) > criteria.failsafe_allowance_s("MEDIUM-ALL", 100.0)
    assert criteria._poisson_quantile(10.0, 0.99) == 18


def test_success_only_after_the_nominal_end_and_with_every_outcome():
    mon, tg, n = _monitor()
    rec = [{"mode": "FORMATION_FOLLOW"}] * n
    r = Ref(n)
    assert mon.update(10.0, tg, r, rec) is None                  # every outcome reached, but before T_nom
    r.formation = "RECOVERING"
    assert mon.update(mon.t_nom + 1.0, tg, r, rec) is None        # formation not recovered yet
    r.formation = "OK"
    assert mon.update(mon.t_nom + 2.0, tg, r, rec) == "SUCCESS"
    assert mon.summary()["t_end"] == mon.t_nom + 2.0 and mon.summary()["criteria"] == "v3"


def test_a_drone_still_waiting_for_a_missing_neighbour_is_not_success():
    mon, tg, n = _monitor()
    rec = [{"mode": "FORMATION_WAIT_REJOIN"}] + [{"mode": "FORMATION_FOLLOW"}] * (n - 1)
    assert mon.update(mon.t_nom + 1.0, tg, Ref(n), rec) is None
    assert "not waiting" in mon.last_missing


def test_a_false_vacancy_blocks_success():
    mon, tg, n = _monitor()
    rec = [{"mode": "FORMATION_FOLLOW", "form": {"vacant": [1]}}] + [{"mode": "FORMATION_FOLLOW"}] * (n - 1)
    assert mon.update(mon.t_nom + 1.0, tg, Ref(n), rec) is None and "no false vacancy" in mon.last_missing
    rec = [{"mode": "FORMATION_FOLLOW", "form": {"vacant": []}}] * n
    assert mon.update(mon.t_nom + 2.0, tg, Ref(n), rec) == "SUCCESS"


def test_progress_is_judged_now_not_latched():
    mon, tg, n = _monitor()
    rec = [{"mode": "FORMATION_FOLLOW"}] * n
    assert mon.update(1.0, tg, Ref(n), rec) is None                # reached early
    assert mon.update(mon.t_nom + 1.0, tg + 10.0, Ref(n), rec) is None and "progress" in mon.last_missing


def test_a_drone_that_goes_over_the_frame_has_not_passed_the_gate():
    sc = SCENARIOS["gate_single"](DEFAULT)
    mon = TerminalMonitor(sc, DEFAULT, sc.sim.names, criteria.deadline_v3(sc, DEFAULT, "CLEAN"))
    g, G = sc.judged_gates[0], DEFAULT.gate
    n = len(sc.sim.names)
    rec = [{"mode": "FORMATION_FOLLOW"}] * n
    P = np.array(criteria.plan_targets(sc))
    for s, dz in ((-3.0, 0.0), (0.0, G.cr_half_height + 1.0), (G.exit_s + 1.0, 0.0)):   # drone 0 above the CR
        P[0] = g.from_gate_frame([s, 0.0, dz])
        mon.update(1.0, P, Ref(n), rec)
    assert mon.passed[0] is None and "drone_0" in mon.bypassed


def test_a_safety_violation_ends_the_run_as_failure():
    mon, tg, n = _monitor()
    r = Ref(n)
    r.collision_edges.append({"t": 5.0, "drone": "drone_0"})
    assert mon.update(5.0, tg, r, [{}] * n) == "FAILURE"
    assert mon.summary()["reason"] == "contact"


def test_the_horizon_ends_as_deadline_with_the_missing_outcomes():
    mon, tg, n = _monitor(deadline=65.0)
    far = tg + 50.0
    assert mon.update(64.9, far, Ref(n), [{"mode": "FORMATION_FOLLOW"}] * n) is None
    assert mon.finalize(64.9, Ref(n), [{"mode": "FORMATION_FOLLOW"}] * n) == "DEADLINE"
    assert "progress" in mon.summary()["reason"]


def test_permanent_loss_requires_the_healthy_formation_and_the_vacancy():
    mon, tg, n = _monitor("lost_drone_timeout")
    assert mon.permanent == {"drone_2"} and 2 not in mon.healthy
    rec = [{"mode": "FORMATION_FOLLOW", "form": {"vacant": []}}] * n
    assert mon.update(mon.t_nom + 1.0, tg, Ref(n), rec) is None and "P3-deg vacancy" in mon.last_missing
    rec = [{"mode": "DEGRADED_FORMATION", "form": {"vacant": [2]}}] * n
    assert mon.update(mon.t_nom + 2.0, tg, Ref(n), rec) == "SUCCESS"
    assert all(mon.must_vacate[k] == {2} for k in mon.healthy)     # every survivor expects slot 2 (square)


def test_v3_referee_declares_only_the_scripted_permanent_fault_absent():
    sc = SCENARIOS["lost_drone_timeout"](DEFAULT)
    ref = Referee(sc.sim.names, sc.template.offsets(), sc.path, [], DEFAULT, True, clock=sc.plans[0].clock,
                  faults=sc.sim.faults)
    P = np.array(criteria.plan_targets(sc, 5.0))
    z = np.zeros((len(P), 3))
    for t in (1.0, 7.9, 8.0, 9.0):
        ref.update(SimpleNamespace(t=t, positions=P, collision=np.zeros(len(P), bool), current_drift=z, velocities=z,
                                   intruders=None, rotations=None), ["FORMATION_FOLLOW"] * len(P))
    assert sorted(ref.absent) == [2] and ref.absence_log[0]["t"] == 8.0
    assert DEFAULT.ref.e_lost == DEFAULT.ref.e_ok == criteria.E_TOL_M      # one tolerance band (no vacuous band)


@pytest.fixture(scope="module")
def camp():
    spec = importlib.util.spec_from_file_location("camp_scope", ROOT / "scripts" / "run_sensor_robustness_campaign.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_degraded_drones_are_deterministic_and_nested(camp):
    for sc in camp.ALL_SCENARIOS:
        one, sub, al = (camp.scope_drones(sc, s) for s in ("ONE", "SUBSET", "ALL"))
        assert len(one) == 1 and set(one) <= set(sub) <= set(al)
        n = len(SCENARIOS[sc](DEFAULT).sim.names)
        assert al == list(range(n)) and len(sub) == (n + 1) // 2 and len(sub) < n
        lost = {f["drone"] for f in SCENARIOS[sc](DEFAULT).sim.faults if f.get("t_off") is None}
        assert f"drone_{one[0]}" not in lost                        # ONE is always a healthy drone
        assert camp.scope_drones(sc, "ONE") == one                    # same choice every time
