"""Terminal criterion of a run (referee/terminal.py) and the deterministic choice of the degraded drones."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.referee.terminal import TerminalMonitor, clean_horizon_s, deadline_s
from holo_fleet.sim.scenarios import SCENARIOS

ROOT = Path(__file__).resolve().parents[1]


class Ref:
    """Minimal referee stand-in: violation lists and the live formation state."""

    def __init__(self):
        self.p1_violations, self.p2_violations, self.contacts, self.collision_edges = [], [], [], []
        self.intruder_contacts, self.intruder_min_t, self.intruder_min_d = [], None, 1e9
        self.formation = "OK"

    def live(self):
        return {"formation": self.formation, "formation_present": self.formation}


def _monitor(name="formation_triangle", t_ref=30.0, deadline=65.0):
    sc = SCENARIOS[name](DEFAULT)
    n = len(sc.sim.names)
    end = [[float(k), 0.0, -5.0] for k in range(n)]
    ref = {"P3_recovered": True, "P3_degraded_holds": False, "end": end}
    return TerminalMonitor(sc, DEFAULT, sc.sim.names, ref, t_ref, deadline), np.array(end), n


def test_deadline_is_derived_from_the_clean_completion():
    assert deadline_s(100.0) == pytest.approx(170.0)
    assert clean_horizon_s(40.0) == pytest.approx(150.0)


def test_success_only_after_the_reference_time_and_with_every_outcome():
    mon, end, n = _monitor()
    rec = [{"mode": "FORMATION_FOLLOW"}] * n
    r = Ref()
    assert mon.update(10.0, end, r, rec) is None                 # every outcome reached, but before T_ref
    r.formation = "RECOVERING"
    assert mon.update(31.0, end, r, rec) is None                 # formation not recovered yet
    r.formation = "OK"
    assert mon.update(32.0, end, r, rec) == "SUCCESS"
    assert mon.summary()["t_end"] == 32.0


def test_a_drone_still_waiting_for_a_missing_neighbour_is_not_success():
    mon, end, n = _monitor()
    rec = [{"mode": "FORMATION_WAIT_REJOIN"}] + [{"mode": "FORMATION_FOLLOW"}] * (n - 1)
    assert mon.update(40.0, end, Ref(), rec) is None


def test_a_safety_violation_ends_the_run_as_failure():
    mon, end, n = _monitor()
    r = Ref()
    r.collision_edges.append({"t": 5.0, "drone": "drone_0"})
    assert mon.update(5.0, end, r, [{}] * n) == "FAILURE"
    assert mon.summary()["reason"] == "contact"


def test_the_horizon_ends_as_deadline_with_the_missing_outcomes():
    mon, end, n = _monitor()
    far = end + 50.0
    assert mon.update(64.9, far, Ref(), [{"mode": "FORMATION_FOLLOW"}] * n) is None
    assert mon.finalize(64.9, Ref(), [{"mode": "FORMATION_FOLLOW"}] * n) == "DEADLINE"
    assert "progress" in mon.summary()["reason"]


@pytest.fixture(scope="module")
def camp():
    spec = importlib.util.spec_from_file_location("camp_scope", ROOT / "scripts" / "run_sensor_robustness_campaign.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_degraded_drones_are_deterministic_and_nested(camp):
    for sc in camp.VALIDATION_SCENARIOS:
        one, sub, al = (camp.scope_drones(sc, s) for s in ("ONE", "SUBSET", "ALL"))
        assert len(one) == 1 and set(one) <= set(sub) <= set(al)
        n = len(SCENARIOS[sc](DEFAULT).sim.names)
        assert al == list(range(n)) and len(sub) == (n + 1) // 2 and len(sub) < n
        lost = {f["drone"] for f in SCENARIOS[sc](DEFAULT).sim.faults if f.get("t_off") is None}
        assert f"drone_{one[0]}" not in lost                        # ONE is always a healthy drone
        assert camp.scope_drones(sc, "ONE") == one                    # same choice every time


def test_validation_plan_has_clean_first_and_no_high(camp):
    runs = camp.plan("validation", (0, 1, 2))
    assert len(runs) == 8 * 3 * (1 + 2 * 3)
    labels = [r[1] for r in runs]
    assert labels[:24] == ["CLEAN"] * 24 and not any("HIGH" in lab for lab in labels)
