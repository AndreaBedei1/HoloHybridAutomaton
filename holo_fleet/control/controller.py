"""One drone's controller (v2): perception -> hybrid automaton -> mode flow -> low-level -> thrusters.

Identical code on every drone; inputs are only the drone's own SensorFrame (six sonars, DVL,
IMU, compass, depth) and its mission plan.  ``uses_ground_truth`` is checked by the tests.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

import numpy as np

from holo_fleet.arena_bridge import thruster_command
from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.control.flows import Flows
from holo_fleet.control.lowlevel import LowLevelController
from holo_fleet.ha.automaton import LocalHybridAutomaton
from holo_fleet.ha.spec import Mode
from holo_fleet.mission import MissionPlan
from holo_fleet.perception.frame import SensorFrame
from holo_fleet.perception.perception import LocalObservation, Perception

AVOIDANCE = (Mode.SEPARATION_WARNING, Mode.COLLISION_AVOIDANCE)


class EnvelopeMonitor:
    """The drone declares itself out of envelope when it cannot hold its commanded velocity for a while
    (persistent actuator saturation): its own, onboard, view of 'the current is stronger than me'."""

    def __init__(self, t_enter: float = 4.0, t_exit: float = 4.0):
        self.t_enter, self.t_exit = t_enter, t_exit
        self.sat_time = 0.0
        self.ok_time = 0.0
        self.ok = True

    def update(self, saturated: bool, dt: float) -> bool:
        if saturated:
            self.sat_time += dt
            self.ok_time = 0.0
        else:
            self.ok_time += dt
            self.sat_time = max(0.0, self.sat_time - dt)
        if self.ok and self.sat_time >= self.t_enter:
            self.ok = False
        elif not self.ok and self.ok_time >= self.t_exit:
            self.ok = True
        return self.ok


class DroneController:
    uses_ground_truth = False

    def __init__(self, plan: MissionPlan, cfg: FleetConfig = DEFAULT, nav_init_err: Optional[np.ndarray] = None):
        self.plan = plan
        self.cfg = cfg
        self.perception = Perception(plan, cfg, nav_init_err)
        self.ha = LocalHybridAutomaton(cfg)
        self.flows = Flows(plan, cfg)
        self.ll = LowLevelController(cfg)
        self.envmon = EnvelopeMonitor()
        self.events: List[Dict[str, Any]] = []
        self.last_obs: Optional[LocalObservation] = None
        self.last_record: Dict[str, Any] = {}

    def step(self, frame: SensorFrame, dt: float) -> np.ndarray:
        t = frame.t
        obs = self.perception.update(frame, dt, sigma=self.flows.sigma, env_ok=self.envmon.ok,
                                     offset=self.flows.offset)
        prev = self.ha.mode
        rec = self.ha.step(obs.ab, t, dt)
        mode = self.ha.mode
        if rec.decision:
            self.events.append({"t": t, "type": "decision", "decision": rec.decision, "edge": rec.edge,
                                "from": rec.source, "to": rec.target,
                                "gate": obs.gate.gate_id, "gate_decision": obs.gate.decision,
                                "relations": obs.gate.relations})
        if rec.decision == "EXITED" or (obs.gate.passed and not self.ha.committed and obs.gate.gate_id is not None):
            self.perception.gp.advance()
        if prev == Mode.FAILSAFE_HOLD_OR_RETREAT and mode != prev:
            self.flows.clear_failsafe()
        # ---------------------------------------------------------------- mission velocity of the calm mode
        if obs.gate.in_zone or self.ha.committed:
            gmode = mode.value if mode in (Mode.GATE_APPROACH, Mode.GATE_YIELD, Mode.GATE_PASS) else (
                "GATE_PASS" if self.ha.committed else "GATE_APPROACH")
            v_mis, yaw_d = self.flows.gate(obs, gmode)
        else:
            v_mis, yaw_d = self.flows.formation(obs, dt, recovering=(mode == Mode.FORMATION_RECOVERY))
        authority = "nominal"
        if mode not in AVOIDANCE:
            self.flows.last_choice = None
            self.flows.escape.reset()
        if mode == Mode.SEPARATION_WARNING:
            v = self.flows.separation_warning(obs, v_mis)
            authority = "brake"
        elif mode == Mode.COLLISION_AVOIDANCE:
            v = self.flows.collision_avoidance(obs, self.ll.current_estimate(), v_mis)
            authority = "escape"
        elif mode == Mode.FAILSAFE_HOLD_OR_RETREAT:
            v = self.flows.failsafe(obs)
            authority = "brake"
        else:
            v = v_mis
        v = self.flows.structure_filter(obs, v)
        cmd = self.ll.command(self.perception.nav.state, v, yaw_d, authority, dt)
        was_ok = self.envmon.ok
        ok = self.envmon.update(self.ll.saturated and mode not in AVOIDANCE, dt)
        if was_ok and not ok:
            self.events.append({"t": t, "type": "ENVELOPE_VIOLATION", "reason": "persistent saturation"})
        elif ok and not was_ok:
            self.events.append({"t": t, "type": "ENVELOPE_RESTORED"})
        self.last_obs = obs
        ch = self.flows.last_choice
        self.last_record = {
            "t": round(t, 3), "mode": mode.value, "committed": self.ha.committed,
            "nav_p": np.round(obs.p, 3).tolist(), "nav_yaw_deg": round(float(np.degrees(obs.yaw)), 1),
            "v_cmd": np.round(v, 3).tolist(), "giveway": self.flows.giveway,
            "offset": np.round(self.flows.offset, 2).tolist(), "d_min": round(obs.d_min, 3) if obs.d_min < 1e8 else None,
            "sense_ok": obs.sense_ok, "env_ok": self.envmon.ok,
            "sectors": {s: {"echoes": [(round(e.r0, 2), e.cls) for e in rd.echoes[:4]],
                            "age": round(min(rd.age, 9.99), 2), "healthy": rd.healthy,
                            "blind_from": None if rd.blind_from is None else round(rd.blind_from, 2)}
                        for s, rd in obs.readings.items()},
            "targets": [{"pattern": "+".join(sorted(tg.pattern)), "r": round(tg.r_min, 2), "d_lower": round(tg.d_lower, 2),
                         "cls": tg.cls, "closing": round(tg.closing_rate, 2)} for tg in obs.targets],
            "escape": None if ch is None else {"dir": ch.label, "guarantee": round(ch.guarantee, 2),
                                               "feasible": ch.feasible},
            "current_est": np.round(self.ll.current_estimate(), 3).tolist(),
            "gate": {"id": obs.gate.gate_id, "s": round(obs.gate.s, 2), "l": round(obs.gate.l, 2),
                     "at_queue": obs.gate.at_queue, "decision": obs.gate.decision, "has_prio": obs.gate.has_prio,
                     "occ": obs.gate.occ_state, "relations": obs.gate.relations, "rank_used": obs.gate.rank_used},
            "form": {"form_err": round(obs.form.form_err, 3), "slot_err": round(obs.form.slot_err_norm, 3),
                     "neighbors_ok": obs.form.neighbors_ok, "sigma": round(self.flows.sigma, 3),
                     "checks": [(c.slot, round(c.expected_d, 2), None if not c.seen else round(c.residual, 2))
                                for c in obs.form.checks]},
            "determinism_violations": self.ha.determinism_violations,
        }
        return thruster_command(cmd["surge"], cmd["sway"], cmd["heave"], cmd["yaw"], self.cfg.plant.thruster_limit)
