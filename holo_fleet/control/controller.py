"""One drone's controller (v2): perception -> hybrid automaton -> mode flow -> low-level -> thrusters.

Identical code on every drone; inputs are only the drone's own SensorFrame (six sonars, DVL,
IMU, compass, depth) and its mission plan.  ``uses_ground_truth`` is checked by the tests.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

import numpy as np

from holo_fleet.arena_bridge import thruster_command
from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.control.current_envelope import CurrentCheck, Persistence, check_onboard
from holo_fleet.control.flows import Flows
from holo_fleet.control.lowlevel import AUTHORITY, LowLevelController
from holo_fleet.ha.automaton import LocalHybridAutomaton
from holo_fleet.ha.observation_invariants import violated as observation_violations
from holo_fleet.ha.spec import Mode
from holo_fleet.mission import MissionPlan
from holo_fleet.perception.frame import SensorFrame
from holo_fleet.perception.perception import LocalObservation, Perception

AVOIDANCE = (Mode.SEPARATION_WARNING, Mode.COLLISION_AVOIDANCE)


class EnvelopeMonitor:
    """Onboard envelope status, ENVELOPE_OK / ENVELOPE_VIOLATION, from the control-feasibility of the
    velocity the drone is asked to deliver (DI-27; control/current_envelope.py).

    A control step is out of envelope when the low level cannot deliver the requested velocity: its
    horizontal command is saturated, or the steady command it needs against the estimated current
    (``check_onboard``: |v_d - w_est| / s, the integral action of the implemented loop) exceeds the
    authority of the mode.  Both depend on the direction of the current relative to the requested motion
    and on the requested speed, not on |w| alone.  The violation is declared after t_enter of such steps
    (leaky) and cleared after t_exit without; avoidance manoeuvres are not judged (higher authority,
    transients).  Saturation is kept as direct evidence: the integrator freezes while saturated, so the
    estimate alone could under-report a current stronger than the drone."""

    def __init__(self, t_enter: float = 4.0, t_exit: float = 4.0):
        self.persist = Persistence(t_enter, t_exit)
        self.check: Optional[CurrentCheck] = None
        self.saturated = False
        self.reason = ""

    @property
    def ok(self) -> bool:
        return self.persist.ok

    def update(self, saturated: bool, check: Optional[CurrentCheck], dt: float) -> bool:
        """``check`` is None while the step is not judged (avoidance manoeuvres)."""
        self.check, self.saturated = check, bool(saturated and check is not None)
        bad = check is not None and (saturated or not check.ok)
        if check is not None and not check.ok:
            self.reason = check.reason
        elif self.saturated:
            self.reason = (f"thrust saturated (steady command {check.required_cmd:.2f} of {check.authority:.2f}; "
                           f"head {check.head:+.2f}, lateral {check.lateral:.2f} m/s estimated)")
        elif self.persist.ok:
            self.reason = ""
        return self.persist.update(bad, dt)

    def record(self) -> Dict[str, Any]:
        """Per-step log entry: estimated current, requested velocity, components, required vs available."""
        c = self.check
        out: Dict[str, Any] = {"status": "ENVELOPE_OK" if self.ok else "ENVELOPE_VIOLATION",
                               "judged": c is not None, "saturated": self.saturated}
        if c is not None:
            out.update({"current_est": [round(float(x), 3) for x in c.current],
                        "v_desired": [round(float(x), 3) for x in c.desired],
                        "head": round(c.head, 3), "lateral": round(c.lateral, 3), "vertical": round(c.vertical, 3),
                        "through_water": round(c.through_water, 3), "required_cmd": round(c.required_cmd, 3),
                        "authority": c.authority, "feasible": c.feasible})
        if self.reason:
            out["reason"] = self.reason
        return out


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
        self.observation_violations = 0            # control steps with an inconsistent abstract observation
        self.obs_consistent = True
        self.events: List[Dict[str, Any]] = []
        self.last_obs: Optional[LocalObservation] = None
        self.last_record: Dict[str, Any] = {}

    def step(self, frame: SensorFrame, dt: float) -> np.ndarray:
        t = frame.t
        obs = self.perception.update(frame, dt, sigma=self.flows.sigma, env_ok=self.envmon.ok,
                                     offset=self.flows.offset)
        # observation consistency check (ha/observation_invariants.py) between perception and automaton:
        # an observation the perception cannot produce reveals a defect; it is logged and handed to the
        # automaton as a sensing fault, i.e. the existing fault edge -> FAILSAFE_HOLD_OR_RETREAT
        bad = observation_violations(obs.ab, self.cfg)
        self.obs_consistent = not bad
        if bad:
            self.observation_violations += 1
            self.events.append({"t": t, "type": "OBSERVATION_INCONSISTENT", "violated": bad,
                                "observation": {**obs.ab.as_dict(), "committed": self.ha.committed},
                                "mode": self.ha.mode.value})
            obs.ab.sense_ok = False
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
        # envelope: can the low level deliver the requested velocity against the (estimated) current?
        judged = mode not in AVOIDANCE
        check = check_onboard(self.ll.current_estimate(), v, self.cfg, AUTHORITY[authority]) if judged else None
        was_ok = self.envmon.ok
        ok = self.envmon.update(self.ll.saturated, check, dt)
        if was_ok and not ok:
            self.events.append({"t": t, "type": "ENVELOPE_VIOLATION", **self.envmon.record()})
        elif ok and not was_ok:
            self.events.append({"t": t, "type": "ENVELOPE_OK", **self.envmon.record()})
        self.last_obs = obs
        ch = self.flows.last_choice
        self.last_record = {
            "t": round(t, 3), "mode": mode.value, "committed": self.ha.committed,
            "nav_p": np.round(obs.p, 3).tolist(), "nav_yaw_deg": round(float(np.degrees(obs.yaw)), 1),
            "v_cmd": np.round(v, 3).tolist(), "giveway": self.flows.giveway,
            "offset": np.round(self.flows.offset, 2).tolist(), "d_min": round(obs.d_min, 3) if obs.d_min < 1e8 else None,
            "sense_ok": obs.sense_ok, "env_ok": self.envmon.ok, "v_mis": np.round(v_mis, 3).tolist(),
            "envelope": self.envmon.record(),
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
            "obs_consistent": self.obs_consistent, "observation_violations": self.observation_violations,
            "saturated": self.ll.saturated,
        }
        return thruster_command(cmd["surge"], cmd["sway"], cmd["heave"], cmd["yaw"], self.cfg.plant.thruster_limit)
