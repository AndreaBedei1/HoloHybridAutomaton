"""One drone's controller (v2): perception -> hybrid automaton -> mode flow -> low-level -> thrusters.

Identical code on every drone; inputs are only the drone's own SensorFrame (six sonars, DVL,
IMU, compass, depth) and its mission plan.  ``uses_ground_truth`` is checked by the tests.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

import numpy as np

from holo_fleet.arena_bridge import thruster_command
from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.control.current_envelope import CurrentCheck, Persistence, check_onboard
from holo_fleet.control.flows import Flows
from holo_fleet.control.lowlevel import AUTHORITY, LowLevelController
from holo_fleet.ha.automaton import LocalHybridAutomaton
from holo_fleet.ha.observation_invariants import violated as observation_violations
from holo_fleet.ha.spec import IN_FORMATION, Mode
from holo_fleet.mission import MissionPlan
from holo_fleet.perception.frame import SensorFrame
from holo_fleet.perception.passage import PassageVerifier, gate_report
from holo_fleet.perception.sonar_geometry import HULL_HALF
from holo_fleet.perception.perception import LocalObservation, Perception

AVOIDANCE = (Mode.SEPARATION_WARNING, Mode.COLLISION_AVOIDANCE)


class EnvelopeMonitor:
    """Onboard envelope status, ENVELOPE_OK / ENVELOPE_VIOLATION, from the control-feasibility of the
    velocity the drone is asked to deliver (control/current_envelope.py).

    A control step is out of envelope when the low level cannot deliver the requested velocity: its
    horizontal command is saturated, or the steady command it needs against the estimated current
    (``check_onboard``: |v_d - w_est| / s, the integral action of the implemented loop) exceeds the
    authority of the mode.  Both depend on the direction of the current relative to the requested motion
    and on the requested speed, not on |w| alone.  The violation is declared after t_enter of such steps
    (leaky) and cleared after t_exit without; avoidance manoeuvres are not judged (higher authority,
    transients).  Saturation is kept as direct evidence: the integrator freezes while saturated, so the
    estimate alone could under-report a current stronger than the drone.

    A declared violation is cleared only after t_exit of steps in which the vehicle DELIVERS a velocity it
    requests (DVL navigation): a vehicle whose thrusters do not respond (a thruster failure) holding
    still in FAILSAFE is neither saturated nor infeasible, and requesting nothing it cannot be told from a
    healthy one.  So while the violation lasts the FAILSAFE flow adds a small heave probe (Flows.failsafe),
    and a step counts toward the clear only when, over the last ``window_s``, the vehicle delivered the
    motion it requested: projection gain sum(v_nav . v_req) / sum(|v_req|^2) >= ``gain_min`` (about 0 for a
    vehicle that does not respond, about 0.7-1 for a healthy one: over half a period of the probe the gain
    of a lagging response is cos(lag) times its amplitude ratio, whatever the window phase).  A window that
    requests motion and does not get it restarts the count; one that requests (almost) nothing (RMS below
    ``v_probe_min``) pauses it; without velocity data the rule is not applied."""

    def __init__(self, t_enter: float = 4.0, t_exit: float = 4.0, v_probe_min: float = 0.04,
                 gain_min: float = 0.5, window_s: float = 4.0):
        self.persist = Persistence(t_enter, t_exit)
        self.v_probe_min, self.gain_min, self.window_s = v_probe_min, gain_min, window_s
        self.window: List = []
        self.gain: Optional[float] = None
        self.check: Optional[CurrentCheck] = None
        self.saturated = False
        self.delivering = True
        self.reason = ""

    @property
    def ok(self) -> bool:
        return self.persist.ok

    def update(self, saturated: bool, check: Optional[CurrentCheck], dt: float, v_nav=None, v_req=None) -> bool:
        """``check`` is None while the step is not judged (avoidance manoeuvres); ``v_nav`` is the navigation
        velocity now and ``v_req`` the velocity requested at the previous step."""
        self.check, self.saturated = check, bool(saturated and check is not None)
        if v_nav is not None and v_req is not None:
            self.window.append((np.asarray(v_nav, float), np.asarray(v_req, float)))
            self.window = self.window[-max(1, int(round(self.window_s / dt))):]
        rr = sum(float(r @ r) for _n, r in self.window)
        rms = math.sqrt(rr / len(self.window)) if self.window else 0.0
        self.gain = (sum(float(n @ r) for n, r in self.window) / rr) if rr > 1e-9 else None
        self.delivering = self.gain is None or self.gain >= self.gain_min
        bad = check is not None and (saturated or not check.ok)
        hold = None
        if not self.persist.ok and not bad and self.window:      # in violation: clearing needs delivered motion
            if rms < self.v_probe_min:
                hold = "pause"
            elif not self.delivering:
                hold = "reset"
        if hold == "reset":
            self.reason = (f"requested motion not delivered (gain {self.gain:.2f} over {self.window_s:.0f} s, "
                           f"RMS request {rms:.2f} m/s)")
        elif check is not None and not check.ok:
            self.reason = check.reason
        elif self.saturated:
            self.reason = (f"thrust saturated (steady command {check.required_cmd:.2f} of {check.authority:.2f}; "
                           f"head {check.head:+.2f}, lateral {check.lateral:.2f} m/s estimated)")
        elif self.persist.ok:
            self.reason = ""
        was_ok = self.persist.ok
        ok = self.persist.update(bad, dt, hold=hold)
        if ok and not was_ok:
            self.reason = ""                            # cleared: the record of ENVELOPE_OK carries no stale reason
        return ok

    def record(self) -> Dict[str, Any]:
        """Per-step log entry: estimated current, requested velocity, components, required vs available."""
        c = self.check
        out: Dict[str, Any] = {"status": "ENVELOPE_OK" if self.ok else "ENVELOPE_VIOLATION",
                               "judged": c is not None, "saturated": self.saturated, "delivering": self.delivering,
                               "delivery_gain": None if self.gain is None else round(self.gain, 2)}
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
        self.passage = PassageVerifier(cfg)        # what the drone reports about its gate passage (payload v2)
        self.ll = LowLevelController(cfg)
        self.envmon = EnvelopeMonitor()
        self.observation_violations = 0            # control steps with an inconsistent abstract observation
        self.obs_consistent = True
        self.events: List[Dict[str, Any]] = []
        self.last_obs: Optional[LocalObservation] = None
        self.last_record: Dict[str, Any] = {}
        self.v_requested = np.zeros(3)             # velocity requested from the low level at the previous step
        self.last_transition: Optional[Dict[str, Any]] = None     # last mode change (transition log)

    def step(self, frame: SensorFrame, dt: float) -> np.ndarray:
        t = frame.t
        ftime = self.flows.ftime                               # own formation time (reduced progress)
        obs = self.perception.update(frame, dt, sigma=self.flows.sigma, env_ok=self.envmon.ok,
                                     offset=self.flows.offset + self.flows.lat_offset, in_formation=self.ha.mode in IN_FORMATION,
                                     t_form=ftime.now(t),
                                     v_req=self.v_requested if (self.envmon.ok and not self.ll.saturated) else None)
        for e in self.perception.events:                       # SLOT_DECLARED_VACANT / SLOT_REOCCUPIED
            self.events.append({**e, "mode": self.ha.mode.value})
        self.perception.events.clear()
        # observation consistency check (ha/observation_invariants.py) between perception and automaton:
        # an observation the perception cannot produce reveals a defect; it is logged and handed to the
        # automaton as a sensing fault, i.e. the existing fault edge -> FAILSAFE_HOLD_OR_RETREAT
        # the stage-2 give-up request of the previous control step (flows.gate), only while committed (I5)
        obs.ab.committed = self.ha.committed
        obs.ab.entry_abort = bool(self.flows.abort_request and self.ha.committed and obs.ab.mutex_zone
                                  and not obs.ab.passed)
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
        if rec.target != rec.source:                           # transition log: every mode change, with its cause
            self.last_transition = {"t": round(t, 2), "from": rec.source, "to": rec.target, "edge": rec.edge,
                                    "reason": rec.reason, "guard": rec.label}
            self.events.append({"type": "transition", **self.last_transition})
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
        gate_flow = obs.gate.in_zone or self.ha.committed
        if gate_flow:
            gmode = mode.value if mode in (Mode.MUTEX_APPROACH, Mode.MUTEX_YIELD, Mode.MUTEX_PASS) else (
                "MUTEX_PASS" if self.ha.committed else "MUTEX_APPROACH")
            v_mis, yaw_d = self.flows.gate(obs, gmode)
        else:
            v_mis, yaw_d = self.flows.formation(obs, dt, recovering=(mode == Mode.FORMATION_RECOVERY),
                                                traffic=(mode != Mode.FAILSAFE_HOLD_OR_RETREAT))
        if not self.ha.committed:
            v_mis = self.flows.keepout_filter(obs, v_mis)   # the mission of a drone not committed never heads for the CR
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
            v = self.flows.failsafe(obs, probe=not self.envmon.ok)
            authority = "brake"
        else:
            v = v_mis
        v = self.flows.structure_filter(obs, v)
        from holo_fleet.perception.nav import NavBound
        dvl_fresh = t - self.perception.nav.state.t_dvl <= NavBound.DVL_STALE_S
        cmd = self.ll.command(self.perception.nav.state, v, yaw_d, authority, dt, dvl_fresh=dvl_fresh)
        # envelope: can the low level deliver the requested velocity against the (estimated) current?
        judged = mode not in AVOIDANCE
        check = check_onboard(self.ll.current_estimate(), v, self.cfg, AUTHORITY[authority]) if judged else None
        was_ok = self.envmon.ok
        # during a DVL outage the navigation velocity is the model's: not evidence of delivery (no circularity)
        ok = self.envmon.update(self.ll.saturated, check, dt, v_nav=obs.v_world if dvl_fresh else None,
                                v_req=self.v_requested)
        self.v_requested = np.asarray(v, dtype=float).copy()
        self._update_gate_report(t, dt, obs)
        for e in self.perception.gp.phantom_events:            # B2: corridor echoes no fleet drone produced
            self.events.append({"t": e["t"], "type": "PHANTOM_ECHO", "excluding_fixes": e["fixes"]})
        self.perception.gp.phantom_events.clear()
        for e in self.flows.events:                            # ENTRY_HOLD_TIMEOUT (stage 2, flows.gate)
            self.events.append({**e, "mode": mode.value})
        self.flows.events.clear()
        if was_ok and not ok:
            self.events.append({"t": t, "type": "ENVELOPE_VIOLATION", **self.envmon.record()})
        elif ok and not was_ok:
            self.events.append({"t": t, "type": "ENVELOPE_OK", **self.envmon.record()})
        # own formation time for the next step: reduced progress while an expected neighbour is missing; a drone that
        # started an episode out of formation keeping and is still away from its slot (it is the one missed) keeps
        # it until it is back near its slot
        incomplete = not obs.form.neighbors_ok or (mode not in IN_FORMATION and ftime.episode_t > 0.0
                                                   and obs.form.slot_err_norm > self.cfg.form.e_ok)
        ftime.update(t, dt, incomplete=incomplete, mutex=gate_flow, front_residual=obs.form.front_residual,
                     keeping=mode in IN_FORMATION, behind=obs.form.behind)
        self.last_obs = obs
        ch = self.flows.last_choice
        self.last_record = {
            "t": round(t, 3), "mode": mode.value, "committed": self.ha.committed,
            "nav_p": np.round(obs.p, 3).tolist(), "nav_yaw_deg": round(float(np.degrees(obs.yaw)), 1),
            "nav_bound": round(float(self.perception.nav_bound.value), 3),
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
                     "occ": obs.gate.occ_state, "exit_witness": obs.gate.exit_witness, "relations": obs.gate.relations,
                     "rank_used": obs.gate.rank_used,
                     "loc_ok": obs.gate.loc_ok, "reloc_hold": self.flows.reloc_hold,
                     "entry_ok": obs.gate.entry_ok, "entry": self.flows.entry_state or None,
                     "tx": [self.perception.tx_gate_idx, self.perception.tx_phase],
                     **({"acoustic": {k: v for k, v in self.perception.gp.acoustic_exit.last.items() if v}}
                        if self.perception.gp.acoustic_exit is not None and obs.gate.occ_state == "BUSY" else {})},
            "form": {"form_err": round(obs.form.form_err, 3), "slot_err": round(obs.form.slot_err_norm, 3),
                     "neighbors_ok": obs.form.neighbors_ok, "sigma": round(self.flows.sigma, 3), "lam": round(self.flows.lam, 3),
                     "checks": [(c.slot, round(c.expected_d, 2), None if not c.seen else round(c.residual, 2))
                                for c in obs.form.checks],
                     "missing": sorted(k for k in obs.form.missing if k not in self.perception.vacant),
                     "missing_t": {k: round(v, 1) for k, v in self.perception.missing_t.items() if v > 0.0},
                     "vacant": sorted(self.perception.vacant), "degraded": bool(self.perception.vacant),
                     "slot": np.round(obs.slot_pos, 2).tolist(), "rejoin": self.flows.rejoin_phase,
                     **ftime.record(t, dt),
                     **({"acoustic_present": sorted(self.perception.acoustic_present),
                         "acoustic_displaced": sorted(self.perception.acoustic_displaced),
                         "acoustic_corr": sorted(self.perception.acoustic_corr),
                         "acoustic_fixed": sorted(k for k in range(len(self.perception.plan.slots))
                                                  if self.perception.acoustic.latest(k, t, self.cfg.acoustic.fresh_s))}
                        if self.perception.acoustic.enabled else {})},
            "nav_aid": None if self.perception.aiding.last is None else {**self.perception.aiding.last,
                                                                         "applied_m": round(self.perception.aiding.applied_m, 3)},
            "transition": self.last_transition,
            "determinism_violations": self.ha.determinism_violations,
            "obs_consistent": self.obs_consistent, "observation_violations": self.observation_violations,
            "saturated": self.ll.saturated,
        }
        return thruster_command(cmd["surge"], cmd["sway"], cmd["heave"], cmd["yaw"], self.cfg.plant.thruster_limit)

    def _apply_crossing(self, t: float, gate, p_est) -> None:
        """A detected crossing is a direct measurement of the position in the opening: the estimate is corrected to it (the
        navigation bound keeps growing, as with every correction; it never decreases)."""
        l_meas, dz = self.passage.crossing
        _s_est, l_est, _dz_est = (float(v) for v in gate.to_gate_frame(np.asarray(p_est, float)))
        axis = np.asarray(gate.axis, float)
        left = np.array([-axis[1], axis[0], 0.0])
        # the LATERAL position only: the along-axis place of the crossing is known to +-0.4 m at best and a jump of s moves
        # the estimate past the post the drone is beside (its echo is then unexplained: gate_single s902 in the first batch)
        if abs(l_meas - l_est) > 1.2:
            return
        corr = (l_meas - l_est) * left
        self.perception.apply_position_correction(corr)
        self.events.append({"t": t, "type": "GATE_CROSSING_MEASURED", "gate": self.passage.idx, "l": round(l_meas, 3),
                            "dz": None if dz is None else round(dz, 3), "correction": np.round(corr, 3).tolist()})

    def _update_gate_report(self, t: float, dt: float, obs: LocalObservation) -> None:
        """Gate phase this drone reports about itself (payload v2): the facts of its own automaton and flow, never a command
        or a request.  It stays on the gate of an unfinished passage until that passage is verified."""
        p, gp = self.passage, self.perception.gp
        if p.idx != gp.idx and not p.in_progress and (not p.verified or self.ha.committed):
            p.next_gate(gp.idx)      # a verified passage keeps being reported until the drone engages the next gate
        gate = self.plan.gates[p.idx] if p.idx < len(self.plan.gates) else None
        if self.flows.cr_cleared and self.ha.committed and gate is not None and not p.admitted:
            p.admit(p.idx, gate, obs.p)
        sectors = {sec: [float(e.r0) for e in obs.readings[sec].echoes] for sec in ("LEFT", "RIGHT", "UP", "DOWN", "REAR")
                   if sec in obs.readings}
        was = p.verified
        crossed = p.crossing is not None
        from holo_fleet.perception.nav import NavBound

        p.update(t, dt, obs.p, obs.v_world, sectors, dvl_fresh=t - self.perception.nav.state.t_dvl <= NavBound.DVL_STALE_S)
        if p.crossing is not None and not crossed and gate is not None:
            self._apply_crossing(t, gate, obs.p)
        if p.verified and not was:
            self.events.append({"t": t, "type": "GATE_PASSAGE_VERIFIED", "by": p.by, "gate": p.idx})
        self.perception.tx_gate_idx, self.perception.tx_phase = gate_report(p, obs.gate, obs.p, self.ha.committed, gp.idx, self.cfg.gate)
