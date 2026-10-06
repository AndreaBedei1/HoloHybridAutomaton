"""DroneController: perception -> local hybrid automaton -> flow -> low-level command.

Every drone runs an identical instance.  The only per-drone inputs are its
MissionPlan (own slot, own launch pose, pre-assigned static rank/layer) and its
own onboard SensorFrame.  There is no access to simulator state and, by
default, no inter-agent message of any kind.
"""

from __future__ import annotations

import math
from dataclasses import asdict
from typing import Any, Dict, List, Optional

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.control.flows import FlowOutput, Flows, structure_filter
from holo_fleet.control.lowlevel import LowLevelController
from holo_fleet.ha.automaton import LocalHybridAutomaton
from holo_fleet.ha.spec import Mode
from holo_fleet.mission import MissionPlan
from holo_fleet.perception.perception import LocalObservation, Perception, SensorFrame

MISSION_MODES = {Mode.FORMATION_FOLLOW, Mode.FORMATION_RECOVERY, Mode.GATE_APPROACH, Mode.GATE_YIELD, Mode.GATE_PASS}


def _jsonable(x: Any) -> Any:
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating,)):
        return float(x)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.bool_,)):
        return bool(x)
    return x


class DroneController:
    uses_ground_truth = False

    def __init__(self, plan: MissionPlan, cfg: FleetConfig = DEFAULT, nav_init_offset: Optional[np.ndarray] = None,
                 comms_inbox: Optional[Any] = None):
        self.plan = plan
        self.cfg = cfg
        self.perception = Perception(plan, cfg, nav_init_offset)
        self.ha = LocalHybridAutomaton(cfg)
        self.flows = Flows(plan, cfg)
        self.low = LowLevelController(cfg)
        self.mission_mode = Mode.FORMATION_FOLLOW
        self.comms_inbox = comms_inbox            # optional, None by default (no communication)
        self._latest_msgs: Dict[str, Dict[str, Any]] = {}
        self.events: List[Dict[str, Any]] = []
        self.mission_complete = False

    def heartbeat(self) -> Dict[str, Any]:
        """Payload for the optional channel: own onboard estimate only."""
        return {"nav_p": self.perception.nav.state.p.round(3).tolist(), "mode": self.ha.mode.value,
                "slot": self.plan.slot_index}

    def _formation_hints(self, t: float, local) -> Dict[int, List[float]]:
        """Path-frame relative offsets of slots that sensing does not currently see, from received
        heartbeats (<= 3 s old).  Empty when communication is off (the default)."""
        if self.comms_inbox is None:
            return {}
        for msg in self.comms_inbox:
            self._latest_msgs[msg["sender"]] = msg
        self.comms_inbox.clear()
        hints: Dict[int, List[float]] = {}
        p = self.perception.nav.state.p
        s_i, _, _ = self.plan.path.project(p[:2])
        _, tan, nrm = self.plan.path.frame_at(s_i)
        for msg in self._latest_msgs.values():
            k = int(msg["slot"])
            if k == self.plan.slot_index or k in local.formation.assigned or t - msg["t_tx"] > 3.0:
                continue
            rel = np.asarray(msg["nav_p"]) - p
            exp_s = self.plan.slots[k].along - self.plan.my_slot.along
            exp_l = self.plan.slots[k].lateral - self.plan.my_slot.lateral
            hints[k] = [float(rel[:2] @ tan - exp_s), float(rel[:2] @ nrm - exp_l)]
        return hints

    def step(self, frame: SensorFrame, dt: float) -> Dict[str, Any]:
        t = frame.t
        local, abstract = self.perception.update(frame, dt)
        rec = self.ha.step(abstract, t, dt)
        mode = self.ha.mode
        gate = self.perception.current_gate()

        events: List[Dict[str, Any]] = []
        if rec.decision:
            events.append({"t": t, "drone": self.plan.drone_id, "type": "decision", "decision": rec.decision,
                           "edge": rec.edge, "from": rec.source, "to": rec.target,
                           "gate": local.gate.gate_id, "gate_frame": local.gate.own_gate_frame})
        elif rec.source != rec.target:
            events.append({"t": t, "drone": self.plan.drone_id, "type": "mode_change", "edge": rec.edge,
                           "from": rec.source, "to": rec.target})
        if rec.edge == "pass_done":
            self.perception.advance_gate()
        elif (gate is not None and not self.ha.committed and local.gate.passed):
            events.append({"t": t, "drone": self.plan.drone_id, "type": "gate_skipped_uncommitted",
                           "gate": gate.gate_id})
            self.perception.advance_gate()
        if mode in MISSION_MODES:
            self.mission_mode = mode

        nav = self.perception.nav.state
        p = nav.p.copy()
        self.flows.comms_hints = self._formation_hints(t, local)   # {} unless the optional channel is on
        flow = self._flow(mode, local, p, nav.yaw)
        v_f, active = structure_filter(flow.v, local.structure_close, self.cfg)
        if active:
            flow = FlowOutput(v=v_f, yaw_d=flow.yaw_d, authority=flow.authority, note=flow.note + " [structure filter]")
        cmd = self.low.command(nav, flow.v, flow.yaw_d, flow.authority, dt)
        track_err = float(np.linalg.norm(flow.v[:2] - nav.v_world[:2]))
        env_before = self.perception.env_ok_flag
        self.perception.report_tracking_error(track_err, self.low.saturated, dt)
        if env_before != self.perception.env_ok_flag:
            events.append({"t": t, "drone": self.plan.drone_id,
                           "type": "ENVELOPE_VIOLATION" if env_before else "ENVELOPE_RESTORED",
                           "tracking_error": round(track_err, 3), "authority": flow.authority})

        s_path, _, _ = self.plan.path.project(p[:2])
        if not self.mission_complete and s_path - self.plan.my_slot.along >= self.plan.s_end \
                and self.perception.gate_index >= len(self.plan.gates):
            self.mission_complete = True
            events.append({"t": t, "drone": self.plan.drone_id, "type": "survey_line_complete"})
        self.events.extend(events)
        return {
            "command": cmd,
            "mode": mode.value,
            "committed": self.ha.committed,
            "transition": asdict(rec),
            "flow": {"v_d": flow.v.round(3).tolist(), "yaw_d_deg": round(math.degrees(flow.yaw_d), 2),
                     "authority": flow.authority, "note": flow.note},
            "observation": _jsonable(asdict(local)),
            "abstract": abstract.as_dict(),
            "events": events,
            "gate_index": self.perception.gate_index,
        }

    def _flow(self, mode: Mode, obs: LocalObservation, p: np.ndarray, yaw: float) -> FlowOutput:
        gate = self.perception.current_gate()
        if mode == Mode.FAILSAFE_HOLD_OR_RETREAT:
            return self.flows.failsafe(obs, p, yaw)
        if mode == Mode.COLLISION_AVOIDANCE:
            return self.flows.collision_avoidance(obs, p, yaw)
        base_mode = self.mission_mode if mode == Mode.SEPARATION_WARNING else mode
        base = self._mission_flow(base_mode, obs, p, gate)
        if mode == Mode.SEPARATION_WARNING:
            return self.flows.separation_warning(base, obs)
        return base

    def _mission_flow(self, mode: Mode, obs: LocalObservation, p: np.ndarray, gate) -> FlowOutput:
        if mode == Mode.GATE_PASS and gate is not None:
            return self.flows.gate_pass(obs, gate, p)
        if mode in (Mode.GATE_APPROACH, Mode.GATE_YIELD) and gate is not None:
            return self.flows.gate_queue(obs, gate, p, yielding=(mode == Mode.GATE_YIELD))
        return self.flows.formation(obs, mode, p, self.perception.gate_index)
