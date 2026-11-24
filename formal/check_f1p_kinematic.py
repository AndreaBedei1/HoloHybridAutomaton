"""F1pk: kinematic reconstruction of the F1p mechanism with the deployed decision rules (reported, not a proof).

F1p / F1pm (formal/check_fleet.py) found two drones in the CR when two drones commit concurrently (a claim learnt late) and
one of them is moved into the CR by its own avoidance.  The fleet model has no positions.  This reconstruction gives it
positions and moving drones, and runs every episode with two controllers:

* NEAR_HOLD: the MUTATION that is the controller before the hold-point correction - the stage-2 hold hull half-length + 0.15
  before the CR entry plane, s = cr_s_min - (0.229 + 0.15) = -1.43 m with the present CR (-1.18 m with the former symmetric
  CR |s| <= 0.8), with the RELOC back-off (that Flows.gate, reproduced verbatim in FlowsNearHold), the queue lines of that
  hold, no follower standoff, and the avoidance output of a drone that is not admitted never edited (the escape planner
  without ``p2``, no P2 mission filter);
* CORRECTED: the current code - Flows.gate (stage-2 hold GateRule.stage2_hold_s, acquisition offsets, standoff), the P2
  mission filter and the gate-aware warning filter / escape (control/gate_safety.py, EscapePlanner with ``p2``), in the
  order of DroneController.step.

Every drone runs, each step of dt = 0.1 s: its mission flow (Flows.gate in MUTEX_APPROACH towards its queue point,
MUTEX_PASS once committed; a drone in transit or one that has passed has a constant-speed mission velocity towards its
goal), Flows.keepout_filter while not committed, then (CORRECTED) the P2 mission filter, the automaton's separation guards
with their hysteresis on the smallest conservative distance (d_ca / d_ca_exit, d_warning / d_warning_exit) and the
separation warning filter or the escape.  Scenarios: (A) concurrent commitments of 2-3 drones at the real queue points with
claim delays 0.5-12 s; (B) a drone inside the CR (moving through, or stalled for 8 s), a second one held at the stage-2
hold by the first's claim, and a THIRD drone approaching the held one from behind along the axis - cooperative (the
deployed avoidance, uncommitted: keep-out and P2 filters) or non-reacting (a vehicle outside the fleet premise, constant
speed) - with a restricted corridor (static hulls beside the hold; the gate 0.9 m below the depth limit so that up escapes
are penalised), a navigation bias of the held drone (estimate - truth along the axis, -1.0 to +0.5 m) and a residual drift
towards the CR.

The claim knowledge (``delay`` s after the TRUE commitment) is idealised: the explicit channel and the local decisions are
formal/check_b2_comm.py, whose commitment-age rule makes a fresh report of an earlier commitment a claim (B2C-E1).
Idealised here (explicitly, not deployed): perception - targets from the TRUE relative positions (sonar_geometry
pattern_of_point and centre_distance_lower: no noise, no missed echo, no stale or unhealthy sector), every heading along
the gate axis; the stage-2 check - entry_ok false while a claim of another committed drone is known (``delay`` s after it
commits) or another centre is in the CR (an occupant always seen); loc_ok always true (a validated localization assumed
whatever the bias: the worst case for P2, the estimate is wrong and admitted); commit times scripted, not produced by the
commit guard; the plant - a velocity loop changing by at most a_brake per second, plus a constant residual drift; the
navigation estimate = truth + a constant bias, the measured velocity = true ground velocity (the P2 envelope adds the
DVL noise bound of a fresh DVL, NavBound.k_rw, and the drift bound of the estimate k_rw + k_lin v_bound); no current estimate in the
escape; no gate bars (no structure filter).  Counted per episode: entries of a true centre into the CR without the stage-2
admission (committed and cr_cleared), those within 3 s of an avoidance mode (moved in by avoidance), instants with two
fleet centres in the CR (a non-reacting vehicle is outside the fleet: its own passages through the CR are counted apart),
the steps whose applied command carries no P2 certificate (Flows.p2_certify; gate_safety), the smallest centre distance of each pair and P1 violations (d < d_safe; between two reacting fleet drones, and with the
non-reacting vehicle), and the joint-feasibility classes of the gate-aware decisions (CORRECTED).  Evidence over a finite set of episodes only.
"""

from __future__ import annotations

import dataclasses
import itertools
import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace as NS
from typing import Dict, List, Optional, Sequence

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from holo_fleet.config import DEFAULT, FleetConfig  # noqa: E402
from holo_fleet.control import gate_safety as gs  # noqa: E402
from holo_fleet.control.flows import RELOC_MARGIN_M, RELOC_ZONE_M, Flows, _cap  # noqa: E402
from holo_fleet.mission import GateSpec  # noqa: E402
from holo_fleet.perception.nav import NavBound  # noqa: E402
from holo_fleet.perception.sonar_geometry import HULL_HALF, MOUNTS, R_IN, R_OUT, centre_distance_lower, pattern_of_point  # noqa: E402

DT = 0.1
NEAR_HOLD_S = DEFAULT.gate.cr_s_min - (float(HULL_HALF[0]) + RELOC_MARGIN_M)
NEAR_HOLD_RELOC_PERIOD_S, NEAR_HOLD_RELOC_BACK_M = 6.0, 0.6
RESULTS = Path(__file__).resolve().parent / "results" / "f1p_kinematic.json"
AVOID = ("SW", "CA")
FLEET = ("GATE", "TRANSIT")


class FlowsNearHold(Flows):
    """The gate flow of the NEAR_HOLD mutation (the controller before the hold-point correction): stage-2 hold NEAR_HOLD_S
    (0.379 m before the CR entry plane; -1.18 m with the former CR) with the RELOC back-off, no standoff."""

    def _ahead(self, obs) -> bool:
        return False

    def gate(self, obs, mode: str):
        self.passing = self.at_level = False
        G = self.cfg.gate
        g = self.plan.gates[min(obs.gate_index, len(self.plan.gates) - 1)]
        go = obs.gate
        yaw = math.atan2(g.axis[1], g.axis[0])
        hold = False
        if mode == "MUTEX_PASS":
            c = self._carrot(self.pass_path(go.queue_s, go.queue_l, go.queue_dz), np.array([go.s, go.l, go.dz]))
            s_hold = NEAR_HOLD_S
            if go.loc_ok and go.entry_ok and go.s >= s_hold - RELOC_ZONE_M:
                self.cr_cleared = True
            s_abort = G.cr_s_min - (float(R_OUT) + G.nav_err_m)
            self.abort_request = False
            was = self.past_centre
            self.past_centre = bool(go.s > G.cr_s_min + G.nav_err_m or (was and go.s > G.cr_s_min))
            if not self.cr_cleared and not self.past_centre and (c[0] > s_hold or self.hold_timeout):
                self.hold_since = obs.t if self.hold_since is None else self.hold_since
                if obs.t - self.hold_since > G.t_occ_max + G.t_clear and not self.hold_timeout:
                    self.hold_timeout = True
                back = int((obs.t - self.hold_since) / NEAR_HOLD_RELOC_PERIOD_S) % 2 == 1
                c = np.array([s_hold - (NEAR_HOLD_RELOC_BACK_M if back else 0.0), 0.0, c[2]])
                if self.hold_timeout:
                    c = np.array([s_abort - RELOC_MARGIN_M, 0.0, c[2]])
                    self.abort_request = go.s <= s_abort
                hold = True
            self.entry_state = "ADMIT" if self.cr_cleared else ("HOLD" if hold else "")
            tgt = g.from_gate_frame(c)
            vmax = G.v_pass
        else:
            self.cr_cleared, self.hold_since, self.entry_state, self.hold_timeout, self.past_centre = False, None, "", False, False
            self.abort_request = False
            tgt = g.from_gate_frame([go.queue_s, go.queue_l, go.queue_dz])
            vmax = G.v_approach
        self.reloc_hold = hold
        v = G.k_track * (tgt - obs.p)
        if mode == "MUTEX_PASS" and not hold:
            n = float(np.linalg.norm(v[:2]))
            if n > 1e-6:
                v[:2] *= vmax / n
        return _cap(v, vmax, 0.25), yaw


def near_hold_config(cfg: FleetConfig = DEFAULT) -> FleetConfig:
    """The queue lines of the NEAR_HOLD mutation: the queue bound (c) of the stage-2 hold evaluated at its hold point."""
    return dataclasses.replace(cfg, gate=dataclasses.replace(cfg.gate, stage2_hold_s=NEAR_HOLD_S))


def make_gate(z_centre: float = -4.3) -> GateSpec:
    return GateSpec("K", center=np.array([0.0, 0.0, z_centre]), axis=np.array([1.0, 0.0, 0.0]),
                    right=np.array([0.0, -1.0, 0.0]), inner_width=2.0, inner_height=2.0, bars=())


def in_cr(cfg: FleetConfig, gate, p) -> bool:
    s, l, dz = gate.to_gate_frame(np.asarray(p, float))
    return cfg.gate.in_cr(s, l, dz)


def target(pi: np.ndarray, pj: np.ndarray):
    """Idealised sonar target of a hull at pj seen from pi (heading along the axis): pattern and conservative distance."""
    rel = np.asarray(pj, float) - np.asarray(pi, float)
    pat = pattern_of_point(rel, R_IN)
    if not pat:
        return None
    dl = max(centre_distance_lower(max(float(np.linalg.norm(rel - MOUNTS[s])) - R_IN, 0.0), s) for s in pat)
    return NS(cls="DYNAMIC", pattern=frozenset(pat), d_lower=float(dl), r_min=float(np.linalg.norm(rel)), closing_rate=0.0,
              possible_neighbour=False)


@dataclass
class Agent:
    """One hull.  kind: GATE (the gate protocol), TRANSIT (uncommitted, constant-speed mission towards ``goal`` with the
    deployed separation layer), SCRIPTED (constant velocity, no reaction), STATIC."""

    name: str
    kind: str
    p: np.ndarray
    bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    queue: tuple = (0.0, 0.0, 0.0)
    commit_t: float = math.inf
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    goal: Optional[np.ndarray] = None
    speed: float = 0.30
    committed: bool = False
    admitted0: bool = False
    stall: tuple = ()
    v: np.ndarray = field(default_factory=lambda: np.zeros(3))
    mode: str = "MUTEX"
    done: bool = False
    flows: Optional[Flows] = None
    avoid_t: float = -math.inf
    first: bool = True
    recommit_after: float = 0.0
    gave_up: int = 0


class World:
    """Closed loop of a set of agents at one gate (module docstring); ``variant`` CORRECTED or NEAR_HOLD.  ``entry_ok`` and
    ``loc_ok`` fixed by the caller override the idealised stage-2 check and localization."""

    def __init__(self, cfg: FleetConfig, gate: GateSpec, agents: Sequence[Agent], variant: str = "CORRECTED",
                 delay: float = 0.0, drift: Sequence[float] = (0.0, 0.0, 0.0), avoidance: bool = True,
                 loc_ok: bool = True, entry_ok: Optional[bool] = None, trace: bool = False):
        self.cfg = cfg if variant == "CORRECTED" else near_hold_config(cfg)
        self.gate, self.agents, self.variant = gate, list(agents), variant
        self.delay, self.drift, self.avoidance = float(delay), np.asarray(drift, float), avoidance
        self.loc_ok, self.entry_ok_fixed, self.trace = loc_ok, entry_ok, trace
        self.t = 0.0
        self.rows: List[Dict] = []
        self.entries: List[Dict] = []
        self.double: Optional[Dict] = None
        self.dmin = math.inf
        self.p1_viol = 0
        self.p1_fleet = 0
        self.pair_min: Dict[str, float] = {}
        self.non_fleet_in_cr = 0
        self.joint = {s: 0 for s in gs.STATUSES}
        self.reasons: Dict[str, int] = {}
        self.p2_filtered = 0
        self.p2_recovery = 0
        self.commit_seen: Dict[str, float] = {}
        nb = NavBound(self.cfg)
        self.v_err = float(nb.k_rw)
        self.rho = self.v_err + float(nb.k_lin) * float(nb.v_bound)
        self.p2_uncertified = 0
        cls = Flows if variant == "CORRECTED" else FlowsNearHold
        for a in self.agents:
            if a.kind in ("GATE", "TRANSIT"):
                plan = NS(gates=[gate], slots=[NS(lateral=a.queue[1], dz=0.0, along=0.0)], slot_index=0,
                          path=NS(depth_z=float(gate.center[2])), clock=None, structures=[])
                a.flows = cls(plan, self.cfg)
                if a.admitted0:
                    a.committed, a.flows.cr_cleared = True, True
                    self.commit_seen[a.name] = -math.inf

    # -- one agent ---------------------------------------------------------------------------------------------------
    def observe(self, a: Agent, others: Sequence[Agent]):
        G = self.cfg.gate
        p_est = a.p + a.bias
        s, l, dz = (float(x) for x in self.gate.to_gate_frame(p_est))
        claims = self._claim_known(others)
        occupied = any(in_cr(self.cfg, self.gate, o.p) for o in others)
        entry_ok = (not claims and not occupied) if self.entry_ok_fixed is None else self.entry_ok_fixed
        go = NS(gate_id=self.gate.gate_id, s=s, l=l, dz=dz, queue_s=a.queue[0], queue_l=a.queue[1], queue_dz=a.queue[2],
                loc_ok=self.loc_ok, entry_ok=entry_ok, passed=s > G.exit_s)
        tg = [x for x in (target(a.p, o.p) for o in others) if x is not None]
        return NS(t=self.t, p=p_est, v_world=a.v + self.drift, R_wb=np.eye(3), yaw=0.0, targets=tg, unhealthy=[],
                  gate=go, gate_index=0, sense_ok=True), entry_ok

    def stalled(self, a: Agent) -> bool:
        """A stalled drone holds its true position (a fault that stops it, its station keeping working: no drift)."""
        return any(t0 <= self.t < t1 for t0, t1 in a.stall)

    def _claim_known(self, others: Sequence[Agent]) -> bool:
        return any(o.kind == "GATE" and o.committed and not o.done
                   and self.t >= self.commit_seen.get(o.name, math.inf) + self.delay for o in others)

    def _may_commit(self, a: Agent, others: Sequence[Agent], obs) -> bool:
        """Re-commitment after a give-up (the stage-1 latch, idealised): at the own queue point, no claim of another committed
        drone known, and no queued drone further left (the precedence of the abreast queue)."""
        G, go = self.cfg.gate, obs.gate
        at_q = abs(go.s - go.queue_s) <= G.queue_tol and abs(go.l - go.queue_l) <= G.queue_tol
        left = any(o.kind == "GATE" and not o.committed and not o.done and o.queue[1] > a.queue[1] for o in others)
        return at_q and not self._claim_known(others) and not left

    def _mode(self, prev: str, d: float) -> str:
        sep = self.cfg.sep
        if not self.avoidance:
            return "MUTEX"
        if d < sep.d_ca or (prev == "CA" and d < sep.d_ca_exit):
            return "CA"
        if d < sep.d_warning or (prev == "SW" and d < sep.d_warning_exit):
            return "SW"
        return "MUTEX"

    def command(self, a: Agent, others: Sequence[Agent]) -> Dict:
        """The final velocity command of one agent, in the order of DroneController.step."""
        if a.kind == "STATIC" or self.stalled(a):
            return {"v": np.zeros(3)}
        if a.kind == "SCRIPTED":
            return {"v": a.vel.copy()}
        fl = a.flows
        obs, entry_ok = self.observe(a, others)
        if a.kind == "GATE" and not a.done:
            if a.committed and fl.abort_request:
                rank = sum(o.kind == "GATE" and o.queue[1] > a.queue[1] for o in others)
                a.committed, a.gave_up, a.recommit_after = False, a.gave_up + 1, self.t + 2.0 + 4.0 * rank
            if not a.committed and ((a.first and self.t >= a.commit_t) or
                                    (not a.first and self.t >= a.recommit_after and self._may_commit(a, others, obs))):
                a.committed, a.first = True, False
                self.commit_seen[a.name] = self.t
            v_mis, _ = fl.gate(obs, "MUTEX_PASS" if a.committed else "MUTEX_APPROACH")
        else:
            goal = a.goal if a.goal is not None else self.gate.from_gate_frame([self.cfg.gate.rally_s + 6.0, a.queue[1], 0.0])
            d = goal - obs.p
            n = float(np.linalg.norm(d))
            v_mis = np.zeros(3) if n < 0.2 else d / n * min(a.speed, 0.6 * n)
        if not a.committed:
            v_mis = fl.keepout_filter(obs, v_mis)
        gc = (fl.gate_constraint(obs, authorized=a.committed and fl.cr_cleared, v_err=self.v_err,
                                  rho=self.rho) if self.variant == "CORRECTED" else None)
        mf = (fl.p2_filter(gc, v_mis, fl.mission_anchor(gc) if a.kind == "GATE" and not a.done else {})
              if self.variant == "CORRECTED" else None)
        if mf is not None:
            v_mis = mf.v
            if mf.modified:
                self.p2_filtered += 1
                self.p2_recovery += mf.status == gs.RECOVERY
        d_min = min([x.d_lower for x in obs.targets] + [math.inf])
        m = self._mode(a.mode, d_min)
        if m not in AVOID:
            fl.last_choice = None
            fl.escape.reset()
            fl.p2_anchor = None
        a.mode = m
        if m == "SW":
            v = fl.separation_warning(obs, v_mis, gc)
        elif m == "CA":
            v = fl.collision_avoidance(obs, np.zeros(3), v_mis, gc)
        else:
            v = v_mis
        if m in AVOID:
            a.avoid_t = self.t
        if self.variant == "CORRECTED" and a.flows is not None:
            loops = [fl.mission_anchor(gc) if a.kind == "GATE" and not a.done else {}]
            if fl.p2_certify(gc, v, loops) is None and gc is not None:
                self.p2_uncertified += 1
        ch = fl.last_choice
        joint = ch.joint if (m in AVOID and ch is not None) else None
        if joint is not None:
            self.joint[joint.status] = self.joint.get(joint.status, 0) + 1
            if joint.reason:
                self.reasons[joint.reason] = self.reasons.get(joint.reason, 0) + 1
        return {"v": np.asarray(v, float), "obs": obs, "entry_ok": entry_ok, "gc": gc, "mf": mf, "joint": joint,
                "label": None if ch is None or m not in AVOID else ch.label, "d_min": d_min}

    # -- the loop -----------------------------------------------------------------------------------------------------
    def step(self) -> List[Dict]:
        cfg, env, G = self.cfg, self.cfg.env, self.cfg.gate
        cmds = [self.command(a, [o for o in self.agents if o is not a]) for a in self.agents]
        for a, c in zip(self.agents, cmds):
            dv = c["v"] - a.v
            nv = float(np.linalg.norm(dv))
            if nv > env.a_brake * DT:
                dv *= env.a_brake * DT / nv
            a.v = a.v + dv
            fixed = a.kind == "STATIC" or self.stalled(a)
            if fixed:
                a.v = np.zeros(3)
            a.p = a.p + (a.v + (0.0 if fixed else self.drift)) * DT
            if a.kind == "GATE" and a.committed and float(self.gate.to_gate_frame(a.p + a.bias)[0]) > G.exit_s:
                a.committed, a.done = False, True
        self.t += DT
        inside = [a.name for a in self.agents if in_cr(cfg, self.gate, a.p) and a.kind in FLEET]
        self.non_fleet_in_cr += any(in_cr(cfg, self.gate, a.p) and a.kind not in FLEET for a in self.agents)
        for a in self.agents:
            if a.name in inside and not (a.committed and a.flows is not None and a.flows.cr_cleared) and a.kind != "SCRIPTED":
                if not any(e["drone"] == a.name for e in self.entries):
                    self.entries.append({"t": round(self.t, 1), "drone": a.name, "by_avoidance": self.t - a.avoid_t <= 3.0,
                                         "s_true": round(float(self.gate.to_gate_frame(a.p)[0]), 2)})
        if len(inside) >= 2 and self.double is None:
            self.double = {"t": round(self.t, 1), "drones": inside}
        for a, b in itertools.combinations(self.agents, 2):
            d = float(np.linalg.norm(a.p - b.p))
            key = f"{a.name}-{b.name}"
            self.pair_min[key] = round(min(self.pair_min.get(key, math.inf), d), 3)
            self.dmin = min(self.dmin, d)
            self.p1_viol += d < cfg.sep.d_safe
            self.p1_fleet += d < cfg.sep.d_safe and a.kind in FLEET and b.kind in FLEET
        if self.trace:
            self.rows.append(self._row(cmds, inside))
        return cmds

    def _row(self, cmds, inside) -> Dict:
        r = {"t": round(self.t, 1), "cr": inside, "agents": {}}
        for a, c in zip(self.agents, cmds):
            j = c.get("joint")
            e = {"true": np.round(a.p, 2).tolist(), "est": np.round(a.p + a.bias, 2).tolist(), "mode": a.mode,
                 "committed": a.committed, "admitted": bool(a.flows is not None and a.flows.cr_cleared),
                 "entry": None if a.flows is None else (a.flows.entry_state or None), "v_cmd": np.round(c["v"], 3).tolist(),
                 "chosen": c.get("label")}
            if j is not None:
                e["joint"] = {"status": j.status, "n": j.n_candidates, "p2_rejected": j.n_p2_rejected,
                              "p1_rejected": j.n_p1_rejected, "reason": j.reason, "uncertified": j.uncertified,
                              "p2_margin": None if j.p2_margin is None else round(j.p2_margin, 3)}
            if c.get("mf") is not None and c["mf"].modified:
                e["p2_filter"] = c["mf"].status
            r["agents"][a.name] = e
        r["dist"] = {f"{a.name}-{b.name}": round(float(np.linalg.norm(a.p - b.p)), 2)
                     for a, b in itertools.combinations(self.agents, 2)}
        return r

    def run(self, horizon: float) -> Dict:
        for _ in range(int(round(horizon / DT))):
            self.step()
            gates = [a for a in self.agents if a.kind == "GATE"]
            if gates and all(a.done for a in gates) and not any(a.kind in ("TRANSIT", "SCRIPTED") for a in self.agents):
                break
        return self.summary()

    def summary(self) -> Dict:
        return {"entries_without_admission": self.entries, "by_avoidance": sum(e["by_avoidance"] for e in self.entries),
                "double": self.double, "dmin": round(self.dmin, 3), "p1_violation_steps": self.p1_viol,
                "p1_fleet_steps": self.p1_fleet, "pair_min": dict(self.pair_min), "non_fleet_in_cr_steps": self.non_fleet_in_cr,
                "joint": dict(self.joint), "reasons": dict(self.reasons), "p2_filtered": self.p2_filtered,
                "p2_recovery": self.p2_recovery, "p2_uncertified": self.p2_uncertified, "passed": [a.name for a in self.agents if a.done],
                "gave_up": sum(a.gave_up for a in self.agents), "t_end": round(self.t, 1)}


# -- episodes -----------------------------------------------------------------------------------------------------------
def _gate_agents(cfg: FleetConfig, gate, n: int, commits, biases) -> List[Agent]:
    G = cfg.gate
    lanes, qs = list(G.queue_laterals(n)), G.queue_s(n)
    out = []
    for k in range(n):
        b = np.asarray(biases[k], float)
        q = (qs, lanes[k], 0.0)
        out.append(Agent(f"q{k}", "GATE", gate.from_gate_frame(q) - b, bias=b, queue=q, commit_t=commits[k]))
    return out


def episodes() -> List[Dict]:
    """The scanned set (module docstring)."""
    out = []
    for n in (2, 3):
        pairs = list(itertools.permutations(range(n), 2))
        delays = (0.5, 2.0, 5.0, 12.0) if n == 2 else (0.5, 5.0)
        for (first, second), gap, delay in itertools.product(pairs, (0.0, 1.0, 3.0), delays):
            for third in ((False, True) if n == 3 else (False,)):
                variants = ((0.0, 0.0), (-0.5, 0.0), (0.0, 0.15), (-0.5, 0.15)) if n == 2 else ((-0.5, 0.15),)
                for bias, drift in variants:
                    commits = [math.inf] * n
                    commits[first], commits[second] = 0.0, gap
                    if third:
                        commits[({0, 1, 2} - {first, second}).pop()] = gap / 2.0
                    bs = [[0.0, 0.0, 0.0] for _ in range(n)]
                    bs[second] = [bias, 0.0, 0.0]
                    out.append({"family": "A_concurrent", "n": n, "commits": commits, "delay": delay, "biases": bs,
                                "drift": [drift, 0.0, 0.0]})
    for a_mode, third, bias, restrict, drift in itertools.product(
            ("moving", "stalled"), ("coop_axis", "coop_offset", "scripted"), (0.0, -0.5, -1.0, 0.5),
            ("none", "lateral_hulls", "shallow"), (0.0, 0.15)):
        out.append({"family": "B_occupant_holder_third", "a_mode": a_mode, "third": third, "bias": bias,
                    "restrict": restrict, "drift": [drift, 0.0, 0.0]})
    return out


def build(ep: Dict, cfg: FleetConfig, variant: str, trace: bool = False) -> World:
    c = cfg if variant == "CORRECTED" else near_hold_config(cfg)
    if ep["family"] == "A_concurrent":
        gate = make_gate()
        ag = _gate_agents(c, gate, ep["n"], ep["commits"], ep["biases"])
        return World(cfg, gate, ag, variant, delay=ep["delay"], drift=ep["drift"], trace=trace)
    gate = make_gate(cfg.env.z_max - 0.9 if ep["restrict"] == "shallow" else -4.3)
    s_hold = c.gate.stage2_hold_s
    qs, lanes = c.gate.queue_s(2), c.gate.queue_laterals(2)
    a = Agent("A", "GATE", gate.from_gate_frame([0.0, 0.0, 0.0]), queue=(qs, lanes[0], 0.0), commit_t=0.0, admitted0=True,
              stall=((0.0, 40.0),) if ep["a_mode"] == "stalled" else ())
    b_bias = np.array([ep["bias"], 0.0, 0.0])
    b = Agent("B", "GATE", gate.from_gate_frame([qs, lanes[1], 0.0]) - b_bias, bias=b_bias, queue=(qs, lanes[1], 0.0),
              commit_t=0.0)
    ag = [a, b]
    l3 = 0.4 if ep["third"] == "coop_offset" else 0.0
    start = gate.from_gate_frame([s_hold - 8.0, l3, 0.0])
    if ep["third"] == "scripted":
        ag.append(Agent("C", "SCRIPTED", start, vel=np.array([0.3, 0.0, 0.0])))
    else:
        ag.append(Agent("C", "TRANSIT", start, goal=gate.from_gate_frame([c.gate.rally_s + 6.0, l3, 0.0]), speed=0.3,
                        queue=(qs, l3, 0.0)))
    if ep["restrict"] == "lateral_hulls":
        ag += [Agent("W1", "STATIC", gate.from_gate_frame([s_hold + 0.3, 2.3, 0.0])),
               Agent("W2", "STATIC", gate.from_gate_frame([s_hold + 0.3, -2.3, 0.0]))]
    return World(cfg, gate, ag, variant, delay=0.0, drift=ep["drift"], trace=trace)


def run_episode(args) -> Dict:
    ep, variant = args
    w = build(ep, DEFAULT, variant)
    return {"ep": ep, "variant": variant, **w.run(90.0 if ep["family"] == "A_concurrent" else 60.0)}


def _low() -> None:
    try:
        import psutil

        psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    except Exception:  # noqa: BLE001 - the priority is a courtesy to the host, never a requirement
        pass


def scan(workers: int = 20) -> Dict:
    jobs = [(ep, v) for ep in episodes() for v in ("NEAR_HOLD", "CORRECTED")]
    try:
        from multiprocessing import Pool

        with Pool(workers, initializer=_low) as pool:
            res = pool.map(run_episode, jobs, chunksize=2)
    except (OSError, RuntimeError, ImportError):  # a pool that cannot start: the same scan, serially
        res = [run_episode(j) for j in jobs]
    summ = {}
    for v in ("NEAR_HOLD", "CORRECTED"):
        rs = [r for r in res if r["variant"] == v]
        fam = {}
        for f in sorted({r["ep"]["family"] for r in rs}):
            rf = [r for r in rs if r["ep"]["family"] == f]
            fam[f] = {"episodes": len(rf),
                      "with_entry_without_admission": sum(bool(r["entries_without_admission"]) for r in rf),
                      "with_entry_by_avoidance": sum(r["by_avoidance"] > 0 for r in rf),
                      "with_double_occupancy": sum(r["double"] is not None for r in rf),
                      "with_p1_violation": sum(r["p1_violation_steps"] > 0 for r in rf),
                      "with_p1_violation_between_fleet_drones": sum(r["p1_fleet_steps"] > 0 for r in rf),
                      "with_non_reacting_vehicle_in_cr": sum(r["non_fleet_in_cr_steps"] > 0 for r in rf),
                      "min_centre_distance_m": round(min(r["dmin"] for r in rf), 2),
                      "joint_steps": {s: sum(r["joint"].get(s, 0) for r in rf) for s in gs.STATUSES},
                      "p2_filtered_steps": sum(r["p2_filtered"] for r in rf),
                      "p2_recovery_steps": sum(r["p2_recovery"] for r in rf),
                      "p2_uncertified_steps": sum(r["p2_uncertified"] for r in rf)}
        summ[v] = fam
    return {"summary": summ, "results": res}


def _worst(res: List[Dict]) -> Optional[Dict]:
    mut = [r for r in res if r["variant"] == "NEAR_HOLD" and (r["double"] or r["by_avoidance"])]
    if not mut:
        return None
    mut.sort(key=lambda r: (r["double"] is None, -r["by_avoidance"], r["dmin"]))
    return mut[0]["ep"]


def _json(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, frozenset):
        return sorted(o)
    raise TypeError(type(o))


def _finite(o):
    if isinstance(o, float) and math.isinf(o):
        return "inf"
    if isinstance(o, dict):
        return {k: _finite(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_finite(v) for v in o]
    return o


def main(workers: int = 20) -> Dict:
    t0 = time.time()
    out = scan(workers)
    worst = _worst(out["results"])
    traces = {}
    if worst is not None:
        for v in ("NEAR_HOLD", "CORRECTED"):
            w = build(worst, DEFAULT, v, trace=True)
            s = w.run(90.0 if worst["family"] == "A_concurrent" else 60.0)
            traces[v] = {"summary": s, "rows": w.rows[::5]}
    compact = [{"family": r["ep"]["family"], "ep": {k: v for k, v in r["ep"].items() if k != "family"},
                "variant": r["variant"], "entries": r["entries_without_admission"], "by_avoidance": r["by_avoidance"],
                "double": r["double"], "dmin": r["dmin"], "p1_steps": r["p1_violation_steps"],
                "p1_fleet_steps": r["p1_fleet_steps"], "pair_min": r["pair_min"], "joint": r["joint"],
                "reasons": r["reasons"], "passed": r["passed"]} for r in out["results"]]
    doc = _finite({"generated_s": round(time.time() - t0, 1), "summary": out["summary"], "worst_near_hold_episode": worst,
                   "traces": traces, "episodes": compact})
    RESULTS.parent.mkdir(parents=True, exist_ok=True)
    RESULTS.write_text(json.dumps(doc, default=_json), encoding="utf-8")
    return doc


def check(cfg: FleetConfig = DEFAULT):
    """F1pk: the scan (module docstring), NEAR_HOLD against CORRECTED; reported, expect none in CORRECTED."""
    from common import CheckResult

    t0 = time.time()
    doc = main()
    s = doc["summary"]
    c = s["CORRECTED"]
    bad = sum(f["with_entry_without_admission"] + f["with_double_occupancy"] for f in c.values())
    note = {"NEAR_HOLD": s["NEAR_HOLD"], "CORRECTED": c,
            "reading": ("NEAR_HOLD against CORRECTED on the same episodes; CORRECTED keeps every not-admitted centre out of the "
                        "CR in this set" if bad == 0 else
                        "CORRECTED has entries without admission or double occupancy: see formal/results/f1p_kinematic.json")}
    return CheckResult("F1pk reported (not a proof): kinematic reconstruction with moving drones, NEAR_HOLD mutation vs CORRECTED controller, "
                       "idealised perception and plant; concurrent commitments, delayed claims, an occupant, a held drone and a "
                       "third drone from behind, restricted corridors, navigation bias, drift",
                       "count entries of a not-admitted centre into the CR and instants with two centres in the CR (CORRECTED)",
                       "formal/check_f1p_kinematic.py", "none", "observed" if bad else "none", bad == 0,
                       round(time.time() - t0, 1), note=json.dumps(note, default=_json))


if __name__ == "__main__":
    d = main()
    print(json.dumps(d["summary"], indent=1, default=_json))
    print("worst NEAR_HOLD episode:", d["worst_near_hold_episode"])
