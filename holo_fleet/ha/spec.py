"""Hybrid-automaton specification shared by the runtime controller and the Z3 encoding.

The discrete part of the local automaton H_i is an explicit edge table
``EDGES[source] = [Edge(name, guard, target, reset), ...]``.  Every guard is a
function ``guard(o, L)`` where

* ``o`` exposes the abstract observation (``o.d_min``, ``o.occ_busy``, ...) and the
  latched discrete variable ``o.committed``;
* ``L`` is a logic backend with ``And``, ``Or``, ``Not``.

At runtime ``o`` holds Python floats/bools and ``L`` is :data:`PY_LOGIC`; in
``formal/`` the very same functions are called with Z3 variables and Z3's
connectives.  The runtime evaluates *all* guards of the current mode and checks
that exactly one is enabled (a determinism monitor whose verdict is logged); the
formal scripts prove the same fact symbolically for every legal observation.

Every edge also carries a readable ``label`` (a Python boolean expression over the
observation variables and the configured thresholds, holo_fleet/ha/guard_text.py),
exact given that no edge listed before it in the same mode is enabled (the edges of a
mode are listed in priority order; formal/check_determinism.py proves the equivalence
with Z3), and a short ``reason`` written to the transition log when the edge changes the
mode.  The automaton diagram (scripts/make_automaton_diagram.py) is generated from them.

Continuous flows (one control law per mode) live in ``holo_fleet.control``.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional

from holo_fleet.config import DEFAULT, FleetConfig


class Mode(str, Enum):
    FORMATION_FOLLOW = "FORMATION_FOLLOW"
    SEPARATION_WARNING = "SEPARATION_WARNING"
    COLLISION_AVOIDANCE = "COLLISION_AVOIDANCE"
    MUTEX_APPROACH = "MUTEX_APPROACH"
    MUTEX_YIELD = "MUTEX_YIELD"
    MUTEX_PASS = "MUTEX_PASS"
    FORMATION_RECOVERY = "FORMATION_RECOVERY"
    FAILSAFE_HOLD_OR_RETREAT = "FAILSAFE_HOLD_OR_RETREAT"
    # formation keeping with an incomplete formation (same flow and priority level as FORMATION_FOLLOW):
    FORMATION_WAIT_REJOIN = "FORMATION_WAIT_REJOIN"    # own slot ok, an expected neighbour missing: keep the
                                                       # slot, follow the shared clock, wait (bounded) for it
    DEGRADED_FORMATION = "DEGRADED_FORMATION"          # a slot declared vacant after the timeout: keep the own
                                                       # original slot, the vacant slot stays empty


MODES: List[Mode] = list(Mode)
# modes in which the drone holds its own slot of the formation (the missing-neighbour timers run only here)
IN_FORMATION = (Mode.FORMATION_FOLLOW, Mode.FORMATION_WAIT_REJOIN, Mode.DEGRADED_FORMATION)

# Abstract observation variables consumed by the guards.  Reals and Booleans are
# declared separately so the Z3 encoding can create correctly-typed symbols.
REAL_VARS = ("d_min", "form_err", "t_ok")
BOOL_VARS = (
    "sense_ok",      # every safety-relevant sensor stream is fresh
    "env_ok",        # vehicle believes it is inside the operational envelope
    "mutex_zone",     # next gate exists and own estimate is inside its approach zone
    "at_queue",      # own gate-frame along position is inside the commit window
    "occ_busy",      # some other drone is perceived inside the occupied zone
    "has_prio",      # robust priority over every perceived queued drone
    "passed",        # own estimate is beyond the occupied zone of the current gate
    "neighbors_ok",  # every expected neighbour not declared vacant perceived recently
    "degraded",      # latched: some expected neighbour was missing for t_rejoin, its slot is declared vacant
    "committed",     # latched: this drone committed to the current gate
)


class Logic:
    """Logic backend interface (Python bools or Z3 BoolRefs)."""

    def And(self, *args: Any) -> Any:  # pragma: no cover - interface
        raise NotImplementedError

    def Or(self, *args: Any) -> Any:  # pragma: no cover - interface
        raise NotImplementedError

    def Not(self, a: Any) -> Any:  # pragma: no cover - interface
        raise NotImplementedError


class _PyLogic(Logic):
    def And(self, *args: Any) -> bool:
        return all(bool(a) for a in args)

    def Or(self, *args: Any) -> bool:
        return any(bool(a) for a in args)

    def Not(self, a: Any) -> bool:
        return not bool(a)


PY_LOGIC = _PyLogic()

Guard = Callable[[Any, Logic], Any]


@dataclass(frozen=True)
class Edge:
    name: str
    guard: Guard
    target: Mode
    # reset of the latched discrete variable: None = keep, True/False = set
    set_committed: Optional[bool] = None
    # semantic label written to the event log (PASS / YIELD / WAIT / RETRY / ...)
    decision: Optional[str] = None
    # readable guard, exact under the priority order (see the module docstring)
    label: str = ""
    # short cause of the mode change, written to the transition log (refined by transition_reason)
    reason: str = ""


class Predicates:
    """Named sub-formulas shared by all edges (pure functions of ``o``)."""

    def __init__(self, cfg: FleetConfig = DEFAULT):
        self.cfg = cfg

    # --- hazards -----------------------------------------------------------
    def fault(self, o, L):
        return L.Or(L.Not(o.sense_ok), L.Not(o.env_ok))

    def ca_enter(self, o, L):
        return o.d_min < self.cfg.sep.d_ca

    def ca_stay(self, o, L):
        return o.d_min < self.cfg.sep.d_ca_exit

    def sw_enter(self, o, L):
        return o.d_min < self.cfg.sep.d_warning

    def sw_stay(self, o, L):
        return o.d_min < self.cfg.sep.d_warning_exit

    def ca_cond(self, o, L, src: Mode):
        if src == Mode.COLLISION_AVOIDANCE:
            return L.And(L.Not(self.fault(o, L)), self.ca_stay(o, L))
        return L.And(L.Not(self.fault(o, L)), self.ca_enter(o, L))

    def sw_cond(self, o, L, src: Mode):
        hazard = self.sw_stay(o, L) if src == Mode.SEPARATION_WARNING else self.sw_enter(o, L)
        return L.And(L.Not(self.fault(o, L)), L.Not(self.ca_cond(o, L, src)), hazard)

    def calm(self, o, L, src: Mode):
        """No fault and no separation hazard: mission-level logic may act."""
        return L.And(L.Not(self.fault(o, L)), L.Not(self.ca_cond(o, L, src)), L.Not(self.sw_cond(o, L, src)))

    # --- gate protocol -----------------------------------------------------
    def can_go(self, o, L):
        return L.And(o.at_queue, L.Not(o.occ_busy), o.has_prio)

    # --- formation ---------------------------------------------------------
    # A drone separates what it can fix from what it cannot: its OWN slot error (navigation + sonar
    # residuals of the neighbours it sees) drives FORMATION_RECOVERY; a MISSING neighbour is not its own
    # error and drives FORMATION_WAIT_REJOIN, then DEGRADED_FORMATION once the slot is declared vacant.
    def self_lost(self, o, L):
        return o.form_err > self.cfg.form.e_lost

    def self_recovered(self, o, L):
        return L.And(o.form_err < self.cfg.form.e_ok, o.t_ok >= self.cfg.form.t_ok_hold)

    def formation_target(self, o, L):
        """Formation-keeping mode selected by the neighbour knowledge (own slot ok): {mode: condition}."""
        return {Mode.FORMATION_WAIT_REJOIN: L.Not(o.neighbors_ok),
                Mode.DEGRADED_FORMATION: L.And(o.neighbors_ok, o.degraded),
                Mode.FORMATION_FOLLOW: L.And(o.neighbors_ok, L.Not(o.degraded))}


HAZARD_MODES = (Mode.FAILSAFE_HOLD_OR_RETREAT, Mode.COLLISION_AVOIDANCE, Mode.SEPARATION_WARNING)
MUTEX_MODES = (Mode.MUTEX_APPROACH, Mode.MUTEX_YIELD, Mode.MUTEX_PASS)
# leaving a hazard mode for a lower one is caused by the hazard clearing
CLEARED = {Mode.FAILSAFE_HOLD_OR_RETREAT: "fault_cleared", Mode.COLLISION_AVOIDANCE: "collision_risk_cleared",
           Mode.SEPARATION_WARNING: "separation_clear"}
SELF_RECOVERED = "form_err < e_ok and t_ok >= t_ok_hold"


def build_edges(cfg: FleetConfig = DEFAULT) -> Dict[Mode, List[Edge]]:
    """Explicit per-mode edge table (self-loops included as ordinary edges), in priority order."""

    P = Predicates(cfg)
    edges: Dict[Mode, List[Edge]] = {}

    for src in MODES:
        out: List[Edge] = []

        def why(target: Mode, own: str, s=src) -> str:
            """Cause of a change from ``s`` to ``target``: the hazard that cleared when a hazard mode is left for a
            lower one, else the edge's own cause (empty for a self-loop)."""
            if target == s:
                return ""
            if s in CLEARED and target not in HAZARD_MODES[:HAZARD_MODES.index(s)]:
                return CLEARED[s]
            return own

        # 1. failsafe has the highest priority
        out.append(Edge("fault", lambda o, L: P.fault(o, L), Mode.FAILSAFE_HOLD_OR_RETREAT,
                        label="not sense_ok or not env_ok", reason=why(Mode.FAILSAFE_HOLD_OR_RETREAT, "fault")))
        # 2. collision avoidance
        out.append(Edge("collision_risk", lambda o, L, s=src: P.ca_cond(o, L, s), Mode.COLLISION_AVOIDANCE,
                        label="d_min < d_ca_exit" if src == Mode.COLLISION_AVOIDANCE else "d_min < d_ca",
                        reason=why(Mode.COLLISION_AVOIDANCE, "collision_risk")))
        # 3. separation warning
        out.append(Edge("separation_warning", lambda o, L, s=src: P.sw_cond(o, L, s), Mode.SEPARATION_WARNING,
                        label={Mode.COLLISION_AVOIDANCE: "d_ca_exit <= d_min < d_warning",
                               Mode.SEPARATION_WARNING: "d_ca <= d_min < d_warning_exit"}.get(
                                   src, "d_ca <= d_min < d_warning"),
                        reason=why(Mode.SEPARATION_WARNING, "separation_warning")))

        # 4. mission-level logic, only when calm
        def calm(o, L, s=src):
            return P.calm(o, L, s)

        # 4a. a committed drone keeps passing until it is beyond the occupied zone
        out.append(Edge(
            "pass_continue",
            lambda o, L, c=calm: L.And(c(o, L), o.committed, L.Not(o.passed)),
            Mode.MUTEX_PASS,
            decision=("RESUME_PASS" if src != Mode.MUTEX_PASS else None),
            label="committed and not passed", reason=why(Mode.MUTEX_PASS, "resume_passage"),
        ))
        out.append(Edge(
            "pass_done",
            lambda o, L, c=calm: L.And(c(o, L), o.committed, o.passed),
            Mode.FORMATION_RECOVERY,
            set_committed=False,
            decision="EXITED",
            label="committed and passed", reason=why(Mode.FORMATION_RECOVERY, "mutex_exit_passed"),
        ))
        # 4b. gate approach / yield / commit (not yet committed, inside approach zone)
        out.append(Edge(
            "commit",
            lambda o, L, c=calm: L.And(c(o, L), L.Not(o.committed), o.mutex_zone, P.can_go(o, L)),
            Mode.MUTEX_PASS,
            set_committed=True,
            decision=("RETRY_PASS" if src == Mode.MUTEX_YIELD else "PASS"),
            label="mutex_zone and at_queue and has_prio and not occ_busy",
            reason=why(Mode.MUTEX_PASS, "mutex_priority_and_free"),
        ))
        out.append(Edge(
            "yield",
            lambda o, L, c=calm: L.And(c(o, L), L.Not(o.committed), o.mutex_zone, L.Not(P.can_go(o, L)), o.at_queue),
            Mode.MUTEX_YIELD,
            decision=(None if src == Mode.MUTEX_YIELD else "YIELD"),
            label="mutex_zone and at_queue and (occ_busy or not has_prio)",
            reason=why(Mode.MUTEX_YIELD, "mutex_wait"),
        ))
        out.append(Edge(
            "approach",
            lambda o, L, c=calm: L.And(c(o, L), L.Not(o.committed), o.mutex_zone, L.Not(o.at_queue)),
            Mode.MUTEX_APPROACH,
            decision=(None if src == Mode.MUTEX_APPROACH else "APPROACH"),
            label="mutex_zone and not at_queue",
            reason=why(Mode.MUTEX_APPROACH, "queue_point_left" if src == Mode.MUTEX_YIELD else "mutex_zone_entered"),
        ))
        # 4c. no gate in view: formation keeping.  From a formation-keeping mode the own slot error
        #     decides first (lost -> RECOVERY), then the neighbour knowledge (missing -> WAIT_REJOIN,
        #     vacant slot -> DEGRADED, else FOLLOW); from any other mode the drone first recovers its
        #     own slot (RECOVERY until self_recovered), then joins the mode selected the same way.
        def base(o, L, c=calm):
            return L.And(c(o, L), L.Not(o.committed), L.Not(o.mutex_zone))

        left = "mutex_zone_left" if src in MUTEX_MODES else ""    # out of the zone without committing
        if src in IN_FORMATION:
            out.append(Edge(
                "formation_lost",
                lambda o, L, b=base: L.And(b(o, L), P.self_lost(o, L)),
                Mode.FORMATION_RECOVERY, decision="FORMATION_LOST",
                label="form_err > e_lost", reason="own_slot_lost",
            ))
            for tgt, name, label, guard_text, reason in (
                    (Mode.FORMATION_WAIT_REJOIN, "neighbour_missing", "NEIGHBOUR_MISSING", "not neighbors_ok",
                     "neighbor_missing"),
                    (Mode.DEGRADED_FORMATION, "slot_vacant", "SLOT_VACANT", "neighbors_ok and degraded",
                     "rejoin_timeout" if src == Mode.FORMATION_WAIT_REJOIN else "slot_vacant"),
                    (Mode.FORMATION_FOLLOW, "follow",
                     {Mode.FORMATION_WAIT_REJOIN: "NEIGHBOUR_REJOINED",
                      Mode.DEGRADED_FORMATION: "FORMATION_RESTORED"}.get(src), "neighbors_ok and not degraded",
                     "neighbor_rejoined" if src == Mode.FORMATION_WAIT_REJOIN else "slot_reoccupied")):
                out.append(Edge(
                    name,
                    lambda o, L, b=base, k=tgt: L.And(b(o, L), L.Not(P.self_lost(o, L)), P.formation_target(o, L)[k]),
                    tgt, decision=(None if tgt == src else label),
                    label=guard_text, reason=why(tgt, reason),
                ))
        else:
            for tgt, name, resume, guard_text in (
                    (Mode.FORMATION_FOLLOW, "formation_recovered", "RESUME_FOLLOW", "neighbors_ok and not degraded"),
                    (Mode.FORMATION_WAIT_REJOIN, "recovered_wait_rejoin", "RESUME_WAIT_REJOIN", "not neighbors_ok"),
                    (Mode.DEGRADED_FORMATION, "recovered_degraded", "RESUME_DEGRADED", "neighbors_ok and degraded")):
                out.append(Edge(
                    name,
                    lambda o, L, b=base, k=tgt: L.And(b(o, L), P.self_recovered(o, L), P.formation_target(o, L)[k]),
                    tgt, decision=("FORMATION_RECOVERED" if src == Mode.FORMATION_RECOVERY else resume),
                    label=f"{SELF_RECOVERED} and {guard_text}", reason=why(tgt, left or "slot_recovered"),
                ))
            out.append(Edge(
                "recover",
                lambda o, L, b=base: L.And(b(o, L), L.Not(P.self_recovered(o, L))),
                Mode.FORMATION_RECOVERY,
                label=f"not ({SELF_RECOVERED})", reason=why(Mode.FORMATION_RECOVERY, left or "slot_not_recovered"),
            ))
        edges[src] = out
    return edges


def transition_reason(edge: Edge, o) -> str:
    """Short cause of a mode change by ``edge`` for the transition log: the edge's reason, refined by the part of
    its guard that holds (fault: sensor and / or envelope; waiting at the queue: occupied or no priority)."""
    if edge.reason == "fault":
        return "+".join(r for r, bad in (("sensor_fault", not o.sense_ok), ("envelope_violation", not o.env_ok))
                        if bad) or "fault"
    if edge.reason == "mutex_wait":
        return "mutex_occupied" if o.occ_busy else "no_priority"
    return edge.reason


EDGES = build_edges()

# Priority hierarchy (documentation + checked in formal/check_determinism.py):
# FAILSAFE > COLLISION_AVOIDANCE > SEPARATION_WARNING > MUTEX (pass, yield, approach) > FORMATION_RECOVERY
# > formation keeping (FORMATION_WAIT_REJOIN, DEGRADED_FORMATION, FORMATION_FOLLOW: one level, the same flow;
# the three modes differ only in what the drone knows about its neighbours).
PRIORITY_ORDER = [
    Mode.FAILSAFE_HOLD_OR_RETREAT,
    Mode.COLLISION_AVOIDANCE,
    Mode.SEPARATION_WARNING,
    Mode.MUTEX_PASS,
    Mode.MUTEX_YIELD,
    Mode.MUTEX_APPROACH,
    Mode.FORMATION_RECOVERY,
    Mode.FORMATION_WAIT_REJOIN,
    Mode.DEGRADED_FORMATION,
    Mode.FORMATION_FOLLOW,
]
