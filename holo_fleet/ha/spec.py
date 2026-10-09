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


MODES: List[Mode] = list(Mode)

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
    "neighbors_ok",  # every expected neighbour perceived recently
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
    def lost(self, o, L):
        return L.Or(o.form_err > self.cfg.form.e_lost, L.Not(o.neighbors_ok))

    def recovered(self, o, L):
        return L.And(o.form_err < self.cfg.form.e_ok, o.t_ok >= self.cfg.form.t_ok_hold, o.neighbors_ok)


def build_edges(cfg: FleetConfig = DEFAULT) -> Dict[Mode, List[Edge]]:
    """Explicit per-mode edge table (self-loops included as ordinary edges)."""

    P = Predicates(cfg)
    edges: Dict[Mode, List[Edge]] = {}

    for src in MODES:
        out: List[Edge] = []
        # 1. failsafe has the highest priority
        out.append(Edge("fault", lambda o, L: P.fault(o, L), Mode.FAILSAFE_HOLD_OR_RETREAT))
        # 2. collision avoidance
        out.append(Edge("collision_risk", lambda o, L, s=src: P.ca_cond(o, L, s), Mode.COLLISION_AVOIDANCE))
        # 3. separation warning
        out.append(Edge("separation_warning", lambda o, L, s=src: P.sw_cond(o, L, s), Mode.SEPARATION_WARNING))

        # 4. mission-level logic, only when calm
        def calm(o, L, s=src):
            return P.calm(o, L, s)

        # 4a. a committed drone keeps passing until it is beyond the occupied zone
        out.append(Edge(
            "pass_continue",
            lambda o, L, c=calm: L.And(c(o, L), o.committed, L.Not(o.passed)),
            Mode.MUTEX_PASS,
            decision=("RESUME_PASS" if src != Mode.MUTEX_PASS else None),
        ))
        out.append(Edge(
            "pass_done",
            lambda o, L, c=calm: L.And(c(o, L), o.committed, o.passed),
            Mode.FORMATION_RECOVERY,
            set_committed=False,
            decision="EXITED",
        ))
        # 4b. gate approach / yield / commit (not yet committed, inside approach zone)
        out.append(Edge(
            "commit",
            lambda o, L, c=calm: L.And(c(o, L), L.Not(o.committed), o.mutex_zone, P.can_go(o, L)),
            Mode.MUTEX_PASS,
            set_committed=True,
            decision=("RETRY_PASS" if src == Mode.MUTEX_YIELD else "PASS"),
        ))
        out.append(Edge(
            "yield",
            lambda o, L, c=calm: L.And(c(o, L), L.Not(o.committed), o.mutex_zone, L.Not(P.can_go(o, L)), o.at_queue),
            Mode.MUTEX_YIELD,
            decision=(None if src == Mode.MUTEX_YIELD else "YIELD"),
        ))
        out.append(Edge(
            "approach",
            lambda o, L, c=calm: L.And(c(o, L), L.Not(o.committed), o.mutex_zone, L.Not(o.at_queue)),
            Mode.MUTEX_APPROACH,
            decision=(None if src == Mode.MUTEX_APPROACH else "APPROACH"),
        ))
        # 4c. no gate in view: formation follow / recovery
        if src == Mode.FORMATION_FOLLOW:
            out.append(Edge(
                "formation_lost",
                lambda o, L, c=calm: L.And(c(o, L), L.Not(o.committed), L.Not(o.mutex_zone), P.lost(o, L)),
                Mode.FORMATION_RECOVERY, decision="FORMATION_LOST",
            ))
            out.append(Edge(
                "follow",
                lambda o, L, c=calm: L.And(c(o, L), L.Not(o.committed), L.Not(o.mutex_zone), L.Not(P.lost(o, L))),
                Mode.FORMATION_FOLLOW,
            ))
        else:
            out.append(Edge(
                "formation_recovered",
                lambda o, L, c=calm: L.And(c(o, L), L.Not(o.committed), L.Not(o.mutex_zone), P.recovered(o, L)),
                Mode.FORMATION_FOLLOW,
                decision=("FORMATION_RECOVERED" if src == Mode.FORMATION_RECOVERY else "RESUME_FOLLOW"),
            ))
            out.append(Edge(
                "recover",
                lambda o, L, c=calm: L.And(c(o, L), L.Not(o.committed), L.Not(o.mutex_zone), L.Not(P.recovered(o, L))),
                Mode.FORMATION_RECOVERY,
            ))
        edges[src] = out
    return edges


EDGES = build_edges()

# Priority hierarchy (documentation + checked in formal/check_determinism.py):
# collision avoidance > critical-region mutual exclusion > formation recovery > mission following.
PRIORITY_ORDER = [
    Mode.FAILSAFE_HOLD_OR_RETREAT,
    Mode.COLLISION_AVOIDANCE,
    Mode.SEPARATION_WARNING,
    Mode.MUTEX_PASS,
    Mode.MUTEX_YIELD,
    Mode.MUTEX_APPROACH,
    Mode.FORMATION_RECOVERY,
    Mode.FORMATION_FOLLOW,
]
