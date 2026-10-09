"""Runtime execution of the local hybrid automaton (discrete part).

The automaton is driven by an :class:`AbstractObservation` produced by the
perception layer.  It never sees simulator state.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.ha.spec import BOOL_VARS, PY_LOGIC, REAL_VARS, Edge, Mode, build_edges


@dataclass
class AbstractObservation:
    d_min: float = 1e9
    form_err: float = 0.0
    t_ok: float = 0.0
    sense_ok: bool = True
    env_ok: bool = True
    mutex_zone: bool = False
    at_queue: bool = False
    occ_busy: bool = False
    has_prio: bool = False
    passed: bool = False
    neighbors_ok: bool = True
    committed: bool = False

    def as_dict(self) -> Dict:
        return asdict(self)


assert set(REAL_VARS) | set(BOOL_VARS) == set(AbstractObservation.__dataclass_fields__), "spec/runtime variable mismatch"


@dataclass
class TransitionRecord:
    t: float
    source: str
    target: str
    edge: str
    decision: Optional[str]
    enabled_edges: List[str] = field(default_factory=list)


class LocalHybridAutomaton:
    """Discrete state = (mode, committed latch).  One instance per drone, identical code."""

    def __init__(self, cfg: FleetConfig = DEFAULT, initial: Mode = Mode.FORMATION_FOLLOW):
        self.cfg = cfg
        self.edges = build_edges(cfg)
        self.mode = initial
        self.committed = False
        self.determinism_violations = 0
        self.time_in_mode = 0.0

    def step(self, obs: AbstractObservation, t: float, dt: float) -> TransitionRecord:
        obs.committed = self.committed
        enabled: List[Edge] = [e for e in self.edges[self.mode] if e.guard(obs, PY_LOGIC)]
        if len(enabled) != 1:
            # Never expected (proved UNSAT in formal/check_determinism.py); fall back to
            # the conservative mode so the vehicle remains safe, and record it.
            self.determinism_violations += 1
            chosen = Edge("nondeterminism_fallback", lambda o, L: True, Mode.FAILSAFE_HOLD_OR_RETREAT)
        else:
            chosen = enabled[0]
        source = self.mode
        if chosen.set_committed is not None:
            self.committed = chosen.set_committed
        if chosen.target != source:
            self.time_in_mode = 0.0
        else:
            self.time_in_mode += dt
        self.mode = chosen.target
        return TransitionRecord(
            t=t, source=source.value, target=chosen.target.value, edge=chosen.name,
            decision=chosen.decision, enabled_edges=[e.name for e in enabled],
        )
