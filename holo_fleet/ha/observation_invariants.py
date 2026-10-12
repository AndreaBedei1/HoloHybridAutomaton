"""Semantic invariants of the abstract observation (the perception -> automaton interface).

The automaton (ha/spec.py) evaluates its guards on an :class:`AbstractObservation` of 13 variables.
Not every assignment of those variables can come out of the deployed perception: several flags are
computed from the same quantity (``mutex_zone``, ``at_queue`` and ``passed`` all from the own
gate-frame position), and one is latched on top of another (``has_prio`` is only ever set while
``at_queue`` holds).  This module lists the relations that hold BY CONSTRUCTION of the perception
code, each with its meaning, the reason it holds and the states it excludes.

The same predicates are used

* at runtime, by the observation consistency check between perception and automaton
  (``control/controller.py``): SensorFrame -> Perception -> AbstractObservation -> consistency
  check -> automaton.  An observation that violates an invariant reveals a perception defect; it is
  logged as ``OBSERVATION_INCONSISTENT`` (drone, time, violated invariants, observation values) and
  handed to the automaton with ``sense_ok = False``, so the existing fault edge takes it to
  FAILSAFE_HOLD_OR_RETREAT (no new mode);
* in ``formal/`` with Z3 symbols: ``formal/common.Obs.legal`` (the observations over which the
  determinism, P1, P2 and P3 checks quantify) and ``formal/check_observations.py`` (satisfiability,
  non-vacuity, mutations).

Only relations that the code guarantees are listed.  Latched beliefs that may legitimately persist
are NOT constrained:

* ``occ_busy`` may be set anywhere, also outside the approach zone (the occupancy latch keeps BUSY or
  EXITING until the exit is seen or ``t_occ_max`` expires);
* ``committed`` is the automaton's own latch, not a perception output (a committed drone may be
  outside ``mutex_zone`` and not yet ``passed``, e.g. after a FAILSAFE during the passage);
* ``has_prio`` and ``occ_busy`` may hold together (priority among queued drones and the occupancy of
  the critical region are separate beliefs), and so may ``neighbors_ok`` with any formation error;
* ``degraded`` (some slot declared vacant) is a latch of the perception: it may hold with or without
  ``neighbors_ok`` (another neighbour may be missing as well) and in any gate state.

Derived relations (implied by the list, not checked separately): has_prio -> mutex_zone,
passed -> not at_queue, passed -> not has_prio.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, List

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.ha.spec import BOOL_VARS, REAL_VARS, _PyLogic

D_NONE = 1e9            # d_min when no obstacle target is perceived (perception.py)


class _PyInvLogic(_PyLogic):
    """Python backend of the invariants: the guards' connectives plus a finiteness test."""

    @staticmethod
    def finite(x: Any) -> bool:
        return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)

    @staticmethod
    def boolean(b: Any) -> bool:
        return isinstance(b, bool)


PY_INV = _PyInvLogic()


def _implies(L, a, b):
    return L.Or(L.Not(a), b)


@dataclass(frozen=True)
class Invariant:
    name: str
    formula: str            # readable form
    meaning: str
    why: str                # why the deployed perception guarantees it (code reference)
    excludes: str           # states of the 13 variables it rules out
    pred: Callable[[Any, Any, FleetConfig], Any]      # pred(o, L, cfg), L = PY_INV or the Z3 backend


def _well_formed(o, L, cfg):
    reals = [L.finite(getattr(o, v)) for v in REAL_VARS]
    bools = [L.boolean(getattr(o, v)) for v in BOOL_VARS if v != "committed"]
    return L.And(*reals, *bools)


INVARIANTS: List[Invariant] = [
    Invariant(
        "N0 well-formed",
        "d_min, form_err, t_ok finite reals; the nine perception flags are Booleans",
        "every variable carries a value of its declared type",
        "perception.py casts every field (float(...), bool(...)); a NaN, an infinity or a missing flag can "
        "only come from a defect upstream (e.g. a NaN pose)",
        "NaN/inf distances or errors (every comparison with NaN is False: a NaN d_min would silently "
        "disable both hazard guards); None or non-Boolean flags.  Trivially true in Z3 (typed symbols)",
        _well_formed),
    Invariant(
        "N1 ranges",
        "form_err >= 0  and  t_ok >= 0  and  d_min <= D_NONE",
        "the formation error is a norm, t_ok a duration, d_min a distance or the no-target sentinel",
        "formation_perception.check_formation: form_err = max of norms and |residuals|; perception.py: "
        "t_ok = t - ok_since with ok_since <= t, d_min = min(d_lower of targets, default D_NONE)",
        "negative errors or durations; distances beyond the sentinel",
        lambda o, L, cfg: L.And(o.form_err >= 0, o.t_ok >= 0, o.d_min <= D_NONE)),
    Invariant(
        "I1 priority only at the queue point",
        "has_prio -> at_queue",
        "the robust priority belief exists only for a drone that is holding its queue point",
        "gate_perception.update: prio_since is set only when raw_decision == PRIORITY and at_queue in the "
        "same update and reset to None otherwise; has_prio = prio_since is not None and held t_clear",
        "has_prio & !at_queue: a drone away from its queue point believing it may commit (the commit guard "
        "also requires at_queue, so this state would be a latent defect, not a commit)",
        lambda o, L, cfg: _implies(L, o.has_prio, o.at_queue)),
    Invariant(
        "I2 the queue point lies in the approach zone",
        "at_queue -> mutex_zone",
        "a drone at its queue point is inside the approach zone of the same gate",
        "both flags come from the own gate-frame (s, l): at_queue needs |s - queue_s| <= queue_tol and "
        "|l - queue_l| <= queue_tol; mutex_zone needs queue_s - approach_len <= s <= exit_s and "
        "|l| <= max(corridor_half_width, |queue_l| + 1).  queue_s(n) < 0 < exit_s - queue_tol, and the lateral "
        "bound always contains the own queue point (for n >= 5 the outer queue points lie beyond 6.5 m)",
        "at_queue & !mutex_zone: the automaton would follow the formation (or recover) while queued, and the "
        "yield/commit edges, which require mutex_zone, could never fire for that drone",
        lambda o, L, cfg: _implies(L, o.at_queue, o.mutex_zone)),
    Invariant(
        "I3 a passed gate is out of the approach zone",
        "passed -> !mutex_zone",
        "beyond the exit of the critical region the drone is no longer approaching that gate",
        "gate_perception.update: passed = s > exit_s, mutex_zone requires s <= exit_s (same s)",
        "passed & mutex_zone: approach/yield/commit and pass_done would both describe the same gate",
        lambda o, L, cfg: _implies(L, o.passed, L.Not(o.mutex_zone))),
    Invariant(
        "I4 t_ok counts only while the own slot error is ok",
        "t_ok > 0 -> form_err < e_ok",
        "t_ok is the time for which the own formation error has been continuously below e_ok up to now",
        "perception.py: ok_since is set while form_err < e_ok and reset to None otherwise; t_ok = 0 when "
        "ok_since is None.  A missing neighbour does not reset it: it is not an error of the drone itself "
        "(it selects FORMATION_WAIT_REJOIN, not FORMATION_RECOVERY)",
        "t_ok > 0 with form_err >= e_ok (a stale 'ok for t seconds' counter)",
        lambda o, L, cfg: _implies(L, o.t_ok > 0, o.form_err < cfg.form.e_ok)),
]

BY_NAME: Dict[str, Invariant] = {inv.name.split()[0]: inv for inv in INVARIANTS}     # "N0", "I1", ...


def legal(o, L, cfg: FleetConfig = DEFAULT, drop: Iterable[str] = (), extra: Iterable[Any] = ()) -> Any:
    """Conjunction of the invariants (minus the short names in ``drop``) and of ``extra``."""
    drop = set(drop)
    return L.And(*[inv.pred(o, L, cfg) for inv in INVARIANTS if inv.name.split()[0] not in drop], *extra)


def violated(o, cfg: FleetConfig = DEFAULT) -> List[str]:
    """Short names of the invariants violated by a runtime observation (empty: consistent)."""
    bad = []
    for inv in INVARIANTS:
        try:
            ok = bool(inv.pred(o, PY_INV, cfg))
        except TypeError:                     # e.g. a None real compared with a threshold
            ok = False
        if not ok:
            bad.append(inv.name.split()[0])
    return bad


def consistent(o, cfg: FleetConfig = DEFAULT) -> bool:
    return not violated(o, cfg)


def table() -> List[Dict[str, str]]:
    """Documentation rows (README / REPORT / formal summary)."""
    return [{"name": inv.name, "formula": inv.formula, "meaning": inv.meaning, "why": inv.why,
             "excludes": inv.excludes} for inv in INVARIANTS]
