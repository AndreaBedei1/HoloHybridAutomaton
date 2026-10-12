"""Communication-free mutual-exclusion (MUTEX) precedence on SECTOR PATTERNS (shared runtime / Z3).

A queued drone never knows where its neighbours are, only which of its six sonars see them, and it
knows the shared mission plan.  The rule maps a neighbour's sector pattern to a pairwise relation:

    FRONT or LEFT in the pattern          -> WAIT      (ahead of me or on my left: it goes first)
    otherwise REAR and/or RIGHT           -> PRIORITY  (behind me or on my right: I go first)
    no horizontal sector, only UP         -> WAIT      (stacked above me: the upper drone goes first)
    no horizontal sector, only DOWN       -> PRIORITY  (stacked below me)
    no horizontal sector, UP and DOWN     -> RANK      (one target spanning above and below: ambiguous)

i.e. precedence to the LEFT, then from the TOP down, and the static rank only as the very last
tie-break, for a pattern that cannot be ordered geometrically.  The static rank is the slot order of
the shared plan, compared with the plan's drones whose queue points lie within the queue bracket
(GatePerception): no identity is ever exchanged.  It is counted in every run (0 so far).

Horizontal bearing beta of the neighbour from the own bow (left positive); cones of half-angle 60
deg, so F = (-60, 60), L = (30, 150), B = (120, 240), R = (-150, -30).  PRIORITY needs the
neighbour outside F and L, i.e. beta in (150, 300).  Seen from the neighbour the bearing is
beta' = beta + 180 - delta (delta = heading difference), which then lies in (-30, 120) - delta,
inside F or L.  A cone boundary is uncertain by ``fuzz`` (hull angular size, octree leaf): with
|delta| + 2 fuzz <= 30 deg the two drones of a pair are never both PRIORITY
(formal/check_mutex.py proves it on the bearing model).  For a vertically stacked pair the upper
drone sees the lower one only in DOWN and the lower one sees the upper only in UP: exactly one
PRIORITY, without any rank (formal M2v).

Both can WAIT (e.g. beta = 135: L for one, F for the other).  This never happens between two
drones standing at their queue points: those are abreast (or stacked), so the relation is pure
LEFT/RIGHT (or UP/DOWN) with a margin to the overlap bands, and exactly one queued drone - the
leftmost, then the upper - has no WAIT relation, for every set of occupied queue points with at most
one vacant point between two occupied ones (formal M2: the progress / no-deadlock property of the
queue).  A WAIT caused by a drone that is not at its queue point (approaching, or passing the gate)
ends when that drone reaches its queue point or moves beyond the critical region.  Two different
drones at similar ranges in adjacent sectors may be associated into one target (e.g. FRONT+RIGHT):
the rule still answers WAIT, the safe answer for both readings.

The same functions are evaluated with Python bools at runtime and with Z3 terms in formal/.
"""

from __future__ import annotations

from typing import Iterable

WAIT, PRIORITY, RANK = "WAIT", "PRIORITY", "RANK"


def relation(F, B, L, R, U, D, Lg):
    """Pairwise relation predicates for one neighbour, from its sector-membership flags.

    Returns a dict of mutually exclusive predicates {WAIT, PRIORITY, RANK, NONE}
    (``NONE`` = not seen by any sector).  ``Lg`` is a logic backend with And/Or/Not.
    """
    horiz = Lg.Or(F, B, L, R)
    wait_h = Lg.Or(F, L)                                      # ahead or on the left
    up_only = Lg.And(Lg.Not(horiz), U, Lg.Not(D))            # stacked above me
    down_only = Lg.And(Lg.Not(horiz), D, Lg.Not(U))          # stacked below me
    wait = Lg.Or(wait_h, up_only)
    priority = Lg.Or(Lg.And(Lg.Not(wait_h), Lg.Or(B, R)), down_only)
    rank = Lg.And(Lg.Not(horiz), U, D)                        # ambiguous: above and below at once
    none = Lg.And(Lg.Not(horiz), Lg.Not(U), Lg.Not(D))
    return {WAIT: wait, PRIORITY: priority, RANK: rank, "NONE": none}


class _Py:
    @staticmethod
    def And(*a):
        return all(bool(x) for x in a)

    @staticmethod
    def Or(*a):
        return any(bool(x) for x in a)

    @staticmethod
    def Not(a):
        return not bool(a)


PY_RULE_LOGIC = _Py          # Python backend of ``relation`` (numeric checks in formal/check_mutex.py)


def classify_pattern(pattern: Iterable[str]) -> str:
    p = set(pattern)
    rel = relation("FRONT" in p, "REAR" in p, "LEFT" in p, "RIGHT" in p, "UP" in p, "DOWN" in p, _Py)
    for k in (WAIT, PRIORITY, RANK):
        if rel[k]:
            return k
    return "NONE"


def decide(relations: Iterable[str], my_rank: int, their_ranks: Iterable[int] = ()) -> str:
    """Own decision from the relations to every perceived queue neighbour.

    WAIT dominates; an ambiguous RANK relation is resolved by the static rank: wait if a drone of the
    shared plan whose queue point lies within the bracket (``their_ranks``) has a lower rank; else PRIORITY.
    """
    rel = list(relations)
    if WAIT in rel:
        return WAIT
    ranks = list(their_ranks)
    if RANK in rel and any(r < my_rank for r in ranks):
        return "WAIT_RANK"
    return PRIORITY
