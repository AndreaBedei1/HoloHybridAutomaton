"""Robust, communication-free priority rule for the critical region (shared runtime/Z3).

Drone i evaluates, on its OWN measurement of the relative position of drone j in
the gate frame (ds = s_i - s_j, dl = l_i - l_j, dz = z_i - z_j), a pairwise
decision that is exhaustive and mutually exclusive (checked in formal/check_mutex.py):

  COMMIT        i beats j: lexicographic order (closer to the gate, then left, then
                upper) where each level is decisive only beyond a margin and the next
                level is consulted only inside a strictly narrower tie band;
  WAIT_ROBUST   j beats i even if both measurements were off by up to eps: j will commit;
  BACKOFF_REAR  i is robustly behind (ds < -mu_s_lo) but nobody is decisive: i retreats;
  WAIT_FRONT    i is robustly ahead (ds > mu_s_lo) but not decisively: the rear drone retreats;
  BACKOFF_RIGHT along tie and i robustly on the right: i retreats along the axis;
  BACKOFF_RANK  complete tie: retreat by a pre-configured static rank (last resort, no messages).

Proved properties (Z3): two drones never both COMMIT (mutual exclusion of commits) and
never both WAIT (no deadlock): at least one commits or retreats.

Every function takes a logic backend ``L`` (Python or Z3) and an ``absf``.
"""

from __future__ import annotations

from typing import Any, Callable, Dict

from holo_fleet.config import GateRule

DECISIONS = ("COMMIT", "WAIT_ROBUST", "BACKOFF_REAR", "WAIT_FRONT", "BACKOFF_RIGHT", "BACKOFF_RANK")
BACKOFFS = ("BACKOFF_REAR", "BACKOFF_RIGHT", "BACKOFF_RANK")
WAITS = ("WAIT_ROBUST", "WAIT_FRONT")


def beats(ds: Any, dl: Any, dz: Any, G: GateRule, L: Any, absf: Callable[[Any], Any] = abs) -> Any:
    return L.Or(
        ds > G.mu_s_hi,
        L.And(absf(ds) <= G.mu_s_lo, dl > G.mu_l_hi),
        L.And(absf(ds) <= G.mu_s_lo, absf(dl) <= G.mu_l_lo, dz > G.mu_z),
    )


def beaten_robustly(ds: Any, dl: Any, dz: Any, G: GateRule, eps: float, L: Any, absf=abs) -> Any:
    """The other drone beats me even if each of the two measurements is off by up to eps."""
    m = 2.0 * eps
    return L.Or(
        ds < -(G.mu_s_hi + m),
        L.And(absf(ds) <= G.mu_s_lo - m, dl < -(G.mu_l_hi + m)),
        L.And(absf(ds) <= G.mu_s_lo - m, absf(dl) <= G.mu_l_lo - m, dz < -(G.mu_z + m)),
    )


def decision_predicates(ds: Any, dl: Any, dz: Any, G: GateRule, eps: float, L: Any, absf=abs) -> Dict[str, Any]:
    """Exclusive predicates for every decision class (precedence encoded with negations)."""
    commit = beats(ds, dl, dz, G, L, absf)
    wait_r = L.And(L.Not(commit), beaten_robustly(ds, dl, dz, G, eps, L, absf))
    undecided = L.And(L.Not(commit), L.Not(wait_r))
    rear = L.And(undecided, ds < -G.mu_s_lo)
    front = L.And(undecided, ds > G.mu_s_lo)
    tie_s = L.And(undecided, absf(ds) <= G.mu_s_lo)
    right = L.And(tie_s, dl < -G.mu_l_lo)
    rank = L.And(tie_s, L.Not(dl < -G.mu_l_lo))
    return {"COMMIT": commit, "WAIT_ROBUST": wait_r, "BACKOFF_REAR": rear, "WAIT_FRONT": front,
            "BACKOFF_RIGHT": right, "BACKOFF_RANK": rank}


def decide(ds: float, dl: float, dz: float, G: GateRule, eps: float) -> str:
    """Python-side decision (the unique true predicate)."""
    from holo_fleet.ha.spec import PY_LOGIC

    preds = decision_predicates(ds, dl, dz, G, eps, PY_LOGIC)
    true = [k for k, v in preds.items() if v]
    assert len(true) == 1, (ds, dl, dz, true)
    return true[0]


def backoff_clearance(G: GateRule, eps: float) -> float:
    """Distance a retreating drone puts between itself and the other one (along the axis), enough
    for the other drone to measure a decisive along-track lead (> mu_s_hi) despite eps errors."""
    return G.mu_s_hi + 2.0 * eps + 0.3
