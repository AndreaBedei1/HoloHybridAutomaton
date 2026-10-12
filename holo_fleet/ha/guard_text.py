"""Readable guards of the automaton: the ``label`` of every spec edge (holo_fleet/ha/spec.py).

A label is a Python boolean expression over the abstract observation variables (spec.REAL_VARS,
spec.BOOL_VARS) and the configured thresholds (:func:`thresholds`), e.g. ``d_ca <= d_min < d_warning``.  It
is evaluated with the same logic backends as the guards - Python bools, or Z3 terms in formal/ - so
formal/check_determinism.py proves that, under the priority order of the edges, it selects exactly the
observations the real guard selects.  :func:`pretty` renders it for the automaton diagram and the fleet
view.  Nothing here is used to take a decision: the guards are the spec functions themselves.
"""

from __future__ import annotations

import ast
from typing import Any, Dict, Set

from holo_fleet.config import DEFAULT, FleetConfig


def thresholds(cfg: FleetConfig = DEFAULT) -> Dict[str, float]:
    """Names of the configured thresholds a label may use, with their values."""
    return {"d_ca": cfg.sep.d_ca, "d_ca_exit": cfg.sep.d_ca_exit, "d_warning": cfg.sep.d_warning,
            "d_warning_exit": cfg.sep.d_warning_exit, "e_lost": cfg.form.e_lost, "e_ok": cfg.form.e_ok,
            "t_ok_hold": cfg.form.t_ok_hold}


UNITS = {"d_ca": "m", "d_ca_exit": "m", "d_warning": "m", "d_warning_exit": "m", "e_lost": "m", "e_ok": "m",
         "t_ok_hold": "s"}

_CMP = {ast.Lt: (lambda a, b: a < b, "<"), ast.LtE: (lambda a, b: a <= b, "≤"),
        ast.Gt: (lambda a, b: a > b, ">"), ast.GtE: (lambda a, b: a >= b, "≥")}


def names(expr: str) -> Set[str]:
    """Every name used by a label."""
    return {n.id for n in ast.walk(ast.parse(expr, mode="eval")) if isinstance(n, ast.Name)}


def evaluate(expr: str, o: Any, L, cfg: FleetConfig = DEFAULT):
    """Value of the label for the observation ``o`` with the logic backend ``L`` (And / Or / Not)."""
    th = thresholds(cfg)

    def ev(n):
        if isinstance(n, ast.Expression):
            return ev(n.body)
        if isinstance(n, ast.BoolOp):
            args = [ev(v) for v in n.values]
            return L.And(*args) if isinstance(n.op, ast.And) else L.Or(*args)
        if isinstance(n, ast.UnaryOp) and isinstance(n.op, ast.Not):
            return L.Not(ev(n.operand))
        if isinstance(n, ast.Compare):
            left, parts = ev(n.left), []
            for op, right in zip(n.ops, n.comparators):
                r = ev(right)
                parts.append(_CMP[type(op)][0](left, r))
                left = r
            return parts[0] if len(parts) == 1 else L.And(*parts)
        if isinstance(n, ast.Name):
            return th[n.id] if n.id in th else getattr(o, n.id)
        if isinstance(n, ast.Constant) and isinstance(n.value, (int, float)):
            return n.value
        raise ValueError(f"unsupported construct in a guard label: {ast.dump(n)}")

    return ev(ast.parse(expr, mode="eval"))


def pretty(expr: str, cfg: FleetConfig = DEFAULT, values: bool = False) -> str:
    """Label in mathematical notation (¬ ∧ ∨ ≤ ≥); with ``values`` the thresholds are replaced by their values."""
    th = thresholds(cfg)

    def pr(n, parent=None):
        if isinstance(n, ast.Expression):
            return pr(n.body)
        if isinstance(n, ast.BoolOp):
            sym = " ∧ " if isinstance(n.op, ast.And) else " ∨ "
            txt = sym.join(pr(v, n) for v in n.values)
            return f"({txt})" if parent is not None else txt
        if isinstance(n, ast.UnaryOp) and isinstance(n.op, ast.Not):
            inner = pr(n.operand, n)
            return "¬" + inner
        if isinstance(n, ast.Compare):
            out = pr(n.left, n)
            for op, right in zip(n.ops, n.comparators):
                out += f" {_CMP[type(op)][1]} {pr(right, n)}"
            return f"({out})" if isinstance(parent, ast.UnaryOp) else out
        if isinstance(n, ast.Name):
            if values and n.id in th:
                return f"{th[n.id]:g} {UNITS[n.id]}"
            return n.id
        if isinstance(n, ast.Constant):
            return f"{n.value:g}"
        raise ValueError(ast.dump(n))

    return pr(ast.parse(expr, mode="eval"))
