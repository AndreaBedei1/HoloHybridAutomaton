"""Shared Z3 helpers for the formal checks.

Every check records: property name, readable formula, encoding file, solver
verdict (UNSAT = property holds for the abstraction / SAT = counterexample) and,
for SAT, the counterexample model.  Results are collected in formal/results/.
"""

from __future__ import annotations

import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import z3

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from holo_fleet.ha import observation_invariants as OI  # noqa: E402
from holo_fleet.ha.spec import BOOL_VARS, REAL_VARS, Logic  # noqa: E402

RESULTS_DIR = ROOT / "formal" / "results"


class Z3Logic(Logic):
    def And(self, *args):
        return z3.And(*args)

    def Or(self, *args):
        return z3.Or(*args)

    def Not(self, a):
        return z3.Not(a)

    # observation invariant N0 (holo_fleet/ha/observation_invariants.py): Z3 Real and Bool symbols are
    # finite and Boolean by construction, so the runtime type checks are trivially true here
    def finite(self, x):
        return z3.BoolVal(True)

    def boolean(self, b):
        return z3.BoolVal(True)


Z3L = Z3Logic()


def exact(x: float) -> z3.ArithRef:
    """Exact rational for a configuration constant (shortest decimal repr, e.g. 0.35 -> 7/20).

    Never let Python pre-compute float arithmetic on thresholds that Z3 then compares exactly: float
    rounding (2*0.35 + 2.4 = 3.0999999999999996) can open a spurious 1e-16 gap between two classes.
    """
    return z3.RealVal(repr(float(x)))


class ExactNamespace:
    """Copy of a config dataclass whose float fields are exact Z3 rationals."""

    def __init__(self, dc):
        for k, v in dc.__dict__.items():
            setattr(self, k, exact(v) if isinstance(v, float) else v)


class Obs:
    """Symbolic abstract observation (same attribute names as the runtime dataclass)."""

    def __init__(self, suffix: str = ""):
        for v in REAL_VARS:
            setattr(self, v, z3.Real(v + suffix))
        for v in BOOL_VARS:
            setattr(self, v, z3.Bool(v + suffix))

    def legal(self, cfg, drop=(), extra=()) -> z3.BoolRef:
        """The observations the perception layer can produce: the shared semantic invariants of
        holo_fleet/ha/observation_invariants.py (the same predicate as the runtime consistency check),
        minus the short names in ``drop`` (mutation tests), plus ``extra``."""
        return OI.legal(self, Z3L, cfg, drop=drop, extra=extra)


@dataclass
class CheckResult:
    prop: str
    formula: str
    encoding: str
    expect: str                 # "unsat" (property holds) or "sat" (counterexample expected, e.g. mutation)
    verdict: str = ""
    passed: bool = False
    seconds: float = 0.0
    counterexample: Optional[Dict[str, Any]] = None
    note: str = ""


def _natural_key(name: str):
    import re

    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", name)]


def model_to_dict(m: z3.ModelRef, limit: int = 400) -> Dict[str, Any]:
    out = {}
    for d in sorted(m.decls(), key=lambda x: _natural_key(x.name()))[:limit]:
        v = m[d]
        if z3.is_rational_value(v):
            out[d.name()] = float(v.numerator_as_long()) / float(v.denominator_as_long())
        elif z3.is_algebraic_value(v):
            out[d.name()] = float(v.approx(10).as_fraction())
        elif z3.is_true(v) or z3.is_false(v):
            out[d.name()] = z3.is_true(v)
        else:
            out[d.name()] = str(v)
    return out


def check(prop: str, formula: str, encoding: str, constraints: List[z3.BoolRef], expect: str = "unsat",
          timeout_ms: int = 120000, note: str = "") -> CheckResult:
    s = z3.Solver()
    s.set("timeout", timeout_ms)
    s.add(*constraints)
    t0 = time.time()
    r = s.check()
    res = CheckResult(prop=prop, formula=formula, encoding=encoding, expect=expect, note=note)
    res.seconds = round(time.time() - t0, 3)
    res.verdict = str(r)
    res.passed = (str(r) == expect)
    if r == z3.sat:
        res.counterexample = model_to_dict(s.model())
    return res


def format_counterexample(cex: Dict[str, Any], max_series: int = 40) -> str:
    """Group trace variables x_0, x_1, ... into time series for readability."""
    import re

    series: Dict[str, List] = {}
    scalars = []
    for k, v in cex.items():
        mt = re.fullmatch(r"([A-Za-z]+)_(\d+)", k)
        if mt:
            series.setdefault(mt.group(1), []).append((int(mt.group(2)), v))
        else:
            scalars.append(f"{k}={round(v, 3) if isinstance(v, float) else v}")
    parts = scalars[:30]
    for name, vals in series.items():
        vals.sort()
        shown = [round(v, 3) if isinstance(v, float) else v for _, v in vals[:max_series]]
        parts.append(f"{name}[0..{len(vals) - 1}]={shown}{' ...' if len(vals) > max_series else ''}")
    return "; ".join(parts)


class Report:
    def __init__(self, name: str):
        self.name = name
        self.results: List[CheckResult] = []

    def add(self, r: CheckResult, verbose: bool = True) -> CheckResult:
        r.passed = bool(r.passed)                    # numpy booleans from numeric lemmas -> plain JSON booleans
        self.results.append(r)
        if verbose:
            flag = "PASS" if r.passed else "FAIL"
            print(f"[{flag}] {r.prop}: {r.verdict.upper()} (expected {r.expect.upper()}) in {r.seconds}s")
            if r.counterexample is not None and (not r.passed or r.expect == "sat"):
                print("       counterexample: " + format_counterexample(r.counterexample))
        return r

    def save(self) -> Path:
        RESULTS_DIR.mkdir(parents=True, exist_ok=True)
        path = RESULTS_DIR / f"{self.name}.json"
        payload = {"check": self.name, "all_passed": all(r.passed for r in self.results),
                   "results": [r.__dict__ for r in self.results]}
        path.write_text(json.dumps(payload, indent=2, default=str))
        return path

    @property
    def ok(self) -> bool:
        return all(r.passed for r in self.results)
