"""The formal scripts run and every check returns its expected verdict (fast suites)."""

import sys
from pathlib import Path

import pytest

FORMAL = Path(__file__).resolve().parents[1] / "formal"
sys.path.insert(0, str(FORMAL))


def test_determinism_suite():
    import check_determinism

    rep = check_determinism.run(verbose=False)
    assert rep.ok, [r.prop for r in rep.results if not r.passed]


def test_mutex_suite():
    import check_mutex

    rep = check_mutex.run(verbose=False)
    assert rep.ok, [r.prop for r in rep.results if not r.passed]


@pytest.mark.slow
def test_separation_suite():
    import check_separation

    rep = check_separation.run(verbose=False)
    assert rep.ok, [r.prop for r in rep.results if not r.passed]


def test_formation_results_if_available():
    import json

    p = FORMAL / "results" / "formation.json"
    if not p.exists():
        pytest.skip("P3 BMC suite not run yet (python formal/check_formation.py)")
    data = json.loads(p.read_text())
    assert data["all_passed"], [r["prop"] for r in data["results"] if not r["passed"]]
