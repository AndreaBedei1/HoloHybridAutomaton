"""The formal scripts run and every check returns its expected verdict (mutations included)."""

import sys
from pathlib import Path

import pytest

FORMAL = Path(__file__).resolve().parents[1] / "formal"
sys.path.insert(0, str(FORMAL))


def _failed(rep):
    return [r.prop for r in rep.results if not r.passed]


def test_determinism_suite():
    import check_determinism

    rep = check_determinism.run(verbose=False)
    assert rep.ok, _failed(rep)


def test_formation_liveness_suite():
    import check_formation

    rep = check_formation.run(verbose=False)
    assert rep.ok, _failed(rep)


@pytest.mark.slow
def test_mutex_suite():
    import check_mutex

    rep = check_mutex.run(verbose=False)
    assert rep.ok, _failed(rep)


@pytest.mark.slow
def test_separation_suite():
    import check_separation

    rep = check_separation.run(verbose=False, n_poses=6000)
    assert rep.ok, _failed(rep)


def test_saved_summary_is_all_passed():
    import json

    p = FORMAL / "results" / "SUMMARY.json"
    if not p.exists():
        pytest.skip("python formal/check_properties.py not run yet")
    data = json.loads(p.read_text())
    assert data["all_passed"], [c["prop"] for s in data["suites"] for c in s["checks"] if not c["passed"]]
