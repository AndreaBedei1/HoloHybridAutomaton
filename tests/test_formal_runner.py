"""The development run must leave validation evidence and skipped proofs intact."""

import json

from formal import check_properties as formal
from common import CheckResult, Report


def test_quick_formal_run_uses_only_fast_checks_and_keeps_full_results(monkeypatch, tmp_path):
    class Suite:
        def __init__(self, name):
            self.name = name
            self.calls = []

        def run(self, **kwargs):
            self.calls.append(kwargs)
            report = Report(self.name)
            report.add(CheckResult(self.name, "", "", "counterexample", "counterexample", True,
                                   counterexample={"witness": 1}), verbose=False)
            return report

    quick = Suite("fast")
    slow = Suite("slow")
    comm = Suite("comm")
    monkeypatch.setattr(formal, "SUITES", [
        ("Fast", quick, "fast.py"), ("Slow", slow, "slow.py"), ("B2", comm, "comm.py"),
    ])
    monkeypatch.setattr(formal, "QUICK_SUITES", {quick, comm})
    monkeypatch.setattr(formal, "check_b2_comm", comm)
    full = tmp_path / "results"
    full.mkdir()
    (full / "SUMMARY.json").write_text("full evidence", encoding="utf-8")
    monkeypatch.setattr(formal, "RESULTS_DIR", full)

    assert formal.main(["--quick"]) == 0
    assert quick.calls == [{"verbose": True}]
    assert slow.calls == []
    assert comm.calls == [{"verbose": True, "step": False}]
    assert (full / "SUMMARY.json").read_text(encoding="utf-8") == "full evidence"
    assert {p.name for p in (tmp_path / "results_quick").iterdir()} == {
        "fast.json", "comm.json", "SUMMARY.json", "SUMMARY.csv",
    }

    assert formal.main([]) == 0
    assert slow.calls == [{"verbose": True}]
    assert comm.calls[-1] == {"verbose": True}
    assert (full / "slow.json").is_file()
    summary = json.loads((full / "SUMMARY.json").read_text(encoding="utf-8"))
    assert summary["suites"][1]["checks"][0]["counterexample"] == {"witness": 1}

def test_combined_runner_returns_failure_if_formal_verification_fails(monkeypatch, tmp_path):
    import subprocess

    from scripts import run_tests

    test_dir = tmp_path / "tests"
    test_dir.mkdir()
    (test_dir / "test_example.py").touch()
    calls = []

    def run(command, **kwargs):
        calls.append(command[1:])
        failed = command[1] == "formal/check_properties.py"
        return subprocess.CompletedProcess(command, int(failed), "failed\n" if failed else "1 passed\n")

    monkeypatch.setattr(run_tests, "ROOT", tmp_path)
    monkeypatch.setattr(run_tests.subprocess, "run", run)
    assert run_tests.main([str(tmp_path / "logs"), "1", "--quick"]) == 1
    assert calls[0] == ["formal/check_properties.py", "--quick"]
    assert calls[1][-2:] == ["-m", "not slow and not holoocean"]
    assert "FORMAL rc 1" in (tmp_path / "logs" / "summary.txt").read_text(encoding="utf-8")
