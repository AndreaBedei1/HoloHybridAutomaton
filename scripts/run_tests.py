"""Run formal verification and one pytest process per test file at below-normal priority."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
BN = getattr(subprocess, "BELOW_NORMAL_PRIORITY_CLASS", 0)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path, help="directory for subprocess logs")
    parser.add_argument("workers", type=int, nargs="?", default=20)
    parser.add_argument("--quick", action="store_true", help="use the development suites")
    args = parser.parse_args(argv)
    if args.workers < 1:
        parser.error("workers must be positive")
    args.out.mkdir(parents=True, exist_ok=True)

    def run(command):
        return subprocess.run([sys.executable, *command], cwd=ROOT, stdin=subprocess.DEVNULL,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, creationflags=BN)

    def run_test(path):
        start = time.monotonic()
        markers = ["-m", "not slow and not holoocean"] if args.quick else []
        result = run(["-m", "pytest", str(path), "-q", "-p", "no:cacheprovider", "--tb=short", *markers])
        (args.out / (path.stem + ".txt")).write_text(result.stdout, encoding="utf-8")
        last = result.stdout.rstrip().splitlines()
        return path.name, result.returncode, time.monotonic() - start, last[-1] if last else "(no output)"

    formal = run(["formal/check_properties.py", *(["--quick"] if args.quick else [])])
    (args.out / "formal.txt").write_text(formal.stdout, encoding="utf-8")
    files = sorted((ROOT / "tests").glob("test_*.py"))
    with ThreadPoolExecutor(args.workers) as workers:
        results = list(workers.map(run_test, files))
    failed = [r for r in results if r[1] != 0]
    lines = [f"{name:48s} rc {code} {elapsed:7.1f}s {last}" for name, code, elapsed, last in results]
    footer = [f"FILES {len(results)} FAILED {len(failed)}",
              f"FORMAL rc {formal.returncode}: " + " | ".join(formal.stdout.splitlines()[-2:])]
    (args.out / "summary.txt").write_text("\n".join(lines + footer) + "\n", encoding="utf-8")
    print("\n".join([line for line, result in zip(lines, results) if result[1] != 0] + footer))
    return int(bool(failed or formal.returncode or not results))


if __name__ == "__main__":
    raise SystemExit(main())
