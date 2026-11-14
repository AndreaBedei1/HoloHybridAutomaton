"""Unit suite, one pytest process per file at below-normal priority: python scripts/run_tests.py <out dir> [workers]; then the formal suite."""
import subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
N = int(sys.argv[2]) if len(sys.argv) > 2 else 20
BN = 0x00004000


def run(f):
    t0 = time.time()
    p = subprocess.run([sys.executable, "-m", "pytest", str(f), "-q", "-p", "no:cacheprovider", "--tb=short"], cwd=ROOT, stdin=subprocess.DEVNULL,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, creationflags=BN)
    (out / (f.stem + ".txt")).write_text(p.stdout, encoding="utf-8")
    last = [l for l in p.stdout.splitlines() if l.strip()][-1:] or ["?"]
    return f.name, p.returncode, round(time.time() - t0, 1), last[0]


p = subprocess.run([sys.executable, "formal/check_properties.py"], cwd=ROOT, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, creationflags=BN)
(out / "formal.txt").write_text(p.stdout, encoding="utf-8")
formal_line = f"FORMAL rc {p.returncode}: " + " | ".join(p.stdout.splitlines()[-2:])
files = sorted((ROOT / "tests").glob("test_*.py"))
with ThreadPoolExecutor(N) as ex:
    res = list(ex.map(run, files))
bad = [r for r in res if r[1] != 0]
lines = [f"{r[0]:48s} rc {r[1]} {r[2]:7.1f}s {r[3]}" for r in res]
lines.append(f"FILES {len(res)} FAILED {len(bad)}")
lines.append(formal_line)
(out / "summary.txt").write_text("\n".join(lines), encoding="utf-8")
print("\n".join(l for l in lines if " rc 0 " not in l))
