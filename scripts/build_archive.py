"""Build a versioned, reproducible archive of the scientific evidence outside the repository.

    python scripts/build_archive.py --dest F:/Andrea/HoloFleet_ARCHIVE --version v12-review \
        --refs b2-freeze-v11 b2-v11-validation HEAD --runs results/robustness/raw_v11_final ... \
        --docs REPORT.md analysis docs/SENSOR_ROBUSTNESS_LITERATURE.md ... --evidence results/validation/report ...

Layout of <dest>/<version>/:
  code/        a git bundle of every ref (full history, restorable with ``git clone``) and a source tarball per ref;
  runs/        the raw run directories (logs, configurations, seeds, metrics), copied with their relative paths;
  reports/     the scientific reports, reviews and development logs moved out of the active repository;
  evidence/    tables, figures, audits and formal results;
  MANIFEST.json  every file with size and sha256, the commit of every ref, the command line and the date.
Nothing is deleted from the repository by this script; files are copied.  An existing version directory is refused.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def sha256(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def git(*args) -> str:
    return subprocess.run(["git", *args], cwd=ROOT, check=True, capture_output=True, text=True).stdout.strip()


def copy_into(src: Path, dest_root: Path) -> None:
    rel = src.resolve().relative_to(ROOT) if src.resolve().is_relative_to(ROOT) else Path(src.name)
    target = dest_root / rel
    if src.is_dir():
        shutil.copytree(src, target, dirs_exist_ok=False)
    else:
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, target)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dest", required=True)
    ap.add_argument("--version", required=True)
    ap.add_argument("--refs", nargs="+", required=True)
    ap.add_argument("--runs", nargs="*", default=[])
    ap.add_argument("--docs", nargs="*", default=[])
    ap.add_argument("--evidence", nargs="*", default=[])
    a = ap.parse_args(argv)
    out = Path(a.dest) / a.version
    if out.exists():
        raise SystemExit(f"{out} exists: an archive version is never overwritten")
    for sub in ("code", "runs", "reports", "evidence"):
        (out / sub).mkdir(parents=True)
    refs = {r: git("rev-parse", f"{r}^{{commit}}") for r in a.refs}  # the commit, also for an annotated tag
    git("bundle", "create", str(out / "code" / "repository.bundle"), "--all")
    for r, sha in refs.items():
        name = r.replace("/", "_")
        git("archive", "--format=tar.gz", f"--output={out / 'code' / f'{name}_{sha[:7]}.tar.gz'}", sha)
    for group, items in (("runs", a.runs), ("reports", a.docs), ("evidence", a.evidence)):
        for item in items:
            p = Path(item)
            if not p.exists():
                raise SystemExit(f"missing: {p}")
            copy_into(p, out / group)
    files = []
    for p in sorted(out.rglob("*")):
        if p.is_file():
            files.append({"path": p.relative_to(out).as_posix(), "bytes": p.stat().st_size, "sha256": sha256(p)})
    manifest = {"version": a.version, "created": time.strftime("%Y-%m-%d %H:%M:%S"), "refs": refs,
                "command": " ".join(sys.argv), "files": len(files),
                "bytes": sum(f["bytes"] for f in files), "entries": files}
    (out / "MANIFEST.json").write_text(json.dumps(manifest, indent=1), encoding="utf-8")
    print(f"{out}: {len(files)} files, {manifest['bytes'] / 1e6:.1f} MB; refs {refs}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
