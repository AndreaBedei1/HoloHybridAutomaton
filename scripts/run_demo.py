"""Watch one short demonstration scenario (one seed) with the live fleet view.

    python scripts/run_demo.py --list
    python scripts/run_demo.py --scenario lost_drone_rejoin                       # live only, nothing kept
    python scripts/run_demo.py --scenario mutex_deadlock_resolution --headless   # no HoloOcean window
    python scripts/run_demo.py --scenario sonar_classification                   # echo classifier bench, A-F
    python scripts/run_demo.py --scenario lost_drone_rejoin --save               # keep a NEW result
    python scripts/run_demo.py --scenario lost_drone_rejoin --save --overwrite   # replace the canonical result
    python scripts/run_demo.py --scenario lost_drone_rejoin --save --overwrite --media   # ... with GIF / frames

Two windows: the HoloOcean viewport (the onboard belief is drawn on it: nearest echo per sonar
sector coloured by class - grey structure, brown seabed, red possible vehicle, amber unknown - and
the escape direction in magenta) and the fleet view (top view of the fleet with the formation slots
and the gate queue, P1 / P2 / P3 status, one row per drone, readable events, automaton timeline;
holo_fleet/ui/fleet_view.py).  The post-run summary is printed at the end.  Ctrl+C stops the run.

By default a demo is LIVE ONLY: the files the run needs (logs read back for the summary) live in a
temporary folder that is deleted at the end; no GIF, no frames, nothing under results/, nothing in
git status.  ``--save`` keeps a COMPLETE run in results/v2/demos/<scenario>/ (logs and metrics) and
refuses to replace an existing result unless ``--overwrite`` is given; an incomplete run is never
saved.  ``--media`` (with ``--save``) also keeps the fleet-view / chase frames and the GIF.  ``--out``
(with ``--save``) saves under another folder, for tests and smoke runs.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.config import DEFAULT  # noqa: E402
from holo_fleet.runner import print_summary, run  # noqa: E402
from holo_fleet.sim.scenarios import SCENARIOS  # noqa: E402
from holo_fleet.ui.gif import make_gif  # noqa: E402
from holo_fleet.ui.live import DemoUI  # noqa: E402

OUT = ROOT / "results" / "v2" / "demos"
STAGE_PREFIX = "holo_fleet_demo_"
MEDIA_DIRS = ("fleet_view", "frames", "dashboard")       # frame sequences (GIFs: *.gif)


def _alive(pid: int) -> bool:
    try:
        import psutil
    except ImportError:                                   # cannot tell: keep the folder
        return True
    return psutil.pid_exists(pid)


def _clean_orphan_stages() -> None:
    """Remove staging folders of demos that were killed.  Closing the HoloOcean window ends the
    process at once (sim/engine_watchdog.py), so that demo could not delete its own folder."""
    for d in Path(tempfile.gettempdir()).glob(STAGE_PREFIX + "*"):
        try:
            orphan = not _alive(int((d / "owner.pid").read_text(encoding="utf-8")))
        except (OSError, ValueError):                     # no pid yet: only when clearly stale
            orphan = time.time() - d.stat().st_mtime > 3600.0
        if orphan:
            shutil.rmtree(d, ignore_errors=True)


def _strip_media(run_dir: Path) -> None:
    for sub in MEDIA_DIRS:
        shutil.rmtree(run_dir / sub, ignore_errors=True)
    for gif in run_dir.glob("*.gif"):
        gif.unlink()


def _promote(src: Path, dest: Path, overwrite: bool) -> None:
    """Copy the staged run to ``dest``; an existing result is replaced only with ``overwrite``, and only
    once the new copy is complete."""
    if dest.exists() and not overwrite:
        raise FileExistsError(dest)
    dest.parent.mkdir(parents=True, exist_ok=True)
    new, old = dest.with_name(f".{dest.name}.new"), dest.with_name(f".{dest.name}.old")
    for d in (new, old):
        shutil.rmtree(d, ignore_errors=True)
    shutil.copytree(src, new)
    if dest.exists():
        dest.rename(old)
    new.rename(dest)
    shutil.rmtree(old, ignore_errors=True)


def _run_scenario(a, stage: Path) -> str:
    """Run in ``stage`` and return the run status."""
    if a.scenario == "sonar_classification":
        sys.path.insert(0, str(ROOT / "scripts"))
        from demo_classification import run_classification_demo  # noqa: E402

        run_classification_demo(stage, headless=a.headless, show=not a.no_window, media=a.media)
        try:
            return json.loads((stage / a.scenario / "run_status.json").read_text(encoding="utf-8"))["status"]
        except (OSError, ValueError, KeyError):
            return "INCOMPLETE"
    sc = SCENARIOS[a.scenario](DEFAULT)
    ui = DemoUI(sc, DEFAULT, sc.sim.names, show=not a.no_window, draw_viewport=not a.headless, selected=a.select)
    m = run(a.scenario, stage, headless=a.headless, run_id=a.scenario, duration=a.duration, ui=ui,
            frame_every_s=0.5 if a.media else math.inf)          # no frame is written without --media
    print_summary(a.scenario, m)
    if a.media:
        gif = make_gif((stage / a.scenario / "fleet_view").glob("fleet_*.jpg"), stage / a.scenario / f"{a.scenario}.gif",
                       width=1280, fps=6.0)
        if gif:
            print(f"fleet-view GIF: {gif}")
    return m["run"]["status"]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", choices=sorted(SCENARIOS) + ["sonar_classification"])
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--headless", action="store_true", help="no HoloOcean viewport")
    ap.add_argument("--no-window", action="store_true", help="no fleet-view window")
    ap.add_argument("--save", action="store_true", help="keep a complete run in results/v2/demos/<scenario>/")
    ap.add_argument("--overwrite", action="store_true", help="with --save: replace an existing result")
    ap.add_argument("--media", action="store_true", help="with --save: also keep frames and the GIF")
    ap.add_argument("--out", default=None, help="with --save: save under this folder instead of results/v2/demos")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--select", type=int, default=None, metavar="K",
                    help="drone shown in the fleet view's automaton panel (default: the scenario's choice)")
    a = ap.parse_args(argv)
    if a.list or not a.scenario:
        print("scenarios (one seed each):")
        for k in ["sonar_classification"] + list(SCENARIOS):
            if k == "sonar_classification":
                print(f"  {k:24s} ~28 s  bench: gate / drone / gate+drone / behind the gate / abeam / seabed / clutter")
                continue
            sc = SCENARIOS[k](DEFAULT)
            print(f"  {k:24s} {sc.duration_s:4.0f} s  {sc.title}")
        return 0
    if not a.save and (a.overwrite or a.media or a.out):
        ap.error("--overwrite, --media and --out apply only with --save (a plain demo keeps nothing)")
    dest = Path(a.out or OUT) / a.scenario
    if a.save and dest.exists() and not a.overwrite:
        print(f"{dest} already exists and was not touched; add --overwrite to replace it.", file=sys.stderr)
        return 2
    _clean_orphan_stages()
    stage = Path(tempfile.mkdtemp(prefix=STAGE_PREFIX))
    try:
        (stage / "owner.pid").write_text(str(os.getpid()), encoding="utf-8")
        status = _run_scenario(a, stage)
        if not a.save:
            print("live demo: nothing kept (--save keeps the run in results/v2/demos/<scenario>/)")
        elif status != "COMPLETE":
            print(f"run {status}: not saved, {dest} unchanged")
        else:
            if not a.media:
                _strip_media(stage / a.scenario)
            _promote(stage / a.scenario, dest, a.overwrite)
            print(f"saved: {dest}")
        return 0 if status == "COMPLETE" else 1
    finally:
        shutil.rmtree(stage, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
