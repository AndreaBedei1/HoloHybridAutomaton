"""Watch one short demonstration scenario (one seed) with the live dashboard.

    python scripts/run_demo.py --list
    python scripts/run_demo.py --scenario p1_vertical_escape
    python scripts/run_demo.py --scenario gate_single --headless          # no HoloOcean window
    python scripts/run_demo.py --scenario sonar_classification            # echo classifier bench, A-F

Two windows: the HoloOcean viewport (the onboard belief is drawn on it: nearest echo per sonar
sector coloured by class - grey structure, brown seabed, red possible vehicle, amber unknown - and
the escape direction in magenta) and the dashboard (ONBOARD vs REFEREE / GROUND TRUTH).  Results
go to results/v2/demos/<scenario>/ (logs, metrics, dashboard frames, <scenario>.gif); the
post-run summary is printed at the end.  Ctrl+C stops the run and marks it INCOMPLETE.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.config import DEFAULT  # noqa: E402
from holo_fleet.runner import print_summary, run  # noqa: E402
from holo_fleet.sim.scenarios import SCENARIOS  # noqa: E402
from holo_fleet.ui.gif import make_gif  # noqa: E402
from holo_fleet.ui.live import DemoUI  # noqa: E402

OUT = ROOT / "results" / "v2" / "demos"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", choices=sorted(SCENARIOS) + ["sonar_classification"])
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--headless", action="store_true", help="no HoloOcean viewport")
    ap.add_argument("--no-window", action="store_true", help="no dashboard window (frames are still saved)")
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--duration", type=float, default=None)
    a = ap.parse_args()
    if a.list or not a.scenario:
        print("scenarios (one seed each):")
        for k in ["sonar_classification"] + list(SCENARIOS):
            if k == "sonar_classification":
                print(f"  {k:24s} ~28 s  bench: gate / drone / gate+drone / behind the gate / abeam / seabed / clutter")
                continue
            sc = SCENARIOS[k](DEFAULT)
            print(f"  {k:24s} {sc.duration_s:4.0f} s  {sc.title}")
        return 0
    if a.scenario == "sonar_classification":
        sys.path.insert(0, str(ROOT / "scripts"))
        from demo_classification import run_classification_demo  # noqa: E402

        return run_classification_demo(Path(a.out), headless=a.headless, show=not a.no_window)
    sc = SCENARIOS[a.scenario](DEFAULT)
    ui = DemoUI(sc, DEFAULT, sc.sim.names, show=not a.no_window, draw_viewport=not a.headless)
    m = run(a.scenario, Path(a.out), headless=a.headless, run_id=a.scenario, duration=a.duration, ui=ui)
    print_summary(a.scenario, m)
    gif = make_gif((Path(a.out) / a.scenario / "dashboard").glob("dash_*.jpg"), Path(a.out) / a.scenario / f"{a.scenario}.gif")
    if gif:
        print(f"dashboard GIF: {gif}")
    return 0 if m["run"]["status"] == "COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
