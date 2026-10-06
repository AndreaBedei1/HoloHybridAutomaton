"""Run one HoloOcean experiment.

Examples::

    python scripts/run_experiment.py --scenario pair_crossing
    python scripts/run_experiment.py --scenario formation_current --current medium --n-drones 3
    python scripts/run_experiment.py --scenario gate_arena --n-drones 3
    python scripts/run_experiment.py --scenario stress
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from holo_fleet.runner import run  # noqa: E402
from holo_fleet.sim.scenarios import SCENARIOS  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", required=True, choices=sorted(SCENARIOS))
    ap.add_argument("--n-drones", type=int, default=3)
    ap.add_argument("--current", default=None, choices=["none", "low", "medium", "high"])
    ap.add_argument("--no-jet", action="store_true", help="formation_current: disable the localized jet")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--run-id", default=None)
    ap.add_argument("--out", default="results")
    ap.add_argument("--viewport", action="store_true", help="show the HoloOcean viewport")
    ap.add_argument("--no-early-stop", action="store_true")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    kw = {}
    if args.scenario != "pair_crossing":
        kw["n_drones"] = args.n_drones
    if args.current is not None:
        kw["current"] = args.current
    if args.scenario == "formation_current":
        kw["jet"] = not args.no_jet
    run_dir = run(args.scenario, out_root=args.out, seed=args.seed, headless=not args.viewport,
                  run_id=args.run_id, duration=args.duration, early_stop=not args.no_early_stop, **kw)
    print(f"RUN_DIR {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
