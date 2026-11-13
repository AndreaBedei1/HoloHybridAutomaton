"""The final campaign of criteria v8 (docs/ACCEPTANCE_CRITERIA.md): the matrix, fixed before the first run.

    python scripts/final_campaign.py <list file>     # one "protocol profile scenario seed" per line

Each line is run by scripts/final_campaign_lane.sh (one HoloOcean instance per lane, below-normal priority) through
scripts/run_sensor_robustness_campaign.py --phase targeted.  Seeds 5000-5404 are used by nothing before the freeze.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import run_sensor_robustness_campaign as c  # noqa: E402

from holo_fleet.sim.scenarios import STRESS_BATCH  # noqa: E402

GATE = c.B2_GATE_SCENARIOS
PROFILES = (("B2_CLEAN", "CLEAN"), ("B2_LOW", "LOW-ALL"), ("B2_MEDIUM", "MEDIUM-ALL"))
ABLATION_SCENARIOS = ("gate_single", "integrated_short", "mutex_deadlock_resolution", "line_parallel_mutex", "lost_drone_mutex")


def matrix() -> list:
    runs = []
    # A: the standard set, three profiles (gate scenarios x 6 seeds, the others x 3)
    for proto, prof in PROFILES:
        for sc in c.ALL_SCENARIOS:
            runs += [(proto, prof, sc, s) for s in (range(5000, 5006) if sc in GATE else range(5000, 5003))]
    # B: the lost-drone family under MEDIUM, 60 runs
    runs += [("B2_MEDIUM", "MEDIUM-ALL", "lost_drone_mutex", s) for s in range(5100, 5140)]
    runs += [("B2_MEDIUM", "MEDIUM-ALL", f"lost_drone_mutex_{v}", s) for v in ("rise_low", "rise_high") for s in range(5100, 5110)]
    # C: the stress batch under MEDIUM, five seeds
    runs += [("B2_MEDIUM", "MEDIUM-ALL", sc, s) for sc in STRESS_BATCH for s in range(5200, 5205)]
    # D: beacon ablation (B0: no beacon; B2NP: the beacon without the payload), MEDIUM sensors and channel
    runs += [(proto, "MEDIUM-ALL", sc, s) for proto in ("B0", "B2NP_MEDIUM") for sc in ABLATION_SCENARIOS for s in range(5300, 5303)]
    # E: sensitivity, beyond the validation profiles: compass bias 3 and 4 deg, the long-tail channel
    runs += [("B2_MEDIUM", f"{prof}-ALL", sc, s) for prof in ("MEDIUM_HB3", "MEDIUM_HB4") for sc in ("gate_single", "lost_drone_mutex")
             for s in range(5400, 5405)]
    runs += [("B2_MEDIUM_HT", "MEDIUM-ALL", sc, s) for sc in ("gate_single", "lost_drone_mutex") for s in range(5400, 5405)]
    assert len(runs) == len(set(runs))
    return runs


if __name__ == "__main__":
    runs = matrix()
    Path(sys.argv[1]).write_text("".join(f"{p} {pr} {sc} {s}\n" for p, pr, sc, s in runs), encoding="utf-8")
    print(len(runs))
