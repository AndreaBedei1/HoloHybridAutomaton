"""Residual risk of the B2 acoustic exit evidence under the channel model (offline Monte Carlo, no simulator).

    python calibration/analyze_b2_channel.py [--trials 20000] [--workers 20]

The formal checks (formal/check_acoustic.py) prove the decision sound for measurements INSIDE the declared envelope.
Two things fall outside it: Gaussian tails beyond 3 sigma, and reflected (NLOS) arrivals that pass the depth and
kinematic gates.  This script measures, with the channel model of sim/acoustic_channel.py at each level, how often
a hull that is still in the CR produces an acoustic "outside the CR" or "beyond the exit face" decision
(perception/acoustic_gate.AcousticExit.outside_now with exit_confirm consecutive accepted fixes):

* an occupant random in the CR (uniform), moving at most v_bound, observed from a random queue point of the abreast
  queue of n = 2..6 for 6 s at the TDMA rate;
* the direct path is clear (LOS errors only) or, with probability 0.5, shadowed or blocked so that reflections occur
  (worst case: every blocked ping that is received is a reflection);
* the decision is evaluated after every reception; any positive decision while the hull is in the CR is a FALSE
  EXIT (it would be a false release if every other drone were also accounted for).

Writes results/calibration/b2_channel/channel_risk.json.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

OUT = ROOT / "results" / "calibration" / "b2_channel" / "channel_risk.json"


def _trial_block(args):
    level, seed, trials, confirm = args
    from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates
    from holo_fleet.config import DEFAULT, BeaconConfig
    from holo_fleet.mission import abreast_slots, queue_line
    from holo_fleet.perception.acoustic import AcousticTracker
    from holo_fleet.perception.acoustic_gate import AcousticExit
    from holo_fleet.sim import acoustic_channel as ac

    cfg = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_RB", channel=level))
    if confirm:
        cfg = dataclasses.replace(cfg, acoustic=dataclasses.replace(cfg.acoustic, exit_confirm=confirm))
    G = cfg.gate
    g = load_arena_gates(HORSESHOE_TRACK, ["G06"])[0]
    yaw = math.atan2(g.axis[1], g.axis[0])
    half = np.array([G.cr_half_len, G.cr_half_width, G.cr_half_height])
    rng = np.random.default_rng([seed, 99])
    ch = ac.channel(level)
    out = {"trials": 0, "receptions": 0, "nlos": 0, "accepted_nlos": 0, "false_outside": 0, "false_beyond": 0,
           "outside_tail": 0}
    qline = {n: queue_line(abreast_slots(n), G) for n in range(2, 7)}
    for _ in range(trials):
        n = int(rng.integers(2, 7))
        q_s = qline[n]
        lat = float(rng.choice(G.queue_laterals(n)))
        p = g.from_gate_frame([q_s, lat, 0.0])
        model = ac.ChannelModel(ch, 2, int(rng.integers(1 << 30)), cfg.beacon.max_range_m)
        model.outages = []
        tr = AcousticTracker(cfg)
        tr.set_identities({1: 1})
        ae = AcousticExit(cfg, tr, [1])
        c = rng.uniform(-1, 1, 3) * half
        vel = rng.normal(0, 1, 3)
        vel *= rng.uniform(0, 0.69) / max(np.linalg.norm(vel), 1e-9)
        blocked = rng.random() < 0.5
        t = 0.0
        tr.update(t, (), p, yaw)
        out["trials"] += 1
        for k in range(6):
            t_tx = float(k)
            c = np.clip(c + vel * 1.0, -half, half)
            w = g.from_gate_frame(c)
            d = w - p
            cy, sy = math.cos(yaw), math.sin(yaw)
            body = np.array([cy * d[0] + sy * d[1], -sy * d[0] + cy * d[1], d[2]])
            dist = float(np.linalg.norm(body))
            geom = (math.atan2(body[1], body[0]), math.asin(body[2] / dist), dist)
            if blocked:
                m = model.reflected(t_tx, 0, 1, geom)
            else:
                m = model.measure(t_tx, 0, 1, np.array([geom[0], geom[1], geom[2], -4.3]))
            for tt in np.arange(t_tx + 0.1, t_tx + 1.0, 0.1):
                tr.update(round(tt, 3), (), p, yaw)
            if m is None:
                continue
            out["receptions"] += 1
            out["nlos"] += int(m[3])
            t_arr = t_tx + dist / 1500.0
            t_now = round(t_arr + ch.latency_s + 0.05, 3)
            ae.note_corridor(-0.5, g, p, yaw, [dist - 0.4])
            fixes = tr.update(t_now, [(1, t_arr, m[0], m[1], m[2])], p, yaw)
            out["accepted_nlos"] += int(m[3] and bool(fixes) and fixes[0].accepted)
            if ae.outside_now(g, 1, t_now, -1.0) is not None:
                out["false_outside"] += 1
                out["outside_tail"] += int(not m[3])
            if ae.outside_now(g, 1, t_now, -1.0, beyond=True) is not None:
                out["false_beyond"] += 1
    return level, out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=20000)
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--confirm", type=int, default=0, help="override exit_confirm (sensitivity only)")
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args()
    try:
        import psutil

        psutil.Process().nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    except Exception:  # noqa: BLE001
        pass
    levels = ("CLEAN", "LOW", "MEDIUM")
    per = max(1, args.trials // args.workers)
    jobs = [(lv, s, per, args.confirm) for lv in levels for s in range(args.workers)]
    res = {lv: {} for lv in levels}
    with ProcessPoolExecutor(args.workers) as ex:
        for lv, o in ex.map(_trial_block, jobs):
            for k, v in o.items():
                res[lv][k] = res[lv].get(k, 0) + v
    for lv in levels:
        r = res[lv]
        r["false_outside_per_reception"] = r["false_outside"] / max(r["receptions"], 1)
        r["false_beyond_per_reception"] = r["false_beyond"] / max(r["receptions"], 1)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(res, indent=1), encoding="utf-8")
    print(json.dumps(res, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
