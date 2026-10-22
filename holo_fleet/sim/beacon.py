"""Simulator side of the optional one-way presence beacon (variant B1): who hears whose ping, and when.

Every drone carries a HoloOcean ``AcousticBeaconSensor`` (id = drone index, engine range limit ``MaxDistance``
and line-of-sight check ``CheckVisible``).  A drone pings every ``period_s`` +- ``jitter_s`` (uniform, seeded).
A ping carries only the sender's static identity: no position, velocity, heading, mode, queue or sensor data,
and nobody replies.

HoloOcean 2.3 facts this module is built on (probe/probe_beacon_api.py, results/robustness/probe/):
* the engine computes, per (sender, receiver) command, [azimuth, elevation, distance, depth] into the receiver's
  buffer, or -1 when the receiver is out of range or the line of sight is blocked (a hull or a prop in between);
  the result appears in the tick of the transmission or the next one (no further propagation model);
* the client's own message bookkeeping (``send_message`` / ``sensor_data``) is not usable for a broadcast: one
  failed reception cancels the transmission for the receivers read after it, and two concurrent senders void every
  reception.  This module therefore enqueues the engine command itself and reads the raw buffer.
* HoloOcean has no acoustic interference model.  Here two pings emitted within the same tick (33 ms) or whose
  results could arrive in the same tick at a receiver are ALL lost at that receiver (a crude, conservative
  collision model; real receivers may separate distinct pinger codes).

On top of the engine: propagation delay distance / sound_speed (stamped on the reception), and a reception
dropout (noise profile ``beacon_dropout``, seeded).  The azimuth/elevation the engine computes are NOT passed on
(presence only).
"""

from __future__ import annotations

import math
from collections import defaultdict
from typing import Dict, List, Sequence, Tuple

import numpy as np

SENSOR_NAME = "Beacon"


class BeaconTransport:
    def __init__(self, names: Sequence[str], cfg, seed: int, dropout: float = 0.0, ticks_per_sec: int = 30):
        self.names = list(names)
        self.cfg = cfg
        self.dropout = float(dropout)
        self.tps = ticks_per_sec
        self.rng = np.random.default_rng([int(seed), 4243])
        self.next_ping = {n: float(self.rng.uniform(0.0, cfg.period_s)) for n in self.names}
        self.pending: Dict[str, List[Tuple[str, float]]] = defaultdict(list)   # receiver -> [(sender, t_tx)]
        self.due: List[Tuple[float, str, int, float]] = []                     # (t_rx, receiver, sender id, t_tx)
        self.tx_log: List[Tuple[float, str]] = []
        self.stats = {"transmissions": 0, "receptions": 0, "lost_collision": 0, "lost_channel": 0,
                      "lost_dropout": 0, "latency_s": []}

    def sensor_config(self, k: int) -> Dict:
        return {"sensor_type": "AcousticBeaconSensor", "sensor_name": SENSOR_NAME, "socket": "",
                "location": [0.0, 0.0, 0.0],
                "configuration": {"id": int(k), "MaxDistance": float(self.cfg.max_range_m),
                                  "CheckVisible": bool(self.cfg.check_visible)}}

    def before_tick(self, env, t: float) -> None:
        """Emit the pings due at onboard time t (one engine command per receiver)."""
        from holoocean.command import SendAcousticMessageCommand

        for n in self.names:
            if t + 1e-9 < self.next_ping[n]:
                continue
            self.next_ping[n] += self.cfg.period_s + float(self.rng.uniform(-self.cfg.jitter_s, self.cfg.jitter_s))
            self.stats["transmissions"] += 1
            self.tx_log.append((round(t, 4), n))
            for r in self.names:
                if r == n:
                    continue
                env._client.command_center.enqueue_command(SendAcousticMessageCommand(n, SENSOR_NAME, r, SENSOR_NAME))
                self.pending[r].append((n, t))

    def after_tick(self, env, t: float) -> None:
        """Resolve the engine results of this tick into receptions (or losses)."""
        win = 2.0 / self.tps + 1e-6                       # a result appears in the tick of the ping or the next one
        for r in self.names:
            pend = self.pending.get(r)
            if not pend:
                continue
            buf = np.asarray(env.agents[r].sensors[SENSOR_NAME]._sensor_data_buffer, dtype=float).ravel()
            event = not np.all(np.isnan(buf))
            expired = [p for p in pend if t - p[1] > win]
            for _p in expired:                             # no result at all (counted as a channel loss)
                self.stats["lost_channel"] += 1
            pend = [p for p in pend if t - p[1] <= win]
            if event and pend:
                live = [p for p in pend if t - p[1] <= win]
                if len(live) > 1:                          # results of two pings can share this buffer: all lost
                    self.stats["lost_collision"] += len(live)
                    pend = []
                else:
                    sender, t_tx = live[0]
                    pend = []
                    if np.all(buf == -1.0):
                        self.stats["lost_channel"] += 1    # out of range or line of sight blocked (engine)
                    elif self.dropout > 0.0 and self.rng.random() < self.dropout:
                        self.stats["lost_dropout"] += 1
                    else:
                        dist = float(buf[2])
                        t_rx = max(t, t_tx + dist / self.cfg.sound_speed)
                        self.due.append((t_rx, r, self.names.index(sender), t_tx))
                        self.stats["receptions"] += 1
                        self.stats["latency_s"].append(t_rx - t_tx)
            self.pending[r] = pend

    def deliver(self, t: float) -> Dict[str, List[Tuple[int, float]]]:
        """Receptions whose time has come: receiver -> [(sender identity, reception time)]."""
        out: Dict[str, List[Tuple[int, float]]] = defaultdict(list)
        keep = []
        for item in self.due:
            if item[0] <= t + 1e-9:
                out[item[1]].append((item[2], round(item[0], 4)))
            else:
                keep.append(item)
        self.due = keep
        return out

    def summary(self) -> Dict:
        lat = self.stats["latency_s"]
        tx, n = self.stats["transmissions"], len(self.names)
        expected = tx * max(0, n - 1)
        return {"mode": self.cfg.mode, "period_s": self.cfg.period_s, "jitter_s": self.cfg.jitter_s,
                "max_range_m": self.cfg.max_range_m, "check_visible": self.cfg.check_visible,
                "dropout": self.dropout, "transmissions": tx, "expected_receptions": expected,
                "receptions": self.stats["receptions"],
                "reception_rate": round(self.stats["receptions"] / expected, 4) if expected else None,
                "lost_collision": self.stats["lost_collision"], "lost_channel": self.stats["lost_channel"],
                "lost_dropout": self.stats["lost_dropout"],
                "latency_mean_ms": round(1000.0 * float(np.mean(lat)), 2) if lat else None,
                "latency_max_ms": round(1000.0 * float(np.max(lat)), 2) if lat else None}
