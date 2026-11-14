"""Simulator side of the variant-B2 one-way acoustic beacon: who hears whose ping, when, and what the receiver measures.

Drone k pings on a fixed TDMA schedule (slot k of every period, slots aligned to simulator ticks so that the
scheduled emission instant is exact), and every reception becomes a range-bearing measurement through the channel
model of sim/acoustic_channel.py (noise, biases, NLOS reflections, dropouts, outages, latency).  A ping carries the
sender's static identity and, with ``payload`` (H6, perception/payload.py), a 64-bit packet with the sender's
ONBOARD position estimate, error bound and gate phase, taken from its own perception through ``payload_source`` (never
from the simulator state).  The packet is lost on its own (``payload_loss``, a separate seeded stream) even when the
ping is detected.  No mode, queue or control data is sent, and nobody replies.

Every drone carries a HoloOcean ``AcousticBeaconSensor`` (id = drone index, engine range limit ``MaxDistance``
and line-of-sight check ``CheckVisible``).

HoloOcean 2.3 facts this module is built on (calibration/probe_beacon_api.py, results/calibration/):
* the engine computes, per (sender, receiver) command, [azimuth, elevation, distance, depth] into the receiver's
  buffer, or -1 when the receiver is out of range or the line of sight is blocked (a hull or a prop in between);
  the result appears in the tick of the transmission or the next one (no further propagation model);
* the client's own message bookkeeping (``send_message`` / ``sensor_data``) is not usable for a broadcast: one
  failed reception cancels the transmission for the receivers read after it, and two concurrent senders void every
  reception.  This module therefore enqueues the engine command itself and reads the raw buffer.
* HoloOcean has no acoustic interference model.  Here two pings emitted within the same tick (33 ms) or whose
  results could arrive in the same tick at a receiver are ALL lost at that receiver (a crude, conservative
  collision model; real receivers may separate distinct pinger codes).

On top of the engine: the propagation delay distance / sound_speed (the arrival time stamped on the reception) and
the receiver-side measurement model of sim/acoustic_channel.py; the engine's exact geometry never reaches a
controller.
"""

from __future__ import annotations

import math
from collections import defaultdict
from typing import Dict, List, Sequence, Tuple

import numpy as np

SENSOR_NAME = "Beacon"
PING_S = 0.04


class BeaconTransport:
    def __init__(self, names: Sequence[str], cfg, seed: int, ticks_per_sec: int = 30):
        from holo_fleet.sim.acoustic_channel import ChannelModel, channel

        self.names = list(names)
        self.cfg = cfg
        self.tps = ticks_per_sec
        self.period_ticks = max(1, int(round(cfg.period_s * ticks_per_sec)))
        self.slot_ticks = max(1, self.period_ticks // max(1, len(self.names)))
        self.channel = ChannelModel(channel(cfg.channel), len(self.names), seed, cfg.max_range_m)
        self.last_clear: Dict[Tuple[str, int], Tuple[float, Tuple[float, float, float]]] = {}
        self.rb_log: List[Tuple] = []
        self.scene = None
        self.pending: Dict[str, List[Tuple[str, float]]] = defaultdict(list)
        self.due: List[Tuple[float, str, int, float]] = []
        self.tx_log: List[Tuple[float, str]] = []
        self.stats = {"transmissions": 0, "receptions": 0, "lost_collision": 0, "lost_channel": 0,
                      "lost_dropout": 0, "latency_s": []}
        self.last_rx: Dict[Tuple[str, int], float] = {}
        self.longest_gap = 0.0
        self.t_now = 0.0
        self.payload_source = None
        self.payload_rng = np.random.default_rng([int(seed), 5102])
        self.payload_stats = {"sent": 0, "lost": 0, "delivered": 0}
        self.is_dead = None
        if cfg.payload:
            from holo_fleet.perception.payload import air_time_s

            slot_s = self.slot_ticks / ticks_per_sec
            need = air_time_s(cfg.payload_bitrate_bps) + PING_S + cfg.max_range_m / cfg.sound_speed
            if need > slot_s + 1e-9:
                raise ValueError(f"B2 payload does not fit the TDMA slot: {need:.3f} s > {slot_s:.3f} s "
                                 f"({len(self.names)} drones, period {cfg.period_s} s)")

    def sensor_config(self, k: int) -> Dict:
        return {"sensor_type": "AcousticBeaconSensor", "sensor_name": SENSOR_NAME, "socket": "",
                "location": [0.0, 0.0, 0.0],
                "configuration": {"id": int(k), "MaxDistance": float(self.cfg.max_range_m),
                                  "CheckVisible": bool(self.cfg.check_visible)}}

    def before_tick(self, env, t: float) -> None:
        """Emit the pings due at onboard time t (one engine command per receiver)."""
        from holoocean.command import SendAcousticMessageCommand

        tick = int(round(t * self.tps))
        for k, n in enumerate(self.names):
            if (tick - k * self.slot_ticks) % self.period_ticks != 0:
                continue
            if self.is_dead is not None and self.is_dead(n, t):
                continue
            self.stats["transmissions"] += 1
            self.tx_log.append((round(t, 4), n))
            packet = self._packet(k, t, tick)
            for r in self.names:
                if r == n or (self.is_dead is not None and self.is_dead(r, t)):
                    continue
                env._client.command_center.enqueue_command(SendAcousticMessageCommand(n, SENSOR_NAME, r, SENSOR_NAME))
                self.pending[r].append((n, t, packet))

    def _packet(self, k: int, t: float, tick: int):
        """The payload of drone k's ping at t, from its onboard estimate (None without payload)."""
        if not self.cfg.payload or self.payload_source is None:
            return None
        from holo_fleet.perception.payload import encode

        src = self.payload_source(k, t)
        if src is None:
            return None
        p_est, bound, gate_idx, phase = src
        self.payload_stats["sent"] += 1
        return encode(tick // self.period_ticks, p_est, bound, gate_idx, phase)

    def after_tick(self, env, t: float) -> None:
        """Resolve the engine results of this tick into receptions (or losses)."""
        self.t_now = t
        win = 2.0 / self.tps + 1e-6
        for r in self.names:
            pend = self.pending.get(r)
            if not pend:
                continue
            buf = np.asarray(env.agents[r].sensors[SENSOR_NAME]._sensor_data_buffer, dtype=float).ravel()
            event = not np.all(np.isnan(buf))
            expired = [p for p in pend if t - p[1] > win]
            for _p in expired:
                self.stats["lost_channel"] += 1
            pend = [p for p in pend if t - p[1] <= win]
            if event and pend:
                live = [p for p in pend if t - p[1] <= win]
                if len(live) > 1:
                    self.stats["lost_collision"] += len(live)
                    pend = []
                else:
                    sender, t_tx = live[0][0], live[0][1]
                    pend = []
                    self._measure(r, sender, t_tx, buf, live[0][2] if len(live[0]) > 2 else None)
            self.pending[r] = pend

    def _measure(self, r: str, sender: str, t_tx: float, buf: np.ndarray, packet=None) -> None:
        """Engine geometry of the direct path -> delivered measurement (identity, t_arrival, range, az, el)."""
        k_s, k_r = self.names.index(sender), self.names.index(r)
        ch = self.channel
        m = ch.measure(t_tx, k_s, k_r, buf)
        geom = None
        if m is None and np.all(buf == -1.0):
            geom, shadow = self._blocked_geometry(k_s, k_r)
            if geom is not None and shadow:
                m = ch.shadowed(t_tx, k_s, k_r, geom)
            elif geom is not None:
                m = ch.reflected(t_tx, k_s, k_r, geom)
        elif not np.all(buf == -1.0):
            geom = (float(buf[0]), float(buf[1]), float(buf[2]))
            self.last_clear[(r, k_s)] = (t_tx, geom)
        if m is None:
            if np.all(buf == -1.0):
                self.stats["lost_channel"] += 1
            else:
                self.stats["lost_dropout"] += 1
            return
        rng_m, az_m, el_m, nlos = m
        t_arr = t_tx + geom[2] / self.cfg.sound_speed
        if packet is not None and self.payload_rng.random() < min(1.0, ch.ch.payload_loss * (3.0 if nlos else 1.0)):
            packet = None
            self.payload_stats["lost"] += 1
        elif packet is not None:
            self.payload_stats["delivered"] += 1
        self.due.append((t_arr + ch.ch.latency_s, r, k_s, t_tx,
                         (round(t_arr, 4), float(rng_m), float(az_m), float(el_m), packet)))
        self.stats["receptions"] += 1
        self.stats["latency_s"].append(t_arr + ch.ch.latency_s - t_tx)
        self.rb_log.append((round(t_tx, 4), k_r, k_s, round(geom[2], 3), round(float(rng_m), 3),
                            round(math.degrees(az_m - geom[0]), 2), round(math.degrees(el_m - geom[1]), 2), nlos))

    def _blocked_geometry(self, k_s: int, k_r: int):
        """Direct-path geometry (receiver body frame, level) of a blocked ping from the simulator truth, and whether
        a drone hull or a gate bar lies on the ray (sim/acoustic_channel.py, H4)."""
        from holo_fleet.perception.sonar_geometry import HULL_HALF
        from holo_fleet.sim.acoustic_channel import segment_hits_box

        if self.scene is None:
            return None, False
        P, Y, bars = self.scene()
        a, b = P[k_s], P[k_r]
        d = a - b
        c, s = math.cos(Y[k_r]), math.sin(Y[k_r])
        body = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
        dist = float(np.linalg.norm(body))
        if dist < 1e-6:
            return None, False
        geom = (math.atan2(body[1], body[0]), math.asin(float(np.clip(body[2] / dist, -1.0, 1.0))), dist)
        half = np.asarray(HULL_HALF, float)
        for k in range(len(P)):
            if k in (k_s, k_r):
                continue
            ck, sk = math.cos(Y[k]), math.sin(Y[k])
            axes = np.array([[ck, -sk, 0.0], [sk, ck, 0.0], [0.0, 0.0, 1.0]])
            if segment_hits_box(a, b, P[k], axes, half):
                return geom, True
        for bar in bars or ():
            if segment_hits_box(a, b, bar.center, bar.axes, bar.half):
                return geom, True
        return geom, False

    def deliver(self, t: float) -> Dict[str, List[Tuple[int, float]]]:
        """Receptions whose time has come: receiver -> [(sender identity, t_arrival, range, azimuth, elevation,
        packet bytes or None)]."""
        out: Dict[str, List[Tuple[int, float]]] = defaultdict(list)
        keep = []
        for item in self.due:
            if item[0] <= t + 1e-9:
                out[item[1]].append((item[2],) + tuple(item[4]))
                key = (item[1], item[2])
                prev = self.last_rx.get(key, 0.0)
                if item[0] > prev:
                    self.longest_gap = max(self.longest_gap, item[0] - prev)
                    self.last_rx[key] = item[0]
            else:
                keep.append(item)
        self.due = keep
        return out

    def summary(self) -> Dict:
        lat = self.stats["latency_s"]
        tx, n = self.stats["transmissions"], len(self.names)
        expected = tx * max(0, n - 1)
        open_gaps = [self.t_now - self.last_rx.get((r, k), 0.0) for r in self.names for k in range(n)
                     if self.names[k] != r]
        return {"mode": self.cfg.mode, "period_s": self.cfg.period_s,
                "max_range_m": self.cfg.max_range_m, "check_visible": self.cfg.check_visible,
                "transmissions": tx, "expected_receptions": expected,
                "receptions": self.stats["receptions"],
                "reception_rate": round(self.stats["receptions"] / expected, 4) if expected else None,
                "lost_collision": self.stats["lost_collision"], "lost_channel": self.stats["lost_channel"],
                "lost_dropout": self.stats["lost_dropout"],
                "latency_mean_ms": round(1000.0 * float(np.mean(lat)), 2) if lat else None,
                "latency_max_ms": round(1000.0 * float(np.max(lat)), 2) if lat else None,
                "longest_absence_s": round(max([self.longest_gap] + open_gaps), 2),
                "channel": self.channel.ch.as_dict(), "channel_stats": dict(self.channel.stats),
                "sound_speed_scale": round(self.channel.scale, 5),
                "clock_offset_m": [round(float(x), 3) for x in self.channel.offset],
                "nlos_received": int(sum(1 for x in self.rb_log if x[7])), "payload": dict(self.payload_stats),
                "payload_bitrate_bps": self.cfg.payload_bitrate_bps if self.cfg.payload else None,
                **_error_summary(self.rb_log)}


def _error_summary(log) -> Dict:
    """Measurement errors of the delivered B2 receptions (direct path / reflected), from the simulator truth."""
    out = {}
    for tag, rows in (("los", [x for x in log if not x[7]]), ("nlos", [x for x in log if x[7]])):
        if not rows:
            continue
        dr = np.array([x[4] - x[3] for x in rows])
        da = np.abs(np.array([((x[5] + 180.0) % 360.0) - 180.0 for x in rows]))
        de = np.abs(np.array([x[6] for x in rows]))
        out[f"{tag}_range_err_p50_m"] = round(float(np.median(np.abs(dr))), 3)
        out[f"{tag}_range_err_p99_m"] = round(float(np.percentile(np.abs(dr), 99)), 3)
        out[f"{tag}_az_err_p99_deg"] = round(float(np.percentile(da, 99)), 2)
        out[f"{tag}_el_err_p99_deg"] = round(float(np.percentile(de, 99)), 2)
    return out
