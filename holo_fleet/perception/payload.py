"""B2 beacon payload (hardware assumption H6): the bits a drone broadcasts with its TDMA identity ping.

Strictly one-way: the payload is the sender's ONBOARD navigation estimate in the shared mission frame and a
conservative bound of its error (perception/nav.py, NavBound).  No request, reply, acknowledgement, mode, queue
position or control decision is ever sent.  The packet is 80 bits (10 bytes):

  field     bits  encoding
  seq          8  TDMA frame counter mod 256 (the emission instant is the pre-assigned slot: no timestamp field)
  x, y     16+16  mission-frame position, signed, 0.01 m (+-327.67 m)
  depth       12  -z, unsigned, 0.01 m (0 - 40.95 m)
  bound        8  error-ball radius, 0.05 m steps, rounded UP after adding the quantisation error; 255 = invalid
  crc         16  CRC-16/CCITT-FALSE over the first 60 bits (8 bytes, 4 padding bits zero)

Air time at the assumed 1 kbit/s coherent short-packet rate: 80 ms; with a 20-40 ms identity chirp and the 33 ms
propagation spread of 50 m it fits the 167 ms slot of n = 6 drones at a 1 s period (sim/beacon.py checks it).
Rates of 1 kbit/s and above at tens of metres are within PSK modes of research modems (WHOI Micro-Modem PSK
burst rates up to about 5 kbit/s) and software-defined modems (Subnero M25M, peak 15 kbit/s); the CRC rejects
corrupted packets, and a decoded position inconsistent with the measured range and bearing is not used
(perception/acoustic.py).  The position uncertainty, clock error (synchronised clocks, H2) and packet loss are
modelled; the identity of the sender is the TDMA slot and the ping code.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

PAYLOAD_BITS = 80
POS_Q = 0.01                  # [m] position quantum
BOUND_Q = 0.05                # [m] bound quantum
BOUND_INVALID = 255
QUANT_ERR = math.sqrt(3.0) * POS_Q / 2.0   # largest rounding error of a position (three axes)


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Payload:
    seq: int
    p: np.ndarray                 # decoded mission-frame position (quantised)
    bound: float                  # error-ball radius at the emission instant, quantisation included


def encode(seq: int, p, bound: float) -> bytes:
    """Sender side.  ``bound`` already covers the age of the estimate at the emission instant."""
    x = int(round(float(p[0]) / POS_Q))
    y = int(round(float(p[1]) / POS_Q))
    depth = int(round(-float(p[2]) / POS_Q))
    b = math.ceil((float(bound) + QUANT_ERR) / BOUND_Q - 1e-9) if math.isfinite(bound) else BOUND_INVALID
    if not (-32768 <= x <= 32767 and -32768 <= y <= 32767 and 0 <= depth <= 4095) or not math.isfinite(bound):
        b = BOUND_INVALID                 # out of the representable frame: sent, but marked unusable
        x, y, depth = max(-32768, min(32767, x)), max(-32768, min(32767, y)), max(0, min(4095, depth))
    b = min(b, BOUND_INVALID)
    bits = ((seq & 0xFF) << 52) | ((x & 0xFFFF) << 36) | ((y & 0xFFFF) << 20) | ((depth & 0xFFF) << 8) | (b & 0xFF)
    head = (bits << 4).to_bytes(8, "big")
    return head + crc16(head).to_bytes(2, "big")


def decode(packet: Optional[bytes]) -> Optional[Payload]:
    """Receiver side: None for a missing, corrupted or invalid packet."""
    if packet is None or len(packet) != PAYLOAD_BITS // 8:
        return None
    head, crc = packet[:8], int.from_bytes(packet[8:], "big")
    if crc16(head) != crc:
        return None
    bits = int.from_bytes(head, "big") >> 4
    b = bits & 0xFF
    if b == BOUND_INVALID:
        return None
    def s16(v):
        return v - 0x10000 if v & 0x8000 else v
    x, y = s16((bits >> 36) & 0xFFFF), s16((bits >> 20) & 0xFFFF)
    depth = (bits >> 8) & 0xFFF
    return Payload(seq=(bits >> 52) & 0xFF, p=np.array([x * POS_Q, y * POS_Q, -depth * POS_Q]), bound=b * BOUND_Q)


def air_time_s(rate_bps: float) -> float:
    return PAYLOAD_BITS / float(rate_bps)
