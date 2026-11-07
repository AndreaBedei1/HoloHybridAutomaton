"""B2 beacon payload (hardware assumption H6): the bits a drone sends with its TDMA identity ping.

Strictly one-way: the payload is the sender's ONBOARD navigation estimate in the shared mission frame and a
conservative bound of its error (perception/nav.py, NavBound).  No request, reply, acknowledgement, mode, queue
position or control decision is ever sent.  The packet is 56 bits (7 bytes):

  field     bits  encoding
  frame        2  TDMA frame counter mod 4: the receiver checks it against its own schedule (a packet of another
                  frame is not used); the emission instant is the pre-assigned slot, so there is no timestamp
  x, y     12+12  mission-frame position, signed, 0.05 m (+-102.35 m; the mission area spans +-45 m)
  depth        8  -z, unsigned, 0.05 m (0 - 12.75 m; the operating band is 1.5 - 9 m)
  bound        6  error-ball radius, 0.10 m steps, rounded UP after adding the quantisation error; 63 = invalid
                  (the sender's integrity monitor flags its own bound, NavBound.invalid)
  crc         16  CRC-16/CCITT-FALSE over the first 40 bits (the field is a mutual-exclusion premise: a CRC-8
                  would pass about 1/256 of multi-bit corrupted packets)

Air time (H6, an assumption to be measured on the bench before any field use): at 1 kbit/s net, 56 ms; the
identity chirp doubles as the packet preamble (40 ms), and with the 33 ms propagation spread of 50 m a slot needs
129 ms of the 167 ms of n = 6 drones at the 1 s period (38 ms left for reverberation tails and scheduling jitter;
sim/beacon.py refuses a configuration that does not fit).  Candidate hardware with short-range rates far above
1 kbit/s exists (EvoLogics S2CM-HS, up to 62.5 kbit/s at 120-200 kHz; Subnero M25M, 15 kbit/s peak at short range;
Teledyne Benthos DAT, 140-15 360 bit/s), but a 7-byte frame time on a synchronised TDMA schedule is not documented
by any of them: it is the stated hardware assumption.  A BlueROV2-class 64 bit/s modem (Water Linked M64) cannot
carry it at this period.  The CRC rejects corrupted packets, and a decoded position inconsistent with the measured
range and bearing is not used (perception/acoustic.py).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

PAYLOAD_BITS = 56
POS_Q = 0.05                  # [m] position quantum
BOUND_Q = 0.10                # [m] bound quantum
BOUND_INVALID = 63
FRAMES = 4                    # frame counter modulus
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
    frame: int
    p: np.ndarray                 # decoded mission-frame position (quantised)
    bound: float                  # error-ball radius at the emission instant, quantisation included


def encode(frame: int, p, bound: float) -> bytes:
    """Sender side.  ``bound`` already covers the age of the estimate at the emission instant."""
    x = int(round(float(p[0]) / POS_Q))
    y = int(round(float(p[1]) / POS_Q))
    depth = int(round(-float(p[2]) / POS_Q))
    b = math.ceil((float(bound) + QUANT_ERR) / BOUND_Q - 1e-9) if math.isfinite(bound) else BOUND_INVALID
    if not (-2048 <= x <= 2047 and -2048 <= y <= 2047 and 0 <= depth <= 255):
        b = BOUND_INVALID                 # outside the representable frame: sent, but marked unusable
        x, y, depth = max(-2048, min(2047, x)), max(-2048, min(2047, y)), max(0, min(255, depth))
    b = min(b, BOUND_INVALID)
    bits = ((frame % FRAMES) << 38) | ((x & 0xFFF) << 26) | ((y & 0xFFF) << 14) | ((depth & 0xFF) << 6) | (b & 0x3F)
    head = bits.to_bytes(5, "big")
    return head + crc16(head).to_bytes(2, "big")


def decode(packet: Optional[bytes]) -> Optional[Payload]:
    """Receiver side: None for a missing, corrupted or invalid packet."""
    if packet is None or len(packet) != PAYLOAD_BITS // 8:
        return None
    head, crc = packet[:5], int.from_bytes(packet[5:], "big")
    if crc16(head) != crc:
        return None
    bits = int.from_bytes(head, "big")
    b = bits & 0x3F
    if b == BOUND_INVALID:
        return None

    def s12(v):
        return v - 0x1000 if v & 0x800 else v
    x, y = s12((bits >> 26) & 0xFFF), s12((bits >> 14) & 0xFFF)
    depth = (bits >> 6) & 0xFF
    return Payload(frame=(bits >> 38) & 0x3, p=np.array([x * POS_Q, y * POS_Q, -depth * POS_Q]), bound=b * BOUND_Q)


def air_time_s(rate_bps: float) -> float:
    return PAYLOAD_BITS / float(rate_bps)


FRAME_GUARD_S = 0.02          # >> the error of an emission instant estimated as t_arrival - r / c (a few ms, even with a
                              # 3 m multipath excess), << the 129 ms slot need: slot 0 of a frame is not taken for the last
                              # slot of the previous one


def frame_index(t_tx: float, period_s: float) -> int:
    """TDMA frame of an emission instant (slot k of frame m at m P + k P / n)."""
    return int(math.floor((t_tx + FRAME_GUARD_S) / period_s)) % FRAMES
