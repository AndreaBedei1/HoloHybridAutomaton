"""B2 beacon payload (hardware assumption H6): the bits a drone sends with its TDMA identity ping.

Strictly one-way: the payload is the sender's ONBOARD navigation estimate in the shared mission frame, a conservative
bound of its error (perception/nav.py, NavBound) and, since v11, its own gate phase.  No request, reply, acknowledgement,
queue position, mode or control command is ever sent: the gate phase is a statement of fact about the sender's own
state at the gate (never a request, an order or a grant of precedence), used by the receivers only to exclude the sender
as a possible occupant of a critical region, or as positive evidence of occupancy (perception/acoustic_gate.py).  The
packet is 64 bits (8 bytes):

  field      bits  encoding
  frame         2  TDMA frame counter mod 4: the receiver checks it against its own schedule (a packet of another
                   frame is not used); the emission instant is the pre-assigned slot, so there is no timestamp
  x, y      12+12  mission-frame position, signed, 0.05 m (+-102.35 m; the mission area spans +-45 m)
  depth         8  -z, unsigned, 0.05 m (0 - 12.75 m; the operating band is 1.5 - 9 m)
  bound         6  error-ball radius, rounded UP after adding the quantisation error: 0 - 3.9 m in 0.10 m steps, then
                   0.50 m steps up to 15.4 m; 63 = invalid (the integrity monitor flags its own bound, NavBound.invalid,
                   or a bound beyond the representable range: v10 saturated at 6.2 m and lost the payload there)
  gate index    2  index (mod 4) of the gate the phase refers to
  gate phase    2  GP_NONE not engaged with that gate / GP_COMMITTED committed, held before the CR, not admitted /
                   GP_ADMITTED admitted, possibly inside the CR / GP_PASSED passage verified onboard (perception/passage.py)
  reserved      4  zero
  crc          16  CRC-16/CCITT-FALSE over the first 48 bits (the fields are mutual-exclusion premises: a CRC-8 would
                   pass about 1/256 of multi-bit corrupted packets)

Air time (H6, an assumption to be measured on the bench before any field use): at 1 kbit/s net, 64 ms; the identity
chirp doubles as the packet preamble (40 ms), and with the 33 ms propagation spread of 50 m a slot needs 137 ms of the
167 ms of n = 6 drones at the 1 s period (30 ms left for reverberation tails and scheduling jitter; sim/beacon.py
refuses a configuration that does not fit).  Candidate hardware with short-range rates far above 1 kbit/s exists
(EvoLogics S2CM-HS, Subnero M25M, Teledyne Benthos DAT), but a 8-byte frame time on a synchronised TDMA schedule is not
documented by any of them: it is the stated hardware assumption.  A BlueROV2-class 64 bit/s modem (Water Linked M64)
cannot carry it at this period.  The CRC rejects corrupted packets, and a decoded position inconsistent with the
measured range and bearing is not used (perception/acoustic.py).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

PAYLOAD_BITS = 64
POS_Q = 0.05
BOUND_FINE_Q = 0.10
BOUND_FINE_MAX = 3.9
BOUND_COARSE_Q = 0.50
BOUND_INVALID = 63
BOUND_MAX = BOUND_FINE_MAX + (BOUND_INVALID - 1 - 39) * BOUND_COARSE_Q
FRAMES = 4
QUANT_ERR = math.sqrt(3.0) * POS_Q / 2.0

GP_NONE, GP_COMMITTED, GP_ADMITTED, GP_PASSED = 0, 1, 2, 3


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_bound(bound: float) -> int:
    """6-bit code of an error radius [m], rounded up so that the decoded value is never smaller; 63 = invalid."""
    if not math.isfinite(bound) or bound > BOUND_MAX:
        return BOUND_INVALID
    if bound <= BOUND_FINE_MAX:
        return min(39, math.ceil(bound / BOUND_FINE_Q - 1e-9))
    return min(BOUND_INVALID - 1, 39 + math.ceil((bound - BOUND_FINE_MAX) / BOUND_COARSE_Q - 1e-9))


def decode_bound(code: int) -> float:
    if code <= 39:
        return code * BOUND_FINE_Q
    return BOUND_FINE_MAX + (code - 39) * BOUND_COARSE_Q


@dataclass(frozen=True)
class Payload:
    frame: int
    p: np.ndarray
    bound: float
    gate_idx: int = 0
    phase: int = GP_NONE

    @property
    def valid_position(self) -> bool:
        return math.isfinite(self.bound)


def encode(frame: int, p, bound: float, gate_idx: int = 0, phase: int = GP_NONE) -> bytes:
    """Sender side.  ``bound`` already covers the age of the estimate at the emission instant."""
    x = int(round(float(p[0]) / POS_Q))
    y = int(round(float(p[1]) / POS_Q))
    depth = int(round(-float(p[2]) / POS_Q))
    b = encode_bound(float(bound) + QUANT_ERR) if math.isfinite(bound) else BOUND_INVALID
    if not (-2048 <= x <= 2047 and -2048 <= y <= 2047 and 0 <= depth <= 255):
        b = BOUND_INVALID
        x, y, depth = max(-2048, min(2047, x)), max(-2048, min(2047, y)), max(0, min(255, depth))
    b = min(b, BOUND_INVALID)
    bits = (((frame % FRAMES) << 46) | ((x & 0xFFF) << 34) | ((y & 0xFFF) << 22) | ((depth & 0xFF) << 14) | ((b & 0x3F) << 8)
            | ((int(gate_idx) & 0x3) << 6) | ((int(phase) & 0x3) << 4))
    head = bits.to_bytes(6, "big")
    return head + crc16(head).to_bytes(2, "big")


def decode(packet: Optional[bytes]) -> Optional[Payload]:
    """Receiver side: None for a missing or corrupted packet.  An invalid bound (63) returns a Payload with bound = inf:
    the gate phase is still a statement of the sender about itself, the position is unusable (``valid_position``)."""
    if packet is None or len(packet) != PAYLOAD_BITS // 8:
        return None
    head, crc = packet[:6], int.from_bytes(packet[6:], "big")
    if crc16(head) != crc:
        return None
    bits = int.from_bytes(head, "big")
    b = (bits >> 8) & 0x3F

    def s12(v):
        return v - 0x1000 if v & 0x800 else v
    x, y = s12((bits >> 34) & 0xFFF), s12((bits >> 22) & 0xFFF)
    depth = (bits >> 14) & 0xFF
    return Payload(frame=(bits >> 46) & 0x3, p=np.array([x * POS_Q, y * POS_Q, -depth * POS_Q]),
                   bound=math.inf if b == BOUND_INVALID else decode_bound(b),
                   gate_idx=(bits >> 6) & 0x3, phase=(bits >> 4) & 0x3)


def air_time_s(rate_bps: float) -> float:
    return PAYLOAD_BITS / float(rate_bps)


FRAME_GUARD_S = 0.02


def frame_index(t_tx: float, period_s: float) -> int:
    """TDMA frame of an emission instant (slot k of frame m at m P + k P / n)."""
    return int(math.floor((t_tx + FRAME_GUARD_S) / period_s)) % FRAMES
