"""Onboard side of the optional one-way presence beacon (variant B1, OFF in the baseline).

A reception carries only a static identity agreed before the mission (which drone holds which slot, like the
formation template itself).  Its only meaning is "drone k is currently detectable".  It says nothing about where
drone k is, how it moves or what it decides; an absent reception means nothing either (the channel may have lost
it).  How the perception may use this evidence is fixed in perception.py (``presence`` rules) and can only make a
decision more conservative or delay a declaration of absence: it never clears a collision risk, never frees a
critical region and never grants a priority.
"""

from __future__ import annotations

from typing import Dict, Optional

from holo_fleet.config import DEFAULT, FleetConfig


class PresenceTracker:
    def __init__(self, cfg: FleetConfig = DEFAULT):
        self.cfg = cfg.beacon
        self.enabled = cfg.beacon.enabled
        self.slot_of: Dict[int, int] = {}            # static identity -> slot (pre-mission plan)
        self.last_heard: Dict[int, float] = {}       # slot -> reception time of its latest ping
        self.receptions = 0

    def set_identities(self, ids: Dict[int, int]) -> None:
        self.slot_of = {int(k): int(v) for k, v in ids.items()}

    def update(self, frame) -> None:
        if not self.enabled:
            return
        for ident, t_rx in frame.data.get("BeaconRx") or ():
            slot = self.slot_of.get(int(ident))
            if slot is None:
                continue
            self.receptions += 1
            self.last_heard[slot] = max(self.last_heard.get(slot, -1e9), float(t_rx))

    def heard(self, slot: int, t: float) -> bool:
        """Evidence of presence: drone of ``slot`` heard within ttl_s (False also means "no evidence", never "absent")."""
        return self.enabled and t - self.last_heard.get(slot, -1e9) <= self.cfg.ttl_s + 1e-9

    def age(self, slot: int, t: float) -> Optional[float]:
        return None if slot not in self.last_heard else t - self.last_heard[slot]
