"""Optional, intermittent acoustic channel (OFF by default; used only in the comparison experiment).

Physical model on the simulator side: a broadcast heartbeat is received by a drone only if the true
distance is within range, it is not dropped (Bernoulli loss) and after a fixed latency.  The payload
is the sender's OWN onboard estimate (nav position, mode, slot), never ground truth.

On the controller side messages are used ONLY as a formation hint for a slot that the drone's sensors
do not currently see.  The safety guards (separation, critical region) never read messages.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List

import numpy as np


@dataclass
class IntermittentChannel:
    drop_prob: float = 0.5
    latency_s: float = 0.3
    range_m: float = 30.0
    period_s: float = 1.0
    seed: int = 0
    sent: int = 0
    delivered: int = 0
    _queue: Deque = field(default_factory=deque)
    _last_tx: Dict[str, float] = field(default_factory=dict)

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed + 4242)

    def broadcast(self, t: float, sender: str, payload: Dict, true_positions: Dict[str, np.ndarray]) -> None:
        if t - self._last_tx.get(sender, -1e9) < self.period_s:
            return
        self._last_tx[sender] = t
        self.sent += 1
        for rx, p in true_positions.items():
            if rx == sender:
                continue
            if np.linalg.norm(p - true_positions[sender]) > self.range_m or self.rng.random() < self.drop_prob:
                continue
            self._queue.append((t + self.latency_s, rx, dict(payload, sender=sender, t_tx=t)))

    def deliver(self, t: float, inboxes: Dict[str, List[Dict]]) -> None:
        keep = deque()
        while self._queue:
            t_rx, rx, msg = self._queue.popleft()
            if t_rx <= t:
                inboxes.setdefault(rx, []).append(msg)
                self.delivered += 1
            else:
                keep.append((t_rx, rx, msg))
        self._queue = keep
