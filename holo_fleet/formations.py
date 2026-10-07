"""Generic formation templates: F = {r_1*, ..., r_N*} (relative slots in the formation frame).

The formation frame moves along the survey line: x = along-track (forward), y = lateral (left +),
z = up.  The controller is the same for every template: a drone only needs its own slot and the
slots of the others (to know where its neighbours should appear in its sonar sectors).

Why a formation and why recovering it matters (P3)
--------------------------------------------------
The templates are survey geometries, not shapes for their own sake.  Each drone carries a
down-looking survey sensor with a swath of ``swath_m`` metres; the lateral slot offsets put the
swaths side by side so that one pass covers a contiguous strip without gaps.  A current that
breaks the relative geometry opens holes (lateral spacing > swath) or wastes effort (overlap) in
the coverage, so after a perturbation the geometry must be recovered: that is property P3.
``coverage_gap`` measures the operational effect of a formation error.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

from holo_fleet.mission import Slot


@dataclass(frozen=True)
class FormationTemplate:
    name: str
    slots: Tuple[Slot, ...]
    purpose: str
    swath_m: float = 3.2                  # survey-sensor swath of each drone (lateral)

    @property
    def n(self) -> int:
        return len(self.slots)

    def offsets(self) -> np.ndarray:
        return np.array([[s.along, s.lateral, s.dz] for s in self.slots], dtype=float)

    def min_spacing(self) -> float:
        r = self.offsets()
        d = np.linalg.norm(r[:, None, :] - r[None, :, :], axis=-1)
        return float(d[~np.eye(self.n, dtype=bool)].min())

    def centred(self) -> np.ndarray:
        r = self.offsets()
        return r - r.mean(axis=0)

    def lanes(self) -> List[float]:
        return sorted({round(s.lateral, 3) for s in self.slots})

    def coverage_gap(self, lateral_positions: np.ndarray) -> float:
        """Largest uncovered lateral gap [m] between adjacent swaths for actual lateral positions."""
        y = np.sort(np.asarray(lateral_positions, dtype=float))
        if y.size < 2:
            return 0.0
        return float(max(0.0, np.max(np.diff(y)) - self.swath_m))


def _t(name: str, slots: List[Tuple[float, float, float]], purpose: str, swath: float = 3.2) -> FormationTemplate:
    return FormationTemplate(name, tuple(Slot(a, l, z) for a, l, z in slots), purpose, swath)


TEMPLATES: Dict[str, FormationTemplate] = {
    # 3 drones: two outer lanes and a centre lane leading by 3 m (every pair >= 4.6 m apart)
    "triangle": _t("triangle", [(0.0, 3.5, 0.0), (0.0, -3.5, 0.0), (3.0, 0.0, 0.0)],
                   "3 parallel swaths (lanes -3.5 / 0 / +3.5 m); centre lane leading by 3 m", swath=3.6),
    # 4 drones: 2 x 2 box, two lanes surveyed twice (front pass + rear pass, change detection)
    "square": _t("square", [(1.75, 1.75, 0.0), (1.75, -1.75, 0.0), (-1.75, 1.75, 0.0), (-1.75, -1.75, 0.0)],
                 "2 lanes 3.5 m apart, each surveyed by a front and a rear drone", swath=3.6),
    # 6 drones: survey line abreast, 6 contiguous swaths = 21 m strip in one pass
    "line6": _t("line6", [(0.0, 8.75, 0.0), (0.0, 5.25, 0.0), (0.0, 1.75, 0.0), (0.0, -1.75, 0.0),
                          (0.0, -5.25, 0.0), (0.0, -8.75, 0.0)],
                "6 contiguous swaths (3.5 m lane spacing): one pass covers a 21 m strip", swath=3.6),
    # 2 drones on one lane (pair tests)
    "pair_column": _t("pair_column", [(0.0, 0.0, 0.0), (-4.0, 0.0, 0.0)], "two drones on one lane"),
}


def get(name: str) -> FormationTemplate:
    return TEMPLATES[name]
