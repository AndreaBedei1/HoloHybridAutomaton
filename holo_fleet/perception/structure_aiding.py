"""Map-aided correction of the dead-reckoning position near a mapped structure (the gate).

Why.  The drone navigates by dead reckoning (DVL velocity rotated by the compass heading).  A compass heading bias
of a few degrees - inside the calibrated range of underwater AHRS (the sensor-robustness method document, section 1) - turns
into a lateral drift of a few percent of the distance travelled: about 0.5 m after the approach to the gate, more
than the lateral margin of the opening (~0.4 m).  The drone then aims at the centre of the opening in its estimate
but is really off-centre: the bar it approaches is not where the map predicts it, its echo is an obstacle, collision
avoidance pushes the drone back and the passage livelocks or, with momentum, the hull touches the bar.  Real
vehicles bound this drift with landmarks; the gate map is the landmark the mission plan already contains.

How.  Candidate horizontal offsets d of the estimate (a coarse-to-fine grid within +-``MAX_CORR_M``) are scored
against the leading edges of the echoes of every healthy sector: for each sector whose cone, seen from the candidate
pose, contains a mapped bar, the predicted nearest-bar range is matched to the nearest echo (squared residual, capped
at ``ASSOC_GATE_M``: a predicted bar that is not seen costs the cap); a sector in whose cone the CURRENT estimate has a
bar costs the cap for a candidate that moves the bar out of it (no offset is rewarded for explaining a missing echo
away: fades and occlusions are ordinary); other sectors add nothing, since their echoes may be vehicles or the
seabed.  The best offset is accepted only if

* at least two sectors are matched within ``FIT_RMS_M`` RMS, within ``AID_RANGE_M`` of the bars only (a single echo never moves
  the estimate),
* it explains the echoes clearly better than the current estimate and is unique (every candidate within
  ``AMBIGUITY`` of the best cost lies within ``UNIQUE_M`` of it: the geometry observes both horizontal directions),
* it is bounded (|d| <= ``MAX_CORR_M``, the drift a run can accumulate) and
* the same offset (within ``CONSIST_M``) is found in ``CONSIST_N`` consecutive updates;

it is then applied with gain ``GAIN`` to the estimate, and only if it is geometrically safe (robust clearance, below).

Robust clearance and gate localization.  A fit locates the drone with an uncertainty ``u``: the radius of the set of
near-optimal offsets (the positions the echoes cannot tell apart) plus the RMS of the matched ranges.  Before the gate
(s <= the end of the critical region) a fit is usable only if the HULL (lateral half-extent ``HULL_LAT_M``, the BlueROV2
half-width plus the yaw allowance) plus ``u``, around the fitted position, lies entirely inside the free opening
(inner half-width of the gate); otherwise it is rejected, whatever its RMS: the fits that moved the estimate the wrong
way in the replay of the MEDIUM runs all placed the drone beside the opening.  A usable fit whose offset is within
``CONSIST_M`` of the current estimate CONFIRMS it; an applied correction confirms the corrected estimate.  The gate
localization is valid for ``LOC_TTL_S`` after a confirmation (``valid``): a committed drone enters the critical region
only with a valid gate localization (control/flows.py, the relocalization hold before the critical region).

The correction changes the drone's OWN position estimate only: the collision-avoidance distance bound uses the raw echo
ranges in the body frame and does not depend on it, and the structure test keeps its tolerance (structure_tol_m is not
widened), so a vehicle in front of a bar stays an obstacle (tests/test_structure_aiding.py).
"""

from __future__ import annotations

from collections import deque
from typing import Deque, Dict, Optional

import numpy as np

from holo_fleet.perception.sonar_geometry import HULL_HALF, MOUNTS, SECTORS
from holo_fleet.perception.sonar_processing import segment

AID_RANGE_M = 2.0        # active only in the passage: within this of a mapped bar (one drone there, P2)
ASSOC_GATE_M = 0.9       # cap of the residual of one sector (predicted bar not seen / matched to a far echo)
FIT_RMS_M = 0.10         # the matched echoes are explained after the correction
MIN_MATCHED = 2         # within AID_RANGE_M of the bars (the passage: one drone, P2); the queue is excluded
GAIN_MIN = 0.05          # the best offset must lower the total squared residual by at least this [m^2]
AMBIGUITY = 0.005        # [m^2] candidates this close to the best total cost must lie within UNIQUE_M of it
UNIQUE_M = 0.4           # ranges near the gate plane change little with the offset: bounded, not zero ambiguity
MAX_CORR_M = 1.2
CONSIST_M = 0.15
CONSIST_N = 4
GAIN = 1.0               # the offset confirmed in CONSIST_N updates is applied in full (a partial step can stop at an ambiguous pose)
COARSE_M, FINE_M = 0.3, 0.05
POINT_STRIDE = 3        # bar sample points used (every 3rd of the classifier's, ~3 cm apart)
HULL_LAT_M = float(HULL_HALF[1] + HULL_HALF[0] * np.sin(np.radians(5.0)))   # lateral half-extent, yaw within 5 deg
LOC_TTL_S = 1.0          # a confirmation of the gate localization is valid this long
CLEARANCE_GATES_CORRECTION = True   # a fit that puts the hull outside the opening is not applied either (else it
                                    # corrects the estimate but never validates the entry)


class StructureAiding:
    def __init__(self, clf, cfg, gates=()):
        self.clf = clf
        self.cfg = cfg
        self.gates = list(gates)                 # GateSpecs: the free opening of the robust clearance
        self.loc_t = -1e9                        # last confirmation of the gate localization
        self.loc_u: Optional[float] = None       # its uncertainty [m]
        self.recent: Deque[np.ndarray] = deque(maxlen=CONSIST_N)
        self.applied_m = 0.0                     # total applied correction (for the log)
        self.last: Optional[Dict] = None
        self.pts = np.asarray(clf.struct_pts)[::POINT_STRIDE]

    def _pred(self, C: np.ndarray, R_wb: np.ndarray, s: str) -> np.ndarray:
        """Predicted nearest-bar range of sector s for every candidate position C (K, 3); NaN: no bar in the cone.

        The NOMINAL cone is used, without the near-field widening of the classifier's structure window: that window
        must contain every echo a bar can produce (a superset, so that a bar is never taken for a vehicle), while
        here the first echo itself is predicted.  Against the CLEAN validation runs (true pose, gate within 2.5 m)
        the nominal cone predicts the first echo of the horizontal sonars within 0.023-0.026 m (SD) and predicts a
        bar that is not seen in 0-2 % of the samples; the widened cone gives 0.03-0.05 m and 10-17 %."""
        a, _M = self.clf._sector_frame(s, R_wb)
        m = self.clf.model
        d = self.pts[None, :, :] - (C + R_wb @ MOUNTS[s])[:, None, :]
        r = np.linalg.norm(d, axis=2)
        ang = np.arccos(np.clip((d @ a) / np.maximum(r, 1e-9), -1.0, 1.0))
        cone = ang <= np.radians(m.half_angle_deg)
        out = np.where(cone & (r >= m.range_min) & (r <= m.range_max), r, np.inf).min(axis=1)
        return np.where(np.isfinite(out), out, np.nan)

    def _score(self, C, R_wb, obs, must):
        """Total squared (capped) residual, RMS over the matched sectors, and matched sectors, for every candidate.

        A sector whose predicted bar has no echo within the cap pays the cap in the total (it selects the
        candidate) but is not part of the RMS, which measures how closely the echoes that ARE there fit: one
        silent sector (a weak grazing return, a fade) must not reject an offset that the others pin down.
        ``must``: sectors in whose cone the current estimate has a bar; a candidate that moves the bar out of
        the cone pays the cap, so no offset is ever rewarded for explaining a missing echo away."""
        total = np.zeros(len(C))
        sq = np.zeros(len(C))
        used = np.zeros(len(C))
        matched = np.zeros(len(C))
        for s, edges in obs.items():
            pr = self._pred(C, R_wb, s)
            has = ~np.isnan(pr)
            if edges.size:
                res = np.min(np.abs(edges[None, :] - np.nan_to_num(pr)[:, None]), axis=1)
            else:
                res = np.full(len(C), np.inf)
            res = np.minimum(res, ASSOC_GATE_M)
            assoc = has & (res < ASSOC_GATE_M)
            total += np.where(has, res ** 2, ASSOC_GATE_M ** 2 if s in must else 0.0)
            sq += np.where(assoc, res ** 2, 0.0)
            used += assoc
            matched += has & (res <= FIT_RMS_M * 1.5)
        rms = np.where(used > 0, np.sqrt(sq / np.maximum(used, 1)), np.inf)
        return total, rms, matched

    def _gate_frame(self, q: np.ndarray):
        g = min(self.gates, key=lambda gg: float(np.linalg.norm(np.asarray(gg.center)[:2] - q[:2])))
        return g, g.to_gate_frame(q)

    def clearance(self, q: np.ndarray, u: float) -> Optional[float]:
        """Free lateral margin of the hull at position q with uncertainty u in the opening of the nearest gate: inner
        half-width - (|l| + HULL_LAT_M + u); None from the critical region on (the localization is needed before it)."""
        if not self.gates:
            return None
        g, (s_g, l_g, _dz) = self._gate_frame(q)
        if s_g > -self.cfg.gate.cr_half_len:
            return None
        return float(g.inner_width / 2.0 - (abs(l_g) + HULL_LAT_M + u))

    def valid(self, t: float) -> bool:
        """Gate localization confirmed within LOC_TTL_S (robust clearance included)."""
        return t - self.loc_t <= LOC_TTL_S

    def update(self, p: np.ndarray, R_wb: np.ndarray, profiles: Dict, ages: Dict, tau: float,
               t: float = 0.0) -> Optional[np.ndarray]:
        """Horizontal correction to add to the position estimate now (None: nothing accepted)."""
        self.last = None
        pts = self.pts
        if len(pts) == 0 or np.min(np.linalg.norm(pts[:, :2] - p[:2], axis=1)) > AID_RANGE_M:
            self.recent.clear()
            return None
        if self.gates and self._gate_frame(p)[1][0] > -self.cfg.gate.cr_half_len:
            self.recent.clear()                  # inside / beyond the critical region: the drone passes as validated
            return None
        obs = {}
        for s in SECTORS:
            prof = profiles.get(s)
            if prof is None or ages.get(s, 1e9) > tau:
                continue
            obs[s] = np.array([seg[0] for seg in segment(prof, self.clf.model, self.cfg.perc.merge_gap_bins)])
        if len(obs) < MIN_MATCHED:
            self.recent.clear()
            return None
        g = np.arange(-MAX_CORR_M, MAX_CORR_M + 1e-9, COARSE_M)
        D = np.array([[x, y, 0.0] for x in g for y in g])
        must = {s for s in obs if not np.isnan(self._pred(p[None, :], R_wb, s)[0])}
        cost, _r, _m = self._score(p + D, R_wb, obs, must)
        D_coarse, cost_coarse = D, cost
        best = D[int(np.argmin(cost))]
        f = np.arange(-COARSE_M, COARSE_M + 1e-9, FINE_M)
        D = best + np.array([[x, y, 0.0] for x in f for y in f])  # fine grid around the coarse best
        D = D[np.linalg.norm(D[:, :2], axis=1) <= MAX_CORR_M]
        if len(D) == 0:                           # the coarse best lies beyond the bound: nothing to accept
            self.recent.clear()
            return None
        cost, rms, matched = self._score(p + D, R_wb, obs, must)
        if not np.isfinite(cost).any():
            self.recent.clear()
            return None
        k = int(np.argmin(cost))
        c0, r0, _m0 = self._score(p[None, :], R_wb, obs, must)
        d = D[k, :2]
        # near-optimal candidates, the positions the echoes cannot separate: on the fine grid AND on the whole coarse
        # grid (an equally good position far from the best one is the ambiguity that matters most)
        near = np.vstack([D[cost <= cost[k] + AMBIGUITY, :2], D_coarse[cost_coarse <= cost[k] + AMBIGUITY, :2]])
        spread = float(np.max(np.linalg.norm(near - d, axis=1)))
        u = spread + float(rms[k])                  # uncertainty of the fitted position
        margin = self.clearance(p + np.array([d[0], d[1], 0.0]), u)
        self.last = {"matched": int(matched[k]), "offset": np.round(d, 3).tolist(), "rms": round(float(rms[k]), 3),
                     "cost": round(float(cost[k]), 4), "cost_current": round(float(c0[0]), 4), "u": round(u, 3),
                     "clearance": None if margin is None else round(margin, 3)}
        # with a gate map: only before the critical region, where the localization is needed
        fit_ok = (matched[k] >= MIN_MATCHED and rms[k] <= FIT_RMS_M and spread <= UNIQUE_M
                  and (margin is not None or not self.gates))
        safe = margin is None or margin >= 0.0          # the hull + uncertainty inside the free opening
        if not fit_ok or (CLEARANCE_GATES_CORRECTION and not safe):
            self.recent.clear()
            return None
        if safe and float(np.linalg.norm(d)) <= CONSIST_M:   # the current estimate is confirmed (consistency band)
            self.loc_t, self.loc_u = t, u
        if not (np.isfinite(c0[0]) and c0[0] - cost[k] >= GAIN_MIN):
            self.recent.clear()
            return None
        self.recent.append(d)
        if len(self.recent) < CONSIST_N:
            return None
        Rr = np.asarray(self.recent)
        if np.max(np.linalg.norm(Rr - Rr.mean(axis=0), axis=1)) > CONSIST_M:
            return None
        corr = GAIN * Rr.mean(axis=0)
        self.recent.clear()
        self.applied_m += float(np.linalg.norm(corr))
        if safe:
            self.loc_t, self.loc_u = t, u           # the corrected estimate is the confirmed one
        return np.array([corr[0], corr[1], 0.0])
