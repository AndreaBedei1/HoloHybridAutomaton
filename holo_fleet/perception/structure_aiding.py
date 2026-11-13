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

* at least two sectors are matched within ``FIT_RMS_M`` RMS (a single echo never moves the estimate),
* it explains the echoes clearly better than the current estimate and is unique (every candidate within
  ``AMBIGUITY`` of the best cost lies within ``UNIQUE_M`` of it: the geometry observes both horizontal directions),
* it is bounded (|d| <= ``MAX_CORR_M``, the drift a run can accumulate) and
* the same offset (within ``CONSIST_M``) is found in ``CONSIST_N`` consecutive updates;

it is then applied with gain ``GAIN`` to the estimate, and only if it is geometrically safe (robust clearance, below).

Robust clearance and gate localization.  A fit locates the drone with an uncertainty ``u``: the radius of the set of
near-optimal offsets (the positions the echoes cannot tell apart) plus the RMS of the matched ranges.  Before the gate
(s <= the end of the critical region) a fit validates the localization only if the HULL (lateral half-extent
``HULL_LAT_M``, the BlueROV2 half-width plus the yaw allowance) plus ``u``, around the fitted position, lies entirely
inside the free opening (inner half-width of the gate); a fit that places the hull beside the opening is not applied
either, whatever its RMS.  Replay of the archived MEDIUM runs with such fits applied (scratch evidence summarised in
docs): they were wrong by 0.4-1.2 m, resting on the echo of another drone taken for a bar in a side sector, and even
four well-matched sectors determine the lateral position beside the opening only to ~0.3 m, more than the near-optimal
spread reports.  A drone whose drift brought it in front of a bar therefore cannot be relocalized there by a range-only
fit without an assumption on the identity of an echo: it holds (safe), the documented C5 limitation.  A usable fit
whose offset is within
``CONSIST_M`` of the current estimate CONFIRMS it; an applied correction confirms the corrected estimate.  The gate
localization is valid for ``LOC_TTL_S`` after a confirmation (``valid``): a committed drone enters the critical region
only with a valid gate localization (control/flows.py, the relocalization hold before the critical region).

Acquisition region.  A fit needs MIN_MATCHED sectors that see a bar.  Before the gate plane, at distance x, the FRONT
cone sees the frame from far away; a side (up / down) cone sees a bar point at offset c from the gate axis only when
x <= tan(half_angle) * |c - e|, e the drone's true offset from the axis.  Over the true positions the fit considers
(|e| <= MAX_CORR_M) the farthest distance at which a drift can be observed at all is therefore
``acquisition_distance`` = tan(half_angle) * (outer half-extent of the frame + MAX_CORR_M) (4.2 m for the 2.0 m opening
and 0.25 m bars): the aiding runs from there to the critical region.  With a small drift the side cones see the bars
only within ~2.2 m, where the bars are also where the map predicts them (explained, no warning); with a drift large
enough to leave a bar unexplained, the far bar enters the far side cone farther away, before the warning band of that
bar (d_warning_exit, tests/test_structure_aiding.py): the fit is no longer acquired only after the drone has been
stopped by the echo it cannot explain.  Nothing is assumed about the identity of an echo: every accepted offset rests
on several sectors matching the bars within FIT_RMS_M, uniquely and consistently over CONSIST_N captures, and the queue
points, beside the opening, never pass the robust clearance below.

Identity from the B2 beacon (v11).  The C5 restriction rests on one premise: an echo matched to a bar may be the hull of
another fleet drone.  When every other drone of the plan is located by a fresh identified acoustic fix (or is dead or
has reported passing this gate) and no echo matched to a bar lies in a sector cone and range that the region of any
of those fixes can reach, the premise is excluded and a fit that places the hull beside the opening may be APPLIED (the
drone learns it is in front of a bar and corrects towards the opening); v10 needed the payload error balls, which grow
with the navigation bound and never excluded anything in MEDIUM.  The test uses the matched echoes only, in the
own frame (the measurement, not any navigation estimate).  It still never VALIDATES the localization: ``valid`` needs a
later fit with the hull inside the opening.  Only fleet drones and the mapped gate are assumed in the gate area (a non-fleet
object there is outside this premise and stays an obstacle for the separation layer).

On the axis of the gate only the FRONT sector sees the frame until about 2.2 m, so an along-axis error leaves the first bar echo
unexplained, the separation layer stops the drone at the band, and the side sectors never see the bars: a deadlock of the perception (RC7,
MEDIUM-ALL lost_drone_mutex s920).  With the same identity premise (every other drone excluded as the source of that echo) a single FRONT echo may
correct the ALONG-AXIS estimate only, uniquely and consistently over CONSIST_N captures, by at most MAX_CORR_M in total until a validated fit.  It
never moves the lateral estimate and never validates the localization.  Identification is by elimination only: any single echo within the correction range is adopted, and is then
labelled STRUCTURE by the classifier.  A hull not accounted for by the premise (a drone silent for t_silent_gate_s) can therefore be taken for a bar.  It relies on the wide FRONT sonar: a
25 deg beam would not see the bars from the axis at this range.

The correction changes the drone's OWN position estimate only: the collision-avoidance distance bound uses the raw echo
ranges in the body frame and does not depend on it, and the structure test keeps its tolerance (structure_tol_m is not
widened), so a vehicle in front of a bar stays an obstacle (tests/test_structure_aiding.py).

A correction accepted without a validated fit (the identity rule, the along-axis correction) is a hypothesis.  Near the gate the sectors see the bars at the edge of their cones and a fit mirrored
about the axis can explain the echoes about as well as the true pose (RC12, MEDIUM-ALL lost_drone_mutex s995: two such fits moved the estimate 2.2 m the wrong way, beyond the reach MAX_CORR_M of any later
fit).  So the step of a correction under the identity rule is capped (IDENTITY_STEP_M): one wrong step stays within reach of a later fit; a right correction arrives in steps.  ``unconfirmed`` is the sum of these
corrections since the start of the current commitment or the last validated fit, whichever is later (a measured crossing also ends it; those made before the commitment include real drift corrected during the
wait at the queue).  When a hold at the gate ends with no valid localization the sum is sealed (``seal_unconfirmed``) and undone after the give-up edge (``take_sealed``; DroneController._drain_flow_events,
Perception.undo_unconfirmed_corrections); what is corrected between the two stays (med7 s981: a right fit between the timeout and the edge would have been removed), and a validated fit or a measured crossing
drops the seal.  This is a recovery heuristic: a hold without validation does not refute a correction.  The bound grows by the step.
"""

from __future__ import annotations

import itertools
import math
from collections import deque
from typing import Deque, Dict, Optional

import numpy as np

from holo_fleet.perception.sonar_geometry import AXES, HULL_HALF, MOUNTS, NEAR_FIELD_WIDEN_M, R_OUT, SECTORS
from holo_fleet.perception.sonar_processing import segment

ASSOC_GATE_M = 0.9       # cap of the residual of one sector (predicted bar not seen / matched to a far echo)
FIT_RMS_M = 0.10         # the matched echoes are explained after the correction
MIN_MATCHED = 2         # sectors matched to the bars (a single echo never moves the estimate)
GAIN_MIN = 0.05          # the best offset must lower the total squared residual by at least this [m^2]
AMBIGUITY = 0.005        # [m^2] candidates this close to the best total cost must lie within UNIQUE_M of it
UNIQUE_M = 0.4           # ranges near the gate plane change little with the offset: bounded, not zero ambiguity
MAX_CORR_M = 1.2
IDENTITY_STEP_M = 0.5    # largest step of a correction under the identity rule (module docstring)
CONSIST_M = 0.15
CONSIST_N = 4
AXIAL_AMBIGUITY_M = 0.04   # a single range: candidates within this of the best residual
GAIN = 1.0               # the offset confirmed in CONSIST_N updates is applied in full (a partial step can stop at an ambiguous pose), but for the cap IDENTITY_STEP_M of a fit under the identity rule
COARSE_M, FINE_M = 0.3, 0.05
POINT_STRIDE = 3        # bar sample points used (every 3rd of the classifier's, ~3 cm apart)
HULL_LAT_M = float(HULL_HALF[1] + HULL_HALF[0] * np.sin(np.radians(5.0)))   # lateral half-extent, yaw within 5 deg
LOC_TTL_S = 1.0          # a confirmation of the gate localization is valid this long


def acquisition_distance(cfg, gate=None, pts=None) -> float:
    """Farthest distance before the gate plane at which a fit can be accepted (module docstring, acquisition region).
    Without a gate map (bar points only) the half-extent is bounded by the largest distance of a point from their
    centroid."""
    if gate is not None:
        corners = [gate.to_gate_frame(b.center + b.axes @ (np.array(sg) * b.half)) for b in gate.bars
                   for sg in itertools.product((-1.0, 1.0), repeat=3)]
        outer = max(max(abs(float(c[1])), abs(float(c[2]))) for c in corners)   # lateral / vertical half-extent
    else:
        outer = float(np.max(np.linalg.norm(pts - pts.mean(axis=0), axis=1)))
    return min(cfg.perc.sonar.range_max, math.tan(math.radians(cfg.perc.sonar.half_angle_deg)) * (outer + MAX_CORR_M))


class StructureAiding:
    def __init__(self, clf, cfg, gates=(), acq_min: float = 0.0):
        self.clf = clf
        self.cfg = cfg
        self.gates = list(gates)                 # GateSpecs: the free opening of the robust clearance
        # the aiding also runs out to the queue line (``acq_min``): off-axis queue points see the frame in two sectors, and
        # with every echo identified as a bar (the identity rule below) a fit there may be applied; v10 stopped at 4.2 m
        self.acq = {g.gate_id: max(acquisition_distance(cfg, g), acq_min) for g in self.gates}
        self.loc_t = -1e9                        # last confirmation of the gate localization
        self.loc_u: Optional[float] = None       # its uncertainty [m]
        self.recent: Deque[np.ndarray] = deque(maxlen=CONSIST_N)
        self.recent_ax: Deque[float] = deque(maxlen=CONSIST_N)
        self.axial_total = 0.0                   # along-axis correction applied since the last validated fit: bounded
        self._ax_touched = False
        self.applied_m = 0.0                     # total applied correction (for the log)
        self.unconfirmed = np.zeros(2)           # sum of the corrections since the last validated fit (module docstring)
        self.sealed: Optional[np.ndarray] = None # that sum when a hold ended with no valid localization: to be undone after the give-up edge
        self.last: Optional[Dict] = None
        self.pts = np.asarray(clf.struct_pts)[::POINT_STRIDE]
        self.acq_pts = acquisition_distance(cfg, pts=self.pts) if len(self.pts) else 0.0
        self.pp = np.einsum("ij,ij->i", self.pts, self.pts) if len(self.pts) else np.zeros(0)

    def _pred(self, C: np.ndarray, R_wb: np.ndarray, s: str) -> np.ndarray:
        """Predicted nearest-bar range of sector s for every candidate position C (K, 3); NaN: no bar in the cone.

        The NOMINAL cone is used, without the near-field widening of the classifier's structure window: that window
        must contain every echo a bar can produce (a superset, so that a bar is never taken for a vehicle), while
        here the first echo itself is predicted.  Against the CLEAN validation runs (true pose, gate within 2.5 m)
        the nominal cone predicts the first echo of the horizontal sonars within 0.023-0.026 m (SD) and predicts a
        bar that is not seen in 0-2 % of the samples; the widened cone gives 0.03-0.05 m and 10-17 %."""
        a, _M = self.clf._sector_frame(s, R_wb)
        m = self.clf.model
        q = C + R_wb @ MOUNTS[s]                                    # sonar position of every candidate
        # exact pre-filter: seen from any candidate within ``rad`` of their centre q0, a point's direction differs from
        # the one seen from q0 by at most asin(rad / |x - q0|); points outside the cone grown by that are never in it
        q0 = q.mean(axis=0)
        rad = float(np.max(np.linalg.norm(q - q0, axis=1)))
        d0 = self.pts - q0
        r0 = np.linalg.norm(d0, axis=1)
        grow = np.arcsin(np.clip(rad / np.maximum(r0, 1e-9), 0.0, 1.0))
        keep = ((d0 @ a >= np.cos(np.minimum(np.radians(m.half_angle_deg) + grow, np.pi)) * r0)
                & (r0 <= m.range_max + rad) & (r0 >= m.range_min - rad))
        P, pp = self.pts[keep], self.pp[keep]
        if len(P) == 0:
            return np.full(len(C), np.nan)
        r = np.sqrt(np.maximum(pp[None, :] - 2.0 * (q @ P.T) + np.einsum("ij,ij->i", q, q)[:, None], 0.0))
        along = (P @ a)[None, :] - (q @ a)[:, None]                 # (point - sonar) . axis, without the K x N x 3 array
        cone = along >= np.cos(np.radians(m.half_angle_deg)) * r
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

    def _fit(self, p: np.ndarray, R_wb: np.ndarray, obs, must):
        """Best offset on a coarse-to-fine grid within MAX_CORR_M and its statistics (offset, total cost, RMS, matched
        sectors, spread of the near-optimal candidates on the fine AND the whole coarse grid: an equally good position
        far from the best one is the ambiguity that matters most), or None."""
        g = np.arange(-MAX_CORR_M, MAX_CORR_M + 1e-9, COARSE_M)
        D_coarse = np.array([[x, y, 0.0] for x in g for y in g])
        cost_c, _r, _m = self._score(p + D_coarse, R_wb, obs, must)
        f = np.arange(-COARSE_M, COARSE_M + 1e-9, FINE_M)
        D = D_coarse[int(np.argmin(cost_c))] + np.array([[x, y, 0.0] for x in f for y in f])
        D = D[np.linalg.norm(D[:, :2], axis=1) <= MAX_CORR_M]
        if len(D) == 0:                           # the coarse best lies beyond the bound: nothing to accept
            return None
        cost, rms, matched = self._score(p + D, R_wb, obs, must)
        if not np.isfinite(cost).any():
            return None
        k = int(np.argmin(cost))
        d = D[k, :2]
        near = np.vstack([D[cost <= cost[k] + AMBIGUITY, :2], D_coarse[cost_c <= cost[k] + AMBIGUITY, :2]])
        return d, float(cost[k]), float(rms[k]), int(matched[k]), float(np.max(np.linalg.norm(near - d, axis=1)))

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

    def start_commitment(self) -> None:
        """Only the corrections of the current commitment can be undone (module docstring)."""
        self.unconfirmed = np.zeros(2)

    def forget_unconfirmed(self) -> None:
        """A validated fit or a measured crossing: nothing before it is left to undo."""
        self.unconfirmed = np.zeros(2)
        self.sealed = None

    def seal_unconfirmed(self) -> None:
        """A hold ended with no valid localization: what is undone after the give-up edge is the sum at this moment."""
        self.sealed = self.unconfirmed.copy()

    def take_sealed(self) -> Optional[np.ndarray]:
        """The correction that undoes the sealed sum (None: there is none to undo); the sums are forgotten."""
        if self.sealed is None:
            return None
        c = -self.sealed
        self.forget_unconfirmed()
        self.axial_total = 0.0
        self.recent.clear()
        self.recent_ax.clear()
        return None if float(np.linalg.norm(c)) == 0.0 else np.array([c[0], c[1], 0.0])

    def valid(self, t: float) -> bool:
        """Gate localization confirmed within LOC_TTL_S (robust clearance included)."""
        return t - self.loc_t <= LOC_TTL_S

    def echoes_not_drones(self, p: np.ndarray, R_wb: np.ndarray, matched: Dict, others) -> bool:
        """None of the echoes matched to a bar can be the hull of a fleet drone: ``others`` = [(samples (n, 3), growth)]
        where each other drone can be (own frame, from its fresh acoustic fix: the navigation of nobody enters); an echo
        whose sector cone and range reach none of them is no drone (module docstring)."""
        env = self.cfg.env
        half0 = math.radians(self.cfg.perc.sonar.half_angle_deg)
        for s, r_e in matched.items():
            m = p + R_wb @ MOUNTS[s]
            u = R_wb @ AXES[s]
            u = u / np.linalg.norm(u)
            lo, hi = max(float(r_e) - env.eps_range_far, 0.0), float(r_e) + env.eps_range_near
            half = half0 + math.atan2(NEAR_FIELD_WIDEN_M, max(lo, 1e-3))
            for pts, rho in others:
                grow = R_OUT + float(rho)
                d = np.asarray(pts, float) - m
                dist = np.maximum(np.linalg.norm(d, axis=1), 1e-6)
                ang = np.arccos(np.clip((d @ u) / dist, -1.0, 1.0))
                if np.any((dist >= lo - grow) & (dist <= hi + grow) & (ang <= half + np.arcsin(np.clip(grow / dist, 0.0, 1.0)))):
                    return False
        return True

    def _axial(self, p: np.ndarray, R_wb: np.ndarray, obs: Dict, g, others) -> Optional[np.ndarray]:
        """Along-axis correction from a single FRONT echo identified as a bar (module docstring), or None."""
        self._ax_touched = True
        edges = obs.get("FRONT")
        if edges is None or not edges.size or others is None:
            self.recent_ax.clear()
            return None
        axis = np.asarray(g.axis, float)
        axis = axis / np.linalg.norm(axis)
        ds = np.arange(-MAX_CORR_M, MAX_CORR_M + 1e-9, FINE_M)
        pr = self._pred(p[None, :] + ds[:, None] * axis, R_wb, "FRONT")
        res = np.min(np.abs(edges[None, :] - np.nan_to_num(pr)[:, None]), axis=1)
        res = np.where(np.isnan(pr), np.inf, res)
        k = int(np.argmin(res))
        near = ds[res <= res[k] + AXIAL_AMBIGUITY_M]
        echo = float(edges[int(np.argmin(np.abs(edges - pr[k])))]) if np.isfinite(res[k]) else None
        if (echo is None or res[k] > FIT_RMS_M * 1.5 or float(near.max() - near.min()) > UNIQUE_M or abs(float(ds[k])) <= CONSIST_M
                or not self.echoes_not_drones(p, R_wb, {"FRONT": echo}, others)):
            self.recent_ax.clear()
            return None
        self.recent_ax.append(float(ds[k]))
        if len(self.recent_ax) < CONSIST_N:
            return None
        a = np.asarray(self.recent_ax)
        if float(a.max() - a.min()) > 2.0 * CONSIST_M:
            return None
        self.recent_ax.clear()
        if abs(self.axial_total + float(a.mean())) > MAX_CORR_M:   # no more than one correction's worth until a validated fit
            return None
        self.axial_total += float(a.mean())
        self.applied_m += abs(float(a.mean()))
        self.last["axial"] = round(float(a.mean()), 3)
        corr = GAIN * float(a.mean()) * np.array([axis[0], axis[1], 0.0])
        self.unconfirmed += corr[:2]
        return corr

    def update(self, p: np.ndarray, R_wb: np.ndarray, profiles: Dict, ages: Dict, tau: float,
               t: float = 0.0, others=None) -> Optional[np.ndarray]:
        """The map fit (``_update``); the along-axis confirmation count survives only consecutive captures that reached it."""
        self._ax_touched = False
        out = self._update(p, R_wb, profiles, ages, tau, t, others)
        if not self._ax_touched:
            self.recent_ax.clear()
        return out

    def _update(self, p: np.ndarray, R_wb: np.ndarray, profiles: Dict, ages: Dict, tau: float,
                t: float = 0.0, others=None) -> Optional[np.ndarray]:
        """Horizontal correction to add to the position estimate now (None: nothing accepted).  ``others``: where every
        other drone of the plan can be (Perception._other_regions), or None when one is not accounted for."""
        self.last = None
        pts = self.pts
        if len(pts) == 0:
            self.recent.clear()
            return None
        if self.gates:
            g, (s_g, _l, _dz) = self._gate_frame(p)
            outside = s_g < -self.acq[g.gate_id] or s_g > -self.cfg.gate.cr_half_len   # from the CR on: passes as validated
        else:                                    # bar points without a gate map: the same distance from the bars
            outside = float(np.min(np.linalg.norm(pts[:, :2] - p[:2], axis=1))) > self.acq_pts
        if outside:
            self.recent.clear()
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
        must = {s for s in obs if not np.isnan(self._pred(p[None, :], R_wb, s)[0])}
        fit = self._fit(p, R_wb, obs, must)
        if fit is None:
            self.recent.clear()
            return None
        d, cost_k, rms_k, matched_k, spread = fit
        c0, _r0, _m0 = self._score(p[None, :], R_wb, obs, must)
        u = spread + rms_k                          # uncertainty of the fitted position
        margin = self.clearance(p + np.array([d[0], d[1], 0.0]), u)
        self.last = {"matched": matched_k, "offset": np.round(d, 3).tolist(), "rms": round(rms_k, 3),
                     "cost": round(cost_k, 4), "cost_current": round(float(c0[0]), 4), "u": round(u, 3),
                     "clearance": None if margin is None else round(margin, 3)}
        # with a gate map: only before the critical region, where the localization is needed
        fit_ok = (matched_k >= MIN_MATCHED and rms_k <= FIT_RMS_M and spread <= UNIQUE_M
                  and (margin is not None or not self.gates))
        safe = margin is None or margin >= 0.0          # the hull + uncertainty inside the free opening
        matched_echoes = {}                        # sector -> the echo matched to a bar by this fit
        cand = (p + np.array([d[0], d[1], 0.0]))[None, :]
        for s, edges in obs.items():
            pr = self._pred(cand, R_wb, s)[0]
            if edges.size and not np.isnan(pr):
                i = int(np.argmin(np.abs(edges - pr)))
                if abs(float(edges[i]) - float(pr)) <= FIT_RMS_M * 1.5:
                    matched_echoes[s] = float(edges[i])
        identified = (not safe and fit_ok and others is not None
                      and self.echoes_not_drones(p, R_wb, matched_echoes, others))
        self.last["identified"] = bool(identified)
        self.last["others"] = None if others is None else len(others)    # None: some other drone not accounted for (log only)
        if not fit_ok or not (safe or identified):       # beside the opening: only with every echo identified as a bar
            self.recent.clear()
            return self._axial(p, R_wb, obs, g, others) if (self.gates and matched_k < MIN_MATCHED) else None
        if safe and float(np.linalg.norm(d)) <= CONSIST_M:   # the current estimate is confirmed (consistency band)
            self.loc_t, self.loc_u = t, u
            self.axial_total = 0.0
            self.forget_unconfirmed()
        if not (np.isfinite(c0[0]) and c0[0] - cost_k >= GAIN_MIN):
            self.recent.clear()
            return None
        self.recent.append(d)
        if len(self.recent) < CONSIST_N:
            return None
        Rr = np.asarray(self.recent)
        if np.max(np.linalg.norm(Rr - Rr.mean(axis=0), axis=1)) > CONSIST_M:
            return None
        corr = GAIN * Rr.mean(axis=0)
        if safe and float(np.linalg.norm(corr)) > self.cfg.gate.nav_err_m:
            safe = False                            # a fit that says the error exceeds the premise of P2 does not validate (round 8, s1103)
        if not safe and float(np.linalg.norm(corr)) > IDENTITY_STEP_M:
            corr = corr * IDENTITY_STEP_M / float(np.linalg.norm(corr))
        self.recent.clear()
        self.applied_m += float(np.linalg.norm(corr))
        if safe:
            self.loc_t, self.loc_u = t, u           # the corrected estimate is the confirmed one
            self.axial_total = 0.0
            self.forget_unconfirmed()
        else:
            self.unconfirmed += corr
        return np.array([corr[0], corr[1], 0.0])
