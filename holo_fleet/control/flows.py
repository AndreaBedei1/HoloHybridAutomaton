"""Continuous flows of the local hybrid automaton (one control law per mode) - v2.

All inputs are onboard: the perception observation (navigation estimate, classified sonar
sectors, gate and formation observations), the mission plan and the onboard current estimate of
the low-level controller.  Output: desired world velocity and heading.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.control.escape import EscapeChoice, EscapePlanner
from holo_fleet.control.formation_time import FormationTime
from holo_fleet.mission import MissionPlan
from holo_fleet.perception.perception import LocalObservation
from holo_fleet.perception.sonar_processing import OBSTACLE_CLASSES
from holo_fleet.perception.sonar_geometry import HULL_HALF, R_OUT

RELOC_MARGIN_M = 0.15     # hold point: the hull this far before the critical region
RELOC_ZONE_M = 0.4        # a gate localization confirmed this close to the hold point clears the entry
RELOC_PERIOD_S = 6.0      # relocalization: hold, then back off for a new viewing geometry, alternately
RELOC_BACK_M = 0.6


def _cap(v: np.ndarray, vmax_xy: float, vmax_z: float) -> np.ndarray:
    v = np.asarray(v, dtype=float).copy()
    n = float(np.linalg.norm(v[:2]))
    if n > vmax_xy:
        v[:2] *= vmax_xy / n
    v[2] = float(np.clip(v[2], -vmax_z, vmax_z))
    return v


def pass_path_points(G, queue_s: float, queue_l: float, queue_dz: float = 0.0, lane: float = 0.0,
                     lane_dz: float = 0.0) -> np.ndarray:
    """Gate-frame polyline (s, l, dz) of a committed drone: queue point -> axis -> through -> own lane.

    Forward along the own queue lane, then down to the merge point (merge_s, 0, 0) on a fixed heading
    ``merge_angle_deg`` from the axis; GateRule.queue_s places the queue line so that this diagonal keeps
    ``merge_clearance`` from the queue point on the right (the next drone in the left-first order).  From
    a stacked queue point (queue_dz != 0) the depth offset is kept along the lane and removed on the
    diagonal, so the drone moves away from the drone below / above it before changing depth and is level
    with the gate axis at the merge point (formal M4s).  Beyond the CR the drone veers towards its own
    formation lane and slot depth (``lane``, ``lane_dz``)."""
    m_s = G.merge_s
    pts = [(queue_s, queue_l, queue_dz)]
    if abs(queue_l) > 0.3:
        k_s = m_s - abs(queue_l) / math.tan(math.radians(G.merge_angle_deg))
        if k_s > queue_s + 0.1:
            pts.append((k_s, queue_l, queue_dz))
    pts += [(m_s, 0.0, 0.0), (G.veer_s, 0.0, 0.0), (G.exit_s + 3.0, lane, lane_dz), (G.rally_s + 6.0, lane, lane_dz)]
    return np.array(pts, dtype=float)


class Flows:
    def __init__(self, plan: MissionPlan, cfg: FleetConfig = DEFAULT):
        self.plan = plan
        self.cfg = cfg
        self.escape = EscapePlanner(cfg)
        self.sigma = 0.0                      # local along-track progress offset (sonar consensus)
        self.lam = 0.0                        # local lateral slot offset (sonar consensus) [m]
        self.lat_offset = np.zeros(3)         # ... as a world vector, added to the slot reference
        self.ftime = FormationTime(plan.clock, cfg)   # local formation time (reduced progress)
        self.last_choice: Optional[EscapeChoice] = None
        self.hold_pos: Optional[np.ndarray] = None
        self.offset = np.zeros(3)             # give-way offset of the mission target (world) [m]
        self.cr_cleared = False               # gate localization valid at the hold point: may enter the CR
        self.reloc_hold = False               # holding before the CR for the gate localization (logged)
        self.hold_since: Optional[float] = None   # relocalization hold before the CR started at
        self.hold_timeout = False             # the stage-2 hold exceeded t_occ_max + t_clear (logged once)
        self.entry_state = ""                 # stage 2: "", "HOLD", "ADMIT" (logged for the gate events E4 / E7)
        self.events: List[Dict] = []          # ENTRY_HOLD_TIMEOUT (drained by the controller)
        self.abort_request = False            # stage-2 hold beyond the slowest release: give the passage up
        self.giveway: str = ""                # "", "RIGHT", "UP", "DOWN"
        self.giveway_t = -1e9                 # last time the head-on condition held
        self.rejoin_phase = ""                # "", "DROP_BACK", "TO_LANE", "PASS" (rejoin from behind, recovering only)
        self.passing = False                  # PASS latch (kept through avoidance episodes); see rejoin_entry
        self.at_level = False                 # ... at the pass level (hysteresis)

    # ------------------------------------------------------------------ give-way (traffic rule)
    def traffic_offset(self, obs: LocalObservation, dt: float, r_trigger: float = 8.0,
                       offset_max: float = 2.5, rate: float = 0.35, active: bool = True,
                       decide: bool = True) -> np.ndarray:
        """Head-on rule (as COLREG rule 14): a drone that sees a DYNAMIC echo closing in its FRONT sector
        shifts its mission target to its right; if a neighbour occupies the right side it shifts vertically
        instead (up when heading east-ish, down otherwise: opposite for two drones meeting head-on).
        Decided before the warning band, from sector patterns only; safety does not rely on it (P1 is
        enforced by the warning filter and the escape), it avoids head-on standoffs.  Not ``active`` (the
        drone is in FAILSAFE, holding): no give-way is decided and a running one is cancelled - a decision
        latched while holding would displace the mission target after the drone resumes.  Not
        ``decide`` (the drone recovers from behind its slot, catching up with its own fleet): no new give-way
        is decided, a running one ends as usual - the drones ahead close in its FRONT sector at the catch-up
        speed and, off its slot, it does not recognise them as neighbours."""
        R = obs.R_wb
        if not self.cfg.traffic_rule:
            return self.offset
        front = [tg for tg in obs.targets if tg.cls == "DYNAMIC" and "FRONT" in tg.pattern
                 and tg.r_min < r_trigger and tg.closing_rate >= 0.45 and not tg.possible_neighbour] if active else []
        if not decide and not self.giveway:
            front = []
        if not active:
            self.giveway = ""
        if front:
            self.giveway_t = obs.t
            if not self.giveway:
                right_busy = any("RIGHT" in tg.pattern and tg.r_min < 4.5 for tg in obs.targets)
                if not right_busy:
                    self.giveway = "RIGHT"
                else:
                    self.giveway = "UP" if math.cos(obs.yaw) > 0.0 else "DOWN"
        elif self.giveway and obs.t - self.giveway_t > 6.0:
            # release 6 s after the last head-on detection, once no unexpected echo within 5 m still closes
            closing = [tg for tg in obs.targets if tg.r_min < 5.0 and tg.closing_rate > 0.1 and not tg.possible_neighbour]
            if not closing:
                self.giveway = ""
        body = {"RIGHT": np.array([0.0, -1.0, 0.0]), "UP": np.array([0.0, 0.0, 1.0]),
                "DOWN": np.array([0.0, 0.0, -1.0])}.get(self.giveway)
        target = np.zeros(3) if body is None else offset_max * (R @ body)
        if body is not None and self.giveway in ("UP", "DOWN"):
            z = obs.p[2] + target[2]
            target[2] = float(np.clip(z, self.cfg.env.z_min + 0.5, self.cfg.env.z_max - 0.5) - obs.p[2])
        step = target - self.offset
        n = float(np.linalg.norm(step))
        if n > rate * dt:
            step *= rate * dt / n
        self.offset = self.offset + step
        return self.offset

    # ------------------------------------------------------------------ formation (P3)
    def formation(self, obs: LocalObservation, dt: float, recovering: bool = False,
                  traffic: bool = True) -> Tuple[np.ndarray, float]:
        fr = self.cfg.form
        # speed of the own formation reference: the planned speed, reduced while a neighbour is missing
        v_clock = self.ftime.speed(obs.t, dt)
        err = obs.slot_pos - obs.p             # slot_pos already includes the give-way offset
        # sonar progress consensus: drift sigma towards the neighbours' actual along-track positions.  Not while the
        # drone is far from its own slot (beyond e_lost): there the range residuals mostly measure its own
        # displacement (a drone left behind sees every neighbour 'ahead') and would carry its reference away from
        # the fleet; near its slot, in any mode, it is what re-aligns the references of the drones
        # lateral consensus, the same integral on the lateral residual: the dead reckoning drifts sideways (a compass
        # bias turns into a lateral error of a few percent of the distance run) and the drone, on its slot in its own
        # estimate, is really beside it; the proportional sonar term below only shares that error with the slot
        # tracking, the integrated offset moves the slot reference until the neighbours' ranges agree
        near = float(np.linalg.norm(err)) < fr.e_lost
        if near:
            self.sigma += dt * (fr.k_progress * obs.form.along_corr - 0.02 * self.sigma)
            self.sigma = float(np.clip(self.sigma, -fr.progress_max, fr.progress_max))
            self.lam += dt * (fr.k_lateral * obs.form.lateral_corr - 0.02 * self.lam)
            self.lam = float(np.clip(self.lam, -fr.progress_max, fr.progress_max))
        elif obs.form.lateral_far is not None:
            # away from the slot the ranges say nothing, the identified fixes still measure how far the own lane is from where
            # the neighbours are (Perception.lateral_far): the reference moves with them, along-track stays with the own clock
            self.lam += dt * (fr.k_lateral * obs.form.lateral_far - 0.02 * self.lam)
            self.lam = float(np.clip(self.lam, -fr.progress_max, fr.progress_max))
        _slot, R_form, s_ref = self._slot(obs)
        t_hat, n_hat = R_form[:, 0], R_form[:, 1]
        self.lat_offset = self.lam * np.array([n_hat[0], n_hat[1], 0.0])
        # a recovering drone behind its slot is catching up with its own fleet, not meeting it head-on: the drones
        # ahead close in its FRONT sector at the catch-up speed and, while it is off its slot, are not recognised
        # as neighbours; no new give-way is decided then (it would push it towards the next lane)
        catching_up = recovering and float(err @ t_hat) > fr.e_ok
        if not traffic:                                          # FAILSAFE: a PASS latch ends with the episode
            self.passing = self.at_level = False
        self.traffic_offset(obs, dt, active=traffic, decide=not catching_up)
        entry = self.rejoin_entry(obs, R_form, s_ref) if recovering else None
        if not recovering:
            self.rejoin_phase = ""
        # the DROP_BACK exists because the fleet is moving: a drone off its lane lets it pass.  With the reference not moving
        # (a planned hold, the end of the path) nothing will pass, and the phase would keep the drone off its slot for good
        # (v10: the slot error constant until the deadline; a reversal made a limit cycle): the drone goes straight to its slot
        # and the separation layer keeps it clear of the others
        if entry is not None and self.rejoin_phase == "DROP_BACK" and v_clock <= 1e-6:
            entry, self.rejoin_phase = None, ""
        if entry is None:
            v = t_hat * v_clock + fr.k_slot * err
            corr = fr.k_sonar * obs.form.lateral_corr * n_hat
            if np.linalg.norm(corr) > fr.v_corr_max:
                corr *= fr.v_corr_max / np.linalg.norm(corr)
            v = v + corr
        elif self.rejoin_phase == "DROP_BACK":   # let the fleet pass: slow down (never reverse), no lateral motion
            e = entry - obs.p
            v_al = float(np.clip(v_clock + fr.k_slot * float(e @ t_hat), 0.0, max(v_clock, 0.0)))
            v = v_al * t_hat + np.array([0.0, 0.0, fr.k_slot * float(e[2])])
        else:                                    # TO_LANE: to the own lane behind the rear-most slot
            v = t_hat * v_clock + fr.k_slot * (entry - obs.p)
        v = _cap(v, fr.v_recovery_max if recovering else fr.v_slot_max, 0.3)
        return v, math.atan2(t_hat[1], t_hat[0])

    def rejoin_entry(self, obs: LocalObservation, R_form: np.ndarray, s_ref: float) -> Optional[np.ndarray]:
        """Rejoin from behind.  A recovering drone that is off its own lane does not head straight for
        its slot, which could take it across the lanes of the drones in formation: it first falls behind the
        formation (DROP_BACK: along-track correction only, no lateral motion, while it is level with the
        fleet), then moves to its own lane at ``rejoin_back_m`` behind the rear-most slot (TO_LANE), and from
        there advances along its own lane to the slot (direct slot tracking, ``rejoin_phase`` = "").
        Only the shared plan and the own navigation are used."""
        fr = self.cfg.form
        me = self.plan.my_slot
        t_hat, n_hat = R_form[:, 0], R_form[:, 1]
        p_lane, _d, _n = self.plan.path.frame_at(s_ref + me.along)
        # the own lane, as corrected by the consensus and displaced by a running give-way: sending a drone back from a give-way
        # to the lane it left puts it head-on again (p1_two_lines s921: a standoff until the deadline)
        lane = me.lateral + self.lam + float(self.offset[:2] @ n_hat[:2])
        lane_err = float((obs.p[:2] - p_lane[:2]) @ n_hat[:2]) - lane
        blk = self._lane_blocked()
        if abs(lane_err) <= fr.rejoin_lane_tol_m:
            if blk is None:
                self.passing = False
                self.rejoin_phase = ""
                return None
            p0, _d, _n = self.plan.path.frame_at(s_ref)
            a = float((obs.p[:2] - p0[:2]) @ t_hat[:2])          # own along-track position in the formation frame
            z, z_pass = float(obs.p[2]), self.plan.path.depth_z + blk[1]
            z_slot = self.plan.path.depth_z + me.dz
            past = min(me.along, blk[0] + fr.pass_dz_m) - 0.5    # rise from the pass level from here on
            in_layer = abs(z - z_pass) < abs(z - z_slot)
            # latched: entered only behind the rear slot (or in the pass layer well before the rise point), left
            # only past it - a drone between the rear slot and its own one, or rising, is never sent back down
            if self.passing and a >= past:
                self.passing = False
            elif not self.passing and (a < blk[0] or (in_layer and a < past - 1.0)):
                self.passing = True
            if not self.passing:
                self.rejoin_phase = ""
                return None
            # pass on the pass level: descend first (advancing at most up to pass_dz_m behind the rear slot, never
            # backwards: a vertical move only ever widens the distance to the drones on the slot level), then advance
            # along the lane at that level up to the rise point (level hysteresis 0.3 / 0.8 m)
            if abs(z - z_pass) <= 0.3:
                self.at_level = True
            elif abs(z - z_pass) > 0.8:
                self.at_level = False
            along = me.along if self.at_level else min(me.along, max(a, blk[0] - fr.pass_dz_m))
            p_e, _d, n_e = self.plan.path.frame_at(s_ref + along)
            self.rejoin_phase = "PASS"
            return np.array([p_e[0] + lane * n_e[0], p_e[1] + lane * n_e[1], z_pass])
        self.passing = False
        rear = min(sl.along for sl in self.plan.slots) - fr.rejoin_back_m
        if blk is not None:                                      # a blocked lane is joined on the pass level, at
            rear = min(rear, blk[0] - fr.pass_dz_m)              # least pass_dz_m behind the rear slot (review)
        p_r, _d, n_r = self.plan.path.frame_at(s_ref + rear)
        dz = me.dz if blk is None else blk[1]
        entry = np.array([p_r[0] + lane * n_r[0], p_r[1] + lane * n_r[1], self.plan.path.depth_z + dz + float(self.offset[2])])
        behind = float((entry - obs.p) @ t_hat) >= -0.5          # at (or behind) the rear entry point
        self.rejoin_phase = "TO_LANE" if behind else "DROP_BACK"
        return entry

    def _lane_blocked(self) -> Optional[Tuple[float, float]]:
        """(along of the front-most slot behind the own one on its lane, dz of the pass level), or None.  A slot is
        on the own lane when its lane and level are nearer than pass_dz_m; the pass level is pass_dz_m below the own
        slot (above when the depth band has no room) and must keep pass_dz_m from every other slot's lane."""
        fr, env = self.cfg.form, self.cfg.env
        me = self.plan.my_slot
        others = [sl for k, sl in enumerate(self.plan.slots) if k != self.plan.slot_index]
        behind = [sl.along for sl in others
                  if sl.along < me.along and math.hypot(sl.lateral - me.lateral, sl.dz - me.dz) < fr.pass_dz_m - 1e-6]
        if not behind:
            return None
        for dz in (me.dz - fr.pass_dz_m, me.dz + fr.pass_dz_m):
            z = self.plan.path.depth_z + dz
            if env.z_min + 0.5 - 1e-6 <= z <= env.z_max - 0.5 + 1e-6 and all(
                    math.hypot(sl.lateral - me.lateral, sl.dz - dz) >= fr.pass_dz_m - 1e-6 for sl in others):
                return max(behind), dz
        return None

    def _slot(self, obs):
        clock = self.plan.clock
        s_ref = (clock.s(self.ftime.now(obs.t)) if clock is not None else 0.0) + self.sigma
        _p, d0, n0 = self.plan.path.frame_at(s_ref)
        R = np.array([[d0[0], n0[0], 0.0], [d0[1], n0[1], 0.0], [0.0, 0.0, 1.0]])
        return None, R, s_ref

    # ------------------------------------------------------------------ gate (P2)
    def pass_path(self, queue_s: float, queue_l: float, queue_dz: float = 0.0) -> np.ndarray:
        """Gate-frame polyline (s, l, dz) of a committed drone: queue point -> axis -> through -> own lane.
        See :func:`pass_path_points`."""
        me = self.plan.slots[self.plan.slot_index]                  # path direction == gate axis
        g = self.plan.gates[0] if self.plan.gates else None
        lane_dz = (self.plan.path.depth_z + me.dz - float(g.center[2])) if g is not None else 0.0
        return pass_path_points(self.cfg.gate, queue_s, queue_l, queue_dz, me.lateral, lane_dz)

    @staticmethod
    def _carrot(path: np.ndarray, q: np.ndarray, look: float = 1.0) -> np.ndarray:
        """Point ``look`` metres ahead of the projection of q on the polyline."""
        best = (1e18, 0, 0.0)
        for k in range(len(path) - 1):
            a, b = path[k], path[k + 1]
            d = b - a
            L = float(np.linalg.norm(d))
            u = float(np.clip((q - a) @ d / max(L * L, 1e-9), 0.0, 1.0))
            dist = float(np.linalg.norm(a + u * d - q))
            if dist < best[0] - 1e-9:
                best = (dist, k, u * L)
        _, k, along = best
        rem = look
        while k < len(path) - 1:
            a, b = path[k], path[k + 1]
            L = float(np.linalg.norm(b - a))
            if along + rem <= L:
                return a + (b - a) * (along + rem) / max(L, 1e-9)
            rem -= L - along
            along = 0.0
            k += 1
        return path[-1]

    def gate(self, obs: LocalObservation, mode: str) -> Tuple[np.ndarray, float]:
        self.passing = self.at_level = False                     # a gate episode ends a PASS latch (review)
        G = self.cfg.gate
        g = self.plan.gates[min(obs.gate_index, len(self.plan.gates) - 1)]
        go = obs.gate
        yaw = math.atan2(g.axis[1], g.axis[0])
        hold = False
        if mode == "MUTEX_PASS":
            c = self._carrot(self.pass_path(go.queue_s, go.queue_l, go.queue_dz), np.array([go.s, go.l, go.dz]))
            # two-stage entry.  Stage 1 is the commit at the queue point (automaton: priority and the latch FREE).
            # Stage 2 is the admission at the hold point before the CR: a valid gate localization (the hull plus the
            # uncertainty of a map fit inside the free opening: perception/structure_aiding.py) AND no hull seen or
            # heard in the CR from there (GateObs.entry_ok).  Short of it the committed drone holds on its path before
            # the CR and every RELOC_PERIOD_S backs off by RELOC_BACK_M and returns (a new viewing geometry); a queued
            # drone sees it in FRONT: a merged target makes a WAIT, a FRONT-only echo holds the latch BUSY.  A hold longer than t_occ_max +
            # t_clear (the slowest release the protocol admits) is logged once as ENTRY_HOLD_TIMEOUT: the evaluation
            # judges it a gate-liveness violation; the drone is never forced into the CR.  It then requests the
            # automaton's give-up edge (ha/spec.py, pass_abort: committed is cleared, MUTEX_APPROACH, back to the
            # queue point, where the stage-1 rule decides again; a flow-only retreat would keep it committed).
            # Once admitted it completes the passage (no stop inside the CR is added)
            s_hold = -(G.cr_half_len + float(HULL_HALF[0]) + RELOC_MARGIN_M)
            if go.loc_ok and go.entry_ok and go.s >= s_hold - RELOC_ZONE_M:
                self.cr_cleared = True
            # give-up plane: a drone whose centre is behind it is outside the CR whatever its declared gate-frame
            # navigation error (the hull's circumscribed ball R_OUT included)
            s_abort = -(G.cr_half_len + float(R_OUT) + G.nav_err_m)
            self.abort_request = False
            if not self.cr_cleared and (c[0] > s_hold or self.hold_timeout):
                self.hold_since = obs.t if self.hold_since is None else self.hold_since
                if obs.t - self.hold_since > G.t_occ_max + G.t_clear and not self.hold_timeout:
                    self.hold_timeout = True
                    self.events.append({"t": obs.t, "type": "ENTRY_HOLD_TIMEOUT", "loc_ok": go.loc_ok,
                                        "entry_ok": go.entry_ok})
                back = int((obs.t - self.hold_since) / RELOC_PERIOD_S) % 2 == 1
                c = np.array([s_hold - (RELOC_BACK_M if back else 0.0), 0.0, c[2]])
                if self.hold_timeout:
                    # give up: still committed (the queued drones keep waiting), back off behind the give-up plane,
                    # then request the automaton's give-up edge, which clears the commitment; recomputed every step,
                    # never once admitted
                    c = np.array([s_abort - RELOC_MARGIN_M, 0.0, c[2]])
                    self.abort_request = go.s <= s_abort
                hold = True
            self.entry_state = "ADMIT" if self.cr_cleared else ("HOLD" if hold else "")
            tgt = g.from_gate_frame(c)
            vmax = G.v_pass
        else:  # MUTEX_APPROACH / MUTEX_YIELD: go to / hold the own queue point
            self.cr_cleared, self.hold_since, self.entry_state, self.hold_timeout = False, None, "", False
            self.abort_request = False
            tgt = g.from_gate_frame([go.queue_s, go.queue_l, go.queue_dz])
            vmax = G.v_approach
        self.reloc_hold = hold
        v = G.k_track * (tgt - obs.p)
        if mode == "MUTEX_PASS" and not hold:                       # carrot: keep the cruise speed
            n = float(np.linalg.norm(v[:2]))
            if n > 1e-6:
                v[:2] *= vmax / n
        return _cap(v, vmax, 0.25), yaw

    # ------------------------------------------------------------------ separation (P1)
    def threats(self, obs: LocalObservation, band: float):
        env = self.cfg.env
        out = []
        for tg in obs.targets:
            if tg.cls not in OBSTACLE_CLASSES or tg.d_lower >= band:
                continue
            req = env.v_open if tg.d_lower < self.cfg.sep.d_warning else 0.0
            out.append((tg.pattern, req))
        return out

    def separation_warning(self, obs: LocalObservation, v_mission_world: np.ndarray) -> np.ndarray:
        sep = self.cfg.sep
        R = obs.R_wb
        v_des_b = R.T @ v_mission_world
        th = self.threats(obs, sep.d_warning_exit)
        ch = self.escape.warning_velocity(v_des_b, th, obs.unhealthy, obs.yaw, self.cfg.env.v_max_nominal)
        self.last_choice = ch
        return R @ ch.v_body

    def collision_avoidance(self, obs: LocalObservation, current_world: np.ndarray,
                            v_mission_world: np.ndarray) -> np.ndarray:
        env, sep = self.cfg.env, self.cfg.sep
        R = obs.R_wb
        th = self.threats(obs, sep.d_ca_exit)
        occupied = {s for tg in obs.targets if tg.r_min < 4.0 for s in tg.pattern}
        z = obs.p[2]
        ch = self.escape.escape_velocity(th, obs.unhealthy, obs.yaw, env.v_escape, current_body=R.T @ current_world,
                                         mission_body=R.T @ v_mission_world, occupied=occupied,
                                         depth_room=(z - env.z_min, env.z_max - z))
        self.last_choice = ch
        return R @ ch.v_body

    # ------------------------------------------------------------------ failsafe
    def failsafe(self, obs: LocalObservation, probe: bool = False, amp: float = 0.3, period: float = 8.0) -> np.ndarray:
        """Hold the position at the own failsafe layer.  With ``probe`` (an envelope violation is declared) the
        held depth oscillates by +-``amp`` (period ``period``, twice the monitor's window): the vehicle shows,
        with its own DVL, that it moves when asked to before the violation is cleared
        (controller.EnvelopeMonitor)."""
        if self.hold_pos is None:
            self.hold_pos = obs.p.copy()
            self.hold_pos[2] = float(np.clip(obs.p[2] + self.plan.failsafe_layer_dz, self.cfg.env.z_min + 0.5,
                                             self.cfg.env.z_max - 0.5))
        target = self.hold_pos.copy()
        if probe:
            target[2] += amp * math.sin(2.0 * math.pi * obs.t / period)
        return _cap(0.5 * (target - obs.p), 0.25, 0.2)

    def clear_failsafe(self) -> None:
        self.hold_pos = None

    # ------------------------------------------------------------------ keep-out of the critical region (no commit: no entry)
    def keepout_filter(self, obs: LocalObservation, v: np.ndarray) -> np.ndarray:
        """Mission velocity of a drone that is not committed to the gate: no along-axis component towards the CR from the
        keep-out plane (the CR, the hull's circumscribed ball, the declared gate-frame navigation error) on.  Mission modes
        only: the output of the avoidance is not edited, and what a drone inside the box reports does not rest on this filter
        (passage.report_phase)."""
        go, G = obs.gate, self.cfg.gate
        if go.gate_id is None or go.passed or go.s <= -(G.cr_half_len + float(R_OUT) + G.nav_err_m) - 1e-6:
            return v
        reach = G.cr_half_width + float(R_OUT) + G.nav_err_m
        if abs(go.l) > reach or abs(go.dz) > G.cr_half_height + float(R_OUT) + G.nav_err_m or go.s > G.cr_half_len:
            return v
        g = self.plan.gates[min(obs.gate_index, len(self.plan.gates) - 1)]
        v = np.asarray(v, dtype=float).copy()
        a = np.asarray(g.axis, float)
        va = float(v @ a)
        if va > 0.0:
            v -= va * a
        return v

    # ------------------------------------------------------------------ mapped structures (all modes)
    def structure_filter(self, obs: LocalObservation, v: np.ndarray) -> np.ndarray:
        """Never close on a mapped gate bar nearer than ``struct_keepout`` (own nav + gate map)."""
        keep = self.cfg.env.struct_keepout
        v = np.asarray(v, dtype=float).copy()
        for g in self.plan.structures:
            if np.linalg.norm(g.center - obs.p) > 6.0:
                continue
            for b in g.bars:
                local = b.axes.T @ (obs.p - b.center)
                q = np.clip(local, -b.half, b.half)
                near = b.center + b.axes @ q
                d = obs.p - near
                dist = float(np.linalg.norm(d))
                if dist < keep and dist > 1e-6:
                    n = d / dist                                  # pointing away from the bar
                    vn = float(v @ n)
                    if vn < 0.0:
                        v -= vn * n                               # remove the closing component
                    if dist < 0.45:
                        v += 0.15 * n                             # and push away when very close
        return v
