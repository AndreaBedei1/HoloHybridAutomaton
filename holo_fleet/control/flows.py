"""Continuous flows of the local hybrid automaton (one control law per mode) - v2.

All inputs are onboard: the perception observation (navigation estimate, classified sonar
sectors, gate and formation observations), the mission plan and the onboard current estimate of
the low-level controller.  Output: desired world velocity and heading.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.control.escape import EscapeChoice, EscapePlanner
from holo_fleet.mission import MissionPlan
from holo_fleet.perception.perception import LocalObservation
from holo_fleet.perception.sonar_processing import OBSTACLE_CLASSES


def _cap(v: np.ndarray, vmax_xy: float, vmax_z: float) -> np.ndarray:
    v = np.asarray(v, dtype=float).copy()
    n = float(np.linalg.norm(v[:2]))
    if n > vmax_xy:
        v[:2] *= vmax_xy / n
    v[2] = float(np.clip(v[2], -vmax_z, vmax_z))
    return v


class Flows:
    def __init__(self, plan: MissionPlan, cfg: FleetConfig = DEFAULT):
        self.plan = plan
        self.cfg = cfg
        self.escape = EscapePlanner(cfg)
        self.sigma = 0.0                      # local along-track progress offset (sonar consensus)
        self.last_choice: Optional[EscapeChoice] = None
        self.hold_pos: Optional[np.ndarray] = None
        self.offset = np.zeros(3)             # give-way offset of the mission target (world) [m]
        self.giveway: str = ""                # "", "RIGHT", "UP", "DOWN"
        self.giveway_t = -1e9                 # last time the head-on condition held

    # ------------------------------------------------------------------ give-way (traffic rule)
    def traffic_offset(self, obs: LocalObservation, dt: float, r_trigger: float = 8.0,
                       offset_max: float = 2.5, rate: float = 0.35) -> np.ndarray:
        """Head-on rule (as COLREG rule 14): a drone that sees a DYNAMIC echo closing in its FRONT sector
        shifts its mission target to its right; if a neighbour occupies the right side it shifts vertically
        instead (up when heading east-ish, down otherwise: opposite for two drones meeting head-on).
        Decided before the warning band, from sector patterns only; safety does not rely on it (P1 is
        enforced by the warning filter and the escape), it avoids head-on standoffs."""
        R = obs.R_wb
        if not self.cfg.traffic_rule:
            return self.offset
        front = [tg for tg in obs.targets if tg.cls == "DYNAMIC" and "FRONT" in tg.pattern
                 and tg.r_min < r_trigger and tg.closing_rate >= 0.45 and not tg.possible_neighbour]
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
    def formation(self, obs: LocalObservation, dt: float, recovering: bool = False) -> Tuple[np.ndarray, float]:
        fr = self.cfg.form
        clock = self.plan.clock
        t = obs.t
        v_clock = (clock.s(t + dt) - clock.s(t)) / dt if clock is not None else 0.0
        # sonar progress consensus: drift sigma towards the neighbours' actual along-track positions
        self.sigma += dt * (fr.k_progress * obs.form.along_corr - 0.02 * self.sigma)
        self.sigma = float(np.clip(self.sigma, -fr.progress_max, fr.progress_max))
        _slot, R_form, _s = self._slot(obs)
        t_hat, n_hat = R_form[:, 0], R_form[:, 1]
        self.traffic_offset(obs, dt)
        err = obs.slot_pos - obs.p             # slot_pos already includes the give-way offset
        v = t_hat * v_clock + fr.k_slot * err
        corr = fr.k_sonar * obs.form.lateral_corr * n_hat
        if np.linalg.norm(corr) > fr.v_corr_max:
            corr *= fr.v_corr_max / np.linalg.norm(corr)
        v = v + corr
        v = _cap(v, fr.v_recovery_max if recovering else fr.v_slot_max, 0.3)
        return v, math.atan2(t_hat[1], t_hat[0])

    def _slot(self, obs):
        clock = self.plan.clock
        s_ref = (clock.s(obs.t) if clock is not None else 0.0) + self.sigma
        _p, d0, n0 = self.plan.path.frame_at(s_ref)
        R = np.array([[d0[0], n0[0], 0.0], [d0[1], n0[1], 0.0], [0.0, 0.0, 1.0]])
        return None, R, s_ref

    # ------------------------------------------------------------------ gate (P2)
    def pass_path(self, queue_s: float, queue_l: float) -> np.ndarray:
        """Gate-frame polyline (s, l) of a committed drone: queue point -> axis -> through -> own lane.

        Forward along the own queue lane, then down to the merge point (merge_s, 0) on a fixed heading
        ``merge_angle_deg`` from the axis; GateRule.queue_s(n) places the queue line so that this
        diagonal keeps ``merge_clearance`` from the queue point on the right (the next drone in the
        left-first order).  Beyond the CR the drone veers towards its own formation lane."""
        G = self.cfg.gate
        lane = self.plan.slots[self.plan.slot_index].lateral       # path direction == gate axis
        m_s = G.merge_s
        pts = [(queue_s, queue_l)]
        if abs(queue_l) > 0.3:
            k_s = m_s - abs(queue_l) / math.tan(math.radians(G.merge_angle_deg))
            if k_s > queue_s + 0.1:
                pts.append((k_s, queue_l))
        pts += [(m_s, 0.0), (G.veer_s, 0.0), (G.exit_s + 3.0, lane), (G.rally_s + 6.0, lane)]
        return np.array(pts, dtype=float)

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
        G = self.cfg.gate
        g = self.plan.gates[min(obs.gate_index, len(self.plan.gates) - 1)]
        go = obs.gate
        yaw = math.atan2(g.axis[1], g.axis[0])
        q = np.array([go.s, go.l])
        if mode == "GATE_PASS":
            c = self._carrot(self.pass_path(go.queue_s, go.queue_l), q)
            tgt = g.from_gate_frame([c[0], c[1], 0.0])
            vmax = G.v_pass
        else:  # GATE_APPROACH / GATE_YIELD: go to / hold the own queue point
            tgt = g.from_gate_frame([go.queue_s, go.queue_l, 0.0])
            vmax = G.v_approach
        v = G.k_track * (tgt - obs.p)
        if mode == "GATE_PASS":                                    # carrot: keep the cruise speed
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
    def failsafe(self, obs: LocalObservation) -> np.ndarray:
        if self.hold_pos is None:
            self.hold_pos = obs.p.copy()
            self.hold_pos[2] = float(np.clip(obs.p[2] + self.plan.failsafe_layer_dz, self.cfg.env.z_min + 0.5,
                                             self.cfg.env.z_max - 0.5))
        return _cap(0.5 * (self.hold_pos - obs.p), 0.25, 0.2)

    def clear_failsafe(self) -> None:
        self.hold_pos = None

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
