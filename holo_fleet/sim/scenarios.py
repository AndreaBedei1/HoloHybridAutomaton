"""Short demonstration scenarios (v2): one seed each, 20-90 s, built to show one behaviour each.

  sonar_classification   (bench, see scripts/run_demo.py)    pinned cases A-F of the echo classifier
  p1_head_on             2 drones, no current                 head-on encounter, right-hand passing
  p1_vertical_escape     4 drones + 1 scripted intruder       T formation crossed head-on by a vehicle
                                                              that does not react; traffic rule off: the
                                                              boxed-in drone escapes upwards (safety layer)
  p1_two_lines           6 drones, no current                 two survey lines meet head-on; flanked
                                                              drones give way vertically (traffic rule)
  p1_close_encounter     2 drones, no current                 right-angle crossing, simultaneous arrival:
                                                              the warning filter keeps them apart (traffic
                                                              rule on, no intruder, full safety layer)
  formation_triangle     3 drones, lateral current 0.25       triangle survey under a constant current
  formation_square       4 drones, diagonal current 0.30      2 x 2 box under a diagonal current
  formation_six          6 drones, lateral + vertical current survey line (6 swaths), current with w_z
  formation_recovery_head_current
                         4 drones, 0.6 m/s head jet           a 0.6 m/s jet against the motion on the rear-left
                                                              drone: not control-feasible (DI-27), lost, recovered
  formation_gust         4 drones, temporary 0.85 m/s jet     out-of-envelope stress test: lost and recovered
  gate_single            3 drones, one arena gate (G06)       one at a time through the gate (P2)
  integrated_short       3 drones, cross-current + G06        P1 + P2 + P3 in one short mission
lost drones and the mutex queue (DI-28, DI-29):
  lost_drone_rejoin      4 drones, lateral current 0.35       the rear-left drone loses its thrusters for 7 s and
                                                              is dragged away; the others wait for it on their
                                                              slots (FORMATION_WAIT_REJOIN), it rejoins from behind
  lost_drone_timeout     4 drones, no current                 the same drone loses its thrusters for good: after
                                                              t_rejoin the others declare its slot vacant and keep
                                                              a degraded formation (DEGRADED_FORMATION)
  mutex_deadlock_resolution
                         3 drones, a stacked pair, G06        stacked queue: left first, then top first; the v2 rule
                                                              (static rank with unknown ranks) deadlocked here
  line_parallel_mutex    3 drones abreast, G06                a line arrives at the gate side by side: left first,
                                                              no static rank
  lost_drone_mutex       3 drones abreast, G06                the left drone is lost before the gate: the leftmost
                                                              drone still present goes first (no ID, no deadlock)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates
from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.formations import TEMPLATES, FormationTemplate
from holo_fleet.mission import FormationClock, GateSpec, MissionPlan, Path, Slot, queue_assignment, queue_line, queue_order
from holo_fleet.sim.currents import CurrentComponent, CurrentField
from holo_fleet.sim.spec import SimSpec, StressSpec

DEPTH = -5.0
FAILSAFE_LAYERS = [0.0, -1.2, 1.2, -2.4, 2.4, 0.0]


@dataclass
class Scenario:
    name: str
    title: str
    description: str
    sim: SimSpec
    plans: List[MissionPlan]
    template: Optional[FormationTemplate]
    path: Path
    judged_gates: List[GateSpec]
    formation_enabled: bool = True
    disturbance_windows: List[Tuple[float, float]] = field(default_factory=list)
    focus: str = ""                      # what to look at (UI)
    cfg_patch: Dict = field(default_factory=dict)   # FleetConfig fields overridden for this scenario

    @property
    def duration_s(self) -> float:
        return self.sim.duration_s

    def disturbance_active(self, t: float) -> bool:
        return any(a <= t <= b for a, b in self.disturbance_windows)


def _fleet(name: str, template: FormationTemplate, path: Path, clock: FormationClock, gates: Sequence[GateSpec],
           structures: Sequence[GateSpec], cfg: FleetConfig, s_start: float = 0.0, formation: bool = True):
    plans, spawns, yaws = [], [], []
    qa = queue_assignment(template.slots, cfg.gate)
    q_s = queue_line(template.slots, cfg.gate)
    rank = queue_order(template.slots, cfg.gate)
    for k, sl in enumerate(template.slots):
        p, d, n = path.frame_at(s_start + sl.along)
        pos = np.array([p[0] + sl.lateral * n[0], p[1] + sl.lateral * n[1], path.depth_z + sl.dz])
        yaw = math.degrees(math.atan2(d[1], d[0]))
        plans.append(MissionPlan(drone_id=f"drone_{k}", slot_index=k, slots=list(template.slots), path=path,
                                 gates=list(gates), structures=list(structures), launch_position=pos,
                                 launch_yaw_deg=yaw, static_rank=rank[k], s_end=clock.s_end,
                                 failsafe_layer_dz=FAILSAFE_LAYERS[k % len(FAILSAFE_LAYERS)],
                                 formation_enabled=formation, mission_name=name, template_name=template.name,
                                 queue_lateral=qa[k][0], queue_dz=qa[k][1], queue_s=q_s, clock=clock))
        spawns.append(pos)
        yaws.append(yaw)
    return plans, spawns, yaws


def _uniform(drift) -> CurrentField:
    return CurrentField([CurrentComponent("uniform", drift=tuple(drift))])


def _sim(name, plans, spawns, yaws, duration, current=None, gate_ids=(), camera=0, stress=StressSpec(), seed=0):
    return SimSpec(name=name, names=[p.drone_id for p in plans], spawn_positions=spawns, spawn_yaw_deg=yaws,
                   gate_ids=tuple(gate_ids), current=current or CurrentField(), stress=stress, duration_s=duration,
                   seed=seed, camera_drone=camera)


# ---------------------------------------------------------------------------------------------- P1
def p1_head_on(cfg: FleetConfig = DEFAULT) -> Scenario:
    single = FormationTemplate("single", (Slot(0.0, 0.0, 0.0),), "one drone")
    plans, spawns, yaws = [], [], []
    for k, (a, b) in enumerate((((-6.0, -30.0), (14.0, -30.0)), ((6.0, -30.0), (-14.0, -30.0)))):
        path = Path(np.array([a, b]), DEPTH)
        clock = FormationClock(0.0, 0.32, 1.0, s_end=path.length)
        pl, sp, yw = _fleet("p1_head_on", single, path, clock, (), (), cfg, formation=False)
        pl[0].drone_id, pl[0].static_rank, pl[0].failsafe_layer_dz = f"drone_{k}", k, FAILSAFE_LAYERS[k]
        plans += pl
        spawns += sp
        yaws += yw
    sim = _sim("p1_head_on", plans, spawns, yaws, 30.0, camera=0)
    sim.chase_offset, sim.side_offset = (-6.0, 0.0, 2.2), (8.0, 9.0, 1.5)
    return Scenario("p1_head_on", "P1 head-on encounter",
                    "Two drones on the same line, 12 m apart, closing at 0.64 m/s. No current, no communication.",
                    sim, plans, None, plans[0].path, [], formation_enabled=False,
                    focus="FRONT echo closing -> both give way to the right (sector rule), pass port to port; "
                          "warning filter as safety net; P1 distance >= d_safe")


def p1_vertical_escape(cfg: FleetConfig = DEFAULT) -> Scenario:
    """A non-cooperative vehicle (scripted, no controller, not part of the fleet) crosses a T formation
    head-on, 1.2 m below it.  The centre drone is boxed in: neighbours on its left, right and rear.
    The head-on traffic rule is switched off, so only the safety layer acts: the warning filter and
    the 3-D escape on sector patterns, which go up (the intruder is seen by FRONT+DOWN)."""
    tee = FormationTemplate("tee", (Slot(0.0, 3.5, 0.0), Slot(0.0, 0.0, 0.0), Slot(0.0, -3.5, 0.0), Slot(-3.5, 0.0, 0.0)),
                            "line of three plus a rear drone", 3.6)
    path = Path(np.array([[-14.0, -30.0], [16.0, -30.0]]), DEPTH)
    clock = FormationClock(10.0, 0.25, 1.0, s_end=path.length - 4.0)
    plans, spawns, yaws = _fleet("p1_vertical_escape", tee, path, clock, (), (), cfg, s_start=10.0)
    sim = _sim("p1_vertical_escape", plans, spawns, yaws, 45.0, camera=1)
    # head-on at 0.40 m/s, 1.2 m below the formation; past the centre drone it dives away (scripted)
    sim.intruders = ({"name": "intruder", "start": [7.0, -30.0, DEPTH - 1.2], "yaw_deg": 180.0,
                      "waypoints": [[0.0, 7.0, -30.0, DEPTH - 1.2], [21.0, -1.4, -30.0, DEPTH - 1.2],
                                    [33.0, -5.0, -30.0, DEPTH - 4.9], [60.0, -12.0, -30.0, DEPTH - 6.0]]},)
    sim.chase_offset, sim.side_offset = (-5.0, 5.0, 1.0), (2.0, 9.0, 0.5)
    return Scenario("p1_vertical_escape", "P1 3-D escape from a non-cooperative vehicle",
                    "T formation (line of three + rear drone) at 0.25 m/s; a scripted vehicle that does not react "
                    "crosses it head-on at 0.40 m/s, 1.2 m below, then dives away. Traffic rule OFF: safety layer only.",
                    sim, plans, tee, path, [], formation_enabled=True,
                    focus="boxed-in centre drone (L, R, rear occupied): SEPARATION_WARNING / COLLISION_AVOIDANCE "
                          "escape with an UP component; depth vs time; P1 among the drones and clearance to the intruder",
                    cfg_patch={"traffic_rule": False})


def p1_close_encounter(cfg: FleetConfig = DEFAULT) -> Scenario:
    """Two fleet drones on perpendicular survey legs reach the crossing point at the same time: without
    the safety layer they would meet there.  A right-angle crossing at the survey speed closes at
    0.30 * sqrt(2) = 0.42 m/s and is not a head-on encounter: the traffic rule (head-on, FRONT, closing
    >= 0.45 m/s) may shift a drone to its right on a noisy closing estimate but cannot resolve a crossing,
    so the warning filter, and the escape if the conservative distance fell below d_ca, keep the drones
    apart.  Real fleet BlueROV2s only: no scripted vehicle, traffic rule ON, full automaton, no current."""
    single = FormationTemplate("single", (Slot(0.0, 0.0, 0.0),), "one drone")
    plans, spawns, yaws = [], [], []
    cx, cy, lead = 0.0, -31.0, 7.5                     # crossing point; both legs start 7.5 m before it
    legs = (((cx - lead, cy), (cx + 14.0, cy)), ((cx, cy - lead), (cx, cy + 12.0)))
    for k, (a, b) in enumerate(legs):
        path = Path(np.array([a, b]), DEPTH)
        clock = FormationClock(0.0, cfg.form.v_nominal, 1.0, s_end=path.length)
        pl, sp, yw = _fleet("p1_close_encounter", single, path, clock, (), (), cfg, formation=False)
        pl[0].drone_id, pl[0].static_rank, pl[0].failsafe_layer_dz = f"drone_{k}", k, FAILSAFE_LAYERS[k]
        plans += pl
        spawns += sp
        yaws += yw
    sim = _sim("p1_close_encounter", plans, spawns, yaws, 40.0, camera=0)
    sim.chase_offset, sim.side_offset = (-6.0, 0.0, 2.2), (6.0, -6.0, 6.0)
    return Scenario("p1_close_encounter", "P1 close encounter: right-angle crossing",
                    "Two drones on perpendicular legs, 7.5 m from the crossing point, arrive there together "
                    "(closing 0.42 m/s). Fleet drones only, traffic rule ON, no current, no communication.",
                    sim, plans, None, plans[0].path, [], formation_enabled=False,
                    focus="conservative onboard distance below d_warning -> SEPARATION_WARNING; true pair distance "
                          "against d_warning / d_ca / d_safe; collision avoidance only if the bound fell below d_ca")


def p1_two_lines(cfg: FleetConfig = DEFAULT) -> Scenario:
    """Two survey lines of three drones (3.5 m apart) meet head-on: the middle drones have a
    neighbour on each side, so the traffic rule shifts them vertically instead of to the right."""
    line3 = TEMPLATES["line3"]
    plans, spawns, yaws = [], [], []
    for side, (a, b) in enumerate((((-9.0, -30.0), (14.0, -30.0)), ((9.0, -30.0), (-14.0, -30.0)))):
        path = Path(np.array([a, b]), DEPTH)
        clock = FormationClock(0.0, 0.30, 1.0, s_end=path.length)
        pl, sp, yw = _fleet("p1_two_lines", line3, path, clock, (), (), cfg, formation=True)
        for j, p in enumerate(pl):
            k = 3 * side + j
            p.drone_id, p.static_rank, p.failsafe_layer_dz = f"drone_{k}", k, FAILSAFE_LAYERS[k]
        plans += pl
        spawns += sp
        yaws += yw
    sim = _sim("p1_two_lines", plans, spawns, yaws, 36.0, camera=1)
    sim.chase_offset, sim.side_offset = (-7.0, 2.0, 2.0), (6.0, 10.0, 0.8)
    return Scenario("p1_two_lines", "P1 traffic rule: two survey lines meet head-on",
                    "Lines of three drones (3.5 m apart) on the same track in opposite directions. "
                    "Middle drones are flanked by neighbours: they give way vertically, the outer ones to the right.",
                    sim, plans, None, plans[0].path, [], formation_enabled=False,
                    focus="head-on give-way from sector patterns: RIGHT when the right side is free, else UP/DOWN")


# ---------------------------------------------------------------------------------------------- P3
def _survey(name: str, template_name: str, length: float, current: CurrentField, duration: float,
            title: str, desc: str, focus: str, windows=(), cfg: FleetConfig = DEFAULT, camera: int = 0) -> Scenario:
    tm = TEMPLATES[template_name]
    path = Path(np.array([[-16.0, -34.0], [-16.0 + length, -34.0]]), DEPTH)
    clock = FormationClock(0.0, cfg.form.v_nominal, 2.0, s_end=path.length - 4.0)
    plans, spawns, yaws = _fleet(name, tm, path, clock, (), (), cfg, s_start=0.0)
    sim = _sim(name, plans, spawns, yaws, duration, current=current, camera=camera)
    return Scenario(name, title, desc, sim, plans, tm, path, [], True, list(windows), focus)


def formation_triangle(cfg: FleetConfig = DEFAULT) -> Scenario:
    return _survey("formation_triangle", "triangle", 30.0, _uniform((0.0, 0.25, 0.0)), 45.0,
                   "Triangle survey under a lateral current",
                   "3 drones, 3 swaths; constant lateral (north) current 0.25 m/s drift (inside the envelope).",
                   "slot tracking + sonar spacing keep the triangle: formation error stays small")


def formation_square(cfg: FleetConfig = DEFAULT) -> Scenario:
    return _survey("formation_square", "square", 30.0, _uniform((0.21, 0.21, 0.0)), 45.0,
                   "Square (2 x 2) survey under a diagonal current",
                   "4 drones; constant diagonal current 0.30 m/s drift (inside the envelope).",
                   "same controller as the triangle, only the template changes")


def formation_six(cfg: FleetConfig = DEFAULT) -> Scenario:
    cur = CurrentField([CurrentComponent("uniform", drift=(0.0, -0.20, 0.0)),
                        CurrentComponent("sinusoid", drift=(0.0, 0.0, 0.0), amplitude=(0.0, 0.0, 0.08), period=20.0)])
    return _survey("formation_six", "line6", 32.0, cur, 50.0,
                   "Six-drone survey line under a current with a vertical component",
                   "6 drones abreast (21 m strip); lateral current 0.20 m/s plus an oscillating vertical drift "
                   "of 0.08 m/s (period 20 s).",
                   "36 sonars; lateral spacing from sonar ranges, depth holding against the vertical current",
                   camera=2)


def formation_recovery_head_current(cfg: FleetConfig = DEFAULT) -> Scenario:
    """P3 under a 0.6 m/s current against the motion: the old scalar bound, which this scenario showed to be
    too general (DI-24, DI-27).  0.6 m/s is the largest current exercised (Envelope.current_validated_max); against
    the motion at the 0.30 m/s survey speed the control-feasible limit is about 0.41 m/s, so the hit drone is
    outside the envelope and declares it (ENVELOPE_VIOLATION).

    Choice of the disturbance (not tuned on the outcome):
    * direction: against the survey direction, the one with the smallest control margin (the survey
      speed adds to the current; a lateral 0.6 m/s jet is rejected with a formation error below 0.5 m);
    * intensity: the old claimed limit itself, 0.6 m/s; no background current, so the drift stays <= 0.6;
    * shape: a localized jet (Gaussian, radius 3 m, as formation_gust), so that it deforms the geometry
      (a uniform current only translates the formation);
    * target: centred on the track of the rear-left drone when it switches on, the front-left drone being
      3.5 m ahead (jet factor 0.07): a front drone pushed back into its rear neighbour would make the
      warning filter move both drones (P1 interplay), the rear drone tests formation keeping alone;
    * timing: on at t = 12 s for 12 s (about five time constants of the low-level integrator, 2.3 s),
      1.5 s sin^2 ramps as in formation_gust."""
    jet = CurrentComponent("jet", drift=(-cfg.env.current_validated_max, 0.0, 0.0), center=(-14.75, -32.25), radius=3.0,
                           t_on=12.0, t_off=24.0, ramp=1.5)
    return _survey("formation_recovery_head_current", "square", 34.0, CurrentField([jet]), 50.0,
                   "P3: a head current at the claimed drift limit hits one drone",
                   "4 drones (square); a 0.6 m/s jet against the motion, the claimed drift limit, acts on the "
                   "rear-left drone between t = 12 s and 24 s. No background current.",
                   "the hit drone cannot hold its slot at the survey speed; formation lost / recovered from the "
                   "true error; the onboard saturation monitor of the hit drone is part of the picture",
                   windows=[(12.0, 24.0)], camera=2)


def formation_gust(cfg: FleetConfig = DEFAULT) -> Scenario:
    """A localized jet (0.85 m/s, beyond the 0.6 m/s envelope) hits only the left lane of the square for
    10 s: unlike a uniform current it deforms the relative geometry, so the formation is lost."""
    jet = CurrentComponent("jet", drift=(0.0, 0.85, 0.0), center=(-9.5, -31.0), radius=3.0, t_on=12.0, t_off=22.0,
                           ramp=1.5)
    cur = CurrentField([CurrentComponent("uniform", drift=(0.0, 0.10, 0.0)), jet])
    return _survey("formation_gust", "square", 34.0, cur, 55.0,
                   "Gust: formation lost and recovered (P3)",
                   "4 drones, 0.10 m/s background current; a 0.85 m/s jet (beyond the 0.6 m/s envelope) hits the "
                   "left lane between t = 12 s and 22 s and pushes it away from the right lane.",
                   "formation_lost while the jet acts, eventual recovery after it; recovery time is measured",
                   windows=[(12.0, 22.0)])


# ---------------------------------------------------------------------------------------------- lost drones (DI-29)
def _lost_drone(name: str, t_on: float, t_off: Optional[float], duration: float, current: CurrentField, title: str,
                desc: str, focus: str, cfg: FleetConfig, faulty: int = 2) -> Scenario:
    """Square survey; drone ``faulty`` (rear-left: nobody runs into it when it stops) loses its thrusters
    (simulator fault, invisible to every controller) at t_on, until t_off (None: for good)."""
    sc = _survey(name, "square", 34.0, current, duration, title, desc, focus, windows=[(t_on, t_off or duration)],
                 cfg=cfg, camera=faulty)
    sc.sim.faults = ({"drone": f"drone_{faulty}", "type": "THRUSTER_FAILURE", "t_on": t_on, "t_off": t_off},)
    sc.sim.chase_offset, sc.sim.side_offset = (-7.0, 0.0, 3.0), (2.0, 10.0, 2.0)
    return sc


def lost_drone_rejoin(cfg: FleetConfig = DEFAULT) -> Scenario:
    return _lost_drone("lost_drone_rejoin", 8.0, 15.0, 60.0, _uniform((0.0, 0.35, 0.0)),
                       "Lost drone: temporary thruster failure, wait and rejoin from behind",
                       "4 drones (square), 0.35 m/s lateral current (inside the envelope). The rear-left drone loses "
                       "its thrusters between t = 8 s and 15 s (simulator fault, no controller is told) and is dragged "
                       "off its lane and behind the fleet.",
                       "the faulty drone detects it onboard (requested velocity not delivered -> FAILSAFE); its "
                       "neighbours miss it and wait on their slots (FORMATION_WAIT_REJOIN, timer < t_rejoin); it "
                       "rejoins from behind along its own lane; formation re-established, no communication", cfg)


def lost_drone_timeout(cfg: FleetConfig = DEFAULT) -> Scenario:
    return _lost_drone("lost_drone_timeout", 8.0, None, 64.0, CurrentField(),
                       "Lost drone: permanent failure, timeout and degraded formation",
                       "4 drones (square), no current. The rear-left drone loses its thrusters for good at t = 8 s "
                       "(simulator fault, no controller is told).",
                       "the neighbours miss it (FORMATION_WAIT_REJOIN), after t_rejoin = 37.5 s they declare its slot "
                       "vacant (DEGRADED_FORMATION) and keep their original slots: no reconfiguration, the hole stays", cfg)


# ---------------------------------------------------------------------------------------------- P2
def _gate_mission(name: str, template_name: str, duration: float, current: CurrentField, cfg: FleetConfig,
                  windows=(), title: str = "", desc: str = "", focus: str = "", lead_in: float = 0.0,
                  v_clock: Optional[float] = None, stress: StressSpec = StressSpec()) -> Scenario:
    """Survey line through arena gate G06 (the only gate spawned).  The formation reference follows the
    line, is re-established at the rendezvous ``rally_s`` beyond the gate as soon as every drone is in
    the approach zone (FormationClock jump), waits there while the drones pass one at a time and then
    resumes the survey.  ``lead_in`` metres of survey precede the approach zone."""
    g6 = {g.gate_id: g for g in load_arena_gates(HORSESHOE_TRACK)}["G06"]
    G = cfg.gate
    tm = TEMPLATES[template_name]
    q_s = queue_line(tm.slots, G)
    s_gate = 40.0                                         # path arclength of the gate centre
    path = Path(np.array([g6.center[:2] - s_gate * g6.axis[:2], g6.center[:2] + 30.0 * g6.axis[:2]]),
                float(g6.center[2]))
    max_al, min_al = max(sl.along for sl in tm.slots), min(sl.along for sl in tm.slots)
    s0 = s_gate + q_s - 0.4 - max_al - lead_in          # leading slot 0.4 m behind the queue line (+ lead-in)
    s_jump = max(s0, s_gate + q_s - G.approach_len + 0.3 - min_al)
    s_rally = s_gate + G.rally_s
    dwell = 26.0 * tm.n + 12.0                          # planned time budget for n one-at-a-time passages
    clock = FormationClock(s0, v_clock or cfg.form.v_nominal, 0.0, holds=((s_rally, dwell),),
                           jumps=((s_jump, s_rally),), s_end=path.length - 3.0)
    plans, spawns, yaws = _fleet(name, tm, path, clock, [g6], [g6], cfg, s_start=s0)
    sim = _sim(name, plans, spawns, yaws, duration, current=current, gate_ids=("G06",), camera=tm.n - 1,
               stress=stress)
    sim.chase_offset, sim.side_offset = (-7.0, 0.0, 2.6), (2.0, 11.0, 1.5)
    return Scenario(name, title, desc, sim, plans, tm, path, [g6], True, list(windows), focus)


def gate_single(cfg: FleetConfig = DEFAULT) -> Scenario:
    return _gate_mission("gate_single", "triangle", 80.0, _uniform((0.0, -0.05, 0.0)), cfg,
                         title="P2: one at a time through arena gate G06",
                         desc="3 drones in triangle reach Marine Race Arena gate G06 (only this gate spawned, 1.5 x 1.5 m "
                              "opening), queue abreast, pass one at a time (left first, sector rule), re-form beyond "
                              "the gate. No communication.",
                         focus="queue relations from sector patterns, CR occupancy belief, occupancy <= 1, entry order")


def integrated_short(cfg: FleetConfig = DEFAULT) -> Scenario:
    g6 = {g.gate_id: g for g in load_arena_gates(HORSESHOE_TRACK)}["G06"]
    left = np.array([-g6.axis[1], g6.axis[0]])
    gust = CurrentComponent("uniform", drift=(round(0.35 * float(left[0]), 3), round(0.35 * float(left[1]), 3), 0.0),
                            t_on=5.0, t_off=45.0, ramp=4.0)
    cur = CurrentField([CurrentComponent("uniform", drift=(0.0, -0.05, 0.0)), gust])
    return _gate_mission("integrated_short", "triangle", 92.0, cur, cfg, windows=[(5.0, 45.0)], lead_in=4.0,
                         v_clock=0.40,
                         title="Integrated: survey, cross-current at the gate, one at a time, re-form",
                         desc="3 drones (triangle) survey towards gate G06; a 0.35 m/s cross-current builds up while "
                              "they approach and queue (onboard current estimate, no ground truth); they pass the "
                              "1.5 m opening one at a time and re-form beyond it. No communication.",
                         focus="P1 throughout, current estimate, P2 at the gate under cross-current, P3 re-formation")


def mutex_deadlock_resolution(cfg: FleetConfig = DEFAULT) -> Scenario:
    return _gate_mission("mutex_deadlock_resolution", "stack_pair", 75.0, CurrentField(), cfg,
                         title="Mutex: stacked queue, left first then top first",
                         desc="3 drones - an upper and a lower drone on the left lane, one on the right lane - reach gate "
                              "G06 and queue with the left pair stacked. The stacked pair sees each other only in UP/DOWN: "
                              "the v2 rule resolved that with a static rank it could not apply (both waited, and the right "
                              "drone waited for them: deadlock); now the upper one goes first. No communication.",
                         focus="queue order left-top, left-bottom, right from sector patterns; static rank never used; "
                               "occupancy <= 1")


def line_parallel_mutex(cfg: FleetConfig = DEFAULT) -> Scenario:
    return _gate_mission("line_parallel_mutex", "line3", 75.0, CurrentField(), cfg,
                         title="Mutex: a line abreast reaches the gate side by side",
                         desc="3 drones abreast arrive together at gate G06: perfect parity. The tie is broken by the "
                              "sector rule alone - left first - without any static rank. No communication.",
                         focus="simultaneous arrival, abreast queue, left-first order, CR occupancy <= 1")


def lost_drone_mutex(cfg: FleetConfig = DEFAULT) -> Scenario:
    """The lead-in (9 m of survey) leaves the stopped drone more than a queue bracket behind the queue line:
    a drone that stops for good INSIDE the bracket of a queued drone, on its left, would block it (M5)."""
    sc = _gate_mission("lost_drone_mutex", "line3", 72.0, CurrentField(), cfg, lead_in=9.0,
                       title="Lost drone before the gate: the leftmost drone present goes first",
                       desc="3 drones abreast survey towards gate G06; the left drone loses its thrusters for good at "
                            "t = 1 s and stays behind. The other two miss it, queue at their own queue points (the left "
                            "one stays vacant) and pass one at a time: the middle drone, now the leftmost present, goes "
                            "first. No communication.",
                       focus="missing neighbour (FORMATION_WAIT_REJOIN; timer frozen in the mutex modes and during the "
                             "rendezvous hold), queue order relative to the drones present, occupancy <= 1")
    sc.sim.faults = ({"drone": "drone_0", "type": "THRUSTER_FAILURE", "t_on": 1.0, "t_off": None},)
    sc.disturbance_windows = [(1.0, sc.duration_s)]
    return sc


SCENARIOS: Dict[str, Callable[..., Scenario]] = {
    "p1_head_on": p1_head_on,
    "p1_vertical_escape": p1_vertical_escape,
    "p1_two_lines": p1_two_lines,
    "p1_close_encounter": p1_close_encounter,
    "formation_triangle": formation_triangle,
    "formation_square": formation_square,
    "formation_six": formation_six,
    "formation_recovery_head_current": formation_recovery_head_current,
    "formation_gust": formation_gust,
    "gate_single": gate_single,
    "integrated_short": integrated_short,
    "lost_drone_rejoin": lost_drone_rejoin,
    "lost_drone_timeout": lost_drone_timeout,
    "mutex_deadlock_resolution": mutex_deadlock_resolution,
    "line_parallel_mutex": line_parallel_mutex,
    "lost_drone_mutex": lost_drone_mutex,
}
