"""Short demonstration scenarios (v2): one seed each, 20-90 s, built to show one behaviour each.

  sonar_classification   (bench, see scripts/run_demo.py)    pinned cases A-F of the echo classifier
  p1_head_on             2 drones, no current                 head-on encounter, right-hand passing
  p1_vertical_escape     4 drones + 1 scripted intruder       T formation crossed head-on by a vehicle
                                                              that does not react; traffic rule off: the
                                                              boxed-in drone escapes upwards (safety layer)
  p1_two_lines           6 drones, no current                 two survey lines meet head-on; flanked
                                                              drones give way vertically (traffic rule)
  formation_triangle     3 drones, lateral current 0.25       triangle survey under a constant current
  formation_square       4 drones, diagonal current 0.30      2 x 2 box under a diagonal current
  formation_six          6 drones, lateral + vertical current survey line (6 swaths), current with w_z
  formation_gust         4 drones, temporary 0.7 m/s gust     formation lost and recovered (P3)
  gate_single            3 drones, one arena gate (G06)       one at a time through the gate (P2)
  integrated_short       3 drones, cross-current + G06        P1 + P2 + P3 in one short mission
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates
from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.formations import TEMPLATES, FormationTemplate
from holo_fleet.mission import FormationClock, GateSpec, MissionPlan, Path, Slot
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


def _queue_laterals(template: FormationTemplate, cfg: FleetConfig) -> List[float]:
    """Abreast queue points assigned in the slots' lateral order (left first): geometry, not IDs."""
    order = sorted(range(template.n), key=lambda k: (-template.slots[k].lateral, -template.slots[k].along))
    lats = cfg.gate.queue_laterals(template.n)
    out = [0.0] * template.n
    for r, k in enumerate(order):
        out[k] = lats[r]
    return out


def _fleet(name: str, template: FormationTemplate, path: Path, clock: FormationClock, gates: Sequence[GateSpec],
           structures: Sequence[GateSpec], cfg: FleetConfig, s_start: float = 0.0, formation: bool = True):
    plans, spawns, yaws = [], [], []
    qlat = _queue_laterals(template, cfg)
    q_s = cfg.gate.queue_s(template.n)
    for k, sl in enumerate(template.slots):
        p, d, n = path.frame_at(s_start + sl.along)
        pos = np.array([p[0] + sl.lateral * n[0], p[1] + sl.lateral * n[1], path.depth_z + sl.dz])
        yaw = math.degrees(math.atan2(d[1], d[0]))
        plans.append(MissionPlan(drone_id=f"drone_{k}", slot_index=k, slots=list(template.slots), path=path,
                                 gates=list(gates), structures=list(structures), launch_position=pos,
                                 launch_yaw_deg=yaw, static_rank=k, s_end=clock.s_end,
                                 failsafe_layer_dz=FAILSAFE_LAYERS[k % len(FAILSAFE_LAYERS)],
                                 formation_enabled=formation, mission_name=name, template_name=template.name,
                                 queue_lateral=qlat[k], queue_s=q_s, clock=clock))
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


def p1_two_lines(cfg: FleetConfig = DEFAULT) -> Scenario:
    """Two survey lines of three drones (3.5 m apart) meet head-on: the middle drones have a
    neighbour on each side, so the traffic rule shifts them vertically instead of to the right."""
    line3 = FormationTemplate("line3", (Slot(0.0, 3.5, 0.0), Slot(0.0, 0.0, 0.0), Slot(0.0, -3.5, 0.0)),
                              "3 swaths 3.5 m apart", 3.6)
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
    q_s = G.queue_s(tm.n)
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


SCENARIOS: Dict[str, Callable[..., Scenario]] = {
    "p1_head_on": p1_head_on,
    "p1_vertical_escape": p1_vertical_escape,
    "p1_two_lines": p1_two_lines,
    "formation_triangle": formation_triangle,
    "formation_square": formation_square,
    "formation_six": formation_six,
    "formation_gust": formation_gust,
    "gate_single": gate_single,
    "integrated_short": integrated_short,
}
