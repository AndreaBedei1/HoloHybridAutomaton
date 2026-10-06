"""Scenario catalogue: mission plans (for the drones) + simulation-only settings.

``ScenarioSpec.plans`` is what each drone is given before launch.  Everything
else in the spec (currents, sensor stress, launch delays, true spawn poses) is
known only to the simulator and the referee.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates
from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.mission import MissionPlan, Path, Slot
from holo_fleet.sim.currents import CurrentComponent, CurrentField

TRIANGLE = [Slot(0.0, +1.75), Slot(0.0, -1.75), Slot(-2.5, 0.0)]
FAILSAFE_LAYERS = [0.0, -1.3, +1.3]

CURRENT_LEVELS = {"none": 0.0, "low": 0.10, "medium": 0.25, "high": 0.40}


@dataclass
class StressSpec:
    beam_dropout: float = 0.0
    blackout_every_s: float = 0.0            # mean interval between proximity-sonar blackouts (0 = off)
    blackout_len_s: float = 0.8
    disable_fls: List[int] = field(default_factory=list)
    nav_init_error_m: float = 0.0


@dataclass
class ScenarioSpec:
    name: str
    description: str
    plans: List[MissionPlan]
    spawn_positions: List[np.ndarray]
    spawn_yaw_deg: List[float]
    release_s: List[float]
    current: CurrentField
    duration_s: float
    use_arena_gates: bool
    mission_gate_ids: List[str]
    stress: StressSpec = field(default_factory=StressSpec)
    comms_enabled: bool = False
    seed: int = 0
    params: Dict = field(default_factory=dict)

    @property
    def n(self) -> int:
        return len(self.plans)


def _plans_for_path(path: Path, slots, gates, structures, s_end, name, formation=True) -> List[MissionPlan]:
    plans = []
    for k, slot in enumerate(slots):
        p, tan, nrm = path.frame_at(slot.along)
        pos = np.array([p[0] + slot.lateral * nrm[0], p[1] + slot.lateral * nrm[1], path.depth_z + slot.dz])
        plans.append(MissionPlan(
            drone_id=f"drone_{k}", slot_index=k, slots=list(slots), path=path, gates=list(gates),
            structures=list(structures), launch_position=pos, launch_yaw_deg=math.degrees(math.atan2(tan[1], tan[0])),
            static_rank=k, s_end=s_end, failsafe_layer_dz=FAILSAFE_LAYERS[k % len(FAILSAFE_LAYERS)],
            formation_enabled=formation, mission_name=name))
    return plans


def pair_crossing(seed: int = 0, cfg: FleetConfig = DEFAULT, **_) -> ScenarioSpec:
    """Two drones on the same line, head-on: the minimal separation test."""
    structures = load_arena_gates(HORSESHOE_TRACK)
    z = -4.5
    path0 = Path(np.array([[-10.0, -4.0], [12.0, -4.0]]), z)
    path1 = Path(np.array([[10.0, -4.0], [-12.0, -4.0]]), z)
    plans = []
    for k, path in enumerate((path0, path1)):
        p, tan, _ = path.frame_at(0.0)
        plans.append(MissionPlan(drone_id=f"drone_{k}", slot_index=0, slots=[Slot(0.0, 0.0)], path=path, gates=[],
                                 structures=structures, launch_position=np.array([p[0], p[1], z]),
                                 launch_yaw_deg=math.degrees(math.atan2(tan[1], tan[0])), static_rank=k,
                                 s_end=20.0, failsafe_layer_dz=FAILSAFE_LAYERS[k], formation_enabled=False,
                                 mission_name="pair_crossing"))
    return ScenarioSpec(
        name="pair_crossing", description="2 drones head-on on the same survey line (P1 sanity check)",
        plans=plans, spawn_positions=[pl.launch_position.copy() for pl in plans],
        spawn_yaw_deg=[pl.launch_yaw_deg for pl in plans], release_s=[0.0, 0.0], current=CurrentField([]),
        duration_s=90.0, use_arena_gates=True, mission_gate_ids=[], seed=seed)


def formation_current(seed: int = 0, current: str = "medium", jet: bool = True, n_drones: int = 3,
                      cfg: FleetConfig = DEFAULT, **_) -> ScenarioSpec:
    """Scenario A: triangle survey in the open interior of the Horseshoe Bay arena, lateral current."""
    structures = load_arena_gates(HORSESHOE_TRACK)
    path = Path(np.array([[-14.0, -3.0], [16.0, -3.0]]), -4.5)
    slots = TRIANGLE[:n_drones]
    plans = _plans_for_path(path, slots, [], structures, s_end=26.0, name="formation_current")
    w = CURRENT_LEVELS[current]
    comps = [CurrentComponent("uniform", drift=(0.0, w, 0.0))] if w > 0 else []
    if jet:
        # time-windowed localized gust on the right-hand lane, pushing drone_1 into its neighbours.
        # Deliberately beyond the nominal authority (and flagged by the referee if beyond the
        # claimed envelope) to force separation reactions and a formation-loss episode.
        comps.append(CurrentComponent("jet", drift=(0.0, 0.80, 0.0), center=(-2.5, -5.0), radius=4.0,
                                      t_on=35.0, t_off=50.0, ramp=3.0))
    return ScenarioSpec(
        name=f"formation_current_{current}" + ("_gust" if jet else ""),
        description=f"Scenario A: triangle survey, uniform lateral drift {w} m/s"
                    + (", plus a 0.80 m/s lateral gust on the right lane for t in [35, 50] s" if jet else ""),
        plans=plans, spawn_positions=[pl.launch_position.copy() for pl in plans],
        spawn_yaw_deg=[pl.launch_yaw_deg for pl in plans], release_s=[0.0] * n_drones,
        current=CurrentField(comps), duration_s=125.0, use_arena_gates=True, mission_gate_ids=[], seed=seed,
        params={"current_level": current, "uniform_drift": w, "jet": jet})


def _gate_path(gates_by_id) -> Path:
    g6, g7 = gates_by_id["G06"], gates_by_id["G07"]
    a6, a7 = g6.axis[:2], g7.axis[:2]
    e6 = g6.center[:2] - 3.0 * a6
    x6 = g6.center[:2] + 2.0 * a6
    e7 = g7.center[:2] - 2.0 * a7
    x7 = g7.center[:2] + 3.0 * a7
    h0 = math.radians(-25.0)
    w0 = e6 - 14.0 * np.array([math.cos(h0), math.sin(h0)])
    h1 = math.radians(35.0)
    w_end = x7 + 17.0 * np.array([math.cos(h1), math.sin(h1)])
    depth = float((g6.center[2] + g7.center[2]) / 2.0)
    return Path(np.array([w0, e6, g6.center[:2], x6, e7, g7.center[:2], x7, w_end]), depth)


def gate_arena(seed: int = 0, current: str = "low", n_drones: int = 3, cfg: FleetConfig = DEFAULT,
               stress: Optional[StressSpec] = None, release=None, extra_current: Optional[List[CurrentComponent]] = None,
               name: str = "gate_arena", comms: bool = False, **_) -> ScenarioSpec:
    """Scenario B: survey line through gates G06 and G07 of the existing Horseshoe Bay arena."""
    structures = load_arena_gates(HORSESHOE_TRACK)
    by_id = {g.gate_id: g for g in structures}
    path = _gate_path(by_id)
    gates = [by_id["G06"], by_id["G07"]]
    slots = TRIANGLE[:n_drones]
    plans = _plans_for_path(path, slots, gates, structures, s_end=path.length - 3.0, name=name)
    w = CURRENT_LEVELS[current]
    comps = [CurrentComponent("uniform", drift=(0.0, -w, 0.0))] if w > 0 else []
    comps += list(extra_current or [])
    return ScenarioSpec(
        name=name, description=f"Scenario B: triangle survey through arena gates G06/G07, drift {w} m/s",
        plans=plans, spawn_positions=[pl.launch_position.copy() for pl in plans],
        spawn_yaw_deg=[pl.launch_yaw_deg for pl in plans],
        release_s=list(release) if release is not None else [0.0] * n_drones,
        current=CurrentField(comps), duration_s=330.0, use_arena_gates=True, mission_gate_ids=["G06", "G07"],
        stress=stress or StressSpec(), comms_enabled=comms, seed=seed,
        params={"current_level": current, "uniform_drift": w})


def stress(seed: int = 0, n_drones: int = 3, cfg: FleetConfig = DEFAULT, **_) -> ScenarioSpec:
    """Scenario C: gate arena with variable current, asynchronous launch, sonar dropout/blackouts."""
    extra = [
        CurrentComponent("sinusoid", drift=(0.0, -0.15, 0.0), amplitude=(0.05, 0.20, 0.0), period=40.0),
        CurrentComponent("gusts", amplitude=(0.12, 0.12, 0.0), seed=seed),
        CurrentComponent("jet", drift=(0.0, 0.6, 0.0), center=(-6.0, 14.0), radius=3.5,
                         t_on=60.0, t_off=95.0, ramp=4.0),
    ]
    st = StressSpec(beam_dropout=0.25, blackout_every_s=25.0, blackout_len_s=0.8, disable_fls=[2],
                    nav_init_error_m=0.15)
    spec = gate_arena(seed=seed, current="none", n_drones=n_drones, stress=st, release=[0.0, 7.0, 14.0][:n_drones],
                      extra_current=extra, name="stress_gate")
    spec.description = ("Scenario C: gate arena, variable current (sinusoid+gusts+jet), launch delays 0/7/14 s, "
                        "25% beam dropout, 0.8 s sonar blackouts, FLS off on drone_2, 0.15 m nav init error")
    spec.duration_s = 380.0
    return spec


SCENARIOS = {
    "pair_crossing": pair_crossing,
    "formation_current": formation_current,
    "gate_arena": gate_arena,
    "stress": stress,
}
