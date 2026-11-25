"""Measure braking and active position hold with the deployed HoloOcean velocity loop.

This finite-window pilot uses the CLEAN navigation sensors without sonar aiding.
Ground-truth displacement is measured at the PoseSensor's IMU socket, at control
samples. The constant DVL bias is a diagnostic injection. The excursions and
sample acceleration extrema are observations, not guaranteed dynamics bounds.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.arena_bridge import thruster_command
from holo_fleet.config import DEFAULT
from holo_fleet.control.gate_safety import HOLD_V_MAX, K_HOLD
from holo_fleet.control.lowlevel import LowLevelController
from holo_fleet.perception.nav import DeadReckoning
from holo_fleet.perception.sensor_suite import SONAR_NAMES, onboard_sensor_configs
from holo_fleet.sim.currents import drift_to_command
from holo_fleet.sim.engine_session import make
from scripts.run_sensor_robustness_campaign import be_nice

TPS = 30
DT = DEFAULT.env.dt
STOP_S = 8
END_S = 48


def summarize(rows, axis):
    start = next(i for i, row in enumerate(rows) if row["t"] == STOP_S)
    # The recorded sample is at the beginning of the first hold command.
    after = rows[start:]
    displacement = np.array([r["p"] for r in after]) - after[0]["p"]
    after_10_s = np.array([row["t"] >= STOP_S + 10 for row in after])
    velocity = np.array([row["velocity"] for row in rows])
    return {
        "velocity_at_switch_m_s": after[0]["velocity"],
        "max_abs_excursion_from_switch_m": np.max(abs(displacement), axis=0).round(5).tolist(),
        "end_excursion_from_switch_m": displacement[-1].round(5).tolist(),
        "max_abs_excursion_over_last_30_s_m": np.max(abs(displacement[after_10_s]), axis=0).round(5).tolist(),
        "max_abs_sample_acceleration_m_s2": np.max(abs(np.diff(velocity, axis=0) / DT), axis=0).round(5).tolist(),
        "along_axis": axis,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    be_nice()
    cases = {}
    for axis, edge_drift in enumerate((DEFAULT.env.current_validated_max, DEFAULT.env.current_validated_max,
                                     DEFAULT.env.current_vertical_max)):
        for sign in (1, -1):
            for drift in (0.0, edge_drift):
                name = f"axis{axis}_{'pos' if sign == 1 else 'neg'}_{'current' if drift else 'calm'}"
                cases[name] = (axis, sign, drift, 0.0)
    cases["axis0_pos_dvl_bias"] = (0, 1, 0.0, 0.02)
    sensors = [s for s in onboard_sensor_configs() if s["sensor_name"] not in SONAR_NAMES]
    sensors += [{"sensor_type": name, "sensor_name": name, "socket": "IMUSocket", "Hz": TPS}
                for name in ("PoseSensor", "VelocitySensor")]
    agents = [{"agent_name": n, "agent_type": "BlueROV2", "location": [-24 + 16 * (k % 4), -24 + 16 * (k // 4), -10],
               "rotation": [0, 0, 0], "control_scheme": 0, "sensors": sensors} for k, n in enumerate(cases)]
    scene = {"name": "p1_stop_hold", "world": "OpenWater", "package_name": "Ocean", "main_agent": next(iter(cases)),
             "ticks_per_sec": TPS, "frames_per_sec": False, "agents": agents}
    nav = {a["agent_name"]: DeadReckoning(a["location"], 0.0) for a in agents}
    low = {n: LowLevelController(DEFAULT) for n in cases}
    anchors = {}
    latest = {n: {} for n in cases}
    stamps = {n: {} for n in cases}
    trace = {n: [] for n in cases}
    with make(scenario_cfg=scene, show_viewport=False, ticks_per_sec=TPS, frames_per_sec=False) as env:
        ports = env._holo_engine["listen_ports"]
        assert ports and 1985 not in ports
        print(f"Isolated engine: {ports}", flush=True)
        env.reset()
        ticks = round(DT * TPS)
        for tick in range(1, TPS * END_S + 1):
            raw = env.tick()
            for name in cases:
                for sensor, value in raw.get(name, {}).items():
                    latest[name][sensor] = value
                    stamps[name][sensor] = tick / TPS
            if tick % ticks:
                continue
            t = round(tick / TPS, 6)
            for name, (axis, direction, drift, bias) in cases.items():
                readings = latest[name]
                dvl = np.array(readings["DVLSensor"], dtype=float).copy()
                dvl.flat[0] += bias
                imu = readings["IMUSensor"]
                state = nav[name].update(t, DT, readings["Compass"], dvl, readings["DepthSensor"], imu, stamps[name])
                unit = np.eye(3)[axis] * direction
                table = DEFAULT.plant.current_cmd_to_drift_z if axis == 2 else DEFAULT.plant.current_cmd_to_drift
                env.set_ocean_currents(name, (unit * drift_to_command(drift, table)).tolist())
                if t < STOP_S:
                    request = unit * (0.25 if axis == 2 else 0.45)
                    if axis != 2:
                        request[2] = np.clip(0.5 * (-10 - state.p[2]), -HOLD_V_MAX, HOLD_V_MAX)
                else:
                    anchor = anchors.setdefault(name, state.p.copy())
                    request = np.clip(K_HOLD * (anchor - state.p), -HOLD_V_MAX, HOLD_V_MAX)
                command = low[name].command(state, request, 0.0, "brake", DT)
                thrust = thruster_command(**command, limit=DEFAULT.plant.thruster_limit)
                env.act(name, thrust)
                trace[name].append({
                    "t": t, "p": readings["PoseSensor"][:3, 3].tolist(),
                    "velocity": np.asarray(readings["VelocitySensor"]).reshape(-1)[:3].tolist(),
                    "nav_p": state.p.tolist(), "dvl_velocity": dvl.reshape(-1)[:3].tolist(),
                    "dvl_t": stamps[name]["DVLSensor"], "imu": np.asarray(imu).tolist(),
                    "imu_t": stamps[name]["IMUSensor"], "actuator": thrust.tolist(), "request": request.tolist(),
                })
            if t % 8 == 0:
                print(f"{t:g}/{END_S} s", flush=True)
    summary = {"dt_s": DT, "switch_s": STOP_S, "hold_duration_s": END_S - STOP_S,
               "hold_gain": K_HOLD, "hold_velocity_cap_m_s": HOLD_V_MAX, "profile": "CLEAN, one run per case",
               "cases": {n: {**summarize(trace[n], axis), "initial_direction": direction,
                              "current_effective_drift_m_s": drift, "dvl_surge_bias_m_s": bias}
                         for n, (axis, direction, drift, bias) in cases.items()}}
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    (args.out / "trace.json").write_text(json.dumps({"non_default_listeners": ports, "cases": trace}), encoding="utf-8")


if __name__ == "__main__":
    main()
