"""Probe: what do HoloOcean 2.3's native noise parameters actually do to our sensors?

    python probe/probe_sensor_noise.py [--seconds 40]

One HoloOcean session, ten pinned "probe" BlueROV2s, each with a different noise setting, each with two
pinned target hulls: one 3.0 m in front (FRONT sonar) and one 6.0 m behind (REAR sonar).  The pairs are
25 m apart, beyond the 12 m sonar range.  Every probe also carries DVL, IMU (bias returned), compass and
depth with its own sigma.  Pinned = zero true velocity and angular rate, so DVL/gyro outputs are pure
error; heading and depth errors are measured against the pinned pose.

Sonar statistics use the onboard segmentation (perception.sonar_processing.segment, threshold 0.30):
* range error = first bin above threshold - exact distance from the sonar to the target's near face;
* recall = fraction of captures with an echo within +-0.5 m of that face;
* false alarm = fraction of captures with any segment nearer than (face - 0.5 m) (open water there);
* open-water intensity samples give the effective Rayleigh scale of AddSigma.

Writes results/robustness/probe/sensor_noise_probe.json.  Nothing here feeds a controller.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.perception.sonar_geometry import DEFAULT_SONAR, HULL_HALF, MOUNTS  # noqa: E402
from holo_fleet.perception.sonar_processing import segment  # noqa: E402
from holo_fleet.sim import holoocean_setup as hs  # noqa: E402

OUT = ROOT / "results" / "robustness" / "probe"
DEPTH = -5.0
SPACING = 25.0
D_FRONT, D_REAR = 3.0, 6.0

# one setting per probe: sonar (add, mult, range) + nav sensors.  Index 0 = the deployed CLEAN suite.
SETTINGS = [
    {"label": "clean", "sonar": (0.05, 0.10, 0.05), "dvl_vel": 0.01, "dvl_rng": 0.02, "gyro": 0.005, "accel": 0.01,
     "gyro_bias": 0.0, "accel_bias": 0.0, "mag": 0.005, "depth": 0.02},
    {"label": "add0.07", "sonar": (0.07, 0.10, 0.05), "dvl_vel": 0.02, "dvl_rng": 0.05, "gyro": 0.01, "accel": 0.02,
     "gyro_bias": 0.0005, "accel_bias": 0.001, "mag": 0.01, "depth": 0.05},
    {"label": "add0.08", "sonar": (0.08, 0.10, 0.05), "dvl_vel": 0.03, "dvl_rng": 0.10, "gyro": 0.02, "accel": 0.05,
     "gyro_bias": 0.001, "accel_bias": 0.002, "mag": 0.02, "depth": 0.10},
    {"label": "add0.09", "sonar": (0.09, 0.10, 0.05), "dvl_vel": 0.05, "dvl_rng": 0.20, "gyro": 0.05, "accel": 0.10,
     "gyro_bias": 0.002, "accel_bias": 0.005, "mag": 0.035, "depth": 0.20},
    {"label": "add0.11", "sonar": (0.11, 0.10, 0.05), "dvl_vel": 0.08, "dvl_rng": 0.30, "gyro": 0.10, "accel": 0.20,
     "gyro_bias": 0.005, "accel_bias": 0.01, "mag": 0.05, "depth": 0.30},
    {"label": "range0.10", "sonar": (0.05, 0.10, 0.10), "dvl_vel": 0.10, "dvl_rng": 0.50, "gyro": 0.2, "accel": 0.3,
     "gyro_bias": 0.01, "accel_bias": 0.02, "mag": 0.07, "depth": 0.50},
    {"label": "range0.20", "sonar": (0.05, 0.10, 0.20), "dvl_vel": 0.15, "dvl_rng": 1.0, "gyro": 0.005, "accel": 0.01,
     "gyro_bias": 0.0, "accel_bias": 0.0, "mag": 0.10, "depth": 0.02},
    {"label": "range0.40", "sonar": (0.05, 0.10, 0.40), "dvl_vel": 0.01, "dvl_rng": 0.02, "gyro": 0.005, "accel": 0.01,
     "gyro_bias": 0.0, "accel_bias": 0.0, "mag": 0.15, "depth": 0.02},
    {"label": "mult0.30", "sonar": (0.05, 0.30, 0.05), "dvl_vel": 0.01, "dvl_rng": 0.02, "gyro": 0.005, "accel": 0.01,
     "gyro_bias": 0.0, "accel_bias": 0.0, "mag": 0.2, "depth": 0.02},
    {"label": "mult0.60", "sonar": (0.05, 0.60, 0.05), "dvl_vel": 0.01, "dvl_rng": 0.02, "gyro": 0.005, "accel": 0.01,
     "gyro_bias": 0.0, "accel_bias": 0.0, "mag": 0.3, "depth": 0.02},
]


# exact distance from each sonar to the target's near face (hulls aligned, pinned): centre distance - mount - half length
FACE = {"front": D_FRONT - MOUNTS["FRONT"][0] - HULL_HALF[0], "rear": D_REAR + MOUNTS["REAR"][0] - HULL_HALF[0]}


def sonar_cfg(name, mount, rot, s):
    m = DEFAULT_SONAR
    return {"sensor_type": "SinglebeamSonar", "sensor_name": name, "socket": "", "location": [float(v) for v in mount],
            "rotation": rot, "Hz": int(m.hz),
            "configuration": {"OpeningAngle": m.opening_deg, "RangeMin": m.range_min, "RangeMax": m.range_max,
                              "RangeBins": m.bins, "AddSigma": s[0], "MultSigma": s[1], "RangeSigma": s[2],
                              "ShowWarning": False, "InitOctreeRange": 15}}


def probe_agent(k, st):
    loc = [0.0, k * SPACING, DEPTH]
    sensors = [sonar_cfg("SonarFront", MOUNTS["FRONT"], [0.0, 0.0, 0.0], st["sonar"]),
               sonar_cfg("SonarRear", MOUNTS["REAR"], [0.0, 0.0, 180.0], st["sonar"]),
               {"sensor_type": "DVLSensor", "sensor_name": "DVLSensor", "socket": "DVLSocket", "Hz": 10,
                "configuration": {"Elevation": 22.5, "VelSigma": st["dvl_vel"], "ReturnRange": True, "MaxRange": 50,
                                  "RangeSigma": st["dvl_rng"]}},
               {"sensor_type": "IMUSensor", "sensor_name": "IMUSensor", "socket": "IMUSocket", "Hz": 30,
                "configuration": {"AccelSigma": st["accel"], "AngVelSigma": st["gyro"], "ReturnBias": True,
                                  **({"AccelBiasSigma": st["accel_bias"]} if st["accel_bias"] else {}),
                                  **({"AngVelBiasSigma": st["gyro_bias"]} if st["gyro_bias"] else {})}},
               {"sensor_type": "MagnetometerSensor", "sensor_name": "Compass", "socket": "IMUSocket", "Hz": 30,
                "configuration": {"Sigma": st["mag"]}},
               {"sensor_type": "DepthSensor", "sensor_name": "DepthSensor", "socket": "DepthSocket", "Hz": 30,
                "configuration": {"Sigma": st["depth"]}},
               {"sensor_type": "PoseSensor", "sensor_name": "PoseSensor", "socket": "IMUSocket", "Hz": 30}]
    return {"agent_name": f"probe_{k}", "agent_type": "BlueROV2", "location": loc, "rotation": [0.0, 0.0, 0.0],
            "control_scheme": 0, "sensors": sensors}


def target_agent(name, loc):
    return {"agent_name": name, "agent_type": "BlueROV2", "location": loc, "rotation": [0.0, 0.0, 0.0],
            "control_scheme": 0, "sensors": [{"sensor_type": "PoseSensor", "socket": "IMUSocket", "Hz": 30}]}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=40.0)
    args = ap.parse_args()
    hs.use_patched_holoocean()
    import holoocean

    hs.wait_for_engine_semaphores()
    agents, pins = [], {}
    for k, st in enumerate(SETTINGS):
        a = probe_agent(k, st)
        agents.append(a)
        pins[a["agent_name"]] = a["location"]
        for nm, dx in ((f"front_{k}", D_FRONT), (f"rear_{k}", -D_REAR)):
            loc = [dx, k * SPACING, DEPTH]
            agents.append(target_agent(nm, loc))
            pins[nm] = loc
    scen = {"name": "probe_sensor_noise", "world": "OpenWater", "package_name": "Ocean", "main_agent": "probe_0",
            "ticks_per_sec": 30, "frames_per_sec": False, "octree_min": hs.OCTREE_MIN, "octree_max": hs.OCTREE_MAX,
            "env_min": [-20.0, -20.0, -20.0], "env_max": [20.0, len(SETTINGS) * SPACING, 2.0], "agents": agents}
    env = holoocean.make(scenario_cfg=scen, show_viewport=False, ticks_per_sec=30, frames_per_sec=False)
    rec = {k: {"front": [], "rear": [], "front_peak": [], "rear_peak": [], "dvl": [], "gyro": [], "gyro_bias": [], "accel": [], "heading": [],
               "depth": [], "pose_z": [], "noise_int": []} for k in range(len(SETTINGS))}
    n_ticks = int(args.seconds * 30)
    t0 = time.time()
    try:
        for tick in range(n_ticks):
            for nm, loc in pins.items():
                env.agents[nm].set_physics_state(list(loc), [0.0, 0.0, 0.0], [0, 0, 0], [0, 0, 0])
                env.act(nm, np.zeros(8))
            raw = env.tick()
            if tick < 30:
                continue
            for k in range(len(SETTINGS)):
                d = raw.get(f"probe_{k}", {})
                r = rec[k]
                for key, slot in (("SonarFront", "front"), ("SonarRear", "rear")):
                    if key in d:
                        prof = np.asarray(d[key], float).ravel()
                        r[slot].append([s[0] for s in segment(prof, DEFAULT_SONAR)])
                        rg = DEFAULT_SONAR.ranges()
                        f = FACE[slot]
                        r[slot + "_peak"].append(float(prof[(rg > f - 0.3) & (rg < f + 0.6)].max()))
                        if slot == "front":            # open water between 0.5 and 2.0 m: pure noise intensities
                            rg = DEFAULT_SONAR.ranges()
                            r["noise_int"].extend(prof[(rg > 0.5) & (rg < 2.0)].tolist())
                if "DVLSensor" in d:
                    r["dvl"].append(np.asarray(d["DVLSensor"], float).ravel()[:3].tolist())
                if "IMUSensor" in d:
                    imu = np.asarray(d["IMUSensor"], float)
                    r["accel"].append(imu[0].tolist())
                    r["gyro"].append(imu[1].tolist())
                    if imu.shape[0] >= 4:
                        r["gyro_bias"].append(imu[3].tolist())
                if "Compass" in d:
                    m = np.asarray(d["Compass"], float).ravel()
                    r["heading"].append(math.atan2(m[1], m[0]))
                if "DepthSensor" in d:
                    r["depth"].append(float(np.asarray(d["DepthSensor"]).ravel()[0]))
                if "PoseSensor" in d:
                    r["pose_z"].append(float(np.asarray(d["PoseSensor"])[2, 3]))
    finally:
        env.__exit__(None, None, None)
    wall = time.time() - t0
    face = FACE
    out = {"seconds": args.seconds, "wall_s": round(wall, 1), "threshold": DEFAULT_SONAR.threshold,
           "bin_m": round(DEFAULT_SONAR.bin_width, 4), "face_m": {k: round(v, 3) for k, v in face.items()}, "settings": []}
    for k, st in enumerate(SETTINGS):
        r = rec[k]
        row = {"setting": st}
        for slot in ("front", "rear"):
            caps = r[slot]
            f = face[slot]
            hit = [min((x for x in c if abs(x - f) <= 0.5), key=lambda x: abs(x - f), default=None) for c in caps]
            errs = np.array([h - f for h in hit if h is not None])
            fa = [any(x < f - 0.5 for x in c) for c in caps]
            row[f"sonar_{slot}"] = {
                "captures": len(caps), "recall": round(float(np.mean([h is not None for h in hit])), 4) if caps else None,
                "false_alarm_rate": round(float(np.mean(fa)), 4) if caps else None,
                "range_err_mean": round(float(errs.mean()), 4) if errs.size else None,
                "range_err_p05": round(float(np.percentile(errs, 5)), 4) if errs.size else None,
                "range_err_p50": round(float(np.percentile(errs, 50)), 4) if errs.size else None,
                "range_err_p95": round(float(np.percentile(errs, 95)), 4) if errs.size else None,
                "range_err_max": round(float(errs.max()), 4) if errs.size else None,
                "range_err_min": round(float(errs.min()), 4) if errs.size else None,
                "peak_p05_p50_p95": np.round(np.percentile(r[slot + "_peak"], [5, 50, 95]), 3).tolist() if caps else None}
        ni = np.asarray(r["noise_int"])
        row["open_water_intensity"] = {"mean": round(float(ni.mean()), 4), "p99": round(float(np.percentile(ni, 99)), 4),
                                       "frac_above_threshold": float(np.mean(ni > DEFAULT_SONAR.threshold)),
                                       "rayleigh_scale_fit": round(float(math.sqrt(np.mean(ni ** 2) / 2.0)), 4)} if ni.size else None
        dvl = np.asarray(r["dvl"])
        row["dvl_vel_std_xyz"] = np.round(dvl.std(axis=0), 4).tolist() if dvl.size else None
        row["dvl_vel_rms_horizontal"] = round(float(np.sqrt(np.mean(dvl[:, 0] ** 2 + dvl[:, 1] ** 2))), 4) if dvl.size else None
        g = np.asarray(r["gyro"])
        row["gyro_z_std"] = round(float(g[:, 2].std()), 5) if g.size else None
        row["gyro_z_mean"] = round(float(g[:, 2].mean()), 5) if g.size else None
        gb = np.asarray(r["gyro_bias"])
        row["gyro_bias_z_end"] = round(float(gb[-1, 2]), 5) if gb.size else None
        row["gyro_bias_z_maxabs"] = round(float(np.abs(gb[:, 2]).max()), 5) if gb.size else None
        a = np.asarray(r["accel"])
        row["accel_std_xyz"] = np.round(a.std(axis=0), 4).tolist() if a.size else None
        h = np.degrees(np.asarray(r["heading"]))
        row["heading_err_deg"] = {"mean": round(float(h.mean()), 3), "std": round(float(h.std()), 3),
                                  "p95_abs": round(float(np.percentile(np.abs(h), 95)), 3)} if h.size else None
        dz = np.asarray(r["depth"])
        row["depth"] = {"mean": round(float(dz.mean()), 4), "std": round(float(dz.std()), 4),
                        "pose_z_mean": round(float(np.mean(r["pose_z"])), 4) if r["pose_z"] else None} if dz.size else None
        out["settings"].append(row)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "sensor_noise_probe.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
    for row in out["settings"]:
        s = row["setting"]
        print(f"{s['label']:10s} sonarF rec {row['sonar_front']['recall']} fa {row['sonar_front']['false_alarm_rate']} "
              f"err p50/p95/max {row['sonar_front']['range_err_p50']}/{row['sonar_front']['range_err_p95']}/{row['sonar_front']['range_err_max']} | "
              f"peak {row['sonar_front']['peak_p05_p50_p95']}/{row['sonar_rear']['peak_p05_p50_p95']} | rear rec {row['sonar_rear']['recall']} fa {row['sonar_rear']['false_alarm_rate']} p95 {row['sonar_rear']['range_err_p95']} | "
              f"rayleigh {row['open_water_intensity']['rayleigh_scale_fit'] if row['open_water_intensity'] else None} | "
              f"dvl {s['dvl_vel']}->{row['dvl_vel_std_xyz']} | gyro {s['gyro']}->{row['gyro_z_std']} bias end {row['gyro_bias_z_end']} | "
              f"mag {s['mag']}->hdg std {row['heading_err_deg']['std'] if row['heading_err_deg'] else None} deg | "
              f"depth {s['depth']}->{row['depth']}")
    print(f"wall {wall:.0f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
