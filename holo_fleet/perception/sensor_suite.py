"""Onboard sensor suite of every drone (hardware description, no simulator state) - v2.

Six identical wide-beam single-beam sonars (``SinglebeamSonar``), one on each hull face, plus DVL
(velocity over ground and four beam ranges), IMU, compass and depth.  No ray-cast rangefinder
ring and no imaging sonar: what a drone knows about its surroundings comes from the six echo
profiles.

Conventions measured on HoloOcean 2.3.0 / BlueROV2 (DISCOVERY.md, docs/v2/SONAR_PROBE.md):
* sensor ``location`` is relative to the agent origin; PoseSensor sits at IMUSocket, (+0.11, 0, -0.065) m;
* rotation pitch is positive downward (pitch -90 points the beam up);
* MagnetometerSensor heading = atan2(m_y, m_x); DVL body velocity x fwd / y left / z up;
* IMU gyro z has the opposite sign of the yaw rate.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.perception.sonar_geometry import MOUNTS, ROTATIONS, SECTORS, sonar_sensor_name

SONAR_NAMES = tuple(sonar_sensor_name(s) for s in SECTORS)
# Keys a controller is allowed to receive.  Everything else (PoseSensor, VelocitySensor,
# CollisionSensor, visualisation cameras, ...) is stripped by the simulator wrapper.
ONBOARD_NAMES = ("DepthSensor", "IMUSensor", "DVLSensor", "Compass") + SONAR_NAMES
PRIVILEGED_NAMES = ("PoseSensor", "VelocitySensor", "CollisionSensor", "ChaseCamera", "SideCamera", "TopCamera")


def is_onboard(name: str) -> bool:
    return name in ONBOARD_NAMES


@dataclass(frozen=True)
class SensorDoc:
    name: str
    holoocean_type: str
    mounting: str
    fov: str
    range_m: str
    rate_hz: float
    noise: str
    role: str


def sonar_sensor_configs(cfg: FleetConfig = DEFAULT, view_region: bool = False) -> List[Dict]:
    m = cfg.perc.sonar
    out = []
    for s in SECTORS:
        out.append({"sensor_type": "SinglebeamSonar", "sensor_name": sonar_sensor_name(s), "socket": "",
                    "location": [float(v) for v in MOUNTS[s]], "rotation": list(ROTATIONS[s]), "Hz": int(m.hz),
                    "configuration": m.holoocean_configuration(view_region)})
    return out


def onboard_sensor_configs(cfg: FleetConfig = DEFAULT, view_region: bool = False) -> List[Dict]:
    return [
        {"sensor_type": "DepthSensor", "sensor_name": "DepthSensor", "socket": "DepthSocket", "Hz": 30,
         "configuration": {"Sigma": 0.02}},
        {"sensor_type": "IMUSensor", "sensor_name": "IMUSensor", "socket": "IMUSocket", "Hz": 30,
         "configuration": {"AccelSigma": 0.01, "AngVelSigma": 0.005, "ReturnBias": False}},
        {"sensor_type": "DVLSensor", "sensor_name": "DVLSensor", "socket": "DVLSocket", "Hz": 10,
         "configuration": {"Elevation": 22.5, "VelSigma": 0.01, "ReturnRange": True, "MaxRange": 50,
                           "RangeSigma": 0.02}},
        {"sensor_type": "MagnetometerSensor", "sensor_name": "Compass", "socket": "IMUSocket", "Hz": 30,
         "configuration": {"Sigma": 0.005}},
    ] + sonar_sensor_configs(cfg, view_region)


def sensor_documentation(cfg: FleetConfig = DEFAULT) -> List[SensorDoc]:
    m = cfg.perc.sonar
    docs = [SensorDoc(sonar_sensor_name(s), "SinglebeamSonar",
                      f"{s.lower()} hull face, {tuple(round(float(v), 2) for v in MOUNTS[s])} m, rpy {ROTATIONS[s]}",
                      f"cone {m.opening_deg:.0f} deg", f"{m.range_min}-{m.range_max}", m.hz,
                      f"intensity: Rayleigh {m.add_sigma}, mult {m.mult_sigma}; range: exp {m.range_sigma} m",
                      "wide-beam directional single-beam sonar: nearest echoes in its sector")
            for s in SECTORS]
    return docs + [
        SensorDoc("DVLSensor", "DVLSensor", "hull bottom, 4 beams at 22.5 deg", "-", "50", 10,
                  "0.01 m/s per beam, range 0.02 m", "velocity over ground (dead reckoning), altitude"),
        SensorDoc("Compass", "MagnetometerSensor", "IMUSocket", "-", "-", 30, "0.005 per axis", "heading"),
        SensorDoc("IMUSensor", "IMUSensor", "IMUSocket", "-", "-", 30, "accel 0.01, gyro 0.005",
                  "yaw-rate damping, roll/pitch from gravity"),
        SensorDoc("DepthSensor", "DepthSensor", "DepthSocket", "-", "-", 30, "0.02 m", "depth"),
    ]
