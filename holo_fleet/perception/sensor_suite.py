"""Onboard sensor suite of every drone (hardware description, no simulator state).

Conventions measured with probe/probe_calib.py on HoloOcean 2.3.0 / BlueROV2:

* RangeFinderSensor beam k of N points along body azimuth ``-k*360/N`` degrees
  (beam index grows clockwise seen from above; +y body = left is beam N*3/4).
* RangeFinderSensor ``LaserAngle`` is positive DOWNWARD (Unreal pitch convention).
* MagnetometerSensor returns the world x-axis with a flipped y component, so
  heading = atan2(m_y, m_x).
* DVLSensor returns body-frame velocity, x forward / y left / z up.
* IMUSensor angular velocity z has the opposite sign of the yaw rate (left-handed).
* ImagingSonar sees other vehicles but NOT props spawned at runtime (gates).
* RangeFinderSensor ray casts DO hit other vehicles and runtime-spawned props.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

from holo_fleet.config import DEFAULT, FleetConfig

# Keys the controller is allowed to receive.  Everything else (PoseSensor,
# VelocitySensor, CollisionSensor, ChaseCamera, ...) is stripped by the simulator
# wrapper before a SensorFrame reaches a controller.
ONBOARD_PREFIXES = ("ProxSonar_", )
ONBOARD_NAMES = ("DepthSensor", "IMUSensor", "DVLSensor", "Compass", "AltimeterUp", "AltimeterDown",
                 "FrontSonar", "FrontCamera")
PRIVILEGED_NAMES = ("PoseSensor", "VelocitySensor", "CollisionSensor", "ChaseCamera", "SideCamera")


def ring_name(elev_deg: int) -> str:
    """Name of the proximity-sonar ring whose beams point ``elev_deg`` above horizontal."""
    return f"ProxSonar_e{elev_deg:+03d}"


def is_onboard(name: str) -> bool:
    return name in ONBOARD_NAMES or any(name.startswith(p) for p in ONBOARD_PREFIXES)


@dataclass(frozen=True)
class SensorDoc:
    name: str
    holoocean_type: str
    socket: str
    mounting: str
    fov: str
    range_m: str
    rate_hz: float
    noise: str
    role: str


def onboard_sensor_configs(cfg: FleetConfig = DEFAULT, with_fls: bool = True, with_camera: bool = True) -> List[Dict]:
    pc = cfg.perc
    sensors: List[Dict] = [
        {"sensor_type": "DepthSensor", "sensor_name": "DepthSensor", "socket": "DepthSocket", "Hz": 30,
         "configuration": {"Sigma": 0.02}},
        {"sensor_type": "IMUSensor", "sensor_name": "IMUSensor", "socket": "IMUSocket", "Hz": 30,
         "configuration": {"AccelSigma": 0.01, "AngVelSigma": 0.005, "ReturnBias": False}},
        {"sensor_type": "DVLSensor", "sensor_name": "DVLSensor", "socket": "DVLSocket", "Hz": 10,
         "configuration": {"Elevation": 22.5, "VelSigma": 0.01, "ReturnRange": True, "MaxRange": 50}},
        {"sensor_type": "MagnetometerSensor", "sensor_name": "Compass", "socket": "IMUSocket", "Hz": 30,
         "configuration": {"Sigma": 0.005}},
        # single-beam vertical rangefinders (altimeter / up-looking echo sounder)
        {"sensor_type": "RangeFinderSensor", "sensor_name": "AltimeterDown", "socket": "IMUSocket", "Hz": 10,
         "configuration": {"LaserMaxDistance": 30, "LaserCount": 1, "LaserAngle": 90}},
        {"sensor_type": "RangeFinderSensor", "sensor_name": "AltimeterUp", "socket": "IMUSocket", "Hz": 10,
         "configuration": {"LaserMaxDistance": 30, "LaserCount": 1, "LaserAngle": -90}},
    ]
    # 3D proximity sonar: rings of beams every 5 deg in elevation, 5 deg in azimuth.
    for elev in pc.ring_elevations_deg:
        sensors.append({
            "sensor_type": "RangeFinderSensor", "sensor_name": ring_name(elev), "socket": "IMUSocket",
            "Hz": pc.ring_hz,
            "configuration": {"LaserMaxDistance": pc.ring_range_m, "LaserCount": pc.ring_beams,
                              "LaserAngle": -elev, "LaserDebug": False},
        })
    if with_fls:
        sensors.append({
            "sensor_type": "ImagingSonar", "sensor_name": "FrontSonar", "socket": "SonarSocket", "Hz": pc.fls_hz,
            "configuration": {"Azimuth": 90, "Elevation": 20, "RangeMin": 0.5, "RangeMax": 12.0,
                              "RangeBins": 128, "AzimuthBins": 128, "AddSigma": 0.02, "MultSigma": 0.05,
                              "InitOctreeRange": 40, "ShowWarning": False},
        })
    if with_camera:
        sensors.append({
            "sensor_type": "RGBCamera", "sensor_name": "FrontCamera", "socket": "CameraSocket", "Hz": pc.camera_hz,
            "configuration": {"CaptureWidth": 320, "CaptureHeight": 240, "FovAngle": 90},
        })
    return sensors


FLS_GEOMETRY = {"azimuth_deg": 90.0, "range_min": 0.5, "range_max": 12.0, "range_bins": 128, "azimuth_bins": 128}


def sensor_documentation(cfg: FleetConfig = DEFAULT) -> List[SensorDoc]:
    pc = cfg.perc
    elevs = pc.ring_elevations_deg
    return [
        SensorDoc("IMUSensor", "IMUSensor", "IMUSocket", "hull centre", "-", "-", 30,
                  "accel 0.01, gyro 0.005 (std)", "yaw-rate damping of the heading loop, attitude monitoring"),
        SensorDoc("Compass", "MagnetometerSensor", "IMUSocket", "hull centre", "-", "-", 30,
                  "0.005 std per axis", "heading (AHRS role)"),
        SensorDoc("DVLSensor", "DVLSensor", "DVLSocket", "hull bottom, 4 beams 22.5 deg", "-", "50", 10,
                  "0.01 m/s per beam", "body velocity over ground -> dead reckoning, velocity loop"),
        SensorDoc("DepthSensor", "DepthSensor", "DepthSocket", "pressure port", "-", "-", 30, "0.02 m",
                  "depth keeping, vertical escape, relative depth"),
        SensorDoc("ProxSonar_e*", "RangeFinderSensor x%d" % len(elevs), "IMUSocket",
                  "hull centre, %d rings" % len(elevs),
                  "360 deg az x [%d,%d] deg el, 5x5 deg" % (min(elevs), max(elevs)),
                  str(pc.ring_range_m), pc.ring_hz,
                  "range %.2f m std + per-beam dropout (added by sensor model)" % pc.range_noise_std,
                  "3D proximity sonar: other drones, gate bars (blind-spot-free safety layer)"),
        SensorDoc("AltimeterDown/Up", "RangeFinderSensor", "IMUSocket", "hull centre", "single beam +/-90 deg",
                  "30", 10, "-", "vertical escape availability (seafloor / overhead)"),
        SensorDoc("FrontSonar", "ImagingSonar", "SonarSocket", "bow", "90 deg az x 20 deg el", "0.5-12",
                  pc.fls_hz, "Rayleigh add 0.02, mult 0.05", "forward-looking sonar: drones ahead (front sector)"),
        SensorDoc("FrontCamera", "RGBCamera", "CameraSocket", "bow", "90 deg HFOV, 320x240", "-", pc.camera_hz,
                  "-", "debug imagery / gate visual check (not used for safety)"),
    ]


def ring_beam_directions(cfg: FleetConfig = DEFAULT):
    """Unit body-frame direction for every (ring, beam) in a fixed order (cached by callers)."""
    import numpy as np

    pc = cfg.perc
    dirs = {}
    k = np.arange(pc.ring_beams)
    az = -np.deg2rad(k * 360.0 / pc.ring_beams)
    for elev in pc.ring_elevations_deg:
        el = np.deg2rad(elev)
        dirs[ring_name(elev)] = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az),
                                          np.full_like(az, np.sin(el))], axis=1)
    return dirs


def ring_elevations(cfg: FleetConfig = DEFAULT) -> Tuple[int, ...]:
    return cfg.perc.ring_elevations_deg
