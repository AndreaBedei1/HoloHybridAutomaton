"""Sensor-degradation profiles, beacon presence semantics and the campaign plan (no simulator)."""

import dataclasses

import numpy as np

from holo_fleet.config import DEFAULT, BeaconConfig
from holo_fleet.perception.beacon_presence import PresenceTracker
from holo_fleet.perception.frame import SensorFrame
from holo_fleet.perception.sensor_suite import DEPLOYED_NOISE, onboard_sensor_configs
from holo_fleet.sim.sensor_noise import CLEAN, FACTORS, HIGH, LOW, MEDIUM, heading_biases, isolated, plan_windows, profile


def test_clean_is_the_deployed_suite():
    assert onboard_sensor_configs() == onboard_sensor_configs(noise=CLEAN)
    for k, v in DEPLOYED_NOISE.items():
        assert getattr(CLEAN, k) == v
    s = DEFAULT.perc.sonar
    assert (CLEAN.sonar_add_sigma, CLEAN.sonar_mult_sigma, CLEAN.sonar_range_sigma) == (s.add_sigma, s.mult_sigma, s.range_sigma)
    assert not CLEAN.wrapper_active and CLEAN.heading_bias_deg == 0.0


def test_levels_are_monotone():
    for f in ("sonar_add_sigma", "dvl_vel_sigma", "mag_sigma", "depth_sigma", "sonar_dropout", "latency_s",
              "sonar_range_jitter_m", "beacon_dropout"):
        assert getattr(CLEAN, f) <= getattr(LOW, f) <= getattr(MEDIUM, f) <= getattr(HIGH, f), f


def test_isolation_changes_only_its_factor():
    for f, fields in FACTORS.items():
        p = isolated(f)
        assert profile(f"ISO_{f}") == p
        for fld in dataclasses.fields(CLEAN):
            if fld.name == "name":
                continue
            want = getattr(HIGH, fld.name) if fld.name in fields else getattr(CLEAN, fld.name)
            assert getattr(p, fld.name) == want, (f, fld.name)


def test_profile_noise_reaches_the_holoocean_configuration():
    cfgs = {c["sensor_name"]: c["configuration"] for c in onboard_sensor_configs(noise=HIGH, heading_bias_deg=4.0)}
    assert cfgs["SonarFront"]["AddSigma"] == HIGH.sonar_add_sigma
    assert cfgs["DVLSensor"]["VelSigma"] == HIGH.dvl_vel_sigma
    assert cfgs["IMUSensor"]["AngVelBiasSigma"] == HIGH.imu_gyro_bias_sigma
    assert np.isclose(np.degrees(np.arctan2(*cfgs["Compass"]["MagneticVector"][1::-1])), 4.0)
    assert cfgs["DepthSensor"]["Sigma"] == HIGH.depth_sigma


def test_seeded_effects_are_reproducible():
    a = plan_windows(np.random.default_rng([3, 7919]), 20.0, 0.5, 80.0)
    b = plan_windows(np.random.default_rng([3, 7919]), 20.0, 0.5, 80.0)
    assert a == b and a and all(abs((y - x) - 0.5) < 1e-9 for x, y in a)
    assert np.array_equal(heading_biases(HIGH, 4, 1), heading_biases(HIGH, 4, 1))
    assert set(np.abs(heading_biases(HIGH, 6, 2))) == {HIGH.heading_bias_deg}
    assert not heading_biases(CLEAN, 3, 0).any()


def test_presence_beacon_is_identity_only_and_off_by_default():
    assert not DEFAULT.beacon.enabled
    off = PresenceTracker(DEFAULT)
    off.set_identities({1: 1})
    off.update(SensorFrame(t=1.0, data={"BeaconRx": ((1, 0.9),)}, stamp={}))
    assert not off.heard(1, 1.0)                     # the baseline ignores receptions entirely
    cfg = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_PRESENCE"))
    pt = PresenceTracker(cfg)
    pt.set_identities({0: 2, 1: 0})
    pt.update(SensorFrame(t=5.0, data={"BeaconRx": ((0, 4.9), (7, 4.95))}, stamp={}))     # unknown id ignored
    assert pt.heard(2, 5.0) and not pt.heard(0, 5.0)
    assert pt.heard(2, 4.9 + cfg.beacon.ttl_s) and not pt.heard(2, 4.9 + cfg.beacon.ttl_s + 0.1)


def test_campaign_plan_sizes():
    import importlib.util
    from pathlib import Path

    spec = importlib.util.spec_from_file_location("camp", Path(__file__).resolve().parents[1] / "scripts" /
                                                  "run_sensor_robustness_campaign.py")
    camp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(camp)
    assert len(camp.plan("main", (0, 1, 2))) == 144
    assert len(camp.plan("beacon", (0, 1, 2))) == 72
    assert len(camp.plan("isolation", (0, 1, 2))) == len(FACTORS) * 3 * 3
    assert {r[3] for r in camp.plan("high", (0, 1, 2))} == {0, 1, 2}
