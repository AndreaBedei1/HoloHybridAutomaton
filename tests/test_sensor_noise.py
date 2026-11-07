"""Sensor-degradation profiles, the beacon default (off) and the campaign plan (no simulator)."""

import dataclasses

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.perception.acoustic import AcousticTracker
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
              "sonar_range_jitter_m"):
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


def test_acoustic_beacon_is_off_by_default_and_ignored_by_the_baseline():
    assert not DEFAULT.beacon.enabled
    off = AcousticTracker(DEFAULT)
    off.set_identities({1: 1})
    assert off.update(1.0, ((1, 0.95, 3.0, 0.0, 0.0),), np.zeros(3), 0.0) == []   # B0 ignores receptions entirely


def test_campaign_plan_sizes():
    import importlib.util
    from pathlib import Path

    spec = importlib.util.spec_from_file_location("camp", Path(__file__).resolve().parents[1] / "scripts" /
                                                  "run_sensor_robustness_campaign.py")
    camp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(camp)
    runs = camp.plan("targeted", (0, 1), only=["gate_single"], protocols=["B2_LOW"], profiles=["LOW-ALL"])
    # criteria v3: the deadline is a priori, so no CLEAN run is planned as a deadline source
    assert runs == [("B2_LOW", "LOW-ALL", "gate_single", 0), ("B2_LOW", "LOW-ALL", "gate_single", 1)]
    final = camp.plan("final", (0, 1, 2))
    assert len(set(final)) == len(final)
