"""Sensor-degradation profiles, the beacon default (off) and the campaign plan (no simulator)."""

import dataclasses

import numpy as np

from holo_fleet.config import DEFAULT, communication_disabled
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


def test_acoustic_beacon_is_on_by_default_and_ignored_when_the_communication_is_disabled():
    assert DEFAULT.beacon.enabled
    off = AcousticTracker(communication_disabled())
    off.set_identities({1: 1})
    assert off.update(1.0, ((1, 0.95, 3.0, 0.0, 0.0),), np.zeros(3), 0.0) == []


def test_campaign_plan_sizes():
    import importlib.util
    from pathlib import Path

    spec = importlib.util.spec_from_file_location("camp", Path(__file__).resolve().parents[1] / "scripts" /
                                                  "run_sensor_robustness_campaign.py")
    camp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(camp)
    runs = camp.plan((0, 1), only=["gate_single"], protocols=["B2_LOW"], profiles=["LOW-ALL"])
    assert runs == [("B2_LOW", "LOW-ALL", "gate_single", 0), ("B2_LOW", "LOW-ALL", "gate_single", 1)]
    both = camp.plan(None, only=["gate_single", "lost_drone_mutex"], protocols=["B0", "B2_MEDIUM"], profiles=["CLEAN"])
    assert len(both) == len(set(both)) == 2 * 2 * len(camp.SEEDS)


def test_validation_levels_truncate_the_sonar_shift_and_draw_per_run_scales():
    """LOW / MEDIUM: whole-profile shift at most one 0.05 m bin, residual sound-speed scale within 0.13 % (eps_range valid to
    12 m), DVL scale within NavBound's 1 %; scales constant per run and reproducible from the seed."""
    from holo_fleet.sim.sensor_noise import scale_factors

    for p in (LOW, MEDIUM):
        assert p.sonar_jitter_max_bins == 1 and 0 < p.sonar_scale_max <= 0.0013 and 0 < p.dvl_scale_max <= 0.01
        s1, d1 = scale_factors(p, 4, 3)
        s2, d2 = scale_factors(p, 4, 3)
        assert np.array_equal(s1, s2) and np.array_equal(d1, d2)
        assert np.all(np.abs(s1) <= p.sonar_scale_max) and np.all(np.abs(d1) <= p.dvl_scale_max)
    assert CLEAN.sonar_scale_max == CLEAN.dvl_scale_max == 0.0 and HIGH.sonar_jitter_max_bins > 1


def _bare_sim(noise, sonar_scale=0.0, dvl_scale=0.0, seed=0):
    from holo_fleet.sim.holo_env import HoloFleetSim

    sim = object.__new__(HoloFleetSim)
    sim.noise, sim.cfg = noise, DEFAULT
    sim.nrng = np.random.default_rng([seed, 7919])
    sim.sonar_scale, sim.dvl_scale = {"d": sonar_scale}, {"d": dvl_scale}
    sim.noise_stats = {k: 0 for k in ("sonar_blackout", "sonar_dropped", "sonar_faded", "sonar_shifted", "dvl_outage",
                                      "dvl_dropped")}
    sim._in_window = lambda name, kind, t: False
    return sim


def test_the_wrapper_truncates_the_shift_and_applies_the_scales():
    from dataclasses import replace as rp

    from holo_fleet.sim.holo_env import SONAR_NAMES

    key = sorted(SONAR_NAMES)[0]
    nb = DEFAULT.perc.sonar.bins
    prof = np.zeros(nb)
    prof[100:103] = 0.9
    noisy = rp(MEDIUM, sonar_range_jitter_m=1.0, sonar_dropout=0.0, sonar_fade_prob=0.0)
    sim = _bare_sim(noisy)
    shifts = set()
    for _ in range(200):
        out = sim._degrade("d", key, prof, 1.0)
        shifts.add(int(np.flatnonzero(np.asarray(out) > 0.5)[0]) - 100)
    assert shifts <= {-1, 0, 1} and len(shifts) > 1
    sim = _bare_sim(rp(MEDIUM, sonar_range_jitter_m=0.0, sonar_dropout=0.0, sonar_fade_prob=0.0), sonar_scale=0.01)
    out = sim._degrade("d", key, prof, 1.0)
    assert int(np.flatnonzero(np.asarray(out) > 0.5)[0]) == 101
    sim = _bare_sim(rp(MEDIUM, dvl_dropout=0.0), dvl_scale=0.01)
    v = sim._degrade("d", "DVLSensor", np.array([0.5, -0.2, 0.1, 3.0, 3.0, 3.0, 3.0]), 1.0)
    assert np.allclose(v[:3], [0.505, -0.202, 0.101]) and np.allclose(v[3:], 3.0)
