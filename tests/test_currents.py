"""Current calibration: effective drift <-> HoloOcean command."""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.sim.currents import CurrentComponent, CurrentField, drift_to_command


def test_inverse_matches_calibration_table():
    for cmd, drift in DEFAULT.plant.current_cmd_to_drift:
        assert abs(drift_to_command(drift) - cmd) < 1e-6


def test_monotone_and_extrapolated():
    xs = np.linspace(0, 1.5, 40)
    ys = [drift_to_command(x) for x in xs]
    assert all(b >= a for a, b in zip(ys, ys[1:]))


def test_gust_window_and_ramp():
    c = CurrentComponent("jet", drift=(0, 0.8, 0), center=(0, 0), radius=4, t_on=10, t_off=20, ramp=2)
    f = CurrentField([c])
    p = np.zeros(3)
    assert np.linalg.norm(f.drift_at(p, 5)) == 0
    assert 0 < np.linalg.norm(f.drift_at(p, 10.5)) < 0.8
    assert abs(np.linalg.norm(f.drift_at(p, 15)) - 0.8) < 1e-9
    assert np.linalg.norm(f.drift_at(p, 25)) == 0
