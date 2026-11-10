"""The heading loop has an integral action: a steady yaw moment does not leave a standing error (a queued drone keeps its heading tolerance through a long wait)."""

import math

from holo_fleet.control.lowlevel import LowLevelController
from holo_fleet.perception.nav import NavState
import numpy as np


def _nav(yaw):
    n = NavState(p=np.zeros(3), yaw=yaw)
    n.yaw_rate = 0.0
    return n


def test_a_standing_heading_error_builds_up_a_bounded_command():
    ll = LowLevelController()
    e = math.radians(8.0)
    first = ll.command(_nav(-e), np.zeros(3), 0.0, "nominal", 0.1)["yaw"]
    for _ in range(600):
        last = ll.command(_nav(-e), np.zeros(3), 0.0, "nominal", 0.1)["yaw"]
    assert last > first + 0.02 and last <= 0.12


def test_a_slew_does_not_wind_up_the_integrator():
    ll = LowLevelController()
    for _ in range(600):
        ll.command(_nav(math.radians(-90.0)), np.zeros(3), 0.0, "nominal", 0.1)
    assert ll.i_psi == 0.0


def test_the_integrator_is_bounded():
    ll = LowLevelController()
    e = math.radians(10.0)
    for _ in range(20000):
        ll.command(_nav(-e), np.zeros(3), 0.0, "nominal", 0.1)
    assert abs(ll.i_psi) <= 8.0 + 1e-9
