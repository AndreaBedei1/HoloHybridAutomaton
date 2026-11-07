"""Mapped gate + navigation error (sensor-robustness campaign: line_parallel_mutex under navigation drift).

The structure test predicts the gate bars from the drone's ESTIMATED pose, with a tolerance of structure_tol_m
(0.30 m on the near side).  When the navigation error exceeds it, a real bar echo is no longer explained by the
map and stays an obstacle: conservative.  In the campaign (DVL velocity noise at its HIGH level, navigation error
up to 1 m) the drone that could not explain the bar was really 0.9 m from it, heading into the bar instead of the
opening; the avoidance kept it off the gate and the run ended in a safe livelock.  The tolerance is deliberately
not widened: in front of a bar, an echo within structure_tol_m (plus the pose error toward the bar) of the predicted
surface is explained by the map, so every centimetre added to the tolerance is a centimetre in which a vehicle
would be explained away.
"""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.mission import BarBox
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.perception.sonar_processing import DYNAMIC, STRUCTURE, EchoClassifier

RANGES = DEFAULT.perc.sonar.ranges()
I3 = np.eye(3)


def _profile(*segs):
    p = np.zeros_like(RANGES)
    for r0, r1 in segs:
        p[(RANGES >= r0) & (RANGES <= r1)] = 0.9
    return p


def _front(clf, segs, pos):
    profiles = {s: _profile() for s in sg.SECTORS}
    profiles["FRONT"] = _profile(*segs)
    return [e.cls for e in clf.classify(profiles, {s: 0.0 for s in sg.SECTORS}, np.asarray(pos, float), I3, None)["FRONT"].echoes]


def _bar(x=4.0):
    return BarBox(center=np.array([x, 0.0, 0.0]), axes=I3, half=np.array([0.09, 0.09, 0.75]))


BAR_ECHO = (3.65, 3.8)          # the near face of a bar 4.0 m ahead, seen by the front sonar


def test_bar_is_structure_with_an_accurate_pose():
    clf = EchoClassifier(gate_bars=[_bar()])
    assert _front(clf, [BAR_ECHO], [0.0, 0.0, 0.0]) == [STRUCTURE]
    clf = EchoClassifier(gate_bars=[_bar()])
    assert _front(clf, [BAR_ECHO], [-0.2, 0.0, 0.0]) == [STRUCTURE]      # pose error inside the tolerance


def test_bar_beyond_the_pose_tolerance_stays_an_obstacle():
    """The drone believes it is 0.5 m further back: the bar is predicted 0.5 m further away than its echo."""
    clf = EchoClassifier(gate_bars=[_bar()])
    _front(clf, [BAR_ECHO], [-0.5, 0.0, 0.0])
    assert _front(clf, [BAR_ECHO], [-0.5, 0.0, 0.0]) == [DYNAMIC]          # confirmed obstacle, never hidden


def test_vehicle_in_front_of_a_mapped_bar_stays_an_obstacle():
    """A vehicle 0.8 m in front of the bar, pose errors up to 0.4 m either way: always a confirmed obstacle."""
    for err in (-0.4, -0.2, 0.0, 0.2, 0.4):
        clf = EchoClassifier(gate_bars=[_bar()])
        _front(clf, [(2.85, 3.0), BAR_ECHO], [err, 0.0, 0.0])
        cls = _front(clf, [(2.85, 3.0), BAR_ECHO], [err, 0.0, 0.0])
        assert cls[0] == DYNAMIC, (err, cls)


def test_the_structure_tolerance_is_not_widened():
    assert DEFAULT.perc.structure_tol_m <= 0.30
