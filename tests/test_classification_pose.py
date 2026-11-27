"""Echo classification at the structure-fit pose (v39 B0 gate MEDIUM: a lateral estimate error of 0.72 m put the bars
ahead outside the structure window; the drone took them for an obstacle at the CR entry and never crossed)."""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.perception.sonar_processing import STRUCTURE, EchoClassifier
from holo_fleet.perception.structure_aiding import StructureAiding, classification_fit
from test_structure_aiding import BARS, GATES, _R, _profiles, _world

G = DEFAULT.gate


def _non_structure(readings, r_max=3.0):
    return [(s, e.r0, e.cls) for s, rd in readings.items() for e in rd.echoes if e.cls != STRUCTURE and e.r0 < r_max]


def _case(s=-1.2, err_l=0.72):
    clf = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    R = _R()
    truth = _world(s, 0.0)
    est = _world(s, err_l)
    prof = _profiles(clf, truth, R)
    ages = {k: 0.0 for k in sg.SECTORS}
    return clf, R, truth, est, prof, ages


def test_the_bar_echo_is_structure_at_the_true_pose_and_not_at_the_shifted_estimate():
    clf, R, truth, est, prof, ages = _case()
    assert not _non_structure(clf.classify(prof, ages, truth, R, None))
    bad = []
    for _ in range(3):  # M-of-N confirmation
        bad = _non_structure(EchoClassifier(gate_bars=BARS, cfg=DEFAULT).classify(prof, ages, est, R, None))
    clf2 = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    for _ in range(3):
        bad = _non_structure(clf2.classify(prof, ages, est, R, None))
    assert bad, "the defect: at the estimated pose a bar echo is unexplained"


def test_the_fit_offset_explains_the_bars():
    clf, R, truth, est, prof, ages = _case()
    aid = StructureAiding(clf, DEFAULT, gates=GATES)
    aid.update(est, R, prof, ages, 0.25, t=0.1)
    assert classification_fit(aid.last)
    d = np.array([aid.last["offset"][0], aid.last["offset"][1], 0.0])
    assert np.linalg.norm(est + d - truth) <= 0.15
    clf2 = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    for _ in range(3):
        out = clf2.classify(prof, ages, est + d, R, None)
    assert not _non_structure(out)


def test_a_hull_off_the_bars_is_still_an_obstacle_at_the_fit_pose():
    """The fit pose moves the comparison with the map, not its tolerances: a hull 0.6 m ahead in the REAR sector (no bar
    there) stays DYNAMIC."""
    clf, R, truth, est, prof, ages = _case()
    prof = _profiles(clf, truth, R, extra={"REAR": 0.6})
    aid = StructureAiding(clf, DEFAULT, gates=GATES)
    aid.update(est, R, prof, ages, 0.25, t=0.1)
    d = np.array([aid.last["offset"][0], aid.last["offset"][1], 0.0])
    clf2 = EchoClassifier(gate_bars=BARS, cfg=DEFAULT)
    for _ in range(3):
        out = clf2.classify(prof, ages, est + d, R, None)
    assert any(e.cls != STRUCTURE and abs(e.r0 - 0.6) < 0.1 for e in out["REAR"].echoes)
