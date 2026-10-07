"""An echo is not a drone: STRUCTURE / SEABED / DYNAMIC / UNKNOWN from onboard information only."""

import json
from pathlib import Path

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.mission import BarBox
from holo_fleet.perception import sonar_geometry as sg
from holo_fleet.perception.sonar_processing import (DYNAMIC, SEABED, STRUCTURE, UNCONFIRMED, UNKNOWN, EchoClassifier,
                                                    segment)
from holo_fleet.perception.targets import SectorTracker, build_targets

ROOT = Path(__file__).resolve().parents[1]
RANGES = DEFAULT.perc.sonar.ranges()
I3 = np.eye(3)


def profile(*segs, level=0.9):
    p = np.zeros_like(RANGES)
    for r0, r1 in segs:
        p[(RANGES >= r0) & (RANGES <= r1)] = level
    return p


def classify(clf, front=(), down=(), seabed_z=None, pos=np.zeros(3)):
    profiles = {s: profile() for s in sg.SECTORS}
    profiles["FRONT"] = profile(*front)
    profiles["DOWN"] = profile(*down)
    return clf.classify(profiles, {s: 0.0 for s in sg.SECTORS}, pos, I3, seabed_z)


def post_ahead(x=4.0):
    return BarBox(center=np.array([x, 0.0, 0.0]), axes=I3, half=np.array([0.09, 0.09, 0.75]))


def test_segmentation_merges_small_gaps():
    p = profile((2.0, 2.2), (2.27, 2.4))
    assert len(segment(p, DEFAULT.perc.sonar, DEFAULT.perc.merge_gap_bins)) == 1


def test_mapped_gate_bar_is_structure():
    clf = EchoClassifier(gate_bars=[post_ahead(4.0)])
    out = classify(clf, front=[(3.65, 3.8)])
    assert [e.cls for e in out["FRONT"].echoes] == [STRUCTURE]


def test_unexplained_compact_echo_needs_two_captures_then_is_dynamic():
    clf = EchoClassifier()
    assert [e.cls for e in classify(clf, front=[(2.5, 2.7)])["FRONT"].echoes] == [UNCONFIRMED]
    assert [e.cls for e in classify(clf, front=[(2.45, 2.65)])["FRONT"].echoes] == [DYNAMIC]


def test_single_capture_noise_never_becomes_an_obstacle():
    clf = EchoClassifier()
    classify(clf, front=[(6.0, 6.1)])
    out = classify(clf, front=[(2.0, 2.1)])            # a different range: not confirmed either
    assert all(e.cls == UNCONFIRMED for e in out["FRONT"].echoes)


def test_extended_unexplained_echo_is_unknown():
    clf = EchoClassifier()
    classify(clf, front=[(3.0, 4.6)])
    assert [e.cls for e in classify(clf, front=[(3.0, 4.6)])["FRONT"].echoes] == [UNKNOWN]


def test_drone_in_front_of_a_mapped_gate():
    clf = EchoClassifier(gate_bars=[post_ahead(4.4)])
    classify(clf, front=[(1.8, 2.0), (4.05, 4.2)])
    cls = [e.cls for e in classify(clf, front=[(1.8, 2.0), (4.05, 4.2)])["FRONT"].echoes]
    assert cls == [DYNAMIC, STRUCTURE]


def test_seabed_from_depth_dvl_altitude_and_cone():
    clf = EchoClassifier()
    sb = -2.0                                           # 2 m below the sonars
    onset = 2.0 / np.cos(np.radians(30.0))              # horizontal 120 deg cone grazes the bottom at h / sin(60)
    out = classify(clf, front=[(onset + 0.05, onset + 1.5)], down=[(1.86, 2.1)], seabed_z=sb)
    assert [e.cls for e in out["DOWN"].echoes] == [SEABED]
    assert [e.cls for e in out["FRONT"].echoes] == [SEABED]
    assert out["FRONT"].blind_from is not None and out["FRONT"].blind_from < onset


def test_drone_before_the_seabed_onset_is_dynamic():
    clf = EchoClassifier()
    onset = 2.0 / np.cos(np.radians(30.0))
    for _ in range(2):
        out = classify(clf, front=[(1.2, 1.4), (onset + 0.05, onset + 1.5)], seabed_z=-2.0)
    assert [e.cls for e in out["FRONT"].echoes] == [DYNAMIC, SEABED]


def test_seabed_clutter_is_a_conservative_unknown_target():
    clf = EchoClassifier()
    for _ in range(2):
        out = classify(clf, front=[(2.4, 3.9)], seabed_z=-2.0)
    tg = build_targets(0.0, out, SectorTracker().update(0.0, out), DEFAULT)
    pseudo = [t for t in tg if "FRONT" in t.pattern]
    assert pseudo and all(t.cls == UNKNOWN for t in pseudo)


BENCH = ROOT / "results" / "v2" / "sonar_bench" / "classify.json"


@pytest.mark.skipif(not BENCH.exists(), reason="HoloOcean classification bench not run (scripts/sonar_bench.py classify)")
def test_holoocean_bench_cases():
    cases = {c["case"]: c["echoes"] for c in json.loads(BENCH.read_text())["cases"]}
    front = {k: [e["cls"] for e in v.get("FRONT", [])] for k, v in cases.items()}
    assert front["A_gate_only"] and set(front["A_gate_only"]) == {STRUCTURE}
    assert DYNAMIC in front["B_drone_only"]
    assert front["D_gate_plus_drone"][:1] == [DYNAMIC] and STRUCTURE in front["D_gate_plus_drone"]
    assert front["F_two_drones_one_cone"].count(DYNAMIC) == 2
    # the DOWN beam at normal incidence is too weak to cross the threshold (probe H): the seabed is known from the
    # DVL altitude, and every echo the horizontal cones get from the bottom is SEABED
    seabed = [e["cls"] for v in cases["C_seabed_only"].values() for e in v]
    assert seabed and DYNAMIC not in seabed and SEABED in seabed       # bottom echoes never become a vehicle
    assert front["E1_seabed_plus_near_drone"][:1] == [DYNAMIC]
    assert DYNAMIC not in front["E2_seabed_plus_far_drone"]          # inside the clutter: never declared a clear drone
    blind = json.loads(BENCH.read_text())["cases"]
    e2 = [c for c in blind if c["case"] == "E2_seabed_plus_far_drone"][0]
    if "blind_from" in e2:                                           # bench v2: the clutter is an UNKNOWN pseudo-target
        assert e2["blind_from"]["FRONT"] is not None
