"""Near-field clearance of the admission and the queue points that leave it to the lead drone (v31 regression:
no B0 lead drone admitted, the drone queued behind the hold renewed the 5f9f049 near-field restart of the timed bound)."""

import math

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.control.flows import ACQ_OFFSETS, RELOC_ZONE_M
from holo_fleet.mission import abreast_slots, queue_assignment, queue_line
from holo_fleet.sim.scenarios import SCENARIOS

G = DEFAULT.gate


def _zone_dist(s, l, dz):
    ds = max(G.stage2_hold_s - G.admit_zone_m - s, 0.0)
    dl = max(abs(l) - G.admit_zone_lat_m, 0.0)
    dv = max(abs(dz) - G.zone_dz_half(), 0.0)
    return math.sqrt(ds * ds + dl * dl + dv * dv)


def test_the_acquisition_offsets_and_the_zone_constants_agree():
    assert RELOC_ZONE_M == G.admit_zone_m
    assert sorted(dz for _dl, dz in ACQ_OFFSETS) == [-G.acq_dz_m, G.acq_dz_m]
    assert all(G.in_admission_zone(G.stage2_hold_s, dl, dz) for dl, dz in ACQ_OFFSETS)
    assert G.in_admission_zone(G.stage2_hold_s, 0.0, 0.0)


def test_the_clearance_exceeds_the_largest_distance_of_two_drones_in_the_zone():
    from formal.check_stale import zone_distance
    from holo_fleet.perception.structure_aiding import CONSIST_M, FIT_RMS_M, UNIQUE_M

    d = zone_distance(DEFAULT, G.nav_err_m, CONSIST_M + UNIQUE_M + FIT_RMS_M)
    assert 3.2 < d < G.admit_clear_m < G.queue_hold_clear_m


@pytest.mark.parametrize("n", [2, 3, 4, 5])
def test_every_abreast_queue_point_keeps_the_hold_clearance(n):
    sl = abreast_slots(n)
    line = queue_line(sl, G)
    for lq, dzq in queue_assignment(sl, G):
        sq = G.queue_s_for(line, lq, dzq, n)
        assert sq <= line
        assert _zone_dist(sq, lq, dzq) >= G.queue_hold_clear_m - 1e-9


def test_the_middle_column_of_the_three_drone_queue_waits_farther_back():
    plans = SCENARIOS["line_parallel_mutex"](DEFAULT).plans
    by_l = {round(p.queue_lateral, 3): p.queue_s for p in plans}
    line = queue_line(plans[0].slots, G)
    assert abs(by_l[0.0] - (G.stage2_hold_s - G.admit_zone_m - G.queue_hold_clear_m)) < 1e-9
    assert by_l[3.5] == line and by_l[-3.5] == line
    for p in plans:
        assert _zone_dist(p.queue_s, p.queue_lateral, p.queue_dz) >= G.queue_hold_clear_m - 1e-9


def test_a_single_drone_keeps_the_queue_line():
    assert G.queue_s_for(-4.0, 0.0, 0.0, 1) == -4.0
