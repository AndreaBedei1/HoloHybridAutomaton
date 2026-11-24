"""The parametric channel of the communication sweep and the no-adaptation baseline."""

import numpy as np

from holo_fleet.config import DEFAULT
from holo_fleet.sim.acoustic_channel import LOW, ChannelModel, channel, sweep_spec
from holo_fleet.sim.scenarios import SCENARIOS


def test_one_factor_of_the_low_channel_is_varied():
    assert sweep_spec("SW-per4-lat0.5") == {"period_s": 4.0, "latency_s": 0.5}
    c = channel("SW-loss0.7")
    assert c.dropout == c.dropout_far == 0.7 and c.latency_s == LOW.latency_s and c.sigma_az_deg == LOW.sigma_az_deg
    b = channel("SW-burst30")
    assert b.outage_len_s == 30.0 and b.outage_every_s == 90.0
    assert channel("SW-jit1.0").jitter_s == 1.0 and channel("SW-asym0.9").asym_loss == 0.9


def test_asymmetric_loss_drops_one_direction_only():
    st = ChannelModel(channel("SW-asym1.0-loss0"), 2, 0, 50.0)
    buf = np.array([0.0, 0.0, 5.0, -5.0])
    assert st.measure(1.0, 1, 0, buf.copy()) is None and st.measure(1.0, 0, 1, buf.copy()) is not None


def test_the_emergence_scenario_delays_one_drone_with_a_head_jet():
    sc = SCENARIOS["em_delay_head"](DEFAULT)
    (jet,) = [c for c in sc.sim.current.components if c.kind == "jet"]
    assert tuple(jet.drift) == (-0.70, 0.0, 0.0) and len(sc.sim.current.components) == 1 and not sc.sim.faults
