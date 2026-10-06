"""Inter-agent communication is absent by default."""

import inspect

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.sim.scenarios import SCENARIOS


def test_config_default_has_no_comms():
    assert DEFAULT.comms_enabled is False
    assert FleetConfig().comms_enabled is False


def test_every_scenario_defaults_to_no_comms():
    for name, factory in SCENARIOS.items():
        spec = factory(seed=0)
        if name == "gate_arena_comms":           # the explicit, optional comparison experiment
            assert spec.comms_enabled is True
            continue
        assert spec.comms_enabled is False, name


def test_controller_has_no_message_channel_by_default():
    from holo_fleet.control.controller import DroneController

    sig = inspect.signature(DroneController.__init__)
    assert sig.parameters["comms_inbox"].default is None
    plan = SCENARIOS["gate_arena"](seed=0).plans[0]
    ctrl = DroneController(plan)
    assert ctrl.comms_inbox is None


def test_safety_guards_never_read_messages():
    """Messages may only become a formation hint; the abstract observation used by every guard is
    produced by the perception layer from onboard sensors only."""
    from holo_fleet.perception import perception

    src = inspect.getsource(perception)
    assert "comms" not in src and "inbox" not in src and "heartbeat" not in src
