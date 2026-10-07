"""There is no inter-agent communication: no channel exists, the configuration disables it, runs log 0 messages."""

import inspect
from pathlib import Path

from holo_fleet.config import DEFAULT, FleetConfig
from holo_fleet.sim.scenarios import SCENARIOS

ROOT = Path(__file__).resolve().parents[1]


def test_config_default_has_no_comms():
    assert DEFAULT.comms_enabled is False
    assert FleetConfig().comms_enabled is False


def test_no_scenario_enables_comms():
    for name, factory in SCENARIOS.items():
        sc = factory(DEFAULT)
        assert "comms_enabled" not in sc.cfg_patch, name


def test_controller_has_no_message_channel():
    from holo_fleet.control.controller import DroneController

    sig = inspect.signature(DroneController.__init__)
    assert set(sig.parameters) == {"self", "plan", "cfg", "nav_init_err"}
    assert not (ROOT / "holo_fleet" / "comms").exists()


def test_perception_and_guards_never_read_messages():
    from holo_fleet.control import controller, flows
    from holo_fleet.perception import perception

    for mod in (perception, controller, flows):
        src = inspect.getsource(mod)
        for word in ("inbox", "heartbeat", "broadcast", "socket", "message"):
            assert word not in src, (mod.__name__, word)
