"""One controller: B2 (passive one-way acoustic beacons) is the default and the only controller; B0 is that same controller with
the communication disabled.  Guards against the return of the historic switches and version labels."""

import dataclasses
import importlib.util
import inspect
import re
from pathlib import Path

import pytest

from holo_fleet import config
from holo_fleet.config import (COMMUNICATION_ACOUSTIC, COMMUNICATION_DISABLED, DEFAULT, BeaconConfig, FleetConfig,
                               communication_disabled)
from holo_fleet.sim.scenarios import SCENARIOS

ROOT = Path(__file__).resolve().parents[1]

REMOVED_FIELDS = {"comms_enabled", "payload", "neighbour_adaptation", "v12", "variant", "version"}
# switches and labels of the controller variants that preceded the single controller
# (a version label inside a tag name, b2-freeze-v11, is not a controller variant: the tags stay)
BANNED = re.compile(r"\bB2N[PA]\b|comms_enabled|neighbour_adaptation|(?<![\w-])[vV]1[0-9]\b|beacon\.payload\b|payload=(?:False|True)")


def test_b2_is_the_default_controller():
    assert DEFAULT.beacon.enabled and DEFAULT.beacon.mode == COMMUNICATION_ACOUSTIC
    assert FleetConfig().beacon == DEFAULT.beacon and BeaconConfig().enabled


def test_b0_differs_from_b2_only_by_the_communication():
    b0 = communication_disabled()
    assert not b0.beacon.enabled and b0.beacon.mode == COMMUNICATION_DISABLED
    d2, d0 = DEFAULT.to_dict(), b0.to_dict()
    assert d2["beacon"].pop("mode") != d0["beacon"].pop("mode")
    assert d2 == d0, "B0 must be B2 with only the communication disabled"
    assert communication_disabled(b0) == b0
    assert dataclasses.replace(b0, beacon=dataclasses.replace(b0.beacon, mode=COMMUNICATION_ACOUSTIC)) == DEFAULT
    other = dataclasses.replace(DEFAULT, beacon=dataclasses.replace(DEFAULT.beacon, channel="MEDIUM"))
    assert communication_disabled(other).beacon == dataclasses.replace(other.beacon, mode=COMMUNICATION_DISABLED)


def test_the_removed_switches_are_not_configuration_fields():
    for cls in (FleetConfig, BeaconConfig, config.FormationRule, config.GateRule, config.AcousticConfig, config.PerceptionConfig):
        assert not {f.name for f in dataclasses.fields(cls)} & REMOVED_FIELDS, cls.__name__
    assert not hasattr(DEFAULT, "comms_enabled") and not hasattr(DEFAULT.beacon, "payload")


def test_no_active_source_carries_a_removed_switch_or_a_controller_version_label():
    hits = []
    files = [p for d, pat in (("holo_fleet", "**/*.py"), ("formal", "*.py"), ("scripts", "*.py"), ("calibration", "*.py"),
                              ("tests", "*.py")) for p in (ROOT / d).glob(pat)]
    assert len(files) > 100
    for p in files:
        if p.name == Path(__file__).name:
            continue
        for n, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1):
            if BANNED.search(line):
                hits.append(f"{p.relative_to(ROOT).as_posix()}:{n}: {line.strip()[:100]}")
    assert not hits, "\n".join(hits)


def test_only_the_documented_scenarios_patch_the_configuration():
    """Scenarios do not select a controller variant: the patches are the safety-layer demo (traffic rule off), the failure model of
    the stress runs and the beacon of the targeted B2 communication cases / the localization case that runs with the communication
    disabled (B0)."""
    patched = {}
    for name, make in SCENARIOS.items():
        sc = make(DEFAULT)
        for key in sc.cfg_patch:
            patched.setdefault(key, set()).add(name)
    assert set(patched) == {"traffic_rule", "failure", "beacon"}
    assert patched["traffic_rule"] == {"p1_vertical_escape"}
    assert patched["beacon"] == {"gate_rear_approach_unloc", "b2c_mute_transmitter", "b2c_blackout"}
    assert SCENARIOS["gate_rear_approach_unloc"](DEFAULT).cfg_patch["beacon"].mode == COMMUNICATION_DISABLED
    for name in ("b2c_mute_transmitter", "b2c_blackout"):
        assert SCENARIOS[name](communication_disabled()).cfg_patch["beacon"].mode == COMMUNICATION_ACOUSTIC


def test_the_campaign_protocols_are_b0_and_b2_channels():
    spec = importlib.util.spec_from_file_location("campaign_protocols", ROOT / "scripts" / "run_sensor_robustness_campaign.py")
    camp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(camp)
    assert camp.protocol_config("B0") == communication_disabled()
    assert camp.protocol_config("B2_CLEAN") == DEFAULT
    assert camp.protocol_config("B2_MEDIUM").beacon == dataclasses.replace(DEFAULT.beacon, channel="MEDIUM")
    assert camp.protocol_config("B2_SW-lat0.35").beacon.channel == "SW-lat0.35"
    for historic in ("B2NP_MEDIUM", "B2NA_LOW", "B1", "B2"):
        with pytest.raises(SystemExit):
            camp.protocol_config(historic)


def test_controller_has_no_message_channel():
    from holo_fleet.control.controller import DroneController

    sig = inspect.signature(DroneController.__init__)
    assert set(sig.parameters) == {"self", "plan", "cfg", "nav_init_err"}
    assert not (ROOT / "holo_fleet" / "comms").exists()


def test_perception_and_guards_never_read_messages():
    from holo_fleet.control import controller, flows, formation_time
    from holo_fleet.ha import automaton, spec
    from holo_fleet.perception import formation_perception, perception

    for mod in (perception, formation_perception, controller, flows, formation_time, spec, automaton):
        src = inspect.getsource(mod)
        for word in ("inbox", "heartbeat", "broadcast", "socket", "message"):
            assert word not in src, (mod.__name__, word)
