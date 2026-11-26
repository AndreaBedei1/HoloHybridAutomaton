"""Explicit multi-engine isolation probe (2 to 4 engines), after the single-engine scenario has passed.

Each native client must receive its own uniquely named sensor, while both engines are alive.
This is infrastructure validation, not a P1/P2/P3 experiment. No engine tests invoke it implicitly.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import sys
import threading

import psutil

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.config import DEFAULT
from holo_fleet.sim.engine_lease import owned_engine
from holo_fleet.sim.engine_session import make
from holo_fleet.sim.holo_env import TICKS_PER_SEC
from scripts.run_sensor_robustness_campaign import be_nice


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--instances", type=int, choices=(2, 3, 4), default=2)
    args = ap.parse_args()
    be_nice()
    args.out.mkdir(parents=True, exist_ok=True)
    n = args.instances
    barrier = threading.Barrier(n)
    before = os.environ.get("HOLO_FLEET_ENGINE_SLOTS")
    os.environ["HOLO_FLEET_ENGINE_SLOTS"] = str(n)

    labels = ["left", "right", "front", "back"][:n]

    def probe(label, x):
        agent, sensor = "isolation_" + label, label + "_pose"
        scenario = {"name": "isolation_" + label, "world": "OpenWater", "package_name": "Ocean",
                    "main_agent": agent, "ticks_per_sec": TICKS_PER_SEC, "frames_per_sec": False,
                    "agents": [{"agent_name": agent, "agent_type": "BlueROV2", "location": [x, 0, -5],
                                "control_scheme": 0, "sensors": [{"sensor_type": "PoseSensor", "sensor_name": sensor,
                                                                "socket": "IMUSocket", "Hz": TICKS_PER_SEC}]}]}
        try:
            with make(scenario_cfg=scenario, show_viewport=False, ticks_per_sec=TICKS_PER_SEC,
                      frames_per_sec=False) as env:
                record = dict(env._holo_engine)
                engine = owned_engine(record["engine"])
                assert engine is not None
                engine.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS if os.name == "nt" else 10)
                if os.name == "nt":
                    engine.ionice(psutil.IOPRIO_LOW)
                command = engine.cmdline()
                assert [a for a in command if a.startswith("--HolodeckUUID=")] == ["--HolodeckUUID=" + env._uuid]
                assert set(env.agents) == {agent}
                record.update(label=label, scenario=scenario["name"], agent=agent, sensor=sensor, client_uuid=env._uuid)
                (args.out / (label + "_ready.json")).write_text(json.dumps(record, indent=1), encoding="utf-8")
                barrier.wait(timeout=100)
                for _ in range(round(DEFAULT.beacon.period_s * TICKS_PER_SEC)):
                    raw = env.tick()
                    frame = raw.get(agent, raw)
                    assert sensor in frame and not any(o + "_pose" in frame for o in labels if o != label), (label, list(raw))
                record["sensor_keys"] = list(frame)
                record["checks_passed"] = True
                return record
        except BaseException:
            barrier.abort()
            raise

    try:
        with ThreadPoolExecutor(max_workers=n) as pool:
            futures = [pool.submit(probe, label, -30 + 20 * k) for k, label in enumerate(labels)]
            records = [future.result() for future in futures]
        assert len({r["client_uuid"] for r in records}) == n
        assert len({r["root"] for r in records}) == n
        ports = [p for r in records for p in r["listen_ports"]]
        assert len(ports) == len(set(ports))
        assert all(1985 not in r["listen_ports"] for r in records)
        result = {"all_passed": True, "instances": records}
        (args.out / "isolation.json").write_text(json.dumps(result, indent=1), encoding="utf-8")
        print(json.dumps(result, indent=1), flush=True)
    finally:
        if before is None:
            os.environ.pop("HOLO_FLEET_ENGINE_SLOTS", None)
        else:
            os.environ["HOLO_FLEET_ENGINE_SLOTS"] = before


if __name__ == "__main__":
    main()
