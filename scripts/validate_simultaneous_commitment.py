"""Run the isolated line3 simultaneous-commitment experiment."""

import argparse
import json
from pathlib import Path

from holo_fleet import runner
from holo_fleet.control.controller import DroneController
from holo_fleet.ha.spec import Mode
from holo_fleet.sim import engine_lease


class ConcurrentCommitment(DroneController):
    injected = False

    def step(self, frame, dt):
        if not self.injected and self.plan.slot_index < 2 and frame.t >= 0.8:
            assert not self.ha.committed
            self.ha.committed = True
            self.ha.mode = Mode.MUTEX_PASS
            self.events.append({"type": "TEST_COMMIT_INJECTION", "t": frame.t})
            self.injected = True
        return super().step(frame, dt)


def main():
    from scripts.run_sensor_robustness_campaign import be_nice

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path)
    parser.add_argument("--slots", type=Path, required=True, help="shared engine slot registry")
    args = parser.parse_args()
    engine_lease.SLOTS = args.slots
    be_nice()
    runner.DroneController = ConcurrentCommitment
    name = "line3_simultaneous_commit"
    runner.run("line_parallel_mutex", args.out, run_id=name, frame_every_s=float("inf"), seed=0, duration=110.0)

    run = args.out / name
    events = [json.loads(line) for line in (run / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    injections = [(event["drone"], event["t"]) for event in events if event["type"] == "TEST_COMMIT_INJECTION"]
    assert len(injections) == 2 and injections[0][1] == injections[1][1], injections
    config = json.loads((run / "run_config.json").read_text(encoding="utf-8"))
    ports = config["setup"]["engine"]["listen_ports"]
    assert ports and 1985 not in ports, ports
    metrics = json.loads((run / "referee_metrics.json").read_text(encoding="utf-8"))
    print(injections, metrics["P2_mutual_exclusion"], flush=True)


if __name__ == "__main__":
    main()
