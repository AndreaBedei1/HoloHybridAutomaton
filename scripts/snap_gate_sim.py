"""One HoloOcean frame of gate G06 as spawned (arena_bridge.GATE_INNER_SIZE_M), with a BlueROV2 in the opening for scale.

The camera drone holds 4.5 m before the gate on its axis (camera on its nose, looking along the passage); a second
drone holds at the gate centre.  Low priority, one engine, closed at the end.  Usage: snap_gate_sim.py OUT.png
"""

import math
import sys

import cv2
import numpy as np
import psutil

from holo_fleet.arena_bridge import HORSESHOE_TRACK, load_arena_gates
from holo_fleet.sim.holo_env import HoloFleetSim
from holo_fleet.sim.spec import bench_spec


def _nice():
    for p in [psutil.Process()] + psutil.Process().children(recursive=True):
        try:
            p.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
        except psutil.Error:
            pass


def main(out):
    _nice()
    g6 = {g.gate_id: g for g in load_arena_gates(HORSESHOE_TRACK)}["G06"]
    yaw = math.degrees(math.atan2(g6.axis[1], g6.axis[0]))
    cam = g6.center - 4.5 * np.asarray(g6.axis, float)
    inside = np.asarray(g6.center, float)
    spec = bench_spec("snap_gate", [cam, inside], yaws=[yaw, yaw], gate_ids=("G06",), camera_drone=0)
    spec.chase_offset = (0.35, 0.0, 0.25)
    sim = HoloFleetSim(spec, headless=True)
    sim.start()
    _nice()
    try:
        for _ in range(15):
            sim.pin(sim.names[0], cam, (0, 0, yaw))
            sim.pin(sim.names[1], inside, (0, 0, yaw))
            sim.step({n: None for n in sim.names}, 0.1)
        img = sim.image("ChaseCamera")
        if img is None:
            raise RuntimeError("no camera frame")
        img = np.ascontiguousarray(img[:, :, :3])
        cv2.putText(img, f"G06 {g6.inner_width:.1f} x {g6.inner_height:.1f} m opening, BlueROV2 at the gate centre "
                    "(camera 4.5 m before the gate)", (12, img.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                    cv2.LINE_AA)
        cv2.imwrite(out, img)
        print(out, img.shape)
    finally:
        sim.close()


if __name__ == "__main__":
    main(sys.argv[1])
