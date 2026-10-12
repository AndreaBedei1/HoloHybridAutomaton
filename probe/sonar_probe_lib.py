"""Shared helpers for the single-beam sonar probe analysis (ground truth allowed: calibration bench)."""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

import numpy as np

RANGE_MIN, RANGE_MAX, BINS = 0.3, 12.0, 234
BIN_W = (RANGE_MAX - RANGE_MIN) / BINS
RANGES = RANGE_MIN + (np.arange(BINS) + 0.5) * BIN_W
SONAR_X = 0.24
# PoseSensor sits at the BlueROV2 IMUSocket, measured offset from the agent origin in the body frame
IMU_OFFSET = np.array([0.11, 0.0, -0.065])
# BlueROV2 (heavy frame, 457 x 575 x 254 mm) as a box centred on the agent origin: near-face truth.
# The probe's own echoes give 0.225 m ahead and 0.275 m abeam, i.e. consistent within one 5 cm bin.
HULL_HALF = np.array([0.229, 0.288, 0.127])


def load(session_dir: Path):
    rows = [json.loads(line) for line in open(session_dir / "samples.jsonl", encoding="utf-8")]
    for r in rows:
        r["sonar"] = np.asarray(r["sonar"], dtype=float)
        r["obs_pose"] = np.asarray(r["obs_pose"], dtype=float)
        r["tgt_pose"] = np.asarray(r["tgt_pose"], dtype=float)
    return rows


def first_return(profile: np.ndarray, thr: float):
    """Range of the first bin whose intensity exceeds ``thr`` (None = no detection)."""
    idx = np.flatnonzero(profile > thr)
    return None if idx.size == 0 else float(RANGES[idx[0]])


def agent_origin(pose: np.ndarray) -> np.ndarray:
    return pose[:3, 3] - pose[:3, :3] @ IMU_OFFSET


def sonar_origin(obs_pose: np.ndarray) -> np.ndarray:
    return agent_origin(obs_pose) + obs_pose[:3, 0] * SONAR_X


def boresight(obs_pose: np.ndarray) -> np.ndarray:
    return obs_pose[:3, 0] / np.linalg.norm(obs_pose[:3, 0])


def box_surface_distance(p: np.ndarray, centre: np.ndarray, rot: np.ndarray, half: np.ndarray) -> float:
    local = rot.T @ (p - centre)
    q = np.abs(local) - half
    return float(np.linalg.norm(np.maximum(q, 0.0)) + min(max(q.max(), 0.0), 0.0))


def hull_near_distance(obs_pose: np.ndarray, tgt_pose: np.ndarray) -> float:
    """True distance from the sonar origin to the nearest point of the target hull (box model)."""
    return box_surface_distance(sonar_origin(obs_pose), agent_origin(tgt_pose), tgt_pose[:3, :3], HULL_HALF)


def centre_distance(obs_pose: np.ndarray, tgt_pose: np.ndarray) -> float:
    """Distance between the agent origins (what the referee calls d_ij)."""
    return float(np.linalg.norm(agent_origin(tgt_pose) - agent_origin(obs_pose)))


def group(rows, exp: str):
    out = defaultdict(list)
    for r in rows:
        if r["exp"] == exp:
            out[json.dumps(r["case"], sort_keys=True)].append(r)
    return out
