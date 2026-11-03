#!/usr/bin/env bash
# Targeted development diagnostics (exploratory evidence, not results): the representative cases of every diagnosed
# mechanism (analysis/2026-11-03_B2_REVIEW_AND_PLAN.md, Section H.2), one HoloOcean at a time, low priority (the
# orchestrator lowers its own priority, be_nice).  Usage: scripts/run_dev_diagnostics.sh <raw root> [seed offset]
set -u
RAW="${1:?raw root}"
PY="${PY:-C:/Users/andrea.bedei3/.conda/envs/holo_fleet_ha/python.exe}"
cd "$(dirname "$0")/.."
CASES=(
  "B2_MEDIUM MEDIUM-ALL formation_six 1"
  "B2_MEDIUM MEDIUM-ONE formation_square 2"
  "B2_CLEAN CLEAN gate_single 0"
  "B2_CLEAN CLEAN lost_drone_mutex 0"
  "B2_MEDIUM MEDIUM-ONE gate_single 0"
  "B2_MEDIUM MEDIUM-SUBSET lost_drone_mutex 0"
  "B2_MEDIUM MEDIUM-SUBSET integrated_short 2"
  "B2_LOW LOW-ALL mutex_deadlock_resolution 2"
  "B2_MEDIUM MEDIUM-SUBSET gate_single 1"
  "B2_MEDIUM MEDIUM-SUBSET lost_drone_timeout 2"
  "B2_BLACKOUT MEDIUM-ALL formation_six 2"
)
for c in "${CASES[@]}"; do
  read -r proto prof sc seed <<< "$c"
  "$PY" scripts/run_sensor_robustness_campaign.py --phase targeted --only "$sc" --protocols "$proto" \
      --profiles "$prof" --seeds "$seed" --raw "$RAW" --cooldown-s 2
done
