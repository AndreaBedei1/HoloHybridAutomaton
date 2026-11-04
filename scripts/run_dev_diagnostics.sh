#!/usr/bin/env bash
# Targeted development diagnostics (exploratory evidence, not results): the representative cases of every diagnosed
# mechanism (analysis/2026-11-03_B2_REVIEW_AND_PLAN.md, Section H.2), at most two guarded HoloOcean instances,
# low priority (be_nice). Usage: <raw root> [shard] [nshards] [repetitions]
set -u
RAW="${1:?raw root}"
SHARD="${2:-0}"
NSHARDS="${3:-1}"
REPETITIONS="${4:-1}"
if (( NSHARDS < 1 || NSHARDS > 2 || SHARD < 0 || SHARD >= NSHARDS )); then
  echo "use one or two shards, numbered from zero" >&2
  exit 2
fi
(( REPETITIONS >= 1 )) || exit 2
PY="${PY:-C:/Users/andrea.bedei3/.conda/envs/holo_fleet_ha/python.exe}"
cd "$(dirname "$0")/.."
CASES=(
  "B2_LOW LOW-ALL mutex_deadlock_resolution 2"
  "B2_MEDIUM MEDIUM-SUBSET integrated_short 2"
  "B2_MEDIUM MEDIUM-SUBSET gate_single 1"
  "B2_MEDIUM MEDIUM-ALL formation_six 1"
  "B2_MEDIUM MEDIUM-ONE formation_square 2"
  "B2_CLEAN CLEAN gate_single 0"
  "B2_CLEAN CLEAN lost_drone_mutex 0"
  "B2_MEDIUM MEDIUM-ONE gate_single 0"
  "B2_MEDIUM MEDIUM-SUBSET lost_drone_mutex 0"
  "B2_MEDIUM MEDIUM-SUBSET lost_drone_timeout 2"
  "B2_BLACKOUT MEDIUM-ALL formation_six 2"
)
for i in "${!CASES[@]}"; do
  (( i % NSHARDS == SHARD )) || continue
  read -r proto prof sc seed <<< "${CASES[$i]}"
  for ((rep=0; rep<REPETITIONS; rep++)); do
    DEST="$RAW"
    (( REPETITIONS > 1 )) && DEST="$RAW/r$rep"
    "$PY" scripts/run_sensor_robustness_campaign.py --phase targeted --only "$sc" --protocols "$proto" \
        --profiles "$prof" --seeds "$seed" --raw "$DEST" --cooldown-s 2
  done
done
