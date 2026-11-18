#!/usr/bin/env bash
# Night batch of the review (commit 6c6a6b1): four engine slots, four shards per batch, below-normal priority.
cd /f/Andrea/HoloHybridAutomaton || exit 1
PY=/c/Users/andrea.bedei3/.conda/envs/holo_fleet_ha/python.exe
export HOLO_FLEET_ENGINE_SLOTS=4
GATE="gate_single mutex_deadlock_resolution line_parallel_mutex lost_drone_mutex st_lost_middle integrated_short"
S5="7300 7301 7302 7303 7304"
quad() {  # $1 log name, rest: arguments
  local name=$1; shift
  for i in 0 1 2 3; do
    $PY scripts/run_sensor_robustness_campaign.py --phase targeted "$@" --shard $i --nshards 4 > results/review/${name}_shard$i.log 2>&1 &
  done
  wait
  echo "$name done $(date)" >> results/review/queue_night_progress.txt
}
quad heldout_b2 --only $GATE --protocols B2_MEDIUM --profiles MEDIUM-ALL --seeds $S5 --raw results/review/raw_heldout
quad heldout_b0 --only $GATE --protocols B0 --profiles LOW-ALL --seeds $S5 --raw results/review/raw_heldout
quad heldout_s5201 --only st_lost_middle --protocols B2_MEDIUM --profiles MEDIUM-ALL --seeds 7310 7311 7312 7313 7314 7315 7316 7317 7318 7319 --raw results/review/raw_heldout
quad latency --only gate_single lost_drone_mutex --protocols B2_LOW B2_SW-lat0.35 B2_SW-lat0.45 --profiles LOW-ALL --seeds $S5 --raw results/review/raw_latency
quad four_b2 --only gate_four --protocols B2_MEDIUM --profiles MEDIUM-ALL --seeds 7400 7401 7402 7403 7404 --raw results/review/raw_four
quad four_b0 --only gate_four --protocols B0 --profiles LOW-ALL --seeds 7400 7401 7402 7403 7404 --raw results/review/raw_four
echo "QUEUE DONE $(date)" > results/review/queue_done.txt
