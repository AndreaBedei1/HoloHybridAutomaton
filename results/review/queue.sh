#!/usr/bin/env bash
# Review experiments after the adversarial batch, two engine slots, from the committed code.
cd /f/Andrea/HoloHybridAutomaton || exit 1
PY=/c/Users/andrea.bedei3/.conda/envs/holo_fleet_ha/python.exe
export HOLO_FLEET_ENGINE_SLOTS=2
while powershell -c "@(Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | Where-Object { \$_.CommandLine -like '*raw_adversarial*' }).Count" | grep -qv '^0'; do sleep 60; done
GATE="gate_single mutex_deadlock_resolution line_parallel_mutex lost_drone_mutex st_lost_middle integrated_short"
S5="7300 7301 7302 7303 7304"
pair() {  # $1 log name, rest: arguments
  local name=$1; shift
  for i in 0 1; do
    $PY scripts/run_sensor_robustness_campaign.py --phase targeted "$@" --shard $i --nshards 2 > results/review/${name}_shard$i.log 2>&1 &
  done
  wait
}
pair heldout_b2 --only $GATE --protocols B2_MEDIUM --profiles MEDIUM-ALL --seeds $S5 --raw results/review/raw_heldout
pair heldout_b0 --only $GATE --protocols B0 --profiles LOW-ALL --seeds $S5 --raw results/review/raw_heldout
pair heldout_s5201 --only st_lost_middle --protocols B2_MEDIUM --profiles MEDIUM-ALL --seeds 7310 7311 7312 7313 7314 7315 7316 7317 7318 7319 --raw results/review/raw_heldout
pair latency --only gate_single lost_drone_mutex --protocols B2_LOW B2_SW-lat0.35 B2_SW-lat0.45 --profiles LOW-ALL --seeds $S5 --raw results/review/raw_latency
pair four_b2 --only gate_four --protocols B2_MEDIUM --profiles MEDIUM-ALL --seeds 7400 7401 7402 7403 7404 --raw results/review/raw_four
pair four_b0 --only gate_four --protocols B0 --profiles LOW-ALL --seeds 7400 7401 7402 7403 7404 --raw results/review/raw_four
echo "QUEUE DONE $(date)" > results/review/queue_done.txt
