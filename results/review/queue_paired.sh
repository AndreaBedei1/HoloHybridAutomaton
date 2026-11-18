#!/usr/bin/env bash
# Paired v11 (no gate guard) on the held-out st_lost_middle seeds: is the concurrent-commit rate a cost of the guard?
cd /f/Andrea/HoloHybridAutomaton || exit 1
PY=/c/Users/andrea.bedei3/.conda/envs/holo_fleet_ha/python.exe
export HOLO_FLEET_ENGINE_SLOTS=4
for i in 0 1 2 3; do
  $PY scripts/run_sensor_robustness_campaign.py --phase targeted --only st_lost_middle --protocols B2NG_MEDIUM --profiles MEDIUM-ALL \
    --seeds 7300 7301 7302 7303 7304 7310 7311 7312 7313 7314 7315 7316 7317 7318 7319 --raw results/review/raw_paired_ng \
    --shard $i --nshards 4 > results/review/paired_ng_shard$i.log 2>&1 &
done
wait
echo "PAIRED DONE $(date)" > results/review/paired_done.txt
