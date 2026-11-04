#!/usr/bin/env bash
# Development batch after a code change: the unit suite file by file, the quick formal checks, then the targeted
# HoloOcean diagnostics on two guarded engine slots (low priority, scripts/run_dev_diagnostics.sh) and the payload
# ablation (B2NP_) of the gate cases.  Sequential phases: no test suite runs while an engine runs.
set -u
TAG="${1:?tag}"
PY="${PY:-C:/Users/andrea.bedei3/.conda/envs/holo_fleet_ha/python.exe}"
cd "$(dirname "$0")/.."
OUT="results/dev/$TAG"
mkdir -p "$OUT"
bash scripts/run_tests_by_file.sh "$OUT/tests" > /dev/null 2>&1
for c in determinism observations acoustic; do "$PY" formal/check_$c.py > "$OUT/formal_$c.txt" 2>&1 < /dev/null; done
git checkout formal/results/formation.json 2>/dev/null
echo "tests+formal done $(date +%T)" >> "$OUT/progress.txt"
RAW="results/robustness/raw_dev/$TAG"
HOLO_FLEET_ENGINE_SLOTS=2 bash scripts/run_dev_diagnostics.sh "$RAW" 0 2 1 > "$OUT/diag0.log" 2>&1 &
HOLO_FLEET_ENGINE_SLOTS=2 bash scripts/run_dev_diagnostics.sh "$RAW" 1 2 1 > "$OUT/diag1.log" 2>&1 &
wait
echo "diagnostics done $(date +%T)" >> "$OUT/progress.txt"
for c in "B2NP_CLEAN CLEAN gate_single 0" "B2NP_MEDIUM MEDIUM-SUBSET lost_drone_mutex 0"; do
  read -r proto prof sc seed <<< "$c"
  HOLO_FLEET_ENGINE_SLOTS=2 "$PY" scripts/run_sensor_robustness_campaign.py --phase targeted --only "$sc" --protocols "$proto" \
      --profiles "$prof" --seeds "$seed" --raw "$RAW" --cooldown-s 2 >> "$OUT/ablation.log" 2>&1 < /dev/null &
done
wait
echo "ALL DONE $(date +%T)" >> "$OUT/progress.txt"
