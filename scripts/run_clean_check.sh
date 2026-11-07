#!/usr/bin/env bash
# CLEAN liveness check before the freeze: every scenario, B2_CLEAN / CLEAN, seed 0, two guarded engine slots.
set -u
RAW="${1:?raw root}"
PY="${PY:-C:/Users/andrea.bedei3/.conda/envs/holo_fleet_ha/python.exe}"
cd "$(dirname "$0")/.."
SC=$("$PY" -c "import sys; sys.path.insert(0, 'scripts'); import run_sensor_robustness_campaign as c; print(' '.join(c.ALL_SCENARIOS))")
for shard in 0 1; do
  HOLO_FLEET_ENGINE_SLOTS=2 "$PY" scripts/run_sensor_robustness_campaign.py --phase targeted --only $SC --protocols B2_CLEAN \
      --profiles CLEAN --seeds 0 --raw "$RAW" --cooldown-s 2 --shard $shard --nshards 2 > "$RAW.shard$shard.log" 2>&1 &
done
wait
echo "ALL DONE" >> "$RAW.done"
