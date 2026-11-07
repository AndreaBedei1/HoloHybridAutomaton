#!/usr/bin/env bash
# Exploratory reruns at the v5 horizon (3 x the a-priori deadline), or a multiple of it, of a list of "protocol profile scenario seed"
# lines, on two guarded engine slots: does a run that missed the deadline complete eventually?
set -u
LIST="${1:?targets}"; RAW="${2:?raw root}"; FACTOR="${3:-1}"   # a multiple of the v5 horizon (3 x deadline)
PY="${PY:-C:/Users/andrea.bedei3/.conda/envs/holo_fleet_ha/python.exe}"
cd "$(dirname "$0")/.."
i=0
while read -r proto prof sc seed; do
  [ -z "$proto" ] && continue
  shard=$((i % 2)); i=$((i + 1))
  echo "$proto $prof $sc $seed" >> "$RAW.shard$shard.list"
done < "$LIST"
for shard in 0 1; do
  ( while read -r proto prof sc seed; do
      HOLO_FLEET_ENGINE_SLOTS=2 HOLO_FLEET_HORIZON_FACTOR="$FACTOR" "$PY" scripts/run_sensor_robustness_campaign.py \
          --phase targeted --only "$sc" --protocols "$proto" --profiles "$prof" --seeds "$seed" --raw "$RAW" \
          --cooldown-s 2 < /dev/null
    done < "$RAW.shard$shard.list" > "$RAW.shard$shard.log" 2>&1 ) &
done
wait
echo "ALL DONE" > "$RAW.done"
