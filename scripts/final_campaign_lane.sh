#!/usr/bin/env bash
# One lane of a targeted batch: for every "protocol profile scenario seed" line of the list, try the run (claims avoid
# duplicates between lanes). Args: worktree, patched root, raw, list, lane tag
set -u
WT="$1"; ROOT="$2"; RAW="$3"; LIST="$4"; TAG="$5"
PY="C:/Users/andrea.bedei3/.conda/envs/holo_fleet_ha/python.exe"
cd "$WT"
mkdir -p "$RAW"
while read -r proto prof sc seed; do
  [ -z "$proto" ] && continue
  HOLO_FLEET_ENGINE_SLOTS=2 HOLO_FLEET_HOLOOCEAN_ROOT="$ROOT" "$PY" scripts/run_sensor_robustness_campaign.py \
     --phase targeted --only "$sc" --protocols "$proto" --profiles "$prof" --seeds "$seed" --raw "$RAW" --cooldown-s 3 < /dev/null
done < "$LIST" > "$RAW/lane_$TAG.log" 2>&1
echo "LANE DONE $(date +%H:%M:%S)" >> "$RAW/lane_$TAG.log"
