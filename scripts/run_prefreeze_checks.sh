#!/usr/bin/env bash
# Pre-freeze verification: the unit suite file by file, then the full formal suite (formal/check_properties.py).
set -u
OUT="${1:?output dir}"
PY="${PY:-C:/Users/andrea.bedei3/.conda/envs/holo_fleet_ha/python.exe}"
cd "$(dirname "$0")/.."
mkdir -p "$OUT"
bash scripts/run_tests_by_file.sh "$OUT/tests" > /dev/null 2>&1
"$PY" formal/check_properties.py > "$OUT/formal_all.txt" 2>&1 < /dev/null
echo "exit $?" >> "$OUT/formal_all.txt"
echo "ALL DONE" > "$OUT/done"
