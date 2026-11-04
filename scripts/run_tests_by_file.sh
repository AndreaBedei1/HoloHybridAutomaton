#!/usr/bin/env bash
# Run the unit-test suite one file at a time (a file whose pytest process dies without a summary is retried once).
# Engine tests are excluded: tests/test_run_demo.py and the native octree regression launch HoloOcean.
# Usage: scripts/run_tests_by_file.sh <output dir>
set -u
OUT="${1:?output dir}"
PY="${PY:-C:/Users/andrea.bedei3/.conda/envs/holo_fleet_ha/python.exe}"
cd "$(dirname "$0")/.."
mkdir -p "$OUT"
: > "$OUT/summary.txt"
for f in tests/test_*.py; do
  [ "$f" = "tests/test_run_demo.py" ] && continue
  name=$(basename "$f" .py)
  for attempt in 1 2; do
    "$PY" -m pytest -q -p no:cacheprovider "$f" --deselect tests/test_octree_patch.py::test_octree_rebuild_regression \
        > "$OUT/$name.txt" 2>&1 < /dev/null
    code=$?
    last=$(grep -E "passed|failed|error|no tests ran" "$OUT/$name.txt" | tail -1)
    [ -n "$last" ] && break
  done
  echo "$name exit=$code attempt=$attempt :: ${last:-NO SUMMARY (process died)}" >> "$OUT/summary.txt"
done
echo "ALL DONE" >> "$OUT/summary.txt"
