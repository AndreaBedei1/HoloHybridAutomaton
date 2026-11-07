#!/usr/bin/env bash
# CLEAN liveness check on the candidate commit, then the unit suite and the full formal suite (sequential).
set -u
TAG="${1:?tag}"
cd "$(dirname "$0")/.."
bash scripts/run_clean_check.sh "results/robustness/raw_dev/clean_$TAG"
bash scripts/run_prefreeze_checks.sh "results/dev/prefreeze_$TAG"
