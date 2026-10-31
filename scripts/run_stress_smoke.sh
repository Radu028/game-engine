#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${1:-build}"
SMOKE_BIN="${BUILD_DIR}/SmokeFlowTest"
RUNS="${RUNS:-8}"
STEPS="${STEPS:-24000}"
OBSTACLE_SETS="${OBSTACLE_SETS:-24}"
SEED_BASE="${SEED_BASE:-1000}"

if [[ ! -x "${SMOKE_BIN}" ]]; then
  echo "Smoke binary not found: ${SMOKE_BIN}"
  echo "Build first: /opt/homebrew/bin/cmake -S . -B ${BUILD_DIR} && /opt/homebrew/bin/cmake --build ${BUILD_DIR}"
  exit 1
fi

echo "Running stress smoke: runs=${RUNS}, steps=${STEPS}, obstacle_sets=${OBSTACLE_SETS}"
for ((i = 1; i <= RUNS; ++i)); do
  seed=$((SEED_BASE + i))
  echo "[${i}/${RUNS}] ${SMOKE_BIN} --seed ${seed}"
  "${SMOKE_BIN}" --steps "${STEPS}" --obstacle-sets "${OBSTACLE_SETS}" --seed "${seed}"
done

echo "Stress smoke completed successfully."
