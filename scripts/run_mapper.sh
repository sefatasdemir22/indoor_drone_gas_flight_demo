#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESULTS_DIR="${PROJECT_DIR}/results"
CSV_FILE="${RESULTS_DIR}/gas_samples.csv"
SCENARIO="${1:-random}"
SAMPLES="${SAMPLES:-220}"
SEED="${SEED:-}"
VMIN="${VMIN:-0}"
VMAX="${VMAX:-130}"

mkdir -p "${RESULTS_DIR}"

echo "[indoor_drone_gas_flight_demo] Simulated gas mapping smoke test"
echo "Project directory: ${PROJECT_DIR}"
echo "Scenario: ${SCENARIO}"
echo "Samples: ${SAMPLES}"
echo "Color scale: vmin=${VMIN}, vmax=${VMAX}"
if [[ -n "${SEED}" ]]; then
  echo "Seed: ${SEED}"
else
  echo "Seed: system random"
fi
echo

SENSOR_CMD=(
  python3 "${PROJECT_DIR}/src/demo_tools/gas_sensor_node.py"
  --scenario "${SCENARIO}"
  --samples "${SAMPLES}"
  --output "${CSV_FILE}"
)

if [[ -n "${SEED}" ]]; then
  SENSOR_CMD+=(--seed "${SEED}")
fi

"${SENSOR_CMD[@]}"

echo
if python3 "${PROJECT_DIR}/src/demo_tools/gas_mapper_node.py" \
  --input "${CSV_FILE}" \
  --output "${RESULTS_DIR}/gas_heatmap.png" \
  --vmin "${VMIN}" \
  --vmax "${VMAX}"; then
  echo "Heatmap output: ${RESULTS_DIR}/gas_heatmap.png"
  echo "Scenario heatmaps directory: ${RESULTS_DIR}/heatmaps"
else
  echo
  echo "Heatmap generation failed, but CSV generation completed."
  echo "This is usually a Matplotlib/NumPy dependency issue, not a gas simulation error."
fi
