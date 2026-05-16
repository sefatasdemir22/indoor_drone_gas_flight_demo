#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESULTS_DIR="${PROJECT_DIR}/results"
HEATMAP_DIR="${RESULTS_DIR}/heatmaps"
SCENARIO="${1:-random}"

trap 'echo; echo "[run_full_demo] Error on line ${LINENO}. Check the message above for details."; exit 1' ERR

echo "[indoor_drone_gas_flight_demo] Full simulated demo"
echo "Project directory: ${PROJECT_DIR}"
echo "Scenario: ${SCENARIO}"
echo
echo "This script does not start Gazebo, PX4, MAVSDK, or a real drone."
echo "It runs the simulated mission FSM, then generates gas mapping outputs."
echo

echo "== 1/2 Simulated mission FSM =="
python3 "${PROJECT_DIR}/src/mission_manager.py" --mode sim --sleep 0

echo
echo "== 2/2 Simulated gas mapping =="
"${PROJECT_DIR}/scripts/run_mapper.sh" "${SCENARIO}"

echo
echo "== Demo outputs =="
for output_file in \
  "${RESULTS_DIR}/gas_samples.csv" \
  "${RESULTS_DIR}/scenario_info.json" \
  "${RESULTS_DIR}/gas_heatmap.png"; do
  if [[ -f "${output_file}" ]]; then
    ls -lh "${output_file}"
  else
    echo "Missing: ${output_file}"
  fi
done

echo
echo "Scenario heatmaps:"
if [[ -d "${HEATMAP_DIR}" ]]; then
  find "${HEATMAP_DIR}" -maxdepth 1 -type f -name '*.png' -printf '%T@ %p\n' \
    | sort -nr \
    | head -n 5 \
    | cut -d' ' -f2- \
    | while IFS= read -r heatmap_file; do
        ls -lh "${heatmap_file}"
      done
else
  echo "No heatmap directory yet: ${HEATMAP_DIR}"
fi

echo
if command -v xdg-open >/dev/null 2>&1; then
  echo "To open the latest heatmap manually:"
  echo "  xdg-open ${RESULTS_DIR}/gas_heatmap.png"
fi

echo
echo "Full simulated demo complete."
