#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO="${1:-possible_gas_zone_4}"
SEED="${2:-1}"

if [[ "${SCENARIO}" == "-h" || "${SCENARIO}" == "--help" ]]; then
  cat <<'EOF'
Usage:
  ./scripts/run_demo_mission.sh [SCENARIO] [SEED]

Runs the canonical validated inspect-all room mission profile.

Defaults:
  SCENARIO=possible_gas_zone_4
  SEED=1

Examples:
  ./scripts/run_demo_mission.sh
  ./scripts/run_demo_mission.sh no_gas 1
  ./scripts/run_demo_mission.sh multi_1_2 1

Gazebo/PX4/MicroXRCEAgent must already be running.
This script does not run post-process exporters or archive experiment outputs.
EOF
  exit 0
fi

# ROS setup scripts are not nounset-safe; restore nounset immediately after sourcing.
set +u
source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"
source "${HOME}/araswarm_ws/install/setup.bash"
set -u

cd "${PROJECT_DIR}"

echo "[demo] canonical inspect-all room mission profile"
echo "[demo] scenario=${SCENARIO}"
echo "[demo] seed=${SEED}"
echo "[demo] Gazebo/PX4/MicroXRCEAgent must already be running."

python3 src/opening_based_gas_survey_mission.py \
  --position-room-inspection-check \
  --enable-no-backtrack-door-capture \
  --enable-room-facing-yaw-entry \
  --enable-room-facing-post-yaw-realign \
  --enable-position-return-home \
  --enable-position-landing-stabilization \
  --inspect-all-openings \
  --max-inspections 5 \
  --position-step-count 35 \
  --position-forward-step 0.4 \
  --position-hold-seconds 2.5 \
  --position-altitude 1.2 \
  --position-yaw 90.0 \
  --room-facing-step-distance 0.25 \
  --room-facing-max-distance 8.0 \
  --room-facing-front-stop-distance 1.5 \
  --room-facing-exit-step-distance 0.5 \
  --room-facing-step-hold-seconds 0.8 \
  --room-facing-door-forward-offset 0.25 \
  --room-facing-yaw-interpolation-step-deg 15 \
  --room-facing-yaw-interpolation-hold-seconds 0.2 \
  --position-opening-same-side-suppression-distance 2.0 \
  --position-return-step-distance 0.6 \
  --position-return-hold-seconds 1.2 \
  --position-return-arrival-tolerance 0.35 \
  --position-return-max-distance 20.0 \
  --position-landing-final-altitude 0.22 \
  --position-landing-step-hold-seconds 2.0 \
  --position-landing-final-hold-seconds 2.0 \
  --gas-scenario "${SCENARIO}" \
  --gas-seed "${SEED}"
