# Demo Cookbook

This document collects practical commands for validating the current indoor
drone gas survey demo.

## 1. Start PX4/Gazebo

```bash
cd /home/sefa/Desktop/indoor_drone_gas_flight_demo
./baslat.sh
```

Optional topic check:

```bash
ros2 topic list | grep -Ei "front_scan|left_scan|right_scan|decision_scan|vehicle_local_position"
```

## 2. Static Checks

```bash
python3 -m py_compile src/opening_based_gas_survey_mission.py
python3 -m py_compile src/opening_mission_types.py
python3 -m py_compile src/opening_scan_decision.py
```

## 3. Dry-Run Checks

```bash
python3 src/opening_based_gas_survey_mission.py --dry-run --dry-run-scenario normal_corridor_side_distance
python3 src/opening_based_gas_survey_mission.py --dry-run --dry-run-scenario front_blocked_left_open
```

Expected:

- `normal_corridor_side_distance` -> `FOLLOW_FORWARD`
- `front_blocked_left_open` -> `BYPASS_LEFT`

## 4. Current Room-Facing Mission

Use this for the current room-facing yaw interpolation checkpoint.

```bash
python3 src/opening_based_gas_survey_mission.py \
  --position-room-inspection-check \
  --enable-no-backtrack-door-capture \
  --enable-room-facing-yaw-entry \
  --enable-room-facing-post-yaw-realign \
  --position-step-count 30 \
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
  --room-facing-yaw-hold-before-seconds 0.5 \
  --room-facing-yaw-hold-after-seconds 1.0 \
  --room-facing-yaw-settle-repeat-count 5 \
  --room-facing-yaw-settle-repeat-interval 0.2 \
  --room-facing-yaw-interpolation-step-deg 15 \
  --room-facing-yaw-interpolation-hold-seconds 0.2 \
  --room-facing-post-yaw-min-front-clearance 2.0 \
  --room-facing-post-yaw-forward-offset-step 0.10 \
  --room-facing-post-yaw-max-forward-offset 0.30 \
  --door-capture-confirm-frames 3 \
  --door-capture-crawl-step 0.15 \
  --door-capture-max-crawl-steps 2 \
  --door-capture-hold-seconds 0.8 \
  --max-inspections 1 \
  --gas-scenario possible_gas_zone_4 \
  --gas-seed 1
```

Expected high-level behavior:

- opening candidate is detected
- no-backtrack capture pauses normal stepping
- drone aligns with the door using forward offset and post-yaw realign
- drone turns toward the room using yaw interpolation
- drone enters until front stop distance is reached
- gas measurement is sampled
- drone exits incrementally
- corridor continuation remains stable

## 5. Event Output

The mission writes:

```text
results/opening_inspection_events.json
```

Useful fields to inspect:

- `room_traversal_mode`
- `room_traverse_stop_reason`
- `room_traverse_actual_distance_m`
- `room_facing_final_front_min_m`
- `baseline_avg_ppm`
- `inspection_avg_ppm`
- `delta_ppm`
- `gas_candidate`
- `room_facing_yaw_interpolation_step_deg`
- `room_facing_exit_steps`

## 6. Optional Gas Heatmap Tools

These tools do not command the drone.

```bash
./scripts/run_mapper.sh random
./scripts/run_mapper.sh no_gas
./scripts/run_mapper.sh multi_1_2
./scripts/run_mapper.sh multi_all
```

Optional live ROS2 heatmap:

```bash
python3 src/live_gas_mapping_ros2.py \
  --scenario possible_gas_zone_4 \
  --duration-seconds 30 \
  --sample-rate-hz 5 \
  --seed 1 \
  --hide-source-marker \
  --route-min-altitude 0.5
```

