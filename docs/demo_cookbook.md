# Demo Cookbook

This document collects practical commands for validating the current indoor
drone gas survey demo.

Current milestone:

- Git tag: `milestone_inspect_all_v1`
- Main mode: position-based inspect-all room inspection
- Main output: `results/opening_inspection_events.json`

## 1. Start PX4/Gazebo

```bash
cd ~/Desktop/indoor_drone_gas_flight_demo
./baslat.sh
```

Optional topic check:

```bash
ros2 topic list | grep -Ei "front_scan|left_scan|right_scan|decision_scan|vehicle_local_position"
```

## 2. Static Checks

```bash
python3 -m py_compile src/opening_based_gas_survey_mission.py
python3 -m py_compile src/opening_event_builder.py
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

## 4. Current Inspect-All Mission

Use this for the current multi-room validation checkpoint.

```bash
python3 src/opening_based_gas_survey_mission.py \
  --position-room-inspection-check \
  --inspect-all-openings \
  --enable-no-backtrack-door-capture \
  --enable-room-facing-yaw-entry \
  --enable-room-facing-post-yaw-realign \
  --enable-position-return-home \
  --enable-position-landing-stabilization \
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
  --position-return-step-distance 0.6 \
  --position-return-hold-seconds 1.2 \
  --position-return-arrival-tolerance 0.35 \
  --position-return-max-distance 20.0 \
  --position-landing-final-altitude 0.22 \
  --position-landing-step-hold-seconds 2.0 \
  --position-landing-final-hold-seconds 2.0 \
  --max-inspections 5 \
  --gas-scenario possible_gas_zone_4 \
  --gas-seed 1
```

Expected high-level behavior:

- opening candidates are detected while progressing down the corridor
- no-backtrack capture pauses normal stepping near each door
- same-side suppression prevents re-inspecting the same opening continuation
- the drone turns toward each room using yaw interpolation
- room traversal stops on front distance
- gas baseline and inspection samples are recorded
- room exit is incremental
- corridor continuation remains stable after each inspection
- return-home completes near local home
- staged landing stabilization reduces touchdown bounce

## 5. Event Output

The mission writes:

```text
results/opening_inspection_events.json
```

Useful fields to inspect:

- `events`
- `return_home`
- `side`
- `opening_id`
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
- `return_home.status`
- `return_home.final_distance_to_home_m`

Quick validation checklist:

- `events` contains one valid event per distinct inspected room.
- `opening_id` values are not duplicated for the same physical opening.
- `room_traverse_stop_reason` is typically `front_stop_distance`.
- `return_home.status` is `completed`.
- Landing logs show staged descent to the configured final altitude.

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
