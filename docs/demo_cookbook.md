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

Use the canonical launcher for the current multi-room validation checkpoint.
Gazebo/PX4/MicroXRCEAgent must already be running.

```bash
./scripts/run_demo_mission.sh possible_gas_zone_4 1
```

The validated mission parameters are frozen in `scripts/run_demo_mission.sh` to
avoid command drift across experiments. Use script arguments only for scenario
and seed changes, for example `./scripts/run_demo_mission.sh no_gas 1`.

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
