# Opening-Based Gas Survey Mission Checkpoint

This document summarizes the current working checkpoint for the opening-based
indoor drone gas survey prototype. It is a development checkpoint, not a final
full-autonomy claim.

## Current Checkpoint

- Git checkpoint: `ce86e82 Add inspect-all openings mission mode`
- Git tag: `milestone_inspect_all_v1`
- Main mission script: `src/opening_based_gas_survey_mission.py`
- Main output: `results/opening_inspection_events.json`

## Working Capabilities

- Starts PX4/Gazebo indoor corridor-room world with `./baslat.sh`.
- Connects through MAVSDK and runs position-mode corridor steps.
- Reads wide and decision LaserScan topics for opening and safety logic.
- Detects candidate openings from side decision scans.
- Uses no-backtrack door capture to avoid passing the door before inspection.
- Applies same-side suppression so a wide opening continuation is not inspected twice.
- Applies a small door forward offset before room-facing entry.
- Turns toward the room with yaw interpolation and repeated yaw setpoints.
- Performs post-yaw realignment if front clearance is low.
- Enters the room in room-facing mode, using front scan as the room-direction sensor.
- Stops room traversal when front distance reaches the configured stop distance.
- Samples gas baseline and inspection ppm.
- Exits the room incrementally.
- Returns to corridor yaw and continues corridor progression.
- Supports inspect-all mode, where `--max-inspections` is a safety cap.
- Returns near the local home anchor after the corridor mission.
- Uses staged position landing stabilization before sending the land command.

## Validated Behavior

Recent validation accepted as successful:

- inspect-all mode stayed stable
- multi-room inspection worked in the 3-room demo world
- same-side suppression prevented repeated inspection of the same opening
- left and right room-facing yaw geometry worked
- no-backtrack capture worked
- door forward offset worked
- post-yaw realignment worked
- room-facing yaw entry worked
- yaw interpolation made turns smoother
- front-distance stop triggered inside rooms
- gas measurement was written to event JSON
- incremental exit worked
- corridor continuation after each inspection stayed stable
- return-home completed near local home
- staged landing reduced or removed bounce/hop behavior

Representative event fields from the current checkpoint:

- `room_traversal_mode="room_facing_yaw"`
- `room_traverse_stop_reason="front_stop_distance"`
- `room_facing_yaw_interpolation_step_deg=15.0`
- `room_facing_yaw_interpolation_hold_seconds=0.2`
- `room_facing_exit_steps`
- `baseline_avg_ppm`
- `inspection_avg_ppm`
- `delta_ppm`
- `gas_candidate`

Representative return-home fields:

- `return_home.enabled=true`
- `return_home.attempted=true`
- `return_home.status="completed"`
- `return_home.final_distance_to_home_m`

## Main Demo Command

Start the simulator:

```bash
cd /home/sefa/Desktop/indoor_drone_gas_flight_demo
./baslat.sh
```

Run the current inspect-all validation mission:

```bash
./scripts/run_demo_mission.sh possible_gas_zone_4 1
```

The validated command profile is stored in `scripts/run_demo_mission.sh`.
Do not retype the long mission command for experiments; pass only the scenario
and seed to the launcher.

## Dry-Run Checks

These checks do not start PX4/Gazebo and do not command the drone:

```bash
python3 -m py_compile src/opening_based_gas_survey_mission.py
python3 -m py_compile src/opening_event_builder.py
python3 -m py_compile src/opening_mission_types.py
python3 -m py_compile src/opening_scan_decision.py
python3 src/opening_based_gas_survey_mission.py --dry-run --dry-run-scenario normal_corridor_side_distance
python3 src/opening_based_gas_survey_mission.py --dry-run --dry-run-scenario front_blocked_left_open
```

Expected dry-run behavior:

- `normal_corridor_side_distance` should produce `FOLLOW_FORWARD`.
- `front_blocked_left_open` should produce `BYPASS_LEFT`.

## Known Limitations

- No SLAM, frontier exploration, global planner, or map integration yet.
- `position_step_count` is still the corridor mission boundary.
- `--max-inspections` remains a safety cap in inspect-all mode.
- Gas candidate boolean threshold is conservative; event deltas can be visible while `gas_candidate=false`.
- The older velocity/crab traversal paths are retained for diagnostics and history, but they are not the current recommended demo path.
- Mission-level JSON summaries such as `discovered_openings` and `mission_end_reason` are not added yet.

## Next Phases

- Multi-level gas classification: `weak`, `moderate`, `strong`.
- Mission-level JSON summary fields.
- Further architecture cleanup after the validated milestone.
- Optional media update with screenshots, GIFs, or video from the successful inspect-all run.
