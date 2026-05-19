# Opening-Based Gas Survey Mission Checkpoint

This document summarizes the current working checkpoint for the opening-based
indoor drone gas survey prototype. It is a development checkpoint, not a final
full-autonomy claim.

## Current Checkpoint

- Git checkpoint: `133f89f Add room-facing yaw interpolation`
- Main mission script: `src/opening_based_gas_survey_mission.py`
- Main output: `results/opening_inspection_events.json`

## Working Capabilities

- Starts PX4/Gazebo indoor corridor-room world with `./baslat.sh`.
- Connects through MAVSDK and runs position-mode corridor steps.
- Reads wide and decision LaserScan topics for opening and safety logic.
- Detects candidate openings from side decision scans.
- Uses no-backtrack door capture to avoid passing the door before inspection.
- Applies a small door forward offset before room-facing entry.
- Turns toward the room with yaw interpolation and repeated yaw setpoints.
- Performs post-yaw realignment if front clearance is low.
- Enters the room in room-facing mode, using front scan as the room-direction sensor.
- Stops room traversal when front distance reaches the configured stop distance.
- Samples gas baseline and inspection ppm.
- Exits the room incrementally.
- Returns to corridor yaw and continues corridor progression.
- Honors `--max-inspections`; later opening candidates can still be detected without being inspected.

## Validated Behavior

Recent validation accepted as successful:

- no-backtrack capture worked
- door forward offset worked
- post-yaw realignment worked
- room-facing yaw entry worked
- yaw interpolation made the turn smoother
- drone entered the room by roughly 5 m
- front-distance stop triggered
- gas measurement was written to event JSON
- incremental exit worked
- corridor continuation after inspection stayed stable
- revisit loop was not observed

Representative event fields from the successful checkpoint:

- `room_traversal_mode="room_facing_yaw"`
- `room_traverse_stop_reason="front_stop_distance"`
- `room_traverse_actual_distance_m=5.0`
- `room_facing_final_front_min_m≈1.23`
- `inspection_avg_ppm≈8.29`
- `delta_ppm≈3.30`
- `room_facing_yaw_interpolation_step_deg=15.0`
- `room_facing_yaw_interpolation_hold_seconds=0.2`
- `room_facing_exit_steps=12`

## Main Demo Command

Start the simulator:

```bash
cd /home/sefa/Desktop/indoor_drone_gas_flight_demo
./baslat.sh
```

Run the current room-facing validation mission:

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

## Dry-Run Checks

These checks do not start PX4/Gazebo and do not command the drone:

```bash
python3 -m py_compile src/opening_based_gas_survey_mission.py
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
- Return-home is not implemented in the main room-facing mission flow yet.
- Multi-room inspection has not been checkpointed yet.
- Landing bounce can happen in some runs.
- Gas candidate boolean threshold is conservative; event deltas can be visible while `gas_candidate=false`.
- The older velocity/crab traversal paths are retained for diagnostics and history, but they are not the current recommended demo path.

## Next Phases

- EventBuilder and GasDecision cleanup.
- Multi-room V1 with multiple event records.
- Multi-level gas classification: `weak`, `moderate`, `strong`.
- Return-home V1 using local NED start position.
- Landing stability phase.

