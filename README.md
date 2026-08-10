# Indoor Drone Gas Flight Demo

A ROS 2 / PX4 simulation project for autonomous indoor gas-inspection missions in a GPS-denied corridor-and-room environment.

The system detects room openings from LaserScan data, inspects multiple rooms without relying on preconfigured room coordinates, samples simulated gas concentrations, returns to a local home reference, and performs a staged landing.

## Project Scope

This repository represents the final flight-demo phase of my indoor autonomous drone graduation project. The focus is a reproducible mission pipeline in a constrained indoor environment rather than a claim of full unknown-environment autonomy.

**Validated scope:**

- corridor following in a GPS-denied simulation
- LaserScan-based opening detection
- multi-room inspection
- duplicate-opening suppression
- room-facing yaw alignment and controlled entry
- simulated gas baseline and inspection measurements
- corridor re-entry and mission continuation
- return to a local home reference
- staged landing stabilization

The project **does not** implement full SLAM, a global planner, or general-purpose unknown-environment topology mapping.

## Tech Stack

- ROS 2 Humble
- PX4 SITL
- Gazebo Classic
- MAVSDK
- Micro XRCE-DDS Agent
- Python

## Mission Flow

The main mission implementation is:

```text
src/opening_based_gas_survey_mission.py
```

A typical mission follows this sequence:

```text
Takeoff
  -> Follow corridor
  -> Detect opening
  -> Align with room
  -> Enter and inspect
  -> Sample gas
  -> Exit room
  -> Continue corridor
  -> Inspect additional openings
  -> Return home
  -> Stabilize and land
```

Opening candidates are inferred from directional LaserScan observations. During room entry, the vehicle rotates toward the room and uses forward-distance measurements to control traversal rather than navigating to a predefined room coordinate.

## Run the Simulation

Start the PX4/Gazebo environment:

```bash
cd ~/Desktop/indoor_drone_gas_flight_demo
./baslat.sh
```

Run the validated inspect-all mission profile from a second terminal:

```bash
cd ~/Desktop/indoor_drone_gas_flight_demo
./scripts/run_demo_mission.sh possible_gas_zone_4 1
```

Other example scenarios:

```bash
./scripts/run_demo_mission.sh no_gas 1
./scripts/run_demo_mission.sh multi_1_2 1
```

## Experiment Output

Mission events are written to:

```text
results/opening_inspection_events.json
```

The output records information such as:

- inspection index and detected side
- opening identifier
- room traversal distance and stop reason
- final front-distance measurement
- gas baseline and inspection averages
- concentration delta
- gas-candidate decision
- return-home status and final home distance

A successful inspect-all run produces multiple valid inspection events and completes the return-home sequence.

## Gas Mapping Utilities

The repository also contains utilities for simulated gas-field visualization and post-processing:

```bash
./scripts/run_mapper.sh random
./scripts/run_mapper.sh no_gas
./scripts/run_mapper.sh multi_1_2
./scripts/run_mapper.sh multi_all
```

Selected figures, tables, and presentation artifacts can be stored under `thesis_assets/`, while raw runtime outputs remain under `results/`.

## Dry-Run Checks

Core mission logic can be checked without launching PX4 or Gazebo:

```bash
python3 -m py_compile src/opening_based_gas_survey_mission.py
python3 -m py_compile src/opening_event_builder.py
python3 -m py_compile src/opening_mission_types.py
python3 -m py_compile src/opening_scan_decision.py

python3 src/opening_based_gas_survey_mission.py \
  --dry-run \
  --dry-run-scenario normal_corridor_side_distance

python3 src/opening_based_gas_survey_mission.py \
  --dry-run \
  --dry-run-scenario front_blocked_left_open
```

## Current Checkpoint

- Tag: `milestone_inspect_all_v1`
- Main mission: `src/opening_based_gas_survey_mission.py`
- Main event output: `results/opening_inspection_events.json`

Legacy and diagnostic scripts from earlier development phases are intentionally preserved in the repository, but they are not the primary demo path.

## Documentation

- [`docs/phase_checkpoint.md`](docs/phase_checkpoint.md) — validated checkpoint summary
- [`docs/demo_cookbook.md`](docs/demo_cookbook.md) — demo and test commands
- [`docs/flight_demo_notes.md`](docs/flight_demo_notes.md) — earlier implementation notes

## Limitations

- No full SLAM pipeline
- No global planner
- Mission behavior is tuned for the provided corridor-room simulation
- Gas detection uses a conservative threshold and may preserve measured deltas even when a candidate is not flagged
- Early mission termination after detecting high gas concentration is not implemented
