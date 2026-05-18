# Opening-Based Gas Survey Mission Checkpoint

This document summarizes the current checkpoint for the opening-based indoor
drone survey prototype. It is a development checkpoint, not a final autonomous
survey claim.

## Current Working Capabilities

- The project starts the PX4/Gazebo indoor corridor-room world with `./baslat.sh`.
- The drone can connect through MAVSDK, take off, run a low-speed body-frame
  corridor-follow check, stop offboard mode, and land.
- The mission script reads these ROS2 LaserScan topics:
  - `/drone/front_scan`
  - `/drone/left_scan`
  - `/drone/right_scan`
- Front safety is active during corridor-follow. It is useful for front-facing
  wall-like obstacles, but it is not a guaranteed blocker for low floor
  obstacles.
- Passive opening decisions are logged during corridor-follow:
  - normal corridor: `FOLLOW_FORWARD`
  - visible left opening: `DETECT_LEFT_OPENING`
  - visible right opening: `DETECT_RIGHT_OPENING`
- `--enable-opening-probe` enables one short optional lateral body-frame probe
  after the first detected opening.
- The opening probe is intentionally small and conservative. It does not perform
  full room exploration.
- Gas mapping is not integrated into this mission script yet. Live gas mapping
  still runs as a separate ROS2 process with `src/live_gas_mapping_ros2.py`.

## Demo Commands

Start the simulator in the first terminal:

```bash
cd /home/sefa/Desktop/indoor_drone_gas_flight_demo
./baslat.sh
```

Run corridor-follow without opening probe:

```bash
python3 src/opening_based_gas_survey_mission.py \
  --corridor-follow-check \
  --corridor-step-count 8 \
  --body-forward-speed 0.20 \
  --corridor-step-duration-seconds 3.0 \
  --pause-between-steps 1.0 \
  --offboard-warmup-seconds 1.5 \
  --min-takeoff-confirm-altitude 0.65
```

Run corridor-follow with the optional opening probe:

```bash
python3 src/opening_based_gas_survey_mission.py \
  --corridor-follow-check \
  --enable-opening-probe \
  --corridor-step-count 8 \
  --body-forward-speed 0.20 \
  --corridor-step-duration-seconds 3.0 \
  --pause-between-steps 1.0 \
  --offboard-warmup-seconds 1.5 \
  --min-takeoff-confirm-altitude 0.65 \
  --probe-side-speed 0.12 \
  --probe-duration-seconds 1.5
```

Optional live gas mapping can be run in a separate terminal. This script does
not command the drone:

```bash
python3 src/live_gas_mapping_ros2.py \
  --scenario possible_gas_zone_4 \
  --duration-seconds 30 \
  --sample-rate-hz 5 \
  --seed 1 \
  --hide-source-marker \
  --route-min-altitude 0.5
```

The demo command uses `possible_gas_zone_4` as a deterministic gas scenario for
a more repeatable presentation heatmap. It hides the ground-truth gas source
marker and filters low-altitude takeoff samples from the route/scatter drawing
only. CSV and JSON records remain unchanged.

## Dry-Run Checks

These checks do not start PX4/Gazebo and do not command the drone:

```bash
python3 -m py_compile src/opening_based_gas_survey_mission.py
python3 src/opening_based_gas_survey_mission.py --dry-run --seed 1
python3 src/opening_based_gas_survey_mission.py --dry-run --dry-run-scenario normal_corridor_side_distance
python3 src/opening_based_gas_survey_mission.py --dry-run --dry-run-scenario left_opening
python3 src/opening_based_gas_survey_mission.py --dry-run --dry-run-scenario right_opening
python3 src/opening_based_gas_survey_mission.py --dry-run --dry-run-scenario front_blocked_left_open
```

Expected dry-run behavior:

- `normal_corridor_side_distance` should produce `FOLLOW_FORWARD`.
- `left_opening` should produce `DETECT_LEFT_OPENING`.
- `right_opening` should produce `DETECT_RIGHT_OPENING`.
- `front_blocked_left_open` should produce `BYPASS_LEFT` in the decision log.

## Known Limitations

- This is not a full autonomous survey mission yet.
- There is no SLAM, frontier exploration, global path planning, or map-based
  navigation.
- Opening detection is based on the current front/left/right LaserScan readings
  and simple thresholds.
- The opening probe is short, optional, and visually small. It only verifies
  that a detected opening can trigger a small lateral motion response.
- The mission does not return to the corridor after probing. The current V1
  ends the corridor-follow check and lands after the probe.
- Landing bounce has been observed in some runs.
- Scan readiness can occasionally time out on the first run if ROS2 scan topics
  are not ready yet. Re-running the same command has worked in later tests.
- Red low obstacles in the current world can be flown over at the current drone
  altitude. Front safety should not be described as a guaranteed blocker for
  those low floor obstacles.
- Gas mapping is still separate from this mission script. The mission does not
  directly trigger gas samples or heatmap generation.
- Debug heatmaps may show the ground-truth gas source marker. Demo heatmaps can
  hide this marker so the plot does not imply that the drone already knows the
  source location.
- `--route-min-altitude` is only a visualization filter for the route/scatter
  drawing. It does not remove samples from the raw CSV or scenario JSON.
- `--scenario random`, `--scenario no_gas`, and `--scenario clean_air` are valid
  robustness tests, but they can produce weak or no visible gas plume in a
  presentation heatmap.

## Recommended Demo Flow

1. Start the simulator with `./baslat.sh`.
2. Confirm relevant topics are visible:

   ```bash
   ros2 topic list | grep -Ei "front_scan|left_scan|right_scan|vehicle_local_position"
   ```

3. Run the dry-run checks if code changes were made.
4. Run corridor-follow without probe to show stable movement and passive opening
   decisions.
5. Run corridor-follow with `--enable-opening-probe` to show that an opening
   decision can trigger a short lateral probe.
6. Run `live_gas_mapping_ros2.py` in another terminal before the mission when a
   gas mapping demo is needed:

   ```bash
   python3 src/live_gas_mapping_ros2.py \
     --scenario possible_gas_zone_4 \
     --duration-seconds 30 \
     --sample-rate-hz 5 \
     --seed 1 \
     --hide-source-marker \
     --route-min-altitude 0.5
   ```

   Expected outputs:

   - `results/live_ros2_gas_samples.csv`
   - `results/live_ros2_scenario_info.json`
   - `results/live_ros2_gas_heatmap.png`

## Next Phases

- Improve takeoff and landing consistency, especially landing bounce.
- Make the opening probe more visible while keeping it safe.
- Add a short hover/sample window after a successful probe.
- Add optional return-to-corridor behavior after probing.
- Connect mission events with the live gas mapping workflow.
- Build a limited survey flow: corridor-follow, detect opening, probe/sample,
  continue or land.
- Keep heavy SLAM, global path planning, and full unknown-environment
  exploration out of scope until the simple demo is stable.
