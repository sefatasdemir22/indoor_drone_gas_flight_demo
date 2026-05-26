# Experiment Validation Protocol

This document defines the minimum experiment protocol for validating the
indoor drone gas inspection demo at thesis/demo level. It intentionally avoids
new flight behavior: the current inspect-all mission pipeline, return-home,
landing stabilization, event JSON, CSV exporter, and sparse summary heatmap are
treated as fixed.

## Scope

In scope:

- Repeatable experiment naming and expected outputs.
- Mission-level success and inspection coverage criteria.
- Scenario-level expectations for thesis reporting.
- Columns expected by a future `experiment_summary.csv`.

Out of scope:

- Flight pipeline changes.
- World/model changes.
- Obstacle avoidance.
- Continuous gas sampling.
- Committing generated experiment outputs.

## Standard Output Pipeline

Each successful run should preserve the following generated files outside git:

```text
opening_inspection_events.json
opening_mission_summary.json
opening_room_samples.csv
opening_event_summary_heatmap.png
```

The post-process flow is:

```text
opening_inspection_events.json
-> opening_mission_summary.json
-> opening_room_samples.csv
-> opening_event_summary_heatmap.png
```

Recommended archive layout:

```text
results/experiments/
  S1_no_gas_seed1/
  S2_single_source_seed1/
  S3_multi_source_seed1/
  S4_strongest_room_validation_seed1/
  S5_single_source_seed1/
  S5_single_source_seed2/
  S5_single_source_seed3/
```

## Experiment Set

| ID | Scenario | Purpose | Approx. expected duration |
| --- | --- | --- | --- |
| S1 | `no_gas` | Check false-positive behavior when no gas source is active. | 300-480 sec |
| S2 | `single_source` | Validate strongest-room and delta metrics with one gas source. | 300-480 sec |
| S3 | `multi_source` | Compare room-level readings when multiple gas regions are active. | 300-480 sec |
| S4 | `strongest_room_validation` | Verify that `strongest_room` is consistent with the `highest_delta_ppm` strategy. | 300-480 sec |
| S5 | `scenario x 3 seed repeat` | Check behavioral stability across seeds for the selected main scenario. | 900-1440 sec total |

Recommended S5 seeds:

```text
1
2
3
```

Durations are approximate and may vary with simulator startup, PX4 readiness,
and manual validation time.

## Acceptance Criteria

Each experiment should satisfy:

- `mission_status == completed`
- `return_home_status == completed`
- `completed_inspections >= expected_room_count`
- The same opening is not inspected repeatedly.
- CSV row count is consistent with `completed_inspections`.
- The sparse summary PNG is generated.
- The PNG states that it is a sparse event-average summary, not a continuous
  gas concentration map.
- `runtime_sec` is recorded.

Inspection coverage should be evaluated with:

```text
coverage_pass = completed_inspections >= expected_room_count
```

`coverage_pass` is a protocol-level criterion based on `expected_room_count`;
it does not prove complete coverage of an unknown environment.

For the current demo world, `expected_room_count` may be recorded manually in
the experiment notes. If a future mission summary exposes `discovered_room_count`,
coverage can be migrated to:

```text
completed_inspections == discovered_room_count
```

## Scenario-Specific Expectations

### S1 — no_gas

Expected behavior:

- Mission completes normally.
- Return-home completes.
- `highest_delta_ppm` remains low.

Thesis interpretation:

- If a high delta appears, discuss it as false-positive/noise behavior rather
  than as a detected gas source.

### S2 — single_source

Expected behavior:

- Mission completes normally.
- One inspected opening is expected to show a higher delta than the others.
- `strongest_room` should match the largest delta reading.

Thesis interpretation:

- Use this as the primary single-source inspection demonstration.

### S3 — multi_source

Expected behavior:

- Mission completes normally.
- More than one inspected opening may show elevated ppm/delta values.

Thesis interpretation:

- Use this to show room-level comparison rather than continuous mapping.

### S4 — strongest_room_validation

Expected behavior:

- `strongest_room_strategy == highest_delta_ppm`
- `strongest_room.delta_ppm == highest_delta_ppm`

Thesis interpretation:

- This checks that the summary layer selects the strongest room using the
  documented strategy.

### S5 — scenario x 3 seed repeat

Expected behavior:

- The same selected scenario is repeated with seeds `1`, `2`, and `3`.
- Mission flow remains successful across runs.
- PPM values may vary, but coverage and return-home behavior should remain
  stable.

Thesis interpretation:

- Use this as a small repeatability check, not as a full statistical study.

## Future Experiment Summary CSV

PHASE 2.2 should generate an `experiment_summary.csv` from archived experiment
folders. The intended columns are:

```text
experiment_id
scenario
gas_seed
runtime_sec
expected_room_count
completed_inspections
coverage_pass
mission_status
highest_ppm
highest_delta_ppm
strongest_room
return_home_status
csv_rows
heatmap_generated
notes
```

`runtime_sec` is stored in seconds. Minute conversion should be done in the
thesis/report layer.

## Minimum Thesis Outputs

The thesis/demo package should include:

- One main experiment results table.
- One sparse event summary PNG per scenario.
- At least one mission video or screenshot.
- At least one example `opening_mission_summary.json`.
- At least one example `opening_room_samples.csv`.

Use precise wording:

```text
Sparse event-average gas inspection summary
```

Avoid claiming that the event-average PNG is a continuous gas concentration map.
