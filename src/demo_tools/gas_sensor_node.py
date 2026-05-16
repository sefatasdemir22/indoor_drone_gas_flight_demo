#!/usr/bin/env python3
"""Simulated gas sensor data generator for the indoor drone demo.

This is intentionally not a real ROS2 node yet. It generates a simulated
mission route, resolves a gas scenario, computes ppm values, and writes CSV/JSON
outputs for the mapping smoke test.

TODO: Publish simulated gas measurements on a ROS2 topic once the flight stack
and telemetry interfaces are connected.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
from pathlib import Path
from typing import Any


POSSIBLE_GAS_ZONES: dict[str, tuple[float, float]] = {
    "possible_gas_zone_1": (5.0, 4.0),
    "possible_gas_zone_2": (9.0, -4.0),
    "possible_gas_zone_3": (13.0, 3.0),
    "possible_gas_zone_4": (7.0, 0.0),
}

SCENARIO_CHOICES = [
    "no_gas",
    "clean_air",
    "random",
    "random_single",
    "random_multi",
    "possible_gas_zone_1",
    "possible_gas_zone_2",
    "possible_gas_zone_3",
    "possible_gas_zone_4",
    "multi_1_2",
    "multi_1_3",
    "multi_2_3",
    "multi_2_4",
    "multi_all",
]

MULTI_SCENARIOS: dict[str, list[str]] = {
    "multi_1_2": ["possible_gas_zone_1", "possible_gas_zone_2"],
    "multi_1_3": ["possible_gas_zone_1", "possible_gas_zone_3"],
    "multi_2_3": ["possible_gas_zone_2", "possible_gas_zone_3"],
    "multi_2_4": ["possible_gas_zone_2", "possible_gas_zone_4"],
    "multi_all": list(POSSIBLE_GAS_ZONES.keys()),
}

ROUTE_WAYPOINTS: list[tuple[float, float]] = [
    (0.0, 0.0),    # START / SAFE_EXIT
    (2.0, 0.0),    # enter main corridor
    (5.0, 4.0),    # left room center
    (5.0, 0.0),    # return to corridor
    (7.0, 0.0),    # corridor middle
    (9.0, -4.0),   # right room center
    (9.0, 0.0),    # return to corridor
    (13.0, 3.0),   # forward exploration room
    (13.0, 0.0),   # return to corridor
    (7.0, 0.0),    # corridor return
    (0.0, 0.0),    # START / SAFE_EXIT return
]


def parse_args() -> argparse.Namespace:
    project_dir = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description="Generate simulated gas measurements for a fixed exploration route.")
    parser.add_argument("--scenario", default="random", choices=SCENARIO_CHOICES)
    parser.add_argument("--samples", type=int, default=220, help="Total number of route samples to generate.")
    parser.add_argument("--output", type=Path, default=project_dir / "results" / "gas_samples.csv")
    parser.add_argument("--seed", type=int, default=None, help="Optional deterministic random seed.")
    parser.add_argument("--background-ppm", type=float, default=5.0)
    parser.add_argument("--peak-ppm", type=float, default=120.0)
    parser.add_argument("--sigma", type=float, default=1.8)
    parser.add_argument("--noise-std", type=float, default=1.2)
    return parser.parse_args()


def source_dict(zone_name: str) -> dict[str, Any]:
    x, y = POSSIBLE_GAS_ZONES[zone_name]
    return {"name": zone_name, "x": x, "y": y}


def resolve_scenario(requested_scenario: str, rng: random.Random) -> tuple[str, list[dict[str, Any]]]:
    """Return a scenario class and active source list."""
    if requested_scenario in {"no_gas", "clean_air"}:
        return "no_gas", []

    if requested_scenario == "random":
        requested_scenario = rng.choice(["no_gas", "random_single", "random_multi"])

    if requested_scenario == "random_single":
        zone_name = rng.choice(list(POSSIBLE_GAS_ZONES.keys()))
        return "single_source", [source_dict(zone_name)]

    if requested_scenario == "random_multi":
        count = rng.choice([2, 3])
        zone_names = rng.sample(list(POSSIBLE_GAS_ZONES.keys()), count)
        return "multi_source", [source_dict(name) for name in zone_names]

    if requested_scenario in POSSIBLE_GAS_ZONES:
        return "single_source", [source_dict(requested_scenario)]

    if requested_scenario in MULTI_SCENARIOS:
        return "multi_source", [source_dict(name) for name in MULTI_SCENARIOS[requested_scenario]]

    raise ValueError(f"Unsupported scenario: {requested_scenario}")


def route_segment_lengths(waypoints: list[tuple[float, float]]) -> list[float]:
    return [math.dist(start, end) for start, end in zip(waypoints, waypoints[1:])]


def interpolate_route(sample_count: int, z: float = 1.5) -> list[tuple[float, float, float]]:
    if sample_count < 2:
        raise ValueError("--samples must be at least 2")

    lengths = route_segment_lengths(ROUTE_WAYPOINTS)
    total_length = sum(lengths)
    samples: list[tuple[float, float, float]] = []

    for index in range(sample_count):
        target_distance = total_length * index / (sample_count - 1)
        walked = 0.0
        for segment_index, segment_length in enumerate(lengths):
            if target_distance <= walked + segment_length or segment_index == len(lengths) - 1:
                start_x, start_y = ROUTE_WAYPOINTS[segment_index]
                end_x, end_y = ROUTE_WAYPOINTS[segment_index + 1]
                ratio = 0.0 if segment_length == 0 else (target_distance - walked) / segment_length
                samples.append((start_x + (end_x - start_x) * ratio, start_y + (end_y - start_y) * ratio, z))
                break
            walked += segment_length

    return samples


def compute_ppm(
    x: float,
    y: float,
    active_sources: list[dict[str, Any]],
    background_ppm: float,
    peak_ppm: float,
    sigma: float,
    rng: random.Random,
    noise_std: float,
) -> tuple[float, float | None]:
    plume_sum = 0.0
    distances: list[float] = []

    for source in active_sources:
        distance = math.dist((x, y), (float(source["x"]), float(source["y"])))
        distances.append(distance)
        plume_sum += peak_ppm * math.exp(-(distance * distance) / (2.0 * sigma * sigma))

    noise = rng.gauss(0.0, noise_std) if noise_std > 0 else 0.0
    ppm = max(0.0, background_ppm + plume_sum + noise)
    nearest_distance = min(distances) if distances else None
    return ppm, nearest_distance


def write_scenario_info(path: Path, info: dict[str, object]) -> None:
    path.write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    rng = random.Random(args.seed) if args.seed is not None else random.Random()
    scenario, active_sources = resolve_scenario(args.scenario, rng)
    active_zones = [str(source["name"]) for source in active_sources]
    first_source = active_sources[0] if active_sources else None
    positions = interpolate_route(args.samples)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    scenario_info_path = args.output.parent / "scenario_info.json"

    active_sources_json = json.dumps(active_sources, separators=(",", ":"))
    active_zones_text = "|".join(active_zones)

    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "sample_index",
                "timestamp_sec",
                "x",
                "y",
                "z",
                "ppm",
                "scenario",
                "active_zones",
                "active_sources_json",
                "nearest_source_distance",
                "active_zone",
                "active_source_x",
                "active_source_y",
                "distance_to_source",
            ],
        )
        writer.writeheader()
        for sample_index, (x, y, z) in enumerate(positions):
            ppm, nearest_distance = compute_ppm(
                x=x,
                y=y,
                active_sources=active_sources,
                background_ppm=args.background_ppm,
                peak_ppm=args.peak_ppm,
                sigma=args.sigma,
                rng=rng,
                noise_std=args.noise_std,
            )
            distance_text = "" if nearest_distance is None else f"{nearest_distance:.3f}"
            writer.writerow(
                {
                    "sample_index": sample_index,
                    "timestamp_sec": f"{sample_index * 0.2:.2f}",
                    "x": f"{x:.3f}",
                    "y": f"{y:.3f}",
                    "z": f"{z:.3f}",
                    "ppm": f"{ppm:.3f}",
                    "scenario": scenario,
                    "active_zones": active_zones_text,
                    "active_sources_json": active_sources_json,
                    "nearest_source_distance": distance_text,
                    "active_zone": first_source["name"] if first_source else "",
                    "active_source_x": f"{float(first_source['x']):.3f}" if first_source else "",
                    "active_source_y": f"{float(first_source['y']):.3f}" if first_source else "",
                    "distance_to_source": distance_text,
                }
            )

    write_scenario_info(
        scenario_info_path,
        {
            "requested_scenario": args.scenario,
            "scenario": scenario,
            "active_zones": active_zones,
            "active_sources": active_sources,
            "background_ppm": args.background_ppm,
            "peak_ppm": args.peak_ppm,
            "sigma": args.sigma,
            "noise_std": args.noise_std,
            "sample_count": args.samples,
            "seed": args.seed,
            "route_waypoints": [{"x": x, "y": y} for x, y in ROUTE_WAYPOINTS],
            "possible_gas_zones": {name: {"x": xy[0], "y": xy[1]} for name, xy in POSSIBLE_GAS_ZONES.items()},
        },
    )

    if active_sources:
        source_summary = ", ".join(f"{source['name']}({source['x']:.1f},{source['y']:.1f})" for source in active_sources)
    else:
        source_summary = "none"
    print(f"Requested scenario: {args.scenario}")
    print(f"Resolved scenario: {scenario}")
    print(f"Active gas sources: {source_summary}")
    print(f"Seed: {args.seed if args.seed is not None else 'system random'}")
    print(f"Wrote CSV: {args.output}")
    print(f"Wrote scenario info: {scenario_info_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
