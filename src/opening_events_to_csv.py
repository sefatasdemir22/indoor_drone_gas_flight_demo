#!/usr/bin/env python3
"""Export completed opening inspection events as event-average CSV samples."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


CSV_COLUMNS = [
    "sample_type",
    "source",
    "sample_confidence",
    "inspection_index",
    "opening_id",
    "side",
    "north_m",
    "east_m",
    "altitude_m",
    "baseline_avg_ppm",
    "inspection_avg_ppm",
    "delta_ppm",
    "gas_candidate",
    "candidate_reason",
    "room_traversal_mode",
    "room_traverse_stop_reason",
    "room_traverse_actual_distance_m",
    "room_depth_estimate_m",
    "room_width_estimate_m",
]


def parse_args() -> argparse.Namespace:
    project_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Export opening inspection events to CSV samples for analysis/mapping."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=project_dir / "results" / "opening_inspection_events.json",
        help="Input opening inspection events JSON.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=project_dir / "results" / "opening_room_samples.csv",
        help="Output CSV path.",
    )
    return parser.parse_args()


def completed_events(events: object) -> list[dict[str, Any]]:
    if not isinstance(events, list):
        return []
    return [
        event
        for event in events
        if isinstance(event, dict) and event.get("inspection_avg_ppm") is not None
    ]


def sample_position(event: dict[str, Any]) -> dict[str, Any]:
    position = event.get("inspection_position")
    if not isinstance(position, dict):
        position = event.get("room_facing_entry_anchor_position")
    if not isinstance(position, dict):
        position = event.get("entry_anchor_position")
    return position if isinstance(position, dict) else {}


def sample_confidence(event: dict[str, Any]) -> str:
    stop_reason = event.get("room_traverse_stop_reason")
    sample_count = event.get("inspection_sample_count")
    try:
        count = int(sample_count)
    except (TypeError, ValueError):
        count = 0

    if stop_reason == "front_stop_distance" and count > 0:
        return "high"
    if count > 0:
        return "medium"
    return "low"


def csv_row(event: dict[str, Any]) -> dict[str, Any]:
    position = sample_position(event)
    return {
        "sample_type": "event_average",
        "source": "opening_inspection_event",
        "sample_confidence": sample_confidence(event),
        "inspection_index": event.get("inspection_index"),
        "opening_id": event.get("opening_id"),
        "side": event.get("side"),
        "north_m": position.get("north"),
        "east_m": position.get("east"),
        "altitude_m": position.get("altitude"),
        "baseline_avg_ppm": event.get("baseline_avg_ppm"),
        "inspection_avg_ppm": event.get("inspection_avg_ppm"),
        "delta_ppm": event.get("delta_ppm"),
        "gas_candidate": event.get("gas_candidate"),
        "candidate_reason": event.get("candidate_reason"),
        "room_traversal_mode": event.get("room_traversal_mode"),
        "room_traverse_stop_reason": event.get("room_traverse_stop_reason"),
        "room_traverse_actual_distance_m": event.get("room_traverse_actual_distance_m"),
        "room_depth_estimate_m": event.get("room_depth_estimate_m"),
        "room_width_estimate_m": event.get("room_width_estimate_m"),
    }


def export_csv(payload: dict[str, Any], output_path: Path) -> int:
    rows = [csv_row(event) for event in completed_events(payload.get("events"))]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def main() -> int:
    args = parse_args()
    if not args.input.exists():
        print(f"Input event JSON not found: {args.input}")
        return 1

    try:
        payload = json.loads(args.input.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        print(f"Could not parse event JSON: {exc}")
        return 2

    if not isinstance(payload, dict):
        print("Input event JSON must contain an object at the top level")
        return 2

    row_count = export_csv(payload, args.output)
    print(f"CSV samples written: {args.output}")
    print(f"sample_rows={row_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
