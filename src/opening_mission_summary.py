#!/usr/bin/env python3
"""Build a mission-level summary from opening inspection event JSON."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    project_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Generate a mission summary JSON from opening_inspection_events.json."
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
        default=project_dir / "results" / "opening_mission_summary.json",
        help="Output mission summary JSON.",
    )
    return parser.parse_args()


def as_float(value: object) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def completed_events(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [event for event in events if event.get("inspection_avg_ppm") is not None]


def opening_summary(event: dict[str, Any]) -> dict[str, Any]:
    return {
        "inspection_index": event.get("inspection_index"),
        "opening_id": event.get("opening_id"),
        "side": event.get("side"),
        "baseline_avg_ppm": event.get("baseline_avg_ppm"),
        "inspection_avg_ppm": event.get("inspection_avg_ppm"),
        "delta_ppm": event.get("delta_ppm"),
        "gas_candidate": event.get("gas_candidate"),
        "candidate_reason": event.get("candidate_reason"),
        "inspection_position": event.get("inspection_position"),
        "room_traverse_stop_reason": event.get("room_traverse_stop_reason"),
    }


def event_with_max(events: list[dict[str, Any]], field: str) -> dict[str, Any] | None:
    scored: list[tuple[float, dict[str, Any]]] = []
    for event in events:
        value = as_float(event.get(field))
        if value is not None:
            scored.append((value, event))
    if not scored:
        return None
    return max(scored, key=lambda item: item[0])[1]


def mission_status(completed_count: int, return_home: dict[str, Any]) -> str:
    return_status = return_home.get("status")
    if completed_count <= 0:
        return "failed"
    if return_status == "completed":
        return "completed"
    return "partial"


def display_path(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(Path.cwd().resolve()))
    except ValueError:
        return str(path)


def build_summary(payload: dict[str, Any], source_path: Path) -> dict[str, Any]:
    events = payload.get("events", [])
    if not isinstance(events, list):
        events = []
    valid_events = [event for event in events if isinstance(event, dict)]
    completed = completed_events(valid_events)
    return_home = payload.get("return_home") if isinstance(payload.get("return_home"), dict) else {}

    highest_ppm_event = event_with_max(completed, "inspection_avg_ppm")
    highest_delta_event = event_with_max(completed, "delta_ppm")
    strongest_event = highest_delta_event or highest_ppm_event

    highest_ppm = as_float(highest_ppm_event.get("inspection_avg_ppm")) if highest_ppm_event else None
    highest_delta = as_float(highest_delta_event.get("delta_ppm")) if highest_delta_event else None

    return {
        "mission_status": mission_status(len(completed), return_home),
        "source_event_file": display_path(source_path),
        "scenario": payload.get("scenario"),
        "requested_scenario": payload.get("requested_scenario"),
        "gas_seed": payload.get("gas_seed"),
        "completed_inspections": len(completed),
        "event_count": len(valid_events),
        "inspected_openings": [opening_summary(event) for event in completed],
        "strongest_room_strategy": "highest_delta_ppm",
        "strongest_room": opening_summary(strongest_event) if strongest_event else None,
        "highest_ppm": round(highest_ppm, 3) if highest_ppm is not None else None,
        "highest_delta_ppm": round(highest_delta, 3) if highest_delta is not None else None,
        "return_home_status": return_home.get("status"),
        "return_home": {
            "enabled": return_home.get("enabled"),
            "attempted": return_home.get("attempted"),
            "status": return_home.get("status"),
            "final_distance_to_home_m": return_home.get("final_distance_to_home_m"),
            "steps": return_home.get("steps"),
            "actual_distance_m": return_home.get("actual_distance_m"),
        },
        "mission_duration": None,
        "mission_duration_note": "unavailable: source event JSON does not include mission start/end timestamps",
        "discovered_openings": None,
        "discovered_openings_note": "unavailable: source event JSON only records inspected/aborted openings",
    }


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

    summary = build_summary(payload, args.input)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Mission summary written: {args.output}")
    print(f"mission_status={summary['mission_status']}")
    print(f"completed_inspections={summary['completed_inspections']}")
    print(f"highest_ppm={summary['highest_ppm']}")
    print(f"highest_delta_ppm={summary['highest_delta_ppm']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
