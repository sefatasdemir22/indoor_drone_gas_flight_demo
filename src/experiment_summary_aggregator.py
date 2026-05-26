#!/usr/bin/env python3
"""Aggregate archived experiment outputs into one CSV summary."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


CSV_COLUMNS = [
    "experiment_id",
    "scenario",
    "gas_seed",
    "runtime_sec",
    "expected_room_count",
    "completed_inspections",
    "coverage_pass",
    "mission_status",
    "highest_ppm",
    "highest_delta_ppm",
    "strongest_room",
    "return_home_status",
    "csv_rows",
    "heatmap_generated",
    "notes",
]


def parse_args() -> argparse.Namespace:
    project_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Aggregate archived experiment outputs into experiment_summary.csv."
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=project_dir / "results" / "experiments",
        help="Directory containing one subdirectory per experiment.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=project_dir / "results" / "experiment_summary.csv",
        help="Output experiment summary CSV.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Exit non-zero if expected files are missing or invalid.",
    )
    return parser.parse_args()


def read_json(path: Path) -> dict[str, Any] | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"[WARN] could not read JSON {path}: {exc}")
        return None
    if not isinstance(payload, dict):
        print(f"[WARN] JSON root is not an object: {path}")
        return None
    return payload


def as_int(value: object) -> int | None:
    if value in (None, ""):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def count_csv_rows(path: Path) -> int | None:
    if not path.exists():
        return None
    try:
        with path.open(newline="", encoding="utf-8") as csv_file:
            return sum(1 for _ in csv.DictReader(csv_file))
    except OSError as exc:
        print(f"[WARN] could not read CSV {path}: {exc}")
        return None


def coverage_pass(completed: object, expected: object) -> str:
    completed_count = as_int(completed)
    expected_count = as_int(expected)
    if completed_count is None or expected_count is None:
        return ""
    return "true" if completed_count >= expected_count else "false"


def strongest_room_id(summary: dict[str, Any]) -> object:
    strongest_room = summary.get("strongest_room")
    if isinstance(strongest_room, dict):
        return strongest_room.get("opening_id")
    return None


def build_row(
    experiment_dir: Path, summary: dict[str, Any], metadata: dict[str, Any]
) -> tuple[dict[str, Any], int]:
    samples_path = experiment_dir / "opening_room_samples.csv"
    heatmap_path = experiment_dir / "opening_event_summary_heatmap.png"
    artifact_errors = 0

    csv_rows = count_csv_rows(samples_path)
    if csv_rows is None:
        print(f"[WARN] missing or unreadable CSV: {samples_path}")
        artifact_errors += 1
    heatmap_generated = heatmap_path.exists()
    if not heatmap_generated:
        print(f"[WARN] missing heatmap PNG: {heatmap_path}")
        artifact_errors += 1

    expected_room_count = metadata.get("expected_room_count")
    completed_inspections = summary.get("completed_inspections")

    row = {
        "experiment_id": experiment_dir.name,
        "scenario": summary.get("requested_scenario") or summary.get("scenario"),
        "gas_seed": summary.get("gas_seed"),
        "runtime_sec": metadata.get("runtime_sec", ""),
        "expected_room_count": expected_room_count if expected_room_count is not None else "",
        "completed_inspections": completed_inspections,
        "coverage_pass": coverage_pass(completed_inspections, expected_room_count),
        "mission_status": summary.get("mission_status"),
        "highest_ppm": summary.get("highest_ppm"),
        "highest_delta_ppm": summary.get("highest_delta_ppm"),
        "strongest_room": strongest_room_id(summary),
        "return_home_status": summary.get("return_home_status"),
        "csv_rows": csv_rows if csv_rows is not None else "",
        "heatmap_generated": "true" if heatmap_generated else "false",
        "notes": metadata.get("notes", ""),
    }
    return row, artifact_errors


def experiment_dirs(input_dir: Path) -> list[Path]:
    if not input_dir.exists():
        print(f"[WARN] input directory not found: {input_dir}")
        return []
    return sorted(path for path in input_dir.iterdir() if path.is_dir())


def aggregate(input_dir: Path) -> tuple[list[dict[str, Any]], int, int]:
    rows: list[dict[str, Any]] = []
    skipped = 0
    artifact_errors = 0

    for experiment_dir in experiment_dirs(input_dir):
        summary_path = experiment_dir / "opening_mission_summary.json"
        if not summary_path.exists():
            print(f"[WARN] missing summary JSON, skipping: {summary_path}")
            skipped += 1
            continue

        summary = read_json(summary_path)
        if summary is None:
            skipped += 1
            continue

        metadata_path = experiment_dir / "experiment_metadata.json"
        metadata: dict[str, Any] = {}
        if metadata_path.exists():
            parsed_metadata = read_json(metadata_path)
            if parsed_metadata is None:
                artifact_errors += 1
            else:
                metadata = parsed_metadata

        row, row_artifact_errors = build_row(experiment_dir, summary, metadata)
        artifact_errors += row_artifact_errors
        rows.append(row)

    return rows, skipped, artifact_errors


def write_csv(rows: list[dict[str, Any]], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    args = parse_args()
    rows, skipped, artifact_errors = aggregate(args.input_dir)
    write_csv(rows, args.output)

    print(f"Experiment summary written: {args.output}")
    print(f"experiment_rows={len(rows)}")
    print(f"skipped_experiments={skipped}")
    print(f"artifact_errors={artifact_errors}")

    if args.strict and (skipped > 0 or artifact_errors > 0 or not rows):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
