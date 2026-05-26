#!/usr/bin/env python3
"""Render a thesis-friendly experiment summary comparison PNG."""

from __future__ import annotations

import argparse
import csv
import os
import warnings
from pathlib import Path
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", "/tmp/indoor_drone_matplotlib")

import matplotlib

matplotlib.use("Agg")
warnings.filterwarnings("ignore", message="Unable to import Axes3D.*", category=UserWarning)

import matplotlib.pyplot as plt


REQUIRED_COLUMNS = [
    "experiment_id",
    "highest_ppm",
    "highest_delta_ppm",
    "completed_inspections",
    "coverage_pass",
    "mission_status",
    "return_home_status",
]


def parse_args() -> argparse.Namespace:
    project_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Generate a PNG comparison from experiment_summary.csv."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=project_dir / "results" / "experiment_summary.csv",
        help="Input CSV produced by experiment_summary_aggregator.py.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=project_dir / "results" / "experiment_comparison.png",
        help="Output PNG path.",
    )
    parser.add_argument(
        "--title",
        default="Experiment Summary Comparison",
        help="Figure title.",
    )
    return parser.parse_args()


def as_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def as_bool(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y", "pass", "passed"}


def short_experiment_id(experiment_id: str) -> str:
    parts = experiment_id.split("_")
    if len(parts) >= 2 and parts[0].startswith("S"):
        return f"{parts[0]} {parts[1]}"
    return experiment_id


def load_rows(input_path: Path) -> tuple[list[dict[str, Any]], str | None]:
    if not input_path.exists():
        return [], f"Input CSV not found: {input_path}"

    with input_path.open(newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        fieldnames = reader.fieldnames or []
        missing = [column for column in REQUIRED_COLUMNS if column not in fieldnames]
        if missing:
            return [], f"Input CSV missing required columns: {', '.join(missing)}"

        rows: list[dict[str, Any]] = []
        for raw in reader:
            highest_ppm = as_float(raw.get("highest_ppm"))
            highest_delta = as_float(raw.get("highest_delta_ppm"))
            completed = as_float(raw.get("completed_inspections"))
            if highest_ppm is None or highest_delta is None or completed is None:
                continue
            rows.append(
                {
                    "experiment_id": raw["experiment_id"],
                    "label": short_experiment_id(raw["experiment_id"]),
                    "highest_ppm": highest_ppm,
                    "highest_delta_ppm": highest_delta,
                    "completed_inspections": completed,
                    "coverage_pass": as_bool(raw.get("coverage_pass")),
                    "mission_ok": raw.get("mission_status") == "completed",
                    "return_home_ok": raw.get("return_home_status") == "completed",
                    "mission_status": raw.get("mission_status") or "",
                    "return_home_status": raw.get("return_home_status") or "",
                }
            )

    if not rows:
        return [], f"No valid experiment rows found in: {input_path}"
    return rows, None


def annotate_bars(axis: Any, values: list[float], fmt: str) -> None:
    upper = max(values) if values else 1.0
    offset = max(upper * 0.025, 0.05)
    for index, value in enumerate(values):
        axis.text(index, value + offset, fmt.format(value), ha="center", va="bottom", fontsize=8)


def render_bar_panel(
    axis: Any,
    labels: list[str],
    values: list[float],
    title: str,
    ylabel: str,
    color: str,
    fmt: str,
) -> None:
    axis.bar(labels, values, color=color, edgecolor="black", linewidth=0.7)
    annotate_bars(axis, values, fmt)
    axis.set_title(title)
    axis.set_ylabel(ylabel)
    axis.grid(axis="y", alpha=0.28)
    axis.tick_params(axis="x", rotation=20)
    upper = max(values) if values else 1.0
    axis.set_ylim(0.0, upper * 1.18 + 0.1)


def status_symbol(ok: bool) -> str:
    return "✓" if ok else "✗"


def render_status_dashboard(axis: Any, rows: list[dict[str, Any]]) -> None:
    axis.axis("off")
    axis.set_title("Mission Status Dashboard")

    columns = ["Experiment", "Mission", "Return", "Coverage"]
    table_rows = [
        [
            row["label"],
            status_symbol(row["mission_ok"]),
            status_symbol(row["return_home_ok"]),
            status_symbol(row["coverage_pass"]),
        ]
        for row in rows
    ]
    table = axis.table(
        cellText=table_rows,
        colLabels=columns,
        cellLoc="center",
        colLoc="center",
        loc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.0, 1.6)

    for (row_index, column_index), cell in table.get_celld().items():
        cell.set_linewidth(0.6)
        if row_index == 0:
            cell.set_facecolor("#e9eef5")
            cell.set_text_props(weight="bold")
        elif column_index > 0:
            text = cell.get_text().get_text()
            cell.set_facecolor("#e7f4ea" if text == "✓" else "#f9e3e3")


def render_plot(rows: list[dict[str, Any]], output_path: Path, title: str) -> None:
    labels = [row["label"] for row in rows]
    highest_ppm = [row["highest_ppm"] for row in rows]
    highest_delta = [row["highest_delta_ppm"] for row in rows]
    completed = [row["completed_inspections"] for row in rows]

    figure, axes = plt.subplots(2, 2, figsize=(13.2, 8.8))
    figure.subplots_adjust(left=0.07, right=0.97, top=0.88, bottom=0.12, hspace=0.38, wspace=0.25)
    figure.suptitle(title)

    render_bar_panel(
        axes[0][0],
        labels,
        highest_ppm,
        "Highest ppm",
        "ppm",
        "#4c78a8",
        "{:.2f}",
    )
    render_bar_panel(
        axes[0][1],
        labels,
        highest_delta,
        "Highest delta ppm",
        "delta ppm",
        "#f58518",
        "{:.2f}",
    )
    render_bar_panel(
        axes[1][0],
        labels,
        completed,
        "Completed inspections",
        "count",
        "#54a24b",
        "{:.0f}",
    )
    render_status_dashboard(axes[1][1], rows)

    figure.text(
        0.5,
        0.045,
        "Comparison is based on sparse room-level inspection summaries.",
        ha="center",
        fontsize=9,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def main() -> int:
    args = parse_args()
    rows, error = load_rows(args.input)
    if error is not None:
        print(error)
        return 1

    render_plot(rows, args.output, args.title)
    print(f"Experiment comparison written: {args.output}")
    print(f"experiment_rows={len(rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
