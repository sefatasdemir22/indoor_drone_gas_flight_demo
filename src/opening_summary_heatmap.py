#!/usr/bin/env python3
"""Render sparse event-average gas inspection samples as a PNG summary."""

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
from matplotlib.colors import Normalize, TwoSlopeNorm


def parse_args() -> argparse.Namespace:
    project_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Generate a sparse event-average gas inspection PNG from CSV samples."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=project_dir / "results" / "opening_room_samples.csv",
        help="Input CSV produced by opening_events_to_csv.py.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=project_dir / "results" / "opening_event_summary_heatmap.png",
        help="Output PNG path.",
    )
    parser.add_argument(
        "--title",
        default="Sparse Event-Average Gas Inspection Summary",
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


def load_rows(input_path: Path) -> tuple[list[dict[str, Any]], int]:
    rows: list[dict[str, Any]] = []
    skipped = 0
    side_counts: dict[str, int] = {}
    with input_path.open(newline="", encoding="utf-8") as csv_file:
        for raw in csv.DictReader(csv_file):
            north = as_float(raw.get("north_m"))
            east = as_float(raw.get("east_m"))
            ppm = as_float(raw.get("inspection_avg_ppm"))
            delta = as_float(raw.get("delta_ppm"))
            if north is None or east is None or ppm is None or delta is None:
                skipped += 1
                continue
            side = raw.get("side") or "?"
            side_key = side.lower()
            side_prefix = "L" if side_key == "left" else "R" if side_key == "right" else "O"
            side_counts[side_prefix] = side_counts.get(side_prefix, 0) + 1
            short_label = f"{side_prefix}{side_counts[side_prefix]}"
            rows.append(
                {
                    "north_m": north,
                    "east_m": east,
                    "inspection_avg_ppm": ppm,
                    "delta_ppm": delta,
                    "opening_id": raw.get("opening_id") or raw.get("inspection_index") or "?",
                    "side": side,
                    "short_label": short_label,
                    "sample_confidence": raw.get("sample_confidence") or "?",
                }
            )
    return rows, skipped


def expand_limits(values: list[float]) -> tuple[float, float]:
    lower = min(values)
    upper = max(values)
    if lower == upper:
        pad = max(abs(lower) * 0.1, 1.0)
    else:
        pad = (upper - lower) * 0.12
    return lower - pad, upper + pad


def delta_norm(values: list[float]) -> TwoSlopeNorm | Normalize:
    max_abs = max(abs(value) for value in values)
    if max_abs > 0:
        return TwoSlopeNorm(vmin=-max_abs, vcenter=0.0, vmax=max_abs)
    return Normalize(vmin=-1.0, vmax=1.0)


def annotate_points(axis: Any, rows: list[dict[str, Any]]) -> None:
    for row in rows:
        axis.annotate(
            row["short_label"],
            (row["east_m"], row["north_m"]),
            xytext=(6, 6),
            textcoords="offset points",
            fontsize=9,
            weight="bold",
        )


def legend_text(rows: list[dict[str, Any]]) -> str:
    entries = [f"{row['short_label']} -> {row['opening_id']}" for row in rows]
    return "\n".join(entries)


def render_plot(rows: list[dict[str, Any]], output_path: Path, title: str) -> None:
    east = [row["east_m"] for row in rows]
    north = [row["north_m"] for row in rows]
    ppm = [row["inspection_avg_ppm"] for row in rows]
    delta = [row["delta_ppm"] for row in rows]

    figure, axes = plt.subplots(1, 2, figsize=(13.5, 6.8))
    figure.subplots_adjust(left=0.07, right=0.94, top=0.86, bottom=0.24, wspace=0.28)
    figure.suptitle(title)
    figure.text(
        0.5,
        0.13,
        "Sparse event averages from room inspections; not a continuous gas concentration map.",
        ha="center",
        fontsize=9,
    )
    figure.text(0.5, 0.04, legend_text(rows), ha="center", va="bottom", fontsize=8)

    panels = [
        (
            axes[0],
            ppm,
            "Inspection average ppm",
            "Inspection average (ppm)",
            "viridis",
            Normalize(vmin=min(ppm), vmax=max(ppm)),
        ),
        (
            axes[1],
            delta,
            "Delta ppm from baseline",
            "Delta from baseline (ppm)",
            "coolwarm",
            delta_norm(delta),
        ),
    ]

    xlim = expand_limits(east)
    ylim = expand_limits(north)
    for axis, values, panel_title, colorbar_label, cmap, norm in panels:
        scatter = axis.scatter(
            east,
            north,
            c=values,
            cmap=cmap,
            norm=norm,
            s=210,
            edgecolors="black",
            linewidths=0.8,
        )
        annotate_points(axis, rows)
        axis.set_title(panel_title)
        axis.set_xlabel("East (m)")
        axis.set_ylabel("North (m)")
        axis.set_xlim(*xlim)
        axis.set_ylim(*ylim)
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, alpha=0.3)
        colorbar = figure.colorbar(scatter, ax=axis)
        colorbar.set_label(colorbar_label)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def main() -> int:
    args = parse_args()
    if not args.input.exists():
        print(f"Input CSV not found: {args.input}")
        return 1

    rows, skipped = load_rows(args.input)
    if not rows:
        print(f"No valid event-average samples found in: {args.input}")
        return 2

    render_plot(rows, args.output, args.title)
    print(f"Summary heatmap written: {args.output}")
    print(f"sample_rows={len(rows)}")
    print(f"skipped_rows={skipped}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
