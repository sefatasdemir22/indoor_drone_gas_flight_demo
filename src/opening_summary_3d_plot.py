#!/usr/bin/env python3
"""Render sparse event-average room inspection samples as a 3D PNG summary."""

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
from matplotlib.colors import Normalize

try:
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 - registers the 3D projection
except ImportError as exc:
    MLP3D_IMPORT_ERROR: ImportError | None = exc
else:
    MLP3D_IMPORT_ERROR = None


def parse_args() -> argparse.Namespace:
    project_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Generate a sparse 3D event-average gas inspection PNG from CSV samples."
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
        default=project_dir / "results" / "opening_event_summary_3d.png",
        help="Output PNG path.",
    )
    parser.add_argument(
        "--title",
        default="3D Sparse Room Inspection Visualization",
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


def z_limits(values: list[float]) -> tuple[float, float]:
    lower = min(0.0, min(values))
    upper = max(0.0, max(values))
    if lower == upper:
        upper = 1.0
    pad = max((upper - lower) * 0.06, 0.35)
    return lower - pad, upper + pad


def legend_text(rows: list[dict[str, Any]]) -> str:
    entries = [f"{row['short_label']} -> {row['opening_id']}" for row in rows]
    return "    ".join(entries)


def render_plot(rows: list[dict[str, Any]], output_path: Path, title: str) -> None:
    east = [row["east_m"] for row in rows]
    north = [row["north_m"] for row in rows]
    ppm = [row["inspection_avg_ppm"] for row in rows]
    delta = [row["delta_ppm"] for row in rows]

    figure = plt.figure(figsize=(12.0, 8.6))
    figure.subplots_adjust(left=0.01, right=0.9, top=0.88, bottom=0.18)
    axis = figure.add_subplot(111, projection="3d")
    figure.suptitle(title)

    norm = Normalize(vmin=min(ppm), vmax=max(ppm))
    cmap = plt.get_cmap("viridis")
    colors = cmap(norm(ppm))

    for x_value, y_value, z_value, color in zip(east, north, delta, colors, strict=True):
        axis.plot(
            [x_value, x_value],
            [y_value, y_value],
            [0.0, z_value],
            color=color,
            linewidth=3.8,
            alpha=0.76,
        )

    scatter = axis.scatter(
        east,
        north,
        delta,
        c=ppm,
        cmap=cmap,
        norm=norm,
        s=135,
        edgecolors="black",
        linewidths=0.9,
        depthshade=True,
    )

    for row in rows:
        axis.text(
            row["east_m"],
            row["north_m"],
            row["delta_ppm"],
            f" {row['short_label']}",
            fontsize=10,
            weight="bold",
        )

    axis.set_title("Room inspection position with delta-ppm height")
    axis.set_xlabel("East (m)", labelpad=8)
    axis.set_ylabel("North (m)", labelpad=8)
    axis.set_zlabel("Delta ppm from baseline", labelpad=8)
    axis.set_xlim(*expand_limits(east))
    axis.set_ylim(*expand_limits(north))
    axis.set_zlim(*z_limits(delta))
    axis.view_init(elev=22, azim=-54)
    axis.grid(True, alpha=0.3)
    axis.tick_params(axis="both", which="major", labelsize=9)
    axis.tick_params(axis="z", which="major", labelsize=9)

    colorbar = figure.colorbar(scatter, ax=axis, pad=0.08, shrink=0.68)
    colorbar.set_label("Inspection average (ppm)")

    figure.text(
        0.5,
        0.085,
        "Sparse room inspection averages; not a continuous 3D gas concentration map.",
        ha="center",
        fontsize=9,
    )
    figure.text(0.5, 0.045, legend_text(rows), ha="center", fontsize=8)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def main() -> int:
    args = parse_args()
    if MLP3D_IMPORT_ERROR is not None:
        print(f"Could not import Matplotlib 3D projection: {MLP3D_IMPORT_ERROR}")
        print("This environment likely mixes user matplotlib with system mpl_toolkits.")
        print("Try:")
        print("  PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/indoor_drone_matplotlib \\")
        print("  python3 src/opening_summary_3d_plot.py")
        return 3

    if not args.input.exists():
        print(f"Input CSV not found: {args.input}")
        return 1

    rows, skipped = load_rows(args.input)
    if not rows:
        print(f"No valid event-average samples found in: {args.input}")
        return 2

    render_plot(rows, args.output, args.title)
    print(f"3D summary written: {args.output}")
    print(f"sample_rows={len(rows)}")
    print(f"skipped_rows={skipped}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
