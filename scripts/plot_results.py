#!/usr/bin/env python3
"""Compatibility wrapper for generating the gas heatmap.

The main plotting implementation lives in src/demo_tools/gas_mapper_node.py so the demo
has one mapping path. This wrapper keeps the older command usable.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> int:
    project_dir = Path(__file__).resolve().parents[1]
    input_path = Path(sys.argv[1]) if len(sys.argv) > 1 else project_dir / "results" / "gas_samples.csv"
    output_path = project_dir / "results" / "gas_heatmap.png"

    if len(sys.argv) > 2:
        print("Usage: python3 scripts/plot_results.py [results/gas_samples.csv]")
        return 2

    command = [
        sys.executable,
        str(project_dir / "src" / "demo_tools" / "gas_mapper_node.py"),
        "--input",
        str(input_path),
        "--output",
        str(output_path),
    ]
    return subprocess.call(command)


if __name__ == "__main__":
    raise SystemExit(main())
