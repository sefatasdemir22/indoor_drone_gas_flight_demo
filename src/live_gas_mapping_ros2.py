#!/usr/bin/env python3
"""Generate simulated gas mapping output from PX4 ROS2 local position telemetry.

This script does not use MAVSDK and does not command the drone. It subscribes to
/fmu/out/vehicle_local_position_v1, computes simulated gas ppm values at the
reported local position, writes CSV/JSON output, and renders a heatmap with the
existing demo mapper.
"""

from __future__ import annotations

import argparse
import csv
import json
import random
import subprocess
import sys
from pathlib import Path
from typing import Any

from demo_tools.gas_sensor_node import (
    POSSIBLE_GAS_ZONES,
    ROUTE_WAYPOINTS,
    SCENARIO_CHOICES,
    compute_ppm,
    resolve_scenario,
)


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_TOPIC = "/fmu/out/vehicle_local_position_v1"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Listen to PX4 ROS2 local position telemetry and generate simulated gas heatmap outputs."
    )
    parser.add_argument("--scenario", default="random", choices=SCENARIO_CHOICES)
    parser.add_argument("--duration-seconds", type=float, default=30.0)
    parser.add_argument("--sample-rate-hz", type=float, default=5.0)
    parser.add_argument("--output-dir", type=Path, default=PROJECT_DIR / "results")
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--vmin", type=float, default=0.0)
    parser.add_argument("--vmax", type=float, default=130.0)
    parser.add_argument("--seed", type=int, default=None, help="Optional deterministic seed for gas scenario/noise.")
    parser.add_argument("--background-ppm", type=float, default=5.0)
    parser.add_argument("--peak-ppm", type=float, default=120.0)
    parser.add_argument("--sigma", type=float, default=1.8)
    parser.add_argument("--noise-std", type=float, default=1.2)
    return parser.parse_args()


def active_source_summary(active_sources: list[dict[str, Any]]) -> str:
    if not active_sources:
        return "none"
    return ", ".join(f"{source['name']}({float(source['x']):.1f},{float(source['y']):.1f})" for source in active_sources)


def render_heatmap(csv_path: Path, heatmap_path: Path, vmin: float, vmax: float) -> int:
    command = [
        sys.executable,
        str(PROJECT_DIR / "src" / "demo_tools" / "gas_mapper_node.py"),
        "--input",
        str(csv_path),
        "--output",
        str(heatmap_path),
        "--vmin",
        str(vmin),
        "--vmax",
        str(vmax),
    ]
    return subprocess.call(command)


def write_scenario_info(path: Path, info: dict[str, object]) -> None:
    path.write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")


def csv_fieldnames() -> list[str]:
    return [
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
    ]


def build_sample_row(
    sample_index: int,
    timestamp_sec: float,
    x: float,
    y: float,
    z: float,
    ppm: float,
    scenario: str,
    active_sources: list[dict[str, Any]],
    nearest_distance: float | None,
) -> dict[str, str | int]:
    active_zones = [str(source["name"]) for source in active_sources]
    first_source = active_sources[0] if active_sources else None
    distance_text = "" if nearest_distance is None else f"{nearest_distance:.3f}"
    return {
        "sample_index": sample_index,
        "timestamp_sec": f"{timestamp_sec:.3f}",
        "x": f"{x:.3f}",
        "y": f"{y:.3f}",
        "z": f"{z:.3f}",
        "ppm": f"{ppm:.3f}",
        "scenario": scenario,
        "active_zones": "|".join(active_zones),
        "active_sources_json": json.dumps(active_sources, separators=(",", ":")),
        "nearest_source_distance": distance_text,
        "active_zone": first_source["name"] if first_source else "",
        "active_source_x": f"{float(first_source['x']):.3f}" if first_source else "",
        "active_source_y": f"{float(first_source['y']):.3f}" if first_source else "",
        "distance_to_source": distance_text,
    }


def main() -> int:
    args = parse_args()
    if args.duration_seconds <= 0:
        print("--duration-seconds must be greater than 0")
        return 2
    if args.sample_rate_hz <= 0:
        print("--sample-rate-hz must be greater than 0")
        return 2
    if args.vmax <= args.vmin:
        print("--vmax must be greater than --vmin")
        return 2

    try:
        import rclpy
        from px4_msgs.msg import VehicleLocalPosition
        from rclpy.node import Node
        from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    except Exception as exc:
        print(f"Could not import ROS2/PX4 Python dependencies: {exc}")
        print("Make sure ROS2 Humble is sourced and px4_msgs is available, for example:")
        print("  source /opt/ros/humble/setup.bash")
        print("  source ~/araswarm_ws/install/setup.bash")
        return 1

    rng = random.Random(args.seed) if args.seed is not None else random.Random()
    scenario, active_sources = resolve_scenario(args.scenario, rng)
    active_zones = [str(source["name"]) for source in active_sources]

    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / "live_ros2_gas_samples.csv"
    scenario_info_path = args.output_dir / "live_ros2_scenario_info.json"
    heatmap_path = args.output_dir / "live_ros2_gas_heatmap.png"

    print("[live_gas_mapping_ros2] ROS2 telemetry gas mapping")
    print(f"[topic] {args.topic}")
    print("[topic] subscription QoS: BEST_EFFORT / VOLATILE / KEEP_LAST depth=10")
    print(f"[scenario] requested: {args.scenario}")
    print(f"[scenario] resolved: {scenario}")
    print(f"[scenario] active gas sources: {active_source_summary(active_sources)}")

    qos_profile = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

    class LiveGasMappingNode(Node):
        def __init__(self) -> None:
            super().__init__("live_gas_mapping_ros2")
            self.sample_count = 0
            self.latest_msg: VehicleLocalPosition | None = None
            self.started_sec = self.get_clock().now().nanoseconds / 1.0e9
            self.csv_handle = csv_path.open("w", newline="", encoding="utf-8")
            self.writer = csv.DictWriter(self.csv_handle, fieldnames=csv_fieldnames())
            self.writer.writeheader()
            self.create_subscription(VehicleLocalPosition, args.topic, self.position_callback, qos_profile)
            self.create_timer(1.0 / args.sample_rate_hz, self.sample_timer_callback)

        def position_callback(self, msg: VehicleLocalPosition) -> None:
            self.latest_msg = msg

        def sample_timer_callback(self) -> None:
            now_sec = self.get_clock().now().nanoseconds / 1.0e9
            elapsed_sec = now_sec - self.started_sec
            if elapsed_sec >= args.duration_seconds:
                return
            if self.latest_msg is None:
                if self.sample_count == 0:
                    self.get_logger().warn(f"Waiting for {args.topic} messages...")
                return

            x = float(self.latest_msg.x)
            y = float(self.latest_msg.y)
            z = max(0.0, -float(self.latest_msg.z))
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
            self.writer.writerow(
                build_sample_row(
                    sample_index=self.sample_count,
                    timestamp_sec=elapsed_sec,
                    x=x,
                    y=y,
                    z=z,
                    ppm=ppm,
                    scenario=scenario,
                    active_sources=active_sources,
                    nearest_distance=nearest_distance,
                )
            )
            self.sample_count += 1
            if self.sample_count == 1 or self.sample_count % max(1, int(args.sample_rate_hz * 5)) == 0:
                self.get_logger().info(
                    f"sample_count={self.sample_count} x={x:.2f} y={y:.2f} z={z:.2f} ppm={ppm:.2f}"
                )

        def close(self) -> None:
            self.csv_handle.flush()
            self.csv_handle.close()

    rclpy.init()
    node = LiveGasMappingNode()
    try:
        deadline_sec = node.get_clock().now().nanoseconds / 1.0e9 + args.duration_seconds
        while rclpy.ok() and node.get_clock().now().nanoseconds / 1.0e9 < deadline_sec:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\n[live_gas_mapping_ros2] interrupted by user")
    finally:
        sample_count = node.sample_count
        node.close()
        node.destroy_node()
        rclpy.shutdown()

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
            "sample_count": sample_count,
            "sample_rate_hz": args.sample_rate_hz,
            "duration_seconds": args.duration_seconds,
            "seed": args.seed,
            "ros2_topic": args.topic,
            "coordinate_frame": "PX4 VehicleLocalPosition: x, y, z converted to altitude=-z",
            "possible_gas_zones": {name: {"x": xy[0], "y": xy[1]} for name, xy in POSSIBLE_GAS_ZONES.items()},
            "reference_route_waypoints": [{"x": x, "y": y} for x, y in ROUTE_WAYPOINTS],
        },
    )

    if sample_count == 0:
        print(f"[warning] No samples collected from {args.topic}.")
        print("Check that PX4/Gazebo, MicroXRCEAgent, ROS2 environment, and px4_msgs are running/sourced.")
        print(f"[output] CSV path: {csv_path}")
        print(f"[output] scenario info path: {scenario_info_path}")
        return 1

    print(f"[output] samples collected: {sample_count}")
    print(f"[output] CSV path: {csv_path}")
    print(f"[output] scenario info path: {scenario_info_path}")

    if sample_count < 2:
        print("[heatmap] not enough samples to render a heatmap")
        return 1

    mapper_status = render_heatmap(csv_path, heatmap_path, args.vmin, args.vmax)
    if mapper_status == 0:
        print(f"[output] heatmap path: {heatmap_path}")
    else:
        print("[heatmap] heatmap rendering failed; CSV and scenario JSON were still written")
    return mapper_status


if __name__ == "__main__":
    raise SystemExit(main())
