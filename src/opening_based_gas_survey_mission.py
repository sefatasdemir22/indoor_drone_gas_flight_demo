#!/usr/bin/env python3
"""Opening-based gas survey mission prototype.

Dry-run and scan-monitor modes never command a drone. MAVSDK modes import
dependencies lazily and verify scan readiness before any flight command.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import random
import time
from pathlib import Path
from typing import Any

from opening_event_builder import (
    build_body_inspection_event,
    build_position_abort_event,
    build_position_inspection_event,
)
from opening_mission_types import (
    Decision,
    DryRunSummary,
    FrontSectorStats,
    GasSampleSummary,
    MissionMemory,
    MissionState,
    OpeningCandidate,
    OpeningCandidateState,
    PositionOpeningAnchor,
    ScanSnapshot,
    TakeoffAltitudeResult,
)
from opening_scan_decision import (
    action_message,
    build_scan_snapshot,
    decide_corridor_action,
    decide_opening_probe,
    format_distance,
    log_scan,
    log_scan_monitor_snapshot,
    mock_scan_for_scenario,
    scan_ready_for_mission,
    scan_stats,
    side_avg_for_snapshot,
    side_decision_diagnostics,
    side_min_for_snapshot,
    side_open_for_corridor_detection,
    side_open_for_probe_confirm,
)

DEFAULT_SYSTEM_ADDRESS = "udpin://0.0.0.0:14540"


OPENING_CANDIDATES: tuple[OpeningCandidate, ...] = (
    OpeningCandidate("left_room_like_opening", corridor_x=5.0, side="left", opening_score=0.78),
    OpeningCandidate("right_room_like_opening", corridor_x=9.0, side="right", opening_score=0.72),
    OpeningCandidate("forward_left_like_opening", corridor_x=13.0, side="left", opening_score=0.66),
)

DRY_RUN_SCENARIOS = (
    "clear_corridor",
    "normal_corridor_side_distance",
    "left_opening",
    "right_opening",
    "front_blocked_left_open",
    "front_blocked_right_open",
    "front_blocked_both_blocked",
    "narrow_passage",
    "missing_side_scans",
    "all_inf_side_after_ready",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Dry-run an opening-based gas survey mission state flow."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run the deterministic planning/state simulation. No MAVSDK/ROS2 execution is implemented yet.",
    )
    parser.add_argument(
        "--scan-monitor",
        action="store_true",
        help="Read real ROS2 LaserScan topics and run decision logic without commanding the drone.",
    )
    parser.add_argument(
        "--takeoff-land-check",
        action="store_true",
        help="Wait for scan readiness, then run a MAVSDK takeoff/hover/land check. No corridor movement is commanded.",
    )
    parser.add_argument(
        "--corridor-follow-check",
        action="store_true",
        help="Run a low-speed body-frame forward movement check with front scan safety.",
    )
    parser.add_argument(
        "--room-inspection-check",
        action="store_true",
        help="Run corridor-follow with short opening inspections and simulated gas candidate event logging.",
    )
    parser.add_argument(
        "--position-room-inspection-check",
        action="store_true",
        help="Run a position-setpoint corridor and room inspection check using MAVSDK PositionNedYaw.",
    )
    parser.add_argument(
        "--position-side-sign-check",
        action="store_true",
        help="Run a small PositionNedYaw left/right sign diagnostic without room inspection.",
    )
    parser.add_argument(
        "--axis-calibration-check",
        action="store_true",
        help="Run offboard zero-hover plus short north/east NED pulses to identify the usable corridor axis.",
    )
    parser.add_argument("--dry-run-scenario", choices=DRY_RUN_SCENARIOS)
    parser.add_argument("--seed", type=int, default=None, help="Seed for repeatable dry-run decisions.")
    parser.add_argument("--max-openings", type=int, default=3)
    parser.add_argument("--max-corridor-x", type=float, default=14.0)
    parser.add_argument("--corridor-step", type=float, default=1.0)
    parser.add_argument("--side-open-distance", type=float, default=2.2)
    parser.add_argument("--side-confirm-distance", type=float, default=1.4)
    parser.add_argument("--side-stop-distance", type=float, default=0.7)
    parser.add_argument("--front-stop-distance", type=float, default=1.0)
    parser.add_argument("--front-clear-distance", type=float, default=1.3)
    parser.add_argument("--front-unavailable-retry-seconds", type=float, default=1.0)
    parser.add_argument("--front-unavailable-retry-rate-hz", type=float, default=5.0)
    parser.add_argument("--front-unavailable-max-events", type=int, default=3)
    parser.add_argument("--min-valid-samples", type=int, default=5)
    parser.add_argument("--min-valid-ratio", type=float, default=0.35)
    parser.add_argument("--opening-confirm-frames", type=int, default=2)
    parser.add_argument("--front-scan-topic", default="/drone/front_scan")
    parser.add_argument("--left-scan-topic", default="/drone/left_scan")
    parser.add_argument("--right-scan-topic", default="/drone/right_scan")
    parser.add_argument("--front-decision-scan-topic", default="/drone/front_decision_scan")
    parser.add_argument("--left-decision-scan-topic", default="/drone/left_decision_scan")
    parser.add_argument("--right-decision-scan-topic", default="/drone/right_decision_scan")
    parser.add_argument("--scan-monitor-duration-seconds", type=float, default=10.0)
    parser.add_argument("--scan-log-rate-hz", type=float, default=2.0)
    parser.add_argument("--scan-warmup-seconds", type=float, default=0.5)
    parser.add_argument("--scan-ready-timeout-seconds", type=float, default=2.0)
    parser.add_argument(
        "--skip-scan-ready-check",
        action="store_true",
        help="Skip the preflight front/left/right scan readiness gate in --takeoff-land-check mode.",
    )
    parser.add_argument(
        "--system-address",
        default=DEFAULT_SYSTEM_ADDRESS,
        help=f"MAVSDK system address. Default: {DEFAULT_SYSTEM_ADDRESS}",
    )
    parser.add_argument("--connection-timeout", type=float, default=30.0)
    parser.add_argument("--takeoff-altitude", type=float, default=1.2)
    parser.add_argument("--takeoff-timeout", type=float, default=25.0)
    parser.add_argument("--takeoff-altitude-tolerance", type=float, default=0.40)
    parser.add_argument("--min-takeoff-confirm-altitude", type=float, default=0.70)
    parser.add_argument("--post-takeoff-settle-seconds", type=float, default=2.0)
    parser.add_argument("--pre-land-settle-seconds", type=float, default=0.75)
    parser.add_argument("--hover-seconds", type=float, default=5.0)
    parser.add_argument("--corridor-step-count", type=int, default=3)
    parser.add_argument("--corridor-step-duration-seconds", type=float, default=1.5)
    parser.add_argument("--corridor-north-speed", type=float, default=0.15)
    parser.add_argument("--corridor-east-speed", type=float, default=0.0)
    parser.add_argument("--body-forward-speed", type=float, default=0.12)
    parser.add_argument("--body-right-speed", type=float, default=0.0)
    parser.add_argument("--body-down-speed", type=float, default=0.0)
    parser.add_argument("--body-yawspeed", type=float, default=0.0)
    parser.add_argument("--enable-corridor-centering", action="store_true")
    parser.add_argument("--corridor-center-kp", type=float, default=0.08)
    parser.add_argument("--corridor-center-max-right-speed", type=float, default=0.08)
    parser.add_argument("--corridor-center-deadband", type=float, default=0.15)
    parser.add_argument("--enable-altitude-hold", action="store_true")
    parser.add_argument("--target-altitude", type=float, default=1.0)
    parser.add_argument("--altitude-hold-kp", type=float, default=0.25)
    parser.add_argument("--altitude-hold-max-down-speed", type=float, default=0.15)
    parser.add_argument(
        "--enable-opening-probe",
        action="store_true",
        help="After a left/right opening is detected during corridor follow, run one short lateral body-frame probe.",
    )
    parser.add_argument("--probe-side-speed", type=float, default=0.12)
    parser.add_argument("--probe-duration-seconds", type=float, default=1.5)
    parser.add_argument("--probe-max-count", type=int, default=1)
    parser.add_argument("--max-inspections", type=int, default=2)
    parser.add_argument("--inspection-side-speed", type=float, default=0.12)
    parser.add_argument("--inspection-enter-seconds", type=float, default=1.8)
    parser.add_argument("--inspection-hover-seconds", type=float, default=3.0)
    parser.add_argument("--inspection-exit-seconds", type=float, default=1.8)
    parser.add_argument("--inspection-enter-distance", type=float, default=0.5)
    parser.add_argument("--inspection-exit-tolerance", type=float, default=0.5)
    parser.add_argument("--inspection-enter-timeout-seconds", type=float, default=8.0)
    parser.add_argument("--inspection-exit-timeout-seconds", type=float, default=8.0)
    parser.add_argument("--inspection-cooldown-steps", type=int, default=2)
    parser.add_argument("--opening-alignment-forward-distance", type=float, default=0.5)
    parser.add_argument("--opening-alignment-timeout-seconds", type=float, default=4.0)
    parser.add_argument("--opening-alignment-confirm-frames", type=int, default=2)
    parser.add_argument("--disable-opening-alignment", action="store_true")
    parser.add_argument("--opening-min-forward-progress", type=float, default=1.0)
    parser.add_argument("--opening-min-persistence-frames", type=int, default=3)
    parser.add_argument("--opening-peak-drop-distance", type=float, default=0.25)
    parser.add_argument("--opening-require-front-clear", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--gas-scenario", default="possible_gas_zone_4")
    parser.add_argument("--gas-seed", type=int, default=1)
    parser.add_argument("--gas-sample-rate-hz", type=float, default=5.0)
    parser.add_argument("--baseline-sample-seconds", type=float, default=1.5)
    parser.add_argument("--inspection-sample-seconds", type=float, default=3.0)
    parser.add_argument("--gas-delta-threshold", type=float, default=15.0)
    parser.add_argument("--gas-absolute-threshold", type=float, default=35.0)
    parser.add_argument("--background-ppm", type=float, default=5.0)
    parser.add_argument("--peak-ppm", type=float, default=120.0)
    parser.add_argument("--sigma", type=float, default=1.8)
    parser.add_argument("--noise-std", type=float, default=1.2)
    parser.add_argument("--inspection-events-output", type=Path, default=Path("results") / "opening_inspection_events.json")
    parser.add_argument("--move-rate-hz", type=float, default=10.0)
    parser.add_argument("--pause-between-steps", type=float, default=0.75)
    parser.add_argument("--front-sector-deg", type=float, default=35.0)
    parser.add_argument("--axis-calibration-speed", type=float, default=0.15)
    parser.add_argument("--axis-calibration-duration", type=float, default=1.5)
    parser.add_argument("--offboard-zero-hover-seconds", type=float, default=2.0)
    parser.add_argument("--offboard-warmup-seconds", type=float, default=1.5)
    parser.add_argument("--position-step-count", type=int, default=6)
    parser.add_argument("--position-forward-step", type=float, default=0.6)
    parser.add_argument("--position-hold-seconds", type=float, default=2.5)
    parser.add_argument("--position-altitude", type=float, default=1.2)
    parser.add_argument("--position-yaw", type=float, default=90.0)
    parser.add_argument("--position-room-entry-distance", type=float, default=1.5)
    parser.add_argument("--position-room-entry-hold-seconds", type=float, default=2.5)
    parser.add_argument("--enable-no-backtrack-door-capture", action="store_true")
    parser.add_argument("--door-capture-confirm-frames", type=int, default=3)
    parser.add_argument("--door-capture-crawl-step", type=float, default=0.15)
    parser.add_argument("--door-capture-max-crawl-steps", type=int, default=2)
    parser.add_argument("--door-capture-hold-seconds", type=float, default=0.8)
    parser.add_argument("--enable-sensor-room-traversal", action="store_true")
    parser.add_argument("--room-traverse-stop-distance", type=float, default=0.75)
    parser.add_argument("--room-traverse-step-distance", type=float, default=0.25)
    parser.add_argument("--room-traverse-max-distance", type=float, default=3.0)
    parser.add_argument("--room-traverse-hold-seconds", type=float, default=0.8)
    parser.add_argument("--enable-room-facing-yaw-entry", action="store_true")
    parser.add_argument("--room-facing-step-distance", type=float, default=0.25)
    parser.add_argument("--room-facing-max-distance", type=float, default=3.0)
    parser.add_argument("--room-facing-front-stop-distance", type=float, default=0.75)
    parser.add_argument("--room-facing-exit-step-distance", type=float, default=0.5)
    parser.add_argument("--room-facing-step-hold-seconds", type=float, default=0.8)
    parser.add_argument("--room-facing-yaw-settle-seconds", type=float, default=1.0)
    parser.add_argument("--room-facing-door-forward-offset", type=float, default=0.0)
    parser.add_argument("--room-facing-yaw-hold-before-seconds", type=float, default=0.5)
    parser.add_argument("--room-facing-yaw-hold-after-seconds", type=float, default=1.0)
    parser.add_argument("--room-facing-yaw-settle-repeat-count", type=int, default=5)
    parser.add_argument("--room-facing-yaw-settle-repeat-interval", type=float, default=0.2)
    parser.add_argument("--room-facing-yaw-interpolation-step-deg", type=float, default=15.0)
    parser.add_argument("--room-facing-yaw-interpolation-hold-seconds", type=float, default=0.2)
    parser.add_argument("--enable-room-facing-post-yaw-realign", action="store_true")
    parser.add_argument("--room-facing-post-yaw-forward-offset-step", type=float, default=0.10)
    parser.add_argument("--room-facing-post-yaw-max-forward-offset", type=float, default=0.30)
    parser.add_argument("--room-facing-post-yaw-min-front-clearance", type=float, default=2.0)
    parser.add_argument("--return-home", action="store_true", default=True)
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def log(tag: str, message: str) -> None:
    print(f"[{tag}] {message}", flush=True)


def config_summary(args: argparse.Namespace, seed_text: str) -> None:
    log("CONFIG", "opening-based gas survey mission dry-run")
    log("CONFIG", f"seed={seed_text}")
    log("CONFIG", f"max_openings={max(0, args.max_openings)}")
    log("CONFIG", f"max_corridor_x={max(0.0, args.max_corridor_x):.1f}")
    log("CONFIG", f"corridor_step={max(0.1, args.corridor_step):.1f}")
    log("CONFIG", f"side_open_distance={args.side_open_distance:.1f}")
    log("CONFIG", f"front_stop_distance={args.front_stop_distance:.1f}")
    log("CONFIG", f"return_home={args.return_home}")


class LaserScanMonitor:
    def __init__(self, rclpy: object, laser_scan_type: object, args: argparse.Namespace) -> None:
        self.rclpy = rclpy
        self.latest_front: object | None = None
        self.latest_left: object | None = None
        self.latest_right: object | None = None
        self.latest_front_decision: object | None = None
        self.latest_left_decision: object | None = None
        self.latest_right_decision: object | None = None
        self.node = self.rclpy.create_node("opening_based_scan_monitor")
        self.node.create_subscription(laser_scan_type, args.front_scan_topic, self._front_callback, 10)
        self.node.create_subscription(laser_scan_type, args.left_scan_topic, self._left_callback, 10)
        self.node.create_subscription(laser_scan_type, args.right_scan_topic, self._right_callback, 10)
        self.node.create_subscription(
            laser_scan_type,
            args.front_decision_scan_topic,
            self._front_decision_callback,
            10,
        )
        self.node.create_subscription(
            laser_scan_type,
            args.left_decision_scan_topic,
            self._left_decision_callback,
            10,
        )
        self.node.create_subscription(
            laser_scan_type,
            args.right_decision_scan_topic,
            self._right_decision_callback,
            10,
        )

    def _front_callback(self, msg: object) -> None:
        self.latest_front = msg

    def _left_callback(self, msg: object) -> None:
        self.latest_left = msg

    def _right_callback(self, msg: object) -> None:
        self.latest_right = msg

    def _front_decision_callback(self, msg: object) -> None:
        self.latest_front_decision = msg

    def _left_decision_callback(self, msg: object) -> None:
        self.latest_left_decision = msg

    def _right_decision_callback(self, msg: object) -> None:
        self.latest_right_decision = msg

    def has_all_messages(self) -> bool:
        return (
            self.latest_front is not None
            and self.latest_left is not None
            and self.latest_right is not None
            and self.latest_front_decision is not None
            and self.latest_left_decision is not None
            and self.latest_right_decision is not None
        )

    def snapshot(self) -> ScanSnapshot:
        front_stats = scan_stats(self.latest_front)
        left_stats = scan_stats(self.latest_left)
        right_stats = scan_stats(self.latest_right)
        return ScanSnapshot(
            front_min=front_stats.min_distance,
            left_min=left_stats.min_distance,
            right_min=right_stats.min_distance,
            left_avg=left_stats.avg_distance,
            right_avg=right_stats.avg_distance,
            front_ready=front_stats.ready,
            left_ready=left_stats.ready,
            right_ready=right_stats.ready,
            front_valid_count=front_stats.valid_count,
            left_valid_count=left_stats.valid_count,
            right_valid_count=right_stats.valid_count,
            front_finite_count=front_stats.finite_count,
            left_finite_count=left_stats.finite_count,
            right_finite_count=right_stats.finite_count,
            front_inf_count=front_stats.inf_count,
            left_inf_count=left_stats.inf_count,
            right_inf_count=right_stats.inf_count,
            front_valid_ratio=front_stats.valid_ratio,
            left_valid_ratio=left_stats.valid_ratio,
            right_valid_ratio=right_stats.valid_ratio,
        )

    def decision_snapshot(self) -> ScanSnapshot:
        front_stats = scan_stats(self.latest_front_decision)
        left_stats = scan_stats(self.latest_left_decision)
        right_stats = scan_stats(self.latest_right_decision)
        return ScanSnapshot(
            front_min=front_stats.min_distance,
            left_min=left_stats.min_distance,
            right_min=right_stats.min_distance,
            left_avg=left_stats.avg_distance,
            right_avg=right_stats.avg_distance,
            front_ready=front_stats.ready,
            left_ready=left_stats.ready,
            right_ready=right_stats.ready,
            front_valid_count=front_stats.valid_count,
            left_valid_count=left_stats.valid_count,
            right_valid_count=right_stats.valid_count,
            front_finite_count=front_stats.finite_count,
            left_finite_count=left_stats.finite_count,
            right_finite_count=right_stats.finite_count,
            front_inf_count=front_stats.inf_count,
            left_inf_count=left_stats.inf_count,
            right_inf_count=right_stats.inf_count,
            front_valid_ratio=front_stats.valid_ratio,
            left_valid_ratio=left_stats.valid_ratio,
            right_valid_ratio=right_stats.valid_ratio,
        )

    def close(self) -> None:
        self.node.destroy_node()


def run_dry_run_scenario(args: argparse.Namespace, seed: int) -> int:
    scenario = args.dry_run_scenario
    if scenario is None:
        return 0

    scan = mock_scan_for_scenario(scenario)
    memory = MissionMemory(
        visited_openings=set(),
        skipped_openings=set(),
        bypass_attempts=0,
        corridor_x=5.0,
        seed=seed,
    )
    if scenario == "left_opening":
        memory.left_open_frames = max(0, args.opening_confirm_frames - 1)
    elif scenario == "right_opening":
        memory.right_open_frames = max(0, args.opening_confirm_frames - 1)

    log_scan(scan)
    corridor_decision = decide_corridor_action(scan, memory, args)
    log("DECIDE", f"corridor_action={corridor_decision.value}")

    if corridor_decision == Decision.DETECT_LEFT_OPENING:
        opening_decision = decide_opening_probe(scan, "left", memory, args)
        log("DECIDE", f"opening_side=left opening_action={opening_decision.value}")
        log("ACTION", "simulated left opening probe" if opening_decision == Decision.PROBE_OPENING else "simulated left opening skip")
    elif corridor_decision == Decision.DETECT_RIGHT_OPENING:
        opening_decision = decide_opening_probe(scan, "right", memory, args)
        log("DECIDE", f"opening_side=right opening_action={opening_decision.value}")
        log("ACTION", "simulated right opening probe" if opening_decision == Decision.PROBE_OPENING else "simulated right opening skip")
    elif corridor_decision in {Decision.BYPASS_LEFT, Decision.BYPASS_RIGHT}:
        memory.bypass_attempts += 1
        log("ACTION", f"simulated {corridor_decision.value.lower()} attempt={memory.bypass_attempts}")
    elif corridor_decision == Decision.BLOCKED:
        log("ACTION", "simulated stop: corridor blocked")
    elif corridor_decision == Decision.NARROW_FORWARD:
        log("ACTION", "simulated slow forward through narrow passage")
    else:
        log("ACTION", "simulated follow forward")

    return 0


def run_scan_monitor(args: argparse.Namespace) -> int:
    try:
        import rclpy
        from sensor_msgs.msg import LaserScan
    except Exception as exc:
        print(f"Could not import ROS2 LaserScan dependencies: {exc}")
        print("Make sure ROS2 Humble is sourced before using --scan-monitor.")
        return 1

    duration_sec = max(0.1, args.scan_monitor_duration_seconds)
    log_rate_hz = max(0.1, args.scan_log_rate_hz)
    log_interval_sec = 1.0 / log_rate_hz
    warmup_sec = max(0.0, args.scan_warmup_seconds)
    ready_timeout_sec = max(warmup_sec, args.scan_ready_timeout_seconds)
    seed = args.seed if args.seed is not None else 0
    memory = MissionMemory(
        visited_openings=set(),
        skipped_openings=set(),
        bypass_attempts=0,
        corridor_x=0.0,
        seed=seed,
    )

    log("CONFIG", "opening-based scan monitor mode")
    log("CONFIG", f"front_scan_topic={args.front_scan_topic}")
    log("CONFIG", f"left_scan_topic={args.left_scan_topic}")
    log("CONFIG", f"right_scan_topic={args.right_scan_topic}")
    log("CONFIG", f"front_decision_scan_topic={args.front_decision_scan_topic}")
    log("CONFIG", f"left_decision_scan_topic={args.left_decision_scan_topic}")
    log("CONFIG", f"right_decision_scan_topic={args.right_decision_scan_topic}")
    log("CONFIG", f"duration_seconds={duration_sec:.1f}")
    log("CONFIG", f"scan_log_rate_hz={log_rate_hz:.1f}")
    log("CONFIG", f"scan_warmup_seconds={warmup_sec:.1f}")
    log("CONFIG", f"scan_ready_timeout_seconds={ready_timeout_sec:.1f}")

    rclpy.init()
    monitor = LaserScanMonitor(rclpy, LaserScan, args)
    started_at = time.monotonic()
    next_log_at = started_at
    try:
        while rclpy.ok() and time.monotonic() - started_at < duration_sec:
            rclpy.spin_once(monitor.node, timeout_sec=0.05)
            now = time.monotonic()
            if now < next_log_at:
                continue
            elapsed_sec = now - started_at
            if elapsed_sec < warmup_sec or (not monitor.has_all_messages() and elapsed_sec < ready_timeout_sec):
                log(
                    "SCAN",
                    "waiting for scan readiness: "
                    f"front_ready={monitor.latest_front is not None} "
                    f"left_ready={monitor.latest_left is not None} "
                    f"right_ready={monitor.latest_right is not None} "
                    f"front_decision_ready={monitor.latest_front_decision is not None} "
                    f"left_decision_ready={monitor.latest_left_decision is not None} "
                    f"right_decision_ready={monitor.latest_right_decision is not None}",
                )
                log("ACTION", "simulated wait: scan data not ready")
                next_log_at = now + log_interval_sec
                continue
            scan = monitor.decision_snapshot()
            decision = decide_corridor_action(scan, memory, args)
            log_scan_monitor_snapshot(scan)
            log("DECIDE", f"corridor_action={decision.value}")
            if decision in {Decision.BYPASS_LEFT, Decision.BYPASS_RIGHT}:
                memory.bypass_attempts += 1
            log("ACTION", action_message(decision))
            next_log_at = now + log_interval_sec
    except KeyboardInterrupt:
        print("\n[scan-monitor] interrupted by user")
    finally:
        monitor.close()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:
            if "rcl_shutdown" not in str(exc) and "shutdown" not in str(exc).lower():
                raise

    return 0


def import_ros2_scan_dependencies() -> tuple[object, object] | None:
    try:
        import rclpy
        from sensor_msgs.msg import LaserScan
    except Exception as exc:
        print(f"Could not import ROS2 LaserScan dependencies: {exc}")
        print("Make sure ROS2 Humble is sourced before using scan readiness checks.")
        return None
    return rclpy, LaserScan


def import_mavsdk_system() -> object | None:
    try:
        from mavsdk import System
    except Exception as exc:
        print(f"Could not import MAVSDK: {exc}")
        print("Make sure MAVSDK-Python is installed before using --takeoff-land-check.")
        return None
    return System


def import_velocity_ned_yaw() -> object | None:
    try:
        from mavsdk.offboard import VelocityNedYaw
    except Exception as exc:
        print(f"Could not import MAVSDK offboard velocity type: {exc}")
        print("Make sure MAVSDK-Python offboard support is available before using --corridor-follow-check.")
        return None
    return VelocityNedYaw


def import_velocity_body_yawspeed() -> object | None:
    try:
        from mavsdk.offboard import VelocityBodyYawspeed
    except Exception as exc:
        print(f"Could not import MAVSDK body velocity type: {exc}")
        print("Make sure MAVSDK-Python offboard support is available before using --corridor-follow-check.")
        return None
    return VelocityBodyYawspeed


def import_position_ned_yaw() -> object | None:
    try:
        from mavsdk.offboard import PositionNedYaw
    except Exception as exc:
        print(f"Could not import MAVSDK position setpoint type: {exc}")
        print("Make sure MAVSDK-Python offboard support is available before using --position-room-inspection-check.")
        return None
    return PositionNedYaw


def import_gas_model() -> tuple[object, object, object] | None:
    try:
        from demo_tools.gas_sensor_node import POSSIBLE_GAS_ZONES, compute_ppm, resolve_scenario
    except Exception as exc:
        print(f"Could not import gas model helpers: {exc}")
        return None
    return compute_ppm, resolve_scenario, POSSIBLE_GAS_ZONES


def create_scan_monitor(args: argparse.Namespace) -> tuple[object, LaserScanMonitor] | None:
    dependencies = import_ros2_scan_dependencies()
    if dependencies is None:
        return None

    rclpy, laser_scan_type = dependencies
    if not rclpy.ok():
        rclpy.init()
    return rclpy, LaserScanMonitor(rclpy, laser_scan_type, args)


def close_scan_monitor(rclpy: object | None, monitor: LaserScanMonitor | None) -> None:
    if monitor is not None:
        monitor.close()
    if rclpy is None:
        return
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception as exc:
        if "rcl_shutdown" not in str(exc) and "shutdown" not in str(exc).lower():
            raise


async def wait_for_scan_readiness(rclpy: object, monitor: LaserScanMonitor, args: argparse.Namespace) -> bool:
    timeout_sec = max(0.1, args.scan_ready_timeout_seconds)
    log_interval_sec = 1.0 / max(0.1, args.scan_log_rate_hz)
    started_at = time.monotonic()
    next_log_at = started_at

    log("SCAN", f"waiting for scan readiness, timeout={timeout_sec:.1f}s")
    while rclpy.ok() and time.monotonic() - started_at < timeout_sec:
        rclpy.spin_once(monitor.node, timeout_sec=0.05)
        scan = monitor.decision_snapshot()
        if monitor.has_all_messages() and scan_ready_for_mission(scan, args):
            log_scan_monitor_snapshot(scan)
            log("SCAN", "front/left/right wide and decision scan readiness confirmed")
            return True

        now = time.monotonic()
        if now >= next_log_at:
            log(
                "SCAN",
                "waiting for scan readiness: "
                f"front_ready={monitor.latest_front is not None} "
                f"left_ready={monitor.latest_left is not None} "
                f"right_ready={monitor.latest_right is not None} "
                f"front_decision_ready={monitor.latest_front_decision is not None} "
                f"left_decision_ready={monitor.latest_left_decision is not None} "
                f"right_decision_ready={monitor.latest_right_decision is not None}",
            )
            next_log_at = now + log_interval_sec
        await asyncio.sleep(0.02)

    log("SCAN", "scan readiness timed out")
    return False


async def log_airborne_scan_checks(
    rclpy: object,
    monitor: LaserScanMonitor,
    args: argparse.Namespace,
    memory: MissionMemory,
    duration_sec: float,
) -> None:
    duration_sec = max(0.0, duration_sec)
    if duration_sec == 0.0:
        return

    log_interval_sec = 1.0 / max(0.1, args.scan_log_rate_hz)
    started_at = time.monotonic()
    next_log_at = started_at
    log("SCAN", f"airborne scan check during hover, duration={duration_sec:.1f}s")
    while rclpy.ok() and time.monotonic() - started_at < duration_sec:
        rclpy.spin_once(monitor.node, timeout_sec=0.05)
        now = time.monotonic()
        if now < next_log_at:
            await asyncio.sleep(0.02)
            continue

        scan = monitor.decision_snapshot()
        decision = decide_corridor_action(scan, memory, args)
        log_scan_monitor_snapshot(scan)
        log("DECIDE", f"corridor_action={decision.value}")
        log("ACTION", action_message(decision))
        next_log_at = now + log_interval_sec
        await asyncio.sleep(0.02)


async def wait_for_mavsdk_connection(drone: object, timeout_sec: float) -> None:
    log("CONNECT", f"waiting for PX4 connection, timeout={timeout_sec:.1f}s")

    async def _wait() -> None:
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    await asyncio.wait_for(_wait(), timeout=timeout_sec)
    log("CONNECT", "PX4 connection established")


async def wait_for_mavsdk_health(drone: object, timeout_sec: float) -> None:
    log("CONNECT", f"waiting for vehicle health, timeout={timeout_sec:.1f}s")

    async def _wait() -> None:
        async for health in drone.telemetry.health():
            log(
                "CONNECT",
                "health: "
                f"global={health.is_global_position_ok}, "
                f"local={health.is_local_position_ok}, "
                f"home={health.is_home_position_ok}",
            )
            if health.is_global_position_ok and health.is_local_position_ok and health.is_home_position_ok:
                return

    await asyncio.wait_for(_wait(), timeout=timeout_sec)
    log("CONNECT", "vehicle health checks passed")


async def wait_until_takeoff_altitude(
    drone: object,
    target_altitude_m: float,
    timeout_sec: float,
    tolerance_m: float,
    min_confirm_altitude_m: float,
) -> TakeoffAltitudeResult:
    target_altitude_m = max(0.0, target_altitude_m)
    tolerance_m = max(0.05, tolerance_m)
    min_confirm_altitude_m = max(0.05, min_confirm_altitude_m)
    log(
        "TAKEOFF",
        "waiting for altitude "
        f"target={target_altitude_m:.1f} m, tolerance={tolerance_m:.2f} m, "
        f"min_confirm={min_confirm_altitude_m:.2f} m",
    )
    last_altitude_m = 0.0

    async def _wait() -> TakeoffAltitudeResult:
        nonlocal last_altitude_m
        last_log_time = 0.0
        async for position_velocity in drone.telemetry.position_velocity_ned():
            altitude_estimate = abs(position_velocity.position.down_m)
            last_altitude_m = altitude_estimate
            now = time.monotonic()
            if now - last_log_time >= 1.0:
                log("TAKEOFF", f"current altitude estimate: {altitude_estimate:.2f} m")
                last_log_time = now
            target_band_reached = altitude_estimate >= max(0.0, target_altitude_m - tolerance_m)
            safe_hover_altitude = altitude_estimate >= min_confirm_altitude_m
            if target_band_reached and safe_hover_altitude:
                return TakeoffAltitudeResult(
                    confirmed=True,
                    safe_hover_altitude=True,
                    last_altitude_m=last_altitude_m,
                )
        return TakeoffAltitudeResult(
            confirmed=False,
            safe_hover_altitude=last_altitude_m >= min_confirm_altitude_m,
            last_altitude_m=last_altitude_m,
        )

    try:
        return await asyncio.wait_for(_wait(), timeout=max(0.1, timeout_sec))
    except asyncio.TimeoutError:
        log("TAKEOFF", "takeoff altitude wait timed out")
        return TakeoffAltitudeResult(
            confirmed=False,
            safe_hover_altitude=last_altitude_m >= min_confirm_altitude_m,
            last_altitude_m=last_altitude_m,
        )
    except Exception as exc:
        log("TAKEOFF", f"takeoff altitude wait failed: {exc}")
        return TakeoffAltitudeResult(confirmed=False, safe_hover_altitude=False, last_altitude_m=0.0)


async def try_safety_land(drone: object) -> None:
    try:
        log("LAND", "attempting safety land")
        await drone.action.land()
    except Exception as exc:
        log("LAND", f"safety land attempt failed: {exc}")


async def read_initial_yaw_deg(drone: object) -> float:
    try:
        attitude = await asyncio.wait_for(anext(drone.telemetry.attitude_euler()), timeout=2.0)
    except Exception as exc:
        log("MOVE", f"could not read initial yaw; using 0.0 deg fallback: {exc}")
        return 0.0

    yaw_deg = float(attitude.yaw_deg)
    log("MOVE", f"initial yaw={yaw_deg:.1f} deg")
    return yaw_deg


async def read_position_ned(drone: object, label: str) -> object | None:
    try:
        position_velocity = await asyncio.wait_for(anext(drone.telemetry.position_velocity_ned()), timeout=2.0)
    except Exception as exc:
        log("MOVE", f"could not read {label} local position NED: {exc}")
        return None

    position = position_velocity.position
    log(
        "MOVE",
        f"{label} north/east/down: {position.north_m:.2f}, {position.east_m:.2f}, {position.down_m:.2f} m",
    )
    return position


async def read_position_ned_quiet(drone: object) -> object | None:
    try:
        position_velocity = await asyncio.wait_for(anext(drone.telemetry.position_velocity_ned()), timeout=2.0)
    except Exception:
        return None
    return position_velocity.position


def log_position_delta(label: str, position: object | None, start_position: object | None) -> None:
    if position is None or start_position is None:
        log("MOVE", f"{label} displacement unavailable")
        return

    delta_north, delta_east, delta_down = position_delta(position, start_position)
    log(
        "MOVE",
        f"{label} displacement from start: north={delta_north:.2f} m, east={delta_east:.2f} m, down={delta_down:.2f} m",
    )


def position_delta(position: object, start_position: object) -> tuple[float, float, float]:
    delta_north = position.north_m - start_position.north_m
    delta_east = position.east_m - start_position.east_m
    delta_down = position.down_m - start_position.down_m
    return delta_north, delta_east, delta_down


def horizontal_magnitude(delta_north: float, delta_east: float) -> float:
    return math.sqrt(delta_north * delta_north + delta_east * delta_east)


def horizontal_distance(position: object | None, anchor_position: object | None) -> float | None:
    if position is None or anchor_position is None:
        return None
    delta_north, delta_east, _delta_down = position_delta(position, anchor_position)
    return horizontal_magnitude(delta_north, delta_east)


def normalize_yaw_deg(yaw_deg: float) -> float:
    return float(yaw_deg) % 360.0


def shortest_yaw_delta_deg(from_yaw: float, to_yaw: float) -> float:
    return (normalize_yaw_deg(to_yaw) - normalize_yaw_deg(from_yaw) + 540.0) % 360.0 - 180.0


def interpolated_yaw_targets(from_yaw: float, to_yaw: float, step_deg: float) -> list[float]:
    from_yaw = normalize_yaw_deg(from_yaw)
    to_yaw = normalize_yaw_deg(to_yaw)
    delta = shortest_yaw_delta_deg(from_yaw, to_yaw)
    step_deg = max(0.0, step_deg)
    if step_deg <= 0.0 or abs(delta) <= step_deg:
        return [to_yaw]

    step_count = int(math.ceil(abs(delta) / step_deg))
    return [
        normalize_yaw_deg(from_yaw + delta * float(index) / float(step_count))
        for index in range(1, step_count + 1)
    ]


def ned_forward_delta(yaw_deg: float, distance_m: float) -> tuple[float, float]:
    yaw_rad = math.radians(yaw_deg)
    return math.cos(yaw_rad) * distance_m, math.sin(yaw_rad) * distance_m


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def compute_corridor_centering_command(monitor: LaserScanMonitor, args: argparse.Namespace) -> float:
    if not args.enable_corridor_centering:
        return 0.0

    scan = monitor.decision_snapshot()
    min_valid_samples = max(1, args.min_valid_samples)
    min_valid_ratio = max(0.0, min(1.0, args.min_valid_ratio))
    if (
        not scan.left_ready
        or not scan.right_ready
        or scan.left_valid_count < min_valid_samples
        or scan.right_valid_count < min_valid_samples
        or scan.left_valid_ratio < min_valid_ratio
        or scan.right_valid_ratio < min_valid_ratio
    ):
        log(
            "CENTER",
            "side scan unavailable; right_cmd=0.00",
        )
        return 0.0

    error = scan.left_avg - scan.right_avg
    deadband = max(0.0, args.corridor_center_deadband)
    if abs(error) < deadband:
        right_cmd = 0.0
    else:
        max_right_speed = max(0.0, args.corridor_center_max_right_speed)
        right_cmd = clamp(
            -max(0.0, args.corridor_center_kp) * error,
            -max_right_speed,
            max_right_speed,
        )

    log(
        "CENTER",
        f"left_avg={scan.left_avg:.2f} right_avg={scan.right_avg:.2f} "
        f"error={error:.2f} right_cmd={right_cmd:.2f}",
    )
    return right_cmd


async def compute_altitude_hold_command(drone: object, args: argparse.Namespace) -> float:
    if not args.enable_altitude_hold:
        return 0.0

    position = await read_position_ned_quiet(drone)
    if position is None:
        log("ALT", "altitude unavailable; down_cmd=0.00")
        return 0.0

    altitude = max(0.0, -float(position.down_m))
    target_altitude = max(0.0, args.target_altitude)
    altitude_error = target_altitude - altitude
    max_down_speed = max(0.0, args.altitude_hold_max_down_speed)
    down_cmd = clamp(
        -max(0.0, args.altitude_hold_kp) * altitude_error,
        -max_down_speed,
        max_down_speed,
    )
    log(
        "ALT",
        f"altitude={altitude:.2f} target={target_altitude:.2f} down_cmd={down_cmd:.2f}",
    )
    return down_cmd


def position_as_event_dict(position: object | None) -> dict[str, float] | None:
    if position is None:
        return None
    return {
        "north": round(float(position.north_m), 3),
        "east": round(float(position.east_m), 3),
        "altitude": round(max(0.0, -float(position.down_m)), 3),
    }


def opening_id_for_position(side: str, position: object | None) -> str:
    if position is None:
        return f"{side}@unknown"
    north_bucket = round(float(position.north_m) * 2.0) / 2.0
    east_bucket = round(float(position.east_m) * 2.0) / 2.0
    return f"{side}@north={north_bucket:.1f},east={east_bucket:.1f}"


def opening_already_inspected(opening_id: str, inspected_ids: set[str]) -> bool:
    return opening_id in inspected_ids


def valid_laser_range(distance: float, msg: object) -> bool:
    if not math.isfinite(distance) or distance <= 0.0:
        return False
    range_min = float(getattr(msg, "range_min", 0.0))
    range_max = float(getattr(msg, "range_max", 0.0))
    if range_min > 0.0 and distance < range_min:
        return False
    if range_max > 0.0 and distance > range_max:
        return False
    return True


def front_sector_min_distance(msg: object | None, sector_deg: float) -> float | None:
    stats = front_sector_stats(msg, sector_deg)
    return stats.min_finite_distance


def front_sector_stats(msg: object | None, sector_deg: float) -> FrontSectorStats:
    if msg is None:
        return FrontSectorStats(
            sample_count=0,
            valid_count=0,
            finite_count=0,
            inf_count=0,
            valid_ratio=0.0,
            min_finite_distance=None,
        )

    sector_half_rad = math.radians(max(0.0, sector_deg) / 2.0)
    angle = float(getattr(msg, "angle_min", 0.0))
    angle_increment = float(getattr(msg, "angle_increment", 0.0))
    finite_ranges: list[float] = []
    inf_count = 0
    sample_count = 0
    for distance in getattr(msg, "ranges", []):
        if abs(angle) <= sector_half_rad:
            sample_count += 1
            value = float(distance)
            if math.isinf(value):
                inf_count += 1
            elif valid_laser_range(value, msg):
                finite_ranges.append(value)
        angle += angle_increment

    finite_count = len(finite_ranges)
    valid_count = finite_count + inf_count
    valid_ratio = valid_count / sample_count if sample_count else 0.0
    min_finite = min(finite_ranges) if finite_ranges else None
    return FrontSectorStats(
        sample_count=sample_count,
        valid_count=valid_count,
        finite_count=finite_count,
        inf_count=inf_count,
        valid_ratio=valid_ratio,
        min_finite_distance=min_finite,
    )


def front_motion_status_for_scan(msg: object | None, args: argparse.Namespace, source_label: str) -> str:
    stats = front_sector_stats(msg, args.front_sector_deg)
    min_valid_samples = max(1, args.min_valid_samples)
    min_valid_ratio = max(0.0, min(1.0, args.min_valid_ratio))

    if stats.sample_count == 0:
        log("SAFETY", f"{source_label} front sector unavailable")
        return "unavailable"

    if stats.min_finite_distance is not None and stats.min_finite_distance < max(0.0, args.front_stop_distance):
        log(
            "SAFETY",
            f"{source_label} front_sector_min={stats.min_finite_distance:.2f} m, "
            f"sector={max(0.0, args.front_sector_deg):.1f} deg",
        )
        log("SAFETY", f"{source_label} front obstacle detected; stopping corridor motion")
        return "blocked"

    if stats.valid_count < min_valid_samples or stats.valid_ratio < min_valid_ratio:
        log(
            "SAFETY",
            f"{source_label} front sector sparse; "
            f"valid={stats.valid_count}/{stats.sample_count}, ratio={stats.valid_ratio:.2f}",
        )
        return "unavailable"

    if stats.min_finite_distance is None:
        log(
            "SAFETY",
            f"{source_label} front sector clear: no finite obstacle, inf_rays={stats.inf_count}",
        )
        return "clear"

    log(
        "SAFETY",
        f"{source_label} front_sector_min={stats.min_finite_distance:.2f} m, "
        f"sector={max(0.0, args.front_sector_deg):.1f} deg",
    )
    if stats.min_finite_distance >= max(args.front_stop_distance, args.front_clear_distance):
        log("SAFETY", f"{source_label} front sector clear")
    return "clear"


def front_motion_status(monitor: LaserScanMonitor, args: argparse.Namespace) -> str:
    wide_status = front_motion_status_for_scan(monitor.latest_front, args, "wide")
    if wide_status in {"clear", "blocked"}:
        return wide_status

    log("SAFETY", "wide front sector unavailable; checking decision front")
    decision_status = front_motion_status_for_scan(monitor.latest_front_decision, args, "decision")
    if decision_status == "clear":
        log("SAFETY", "decision front fallback clear")
    elif decision_status == "blocked":
        log("SAFETY", "decision front fallback blocked")
    else:
        log("SAFETY", "front sector unavailable on wide and decision scans")
    return decision_status


def front_clearance_for_candidate(monitor: LaserScanMonitor, args: argparse.Namespace) -> tuple[bool, float | None]:
    min_valid_samples = max(1, args.min_valid_samples)
    min_valid_ratio = max(0.0, min(1.0, args.min_valid_ratio))
    front_clear_distance = max(0.0, args.front_clear_distance)

    for msg in (monitor.latest_front, monitor.latest_front_decision):
        stats = front_sector_stats(msg, args.front_sector_deg)
        if stats.sample_count == 0:
            continue
        if stats.valid_count < min_valid_samples or stats.valid_ratio < min_valid_ratio:
            continue
        if stats.min_finite_distance is None:
            return True, math.inf
        return stats.min_finite_distance >= front_clear_distance, stats.min_finite_distance

    return False, None


def front_motion_allowed(monitor: LaserScanMonitor, args: argparse.Namespace) -> bool:
    return front_motion_status(monitor, args) == "clear"


async def front_motion_allowed_with_retry(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    args: argparse.Namespace,
    velocity_type: object,
) -> bool:
    status = front_motion_status(monitor, args)
    if status == "clear":
        setattr(args, "_front_unavailable_events", 0)
        return True
    if status == "blocked":
        return False

    unavailable_events = int(getattr(args, "_front_unavailable_events", 0)) + 1
    setattr(args, "_front_unavailable_events", unavailable_events)
    if unavailable_events > max(0, args.front_unavailable_max_events):
        log("SAFETY", "front sector unavailable persists; ending mission safely")
        return False

    retry_seconds = max(0.0, args.front_unavailable_retry_seconds)
    retry_rate_hz = max(0.1, args.front_unavailable_retry_rate_hz)
    retry_interval = 1.0 / retry_rate_hz
    log(
        "SAFETY",
        "front sector unavailable; holding and retrying scan "
        f"({unavailable_events}/{max(0, args.front_unavailable_max_events)})",
    )
    await send_zero_body_velocity(drone, velocity_type)

    retry_started_at = time.monotonic()
    while time.monotonic() - retry_started_at < retry_seconds:
        rclpy.spin_once(monitor.node, timeout_sec=0.0)
        await asyncio.sleep(retry_interval)
        status = front_motion_status(monitor, args)
        if status == "clear":
            log("SAFETY", "front sector recovered")
            setattr(args, "_front_unavailable_events", 0)
            return True
        if status == "blocked":
            return False

    log("SAFETY", "front sector unavailable after retry")
    return False


def side_motion_allowed(monitor: LaserScanMonitor, args: argparse.Namespace, side: str) -> bool:
    scan = monitor.decision_snapshot()
    side = side.lower()
    if side == "left":
        ready = scan.left_ready
        valid_count = scan.left_valid_count
        valid_ratio = scan.left_valid_ratio
        side_min = scan.left_min
    elif side == "right":
        ready = scan.right_ready
        valid_count = scan.right_valid_count
        valid_ratio = scan.right_valid_ratio
        side_min = scan.right_min
    else:
        log("SAFETY", f"unknown side safety direction: {side}")
        return False

    if not ready or valid_count < max(1, args.min_valid_samples) or valid_ratio < max(0.0, args.min_valid_ratio):
        log("SAFETY", f"{side} scan unavailable or sparse; stopping lateral motion")
        return False

    log("SAFETY", f"{side}_min={format_distance(side_min)} m")
    if side_min < max(0.0, args.side_stop_distance):
        log("SAFETY", f"{side} obstacle detected; stopping lateral motion")
        return False
    return True


def log_passive_corridor_decision(
    monitor: LaserScanMonitor,
    memory: MissionMemory,
    args: argparse.Namespace,
    context: str,
) -> Decision:
    scan = monitor.snapshot()
    decision = decide_corridor_action(scan, memory, args)
    log_scan_monitor_snapshot(scan)
    log("DECIDE", f"{context} corridor_action={decision.value}")
    if decision == Decision.DETECT_LEFT_OPENING:
        log("DETECT", f"{context} candidate opening side=left")
    elif decision == Decision.DETECT_RIGHT_OPENING:
        log("DETECT", f"{context} candidate opening side=right")
    return decision


def opening_side_for_decision(decision: Decision) -> str | None:
    if decision == Decision.DETECT_LEFT_OPENING:
        return "left"
    if decision == Decision.DETECT_RIGHT_OPENING:
        return "right"
    return None


async def goto_position_ned(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    position_type: object,
    north_m: float,
    east_m: float,
    down_m: float,
    yaw_deg: float,
    hold_sec: float,
    label: str,
) -> None:
    hold_sec = max(0.0, hold_sec)
    log(
        "POS",
        f"{label}: N={north_m:.2f}, E={east_m:.2f}, D={down_m:.2f}, yaw={yaw_deg:.1f}, hold={hold_sec:.1f}s",
    )
    await drone.offboard.set_position_ned(position_type(north_m, east_m, down_m, yaw_deg))
    started_at = time.monotonic()
    while time.monotonic() - started_at < hold_sec:
        rclpy.spin_once(monitor.node, timeout_sec=0.02)
        await asyncio.sleep(0.05)


async def sample_gas_at_position(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    position_type: object,
    args: argparse.Namespace,
    label: str,
    duration_sec: float,
    hold_north_m: float,
    hold_east_m: float,
    hold_down_m: float,
    yaw_deg: float,
    compute_ppm: object,
    active_sources: list[dict[str, Any]],
    rng: random.Random,
) -> GasSampleSummary:
    duration_sec = max(0.0, duration_sec)
    rate_hz = max(0.1, args.gas_sample_rate_hz)
    interval_sec = 1.0 / rate_hz
    samples: list[float] = []
    last_position: object | None = None
    started_at = time.monotonic()
    log("GAS", f"{label} position-hold sampling for {duration_sec:.1f}s at {rate_hz:.1f} Hz")

    while time.monotonic() - started_at < duration_sec:
        rclpy.spin_once(monitor.node, timeout_sec=0.0)
        await drone.offboard.set_position_ned(position_type(hold_north_m, hold_east_m, hold_down_m, yaw_deg))
        position = await read_position_ned_quiet(drone)
        if position is not None:
            last_position = position
            ppm, _nearest_distance = compute_ppm(
                x=float(position.north_m),
                y=float(position.east_m),
                active_sources=active_sources,
                background_ppm=args.background_ppm,
                peak_ppm=args.peak_ppm,
                sigma=args.sigma,
                rng=rng,
                noise_std=args.noise_std,
            )
            samples.append(float(ppm))
            log("GAS", f"{label} sample ppm={ppm:.2f}")
        await asyncio.sleep(interval_sec)

    avg_ppm = sum(samples) / len(samples) if samples else 0.0
    max_ppm = max(samples) if samples else 0.0
    log("GAS", f"{label}_avg={avg_ppm:.2f} ppm, max={max_ppm:.2f} ppm, samples={len(samples)}")
    return GasSampleSummary(
        label=label,
        sample_count=len(samples),
        avg_ppm=avg_ppm,
        max_ppm=max_ppm,
        position=position_as_event_dict(last_position),
    )


async def soft_land_position_mode(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    position_type: object,
    north_m: float,
    east_m: float,
    yaw_deg: float,
) -> None:
    for altitude_m in (1.0, 0.7, 0.4):
        await goto_position_ned(
            drone,
            rclpy,
            monitor,
            position_type,
            north_m,
            east_m,
            -altitude_m,
            yaw_deg,
            3.0,
            f"soft land descent {altitude_m:.1f}m",
        )


def log_position_decision_scan(monitor: LaserScanMonitor, args: argparse.Namespace, context: str) -> None:
    scan = monitor.decision_snapshot()
    log(
        "SCAN",
        f"{context} decision "
        f"left_min={format_distance(scan.left_min)} left_avg={format_distance(scan.left_avg)} "
        f"right_min={format_distance(scan.right_min)} right_avg={format_distance(scan.right_avg)} "
        f"valid(left/right)={scan.left_valid_count}/{scan.right_valid_count} "
        f"ratio(left/right)={scan.left_valid_ratio:.2f}/{scan.right_valid_ratio:.2f}",
    )


async def send_zero_velocity(drone: object, velocity_type: object, yaw_deg: float) -> None:
    await drone.offboard.set_velocity_ned(velocity_type(0.0, 0.0, 0.0, yaw_deg))


async def send_zero_body_velocity(drone: object, velocity_type: object) -> None:
    await drone.offboard.set_velocity_body(velocity_type(0.0, 0.0, 0.0, 0.0))


async def run_opening_probe(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    args: argparse.Namespace,
    velocity_type: object,
    side: str,
) -> None:
    duration_sec = max(0.0, args.probe_duration_seconds)
    rate_hz = max(1.0, args.move_rate_hz)
    interval_sec = 1.0 / rate_hz
    side_speed = abs(float(args.probe_side_speed))
    right_speed = -side_speed if side == "left" else side_speed

    if duration_sec == 0.0 or side_speed == 0.0:
        log("PROBE", f"skipping {side} opening probe because speed or duration is zero")
        return

    if not await front_motion_allowed_with_retry(drone, rclpy, monitor, args, velocity_type):
        log("PROBE", f"front safety blocked before {side} opening probe")
        return

    start_position = await read_position_ned(drone, f"{side} probe start")
    log(
        "PROBE",
        f"{side} opening lateral body probe: right_speed={right_speed:.2f} m/s, duration={duration_sec:.1f}s",
    )

    started_at = time.monotonic()
    next_decision_log_at = started_at
    decision_log_interval_sec = 1.0 / max(0.1, args.scan_log_rate_hz)
    memory = MissionMemory(
        visited_openings=set(),
        skipped_openings=set(),
        bypass_attempts=0,
        corridor_x=0.0,
        seed=args.seed if args.seed is not None else 0,
    )
    while time.monotonic() - started_at < duration_sec:
        rclpy.spin_once(monitor.node, timeout_sec=0.0)
        now = time.monotonic()
        if now >= next_decision_log_at:
            log_passive_corridor_decision(monitor, memory, args, f"probe side={side}")
            next_decision_log_at = now + decision_log_interval_sec
        if not await front_motion_allowed_with_retry(drone, rclpy, monitor, args, velocity_type):
            log("PROBE", f"front safety stopped {side} opening probe")
            await send_zero_body_velocity(drone, velocity_type)
            return
        await drone.offboard.set_velocity_body(velocity_type(0.0, right_speed, 0.0, 0.0))
        await asyncio.sleep(interval_sec)

    await send_zero_body_velocity(drone, velocity_type)
    await asyncio.sleep(0.2)
    end_position = await read_position_ned(drone, f"{side} probe end")
    log_position_delta(f"{side} probe", end_position, start_position)


async def sample_gas_for_duration(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    args: argparse.Namespace,
    velocity_type: object,
    label: str,
    duration_sec: float,
    compute_ppm: object,
    active_sources: list[dict[str, Any]],
    rng: random.Random,
) -> GasSampleSummary:
    duration_sec = max(0.0, duration_sec)
    rate_hz = max(0.1, args.gas_sample_rate_hz)
    interval_sec = 1.0 / rate_hz
    samples: list[float] = []
    last_position: object | None = None
    started_at = time.monotonic()
    log("GAS", f"{label} sampling for {duration_sec:.1f}s at {rate_hz:.1f} Hz")

    while time.monotonic() - started_at < duration_sec:
        rclpy.spin_once(monitor.node, timeout_sec=0.0)
        position = await read_position_ned(drone, f"{label} sample")
        if position is not None:
            last_position = position
            ppm, _nearest_distance = compute_ppm(
                x=float(position.north_m),
                y=float(position.east_m),
                active_sources=active_sources,
                background_ppm=args.background_ppm,
                peak_ppm=args.peak_ppm,
                sigma=args.sigma,
                rng=rng,
                noise_std=args.noise_std,
            )
            samples.append(float(ppm))
            log("GAS", f"{label} sample ppm={ppm:.2f}")
        await send_zero_body_velocity(drone, velocity_type)
        await asyncio.sleep(interval_sec)

    if samples:
        avg_ppm = sum(samples) / len(samples)
        max_ppm = max(samples)
    else:
        avg_ppm = 0.0
        max_ppm = 0.0
    log("GAS", f"{label}_avg={avg_ppm:.2f} ppm, max={max_ppm:.2f} ppm, samples={len(samples)}")
    return GasSampleSummary(
        label=label,
        sample_count=len(samples),
        avg_ppm=avg_ppm,
        max_ppm=max_ppm,
        position=position_as_event_dict(last_position),
    )


async def run_lateral_body_motion(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    args: argparse.Namespace,
    velocity_type: object,
    label: str,
    right_speed: float,
    duration_sec: float,
) -> None:
    duration_sec = max(0.0, duration_sec)
    rate_hz = max(1.0, args.move_rate_hz)
    interval_sec = 1.0 / rate_hz
    if duration_sec == 0.0:
        return
    if not await front_motion_allowed_with_retry(drone, rclpy, monitor, args, velocity_type):
        raise RuntimeError(f"front safety blocked before {label}")

    log("INSPECT", f"{label}: right_speed={right_speed:.2f} m/s, duration={duration_sec:.1f}s")
    started_at = time.monotonic()
    while time.monotonic() - started_at < duration_sec:
        rclpy.spin_once(monitor.node, timeout_sec=0.0)
        if not await front_motion_allowed_with_retry(drone, rclpy, monitor, args, velocity_type):
            await send_zero_body_velocity(drone, velocity_type)
            raise RuntimeError(f"front safety stopped {label}")
        await drone.offboard.set_velocity_body(velocity_type(0.0, right_speed, 0.0, 0.0))
        await asyncio.sleep(interval_sec)
    await send_zero_body_velocity(drone, velocity_type)
    await asyncio.sleep(0.2)


async def run_distance_based_lateral_body_motion(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    args: argparse.Namespace,
    velocity_type: object,
    label: str,
    right_speed: float,
    anchor_position: object,
    target_distance_m: float | None,
    exit_tolerance_m: float | None,
    timeout_sec: float,
) -> dict[str, float]:
    rate_hz = max(1.0, args.move_rate_hz)
    interval_sec = 1.0 / rate_hz
    timeout_sec = max(0.1, timeout_sec)
    target_distance_m = None if target_distance_m is None else max(0.0, target_distance_m)
    exit_tolerance_m = None if exit_tolerance_m is None else max(0.0, exit_tolerance_m)
    direction_side = "left" if right_speed < 0.0 else "right"

    if right_speed == 0.0:
        log("INSPECT", f"{label}: skipping lateral motion because right_speed is zero")
        return {"actual_distance_m": 0.0, "final_anchor_distance_m": 0.0, "elapsed_seconds": 0.0}
    if target_distance_m == 0.0:
        log("INSPECT", f"{label}: skipping lateral enter because target distance is zero")
        return {"actual_distance_m": 0.0, "final_anchor_distance_m": 0.0, "elapsed_seconds": 0.0}

    mode = "enter" if target_distance_m is not None else "exit"
    if mode == "enter":
        log(
            "INSPECT",
            f"{label}: distance-based enter, right_speed={right_speed:.2f} m/s, "
            f"target={target_distance_m:.2f} m, timeout={timeout_sec:.1f}s",
        )
    else:
        log(
            "INSPECT",
            f"{label}: distance-based exit, right_speed={right_speed:.2f} m/s, "
            f"tolerance={exit_tolerance_m:.2f} m, timeout={timeout_sec:.1f}s",
        )

    started_at = time.monotonic()
    last_position: object | None = None
    last_anchor_distance = horizontal_distance(await read_position_ned_quiet(drone), anchor_position)
    max_anchor_distance = last_anchor_distance if last_anchor_distance is not None else 0.0
    next_progress_log_at = started_at
    progress_log_interval_sec = 1.0 / max(0.1, args.scan_log_rate_hz)

    while time.monotonic() - started_at < timeout_sec:
        rclpy.spin_once(monitor.node, timeout_sec=0.0)
        if not await front_motion_allowed_with_retry(drone, rclpy, monitor, args, velocity_type):
            await send_zero_body_velocity(drone, velocity_type)
            raise RuntimeError(f"front safety stopped {label}")
        if not side_motion_allowed(monitor, args, direction_side):
            await send_zero_body_velocity(drone, velocity_type)
            raise RuntimeError(f"{direction_side} safety stopped {label}")

        position = await read_position_ned_quiet(drone)
        if position is not None:
            last_position = position
            anchor_distance = horizontal_distance(position, anchor_position)
            if anchor_distance is not None:
                last_anchor_distance = anchor_distance
                max_anchor_distance = max(max_anchor_distance, anchor_distance)
                now = time.monotonic()
                if now >= next_progress_log_at:
                    log("INSPECT", f"{label}: anchor_distance={anchor_distance:.2f} m")
                    next_progress_log_at = now + progress_log_interval_sec
                if mode == "enter" and anchor_distance >= target_distance_m:
                    await send_zero_body_velocity(drone, velocity_type)
                    log("INSPECT", f"{label}: target distance reached at {anchor_distance:.2f} m")
                    return {
                        "actual_distance_m": round(anchor_distance, 3),
                        "final_anchor_distance_m": round(anchor_distance, 3),
                        "elapsed_seconds": round(time.monotonic() - started_at, 3),
                    }
                if mode == "exit" and anchor_distance <= (exit_tolerance_m or 0.0):
                    await send_zero_body_velocity(drone, velocity_type)
                    log("INSPECT", f"{label}: exit tolerance reached at {anchor_distance:.2f} m")
                    return {
                        "actual_distance_m": round(max_anchor_distance, 3),
                        "final_anchor_distance_m": round(anchor_distance, 3),
                        "elapsed_seconds": round(time.monotonic() - started_at, 3),
                    }

        await drone.offboard.set_velocity_body(velocity_type(0.0, right_speed, 0.0, 0.0))
        await asyncio.sleep(interval_sec)

    await send_zero_body_velocity(drone, velocity_type)
    final_distance = horizontal_distance(last_position, anchor_position)
    final_distance_text = "unknown" if final_distance is None else f"{final_distance:.2f} m"
    raise RuntimeError(f"{label} timed out after {timeout_sec:.1f}s; anchor_distance={final_distance_text}")


async def run_opening_alignment_gate(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    args: argparse.Namespace,
    velocity_type: object,
    side: str,
) -> bool:
    if args.disable_opening_alignment:
        log("ALIGN", f"{side} opening alignment disabled; accepting candidate")
        return True

    target_distance_m = max(0.0, args.opening_alignment_forward_distance)
    timeout_sec = max(0.1, args.opening_alignment_timeout_seconds)
    rate_hz = max(1.0, args.move_rate_hz)
    interval_sec = 1.0 / rate_hz
    forward_speed = abs(float(args.body_forward_speed))

    log("ALIGN", f"candidate {side} opening detected; aligning before inspection")
    rclpy.spin_once(monitor.node, timeout_sec=0.0)
    pre_scan = monitor.decision_snapshot()
    pre_open, pre_reason, pre_metrics = side_decision_diagnostics(pre_scan, side, args)
    log(
        "ALIGN",
        f"pre-align decision scan: open={pre_open} reason={pre_reason} {pre_metrics}",
    )
    if target_distance_m > 0.0 and forward_speed > 0.0:
        anchor_position = await read_position_ned_quiet(drone)
        if anchor_position is None:
            raise RuntimeError(f"could not read alignment anchor position for {side} opening")

        log(
            "ALIGN",
            f"moving forward for alignment: target={target_distance_m:.2f} m, "
            f"speed={forward_speed:.2f} m/s, timeout={timeout_sec:.1f}s",
        )
        started_at = time.monotonic()
        last_distance = 0.0
        while time.monotonic() - started_at < timeout_sec:
            rclpy.spin_once(monitor.node, timeout_sec=0.0)
            if not await front_motion_allowed_with_retry(drone, rclpy, monitor, args, velocity_type):
                await send_zero_body_velocity(drone, velocity_type)
                raise RuntimeError(f"front safety stopped {side} opening alignment")

            position = await read_position_ned_quiet(drone)
            distance = horizontal_distance(position, anchor_position)
            if distance is not None:
                last_distance = distance
                if distance >= target_distance_m:
                    log("ALIGN", f"alignment forward target reached at {distance:.2f} m")
                    break

            await drone.offboard.set_velocity_body(velocity_type(forward_speed, 0.0, 0.0, 0.0))
            await asyncio.sleep(interval_sec)
        else:
            log("ALIGN", f"alignment forward timeout; last_distance={last_distance:.2f} m")

        await send_zero_body_velocity(drone, velocity_type)
        await asyncio.sleep(0.2)

    rclpy.spin_once(monitor.node, timeout_sec=0.05)
    post_scan = monitor.decision_snapshot()
    post_open, post_reason, post_metrics = side_decision_diagnostics(post_scan, side, args)
    log(
        "ALIGN",
        f"post-align decision scan: open={post_open} reason={post_reason} {post_metrics}",
    )

    confirm_frames = max(1, args.opening_alignment_confirm_frames)
    confirmed_frames = 0
    for frame_index in range(confirm_frames):
        rclpy.spin_once(monitor.node, timeout_sec=0.05)
        scan = monitor.decision_snapshot()
        open_now, reason, metrics = side_decision_diagnostics(scan, side, args)
        log(
            "ALIGN",
            f"confirm frame {frame_index + 1}/{confirm_frames}: "
            f"open={open_now} reason={reason} {metrics}",
        )
        if open_now:
            confirmed_frames += 1
            log("ALIGN", f"{side} opening confirm frame {frame_index + 1}/{confirm_frames}: open")
        else:
            log(
                "ALIGN",
                f"{side} opening rejected after alignment at frame {frame_index + 1}/{confirm_frames}: "
                f"reason={reason}",
            )
            await send_zero_body_velocity(drone, velocity_type)
            return False
        await asyncio.sleep(0.05)

    log("ALIGN", f"{side} opening confirmed after alignment ({confirmed_frames}/{confirm_frames})")
    return True


def write_inspection_events(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


async def warmup_offboard_ned(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    velocity_type: object,
    yaw_deg: float,
    args: argparse.Namespace,
) -> None:
    warmup_sec = max(0.0, args.offboard_warmup_seconds)
    rate_hz = max(1.0, args.move_rate_hz)
    interval_sec = 1.0 / rate_hz
    if warmup_sec == 0.0:
        await send_zero_velocity(drone, velocity_type, yaw_deg)
        return

    log("MOVE", f"warming up NED offboard setpoints for {warmup_sec:.1f}s")
    started_at = time.monotonic()
    while time.monotonic() - started_at < warmup_sec:
        rclpy.spin_once(monitor.node, timeout_sec=0.0)
        await send_zero_velocity(drone, velocity_type, yaw_deg)
        await asyncio.sleep(interval_sec)


async def warmup_offboard_body(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    velocity_type: object,
    args: argparse.Namespace,
) -> None:
    warmup_sec = max(0.0, args.offboard_warmup_seconds)
    rate_hz = max(1.0, args.move_rate_hz)
    interval_sec = 1.0 / rate_hz
    if warmup_sec == 0.0:
        await send_zero_body_velocity(drone, velocity_type)
        return

    log("MOVE", f"warming up body offboard setpoints for {warmup_sec:.1f}s")
    started_at = time.monotonic()
    while time.monotonic() - started_at < warmup_sec:
        rclpy.spin_once(monitor.node, timeout_sec=0.0)
        await send_zero_body_velocity(drone, velocity_type)
        await asyncio.sleep(interval_sec)


async def run_corridor_follow_steps(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    args: argparse.Namespace,
    velocity_type: object,
    yaw_deg: float,
) -> None:
    step_count = max(0, args.corridor_step_count)
    step_duration_sec = max(0.0, args.corridor_step_duration_seconds)
    rate_hz = max(1.0, args.move_rate_hz)
    pause_sec = max(0.0, args.pause_between_steps)
    interval_sec = 1.0 / rate_hz
    north_speed = float(args.corridor_north_speed)
    east_speed = float(args.corridor_east_speed)

    log("MOVE", "corridor follow check uses fixed yaw and NED velocity")
    log("MOVE", f"step_count={step_count}")
    log("MOVE", f"north_speed={north_speed:.2f} m/s")
    log("MOVE", f"east_speed={east_speed:.2f} m/s")
    log("MOVE", f"step_duration_seconds={step_duration_sec:.1f}")
    log("MOVE", f"move_rate_hz={rate_hz:.1f}")

    if step_count == 0 or step_duration_sec == 0.0:
        log("MOVE", "corridor movement skipped because step count or duration is zero")
        return

    start_position = await read_position_ned(drone, "start")

    await warmup_offboard_ned(drone, rclpy, monitor, velocity_type, yaw_deg, args)
    log("MOVE", "starting offboard mode")
    await drone.offboard.start()

    try:
        for step_index in range(step_count):
            tag = f"MOVE {step_index + 1}/{step_count}"
            rclpy.spin_once(monitor.node, timeout_sec=0.05)
            if not front_motion_allowed(monitor, args):
                await send_zero_velocity(drone, velocity_type, yaw_deg)
                break

            log(
                tag,
                f"NED velocity: north={north_speed:.2f} m/s, east={east_speed:.2f} m/s, yaw={yaw_deg:.1f} deg",
            )
            step_started_at = time.monotonic()
            while time.monotonic() - step_started_at < step_duration_sec:
                rclpy.spin_once(monitor.node, timeout_sec=0.0)
                if not front_motion_allowed(monitor, args):
                    await send_zero_velocity(drone, velocity_type, yaw_deg)
                    return
                await drone.offboard.set_velocity_ned(velocity_type(north_speed, east_speed, 0.0, yaw_deg))
                await asyncio.sleep(interval_sec)

            await send_zero_velocity(drone, velocity_type, yaw_deg)
            end_position = await read_position_ned(drone, f"after step {step_index + 1}")
            log_position_delta(f"step {step_index + 1}", end_position, start_position)
            if pause_sec > 0.0:
                log(tag, f"pausing for {pause_sec:.1f}s")
                pause_started_at = time.monotonic()
                while time.monotonic() - pause_started_at < pause_sec:
                    rclpy.spin_once(monitor.node, timeout_sec=0.02)
                    await asyncio.sleep(0.05)
    finally:
        try:
            await send_zero_velocity(drone, velocity_type, yaw_deg)
        except Exception as exc:
            log("MOVE", f"zero velocity before offboard stop failed: {exc}")
        try:
            log("MOVE", "stopping offboard mode")
            await drone.offboard.stop()
        except Exception as exc:
            log("MOVE", f"offboard stop failed, continuing to land: {exc}")


async def run_body_corridor_follow_steps(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    args: argparse.Namespace,
    velocity_type: object,
) -> None:
    step_count = max(0, args.corridor_step_count)
    step_duration_sec = max(0.0, args.corridor_step_duration_seconds)
    rate_hz = max(1.0, args.move_rate_hz)
    pause_sec = max(0.0, args.pause_between_steps)
    interval_sec = 1.0 / rate_hz
    forward_speed = float(args.body_forward_speed)
    right_speed = float(args.body_right_speed)
    down_speed = float(args.body_down_speed)
    yawspeed = float(args.body_yawspeed)
    decision_log_interval_sec = 1.0 / max(0.1, args.scan_log_rate_hz)
    opening_probe_enabled = bool(args.enable_opening_probe)
    max_probe_count = max(0, args.probe_max_count)
    probe_count = 0
    memory = MissionMemory(
        visited_openings=set(),
        skipped_openings=set(),
        bypass_attempts=0,
        corridor_x=0.0,
        seed=args.seed if args.seed is not None else 0,
    )

    log("MOVE", "corridor follow check uses body-frame forward velocity")
    log("MOVE", f"step_count={step_count}")
    log("MOVE", f"body_forward_speed={forward_speed:.2f} m/s")
    log("MOVE", f"body_right_speed={right_speed:.2f} m/s")
    log("MOVE", f"body_down_speed={down_speed:.2f} m/s")
    log("MOVE", f"body_yawspeed={yawspeed:.2f} deg/s")
    log("MOVE", f"step_duration_seconds={step_duration_sec:.1f}")
    log("MOVE", f"move_rate_hz={rate_hz:.1f}")
    log("PROBE", f"opening_probe_enabled={opening_probe_enabled}")
    if opening_probe_enabled:
        log("PROBE", f"probe_side_speed={abs(float(args.probe_side_speed)):.2f} m/s")
        log("PROBE", f"probe_duration_seconds={max(0.0, args.probe_duration_seconds):.1f}")
        log("PROBE", f"probe_max_count={max_probe_count}")

    if step_count == 0 or step_duration_sec == 0.0:
        log("MOVE", "corridor movement skipped because step count or duration is zero")
        return

    start_position = await read_position_ned(drone, "start")

    await warmup_offboard_body(drone, rclpy, monitor, velocity_type, args)
    log("MOVE", "starting offboard mode")
    await drone.offboard.start()

    try:
        for step_index in range(step_count):
            tag = f"MOVE {step_index + 1}/{step_count}"
            rclpy.spin_once(monitor.node, timeout_sec=0.05)
            decision = log_passive_corridor_decision(monitor, memory, args, f"step={step_index + 1}/{step_count} pre")
            opening_side = opening_side_for_decision(decision)
            if opening_probe_enabled and opening_side is not None and probe_count < max_probe_count:
                probe_count += 1
                await run_opening_probe(drone, rclpy, monitor, args, velocity_type, opening_side)
                log("PROBE", "opening probe complete; ending corridor follow and landing")
                return
            if not await front_motion_allowed_with_retry(drone, rclpy, monitor, args, velocity_type):
                await send_zero_body_velocity(drone, velocity_type)
                break

            log(
                tag,
                "body velocity: "
                f"forward={forward_speed:.2f} m/s, right={right_speed:.2f} m/s, "
                f"down={down_speed:.2f} m/s, yawspeed={yawspeed:.2f} deg/s",
            )
            step_started_at = time.monotonic()
            next_decision_log_at = step_started_at
            while time.monotonic() - step_started_at < step_duration_sec:
                rclpy.spin_once(monitor.node, timeout_sec=0.0)
                now = time.monotonic()
                if now >= next_decision_log_at:
                    decision = log_passive_corridor_decision(
                        monitor,
                        memory,
                        args,
                        f"step={step_index + 1}/{step_count} moving",
                    )
                    next_decision_log_at = now + decision_log_interval_sec
                    opening_side = opening_side_for_decision(decision)
                    if opening_probe_enabled and opening_side is not None and probe_count < max_probe_count:
                        probe_count += 1
                        await run_opening_probe(drone, rclpy, monitor, args, velocity_type, opening_side)
                        log("PROBE", "opening probe complete; ending corridor follow and landing")
                        return
                if not await front_motion_allowed_with_retry(drone, rclpy, monitor, args, velocity_type):
                    await send_zero_body_velocity(drone, velocity_type)
                    return
                command_right_speed = right_speed + compute_corridor_centering_command(monitor, args)
                command_down_speed = (
                    await compute_altitude_hold_command(drone, args)
                    if args.enable_altitude_hold
                    else down_speed
                )
                await drone.offboard.set_velocity_body(
                    velocity_type(forward_speed, command_right_speed, command_down_speed, yawspeed)
                )
                await asyncio.sleep(interval_sec)

            await send_zero_body_velocity(drone, velocity_type)
            end_position = await read_position_ned(drone, f"after step {step_index + 1}")
            log_position_delta(f"step {step_index + 1}", end_position, start_position)
            if pause_sec > 0.0:
                log(tag, f"pausing for {pause_sec:.1f}s")
                pause_started_at = time.monotonic()
                while time.monotonic() - pause_started_at < pause_sec:
                    rclpy.spin_once(monitor.node, timeout_sec=0.02)
                    now = time.monotonic()
                    if now >= next_decision_log_at:
                        decision = log_passive_corridor_decision(
                            monitor,
                            memory,
                            args,
                            f"step={step_index + 1}/{step_count} pause",
                        )
                        next_decision_log_at = now + decision_log_interval_sec
                        opening_side = opening_side_for_decision(decision)
                        if opening_probe_enabled and opening_side is not None and probe_count < max_probe_count:
                            probe_count += 1
                            await run_opening_probe(drone, rclpy, monitor, args, velocity_type, opening_side)
                            log("PROBE", "opening probe complete; ending corridor follow and landing")
                            return
                    await asyncio.sleep(0.05)
    finally:
        try:
            await send_zero_body_velocity(drone, velocity_type)
        except Exception as exc:
            log("MOVE", f"body zero velocity before offboard stop failed: {exc}")
        try:
            log("MOVE", "stopping offboard mode")
            await drone.offboard.stop()
        except Exception as exc:
            log("MOVE", f"offboard stop failed, continuing to land: {exc}")


async def run_room_inspection_steps(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    args: argparse.Namespace,
    velocity_type: object,
    compute_ppm: object,
    active_sources: list[dict[str, Any]],
    scenario: str,
    possible_gas_zones: dict[str, tuple[float, float]],
) -> dict[str, Any]:
    step_count = max(0, args.corridor_step_count)
    step_duration_sec = max(0.0, args.corridor_step_duration_seconds)
    rate_hz = max(1.0, args.move_rate_hz)
    pause_sec = max(0.0, args.pause_between_steps)
    interval_sec = 1.0 / rate_hz
    forward_speed = float(args.body_forward_speed)
    max_inspections = max(0, args.max_inspections)
    cooldown_steps = max(0, args.inspection_cooldown_steps)
    side_speed = abs(float(args.inspection_side_speed))
    rng = random.Random(args.gas_seed)
    memory = MissionMemory(
        visited_openings=set(),
        skipped_openings=set(),
        bypass_attempts=0,
        corridor_x=0.0,
        seed=args.seed if args.seed is not None else 0,
    )
    inspected_ids: set[str] = set()
    events: list[dict[str, Any]] = []
    last_inspection_step = -cooldown_steps - 1
    decision_log_interval_sec = 1.0 / max(0.1, args.scan_log_rate_hz)

    payload: dict[str, Any] = {
        "requested_scenario": args.gas_scenario,
        "scenario": scenario,
        "gas_seed": args.gas_seed,
        "active_sources": active_sources,
        "possible_gas_zones": {name: {"x": xy[0], "y": xy[1]} for name, xy in possible_gas_zones.items()},
        "events": events,
    }

    log("INSPECT", "room inspection check uses body-frame corridor-follow and lateral enter/exit")
    log("INSPECT", f"max_inspections={max_inspections}")
    log("INSPECT", f"inspection_side_speed={side_speed:.2f} m/s")
    log("INSPECT", f"inspection_enter_distance={max(0.0, args.inspection_enter_distance):.2f} m")
    log("INSPECT", f"inspection_exit_tolerance={max(0.0, args.inspection_exit_tolerance):.2f} m")
    log("INSPECT", f"inspection_enter_timeout_seconds={max(0.1, args.inspection_enter_timeout_seconds):.1f}")
    log("INSPECT", f"inspection_exit_timeout_seconds={max(0.1, args.inspection_exit_timeout_seconds):.1f}")
    log("INSPECT", f"inspection_hover_seconds={max(0.0, args.inspection_hover_seconds):.1f}")
    log("ALIGN", f"opening_alignment_enabled={not args.disable_opening_alignment}")
    log("ALIGN", f"opening_alignment_forward_distance={max(0.0, args.opening_alignment_forward_distance):.2f} m")
    log("ALIGN", f"opening_alignment_confirm_frames={max(1, args.opening_alignment_confirm_frames)}")
    log("OPENING", f"opening_min_forward_progress={max(0.0, args.opening_min_forward_progress):.2f} m")
    log("OPENING", f"opening_min_persistence_frames={max(1, args.opening_min_persistence_frames)}")
    log("OPENING", f"opening_peak_drop_distance={max(0.0, args.opening_peak_drop_distance):.2f} m")
    log("OPENING", f"opening_require_front_clear={args.opening_require_front_clear}")
    log("GAS", f"scenario={scenario}, requested={args.gas_scenario}, active_sources={len(active_sources)}")

    if step_count == 0 or step_duration_sec == 0.0:
        log("INSPECT", "room inspection skipped because step count or duration is zero")
        return payload

    active_candidate: OpeningCandidateState | None = None

    async def inspect_opening(side: str, step_index: int) -> None:
        nonlocal last_inspection_step
        entry_anchor_position = await read_position_ned(drone, f"{side} opening entry anchor")
        if entry_anchor_position is None:
            raise RuntimeError(f"could not read entry anchor position for {side} opening")
        opening_id = opening_id_for_position(side, entry_anchor_position)
        if opening_already_inspected(opening_id, inspected_ids):
            log("INSPECT", f"skip already inspected opening {opening_id}")
            return
        if step_index - last_inspection_step < cooldown_steps:
            log("INSPECT", f"skip opening due to cooldown: step={step_index + 1}, last={last_inspection_step + 1}")
            return
        if len(events) >= max_inspections:
            log("INSPECT", "max inspections reached; continuing corridor-follow without inspection")
            return

        inspected_ids.add(opening_id)
        last_inspection_step = step_index
        await send_zero_body_velocity(drone, velocity_type)
        baseline = await sample_gas_for_duration(
            drone,
            rclpy,
            monitor,
            args,
            velocity_type,
            "baseline",
            args.baseline_sample_seconds,
            compute_ppm,
            active_sources,
            rng,
        )

        enter_right_speed = -side_speed if side == "left" else side_speed
        enter_result = await run_distance_based_lateral_body_motion(
            drone,
            rclpy,
            monitor,
            args,
            velocity_type,
            f"enter {side} opening",
            enter_right_speed,
            entry_anchor_position,
            args.inspection_enter_distance,
            None,
            args.inspection_enter_timeout_seconds,
        )

        hover_seconds = max(0.0, args.inspection_hover_seconds)
        if hover_seconds > 0.0:
            log("INSPECT", f"hovering in/near {side} opening for {hover_seconds:.1f}s")
            hover_started_at = time.monotonic()
            while time.monotonic() - hover_started_at < hover_seconds:
                rclpy.spin_once(monitor.node, timeout_sec=0.02)
                await send_zero_body_velocity(drone, velocity_type)
                await asyncio.sleep(0.05)

        inspection = await sample_gas_for_duration(
            drone,
            rclpy,
            monitor,
            args,
            velocity_type,
            "inspection",
            args.inspection_sample_seconds,
            compute_ppm,
            active_sources,
            rng,
        )

        delta_ppm = inspection.avg_ppm - baseline.avg_ppm
        candidate_by_delta = delta_ppm >= max(0.0, args.gas_delta_threshold)
        candidate_by_absolute = inspection.avg_ppm >= max(0.0, args.gas_absolute_threshold)
        gas_candidate = candidate_by_delta or candidate_by_absolute
        if candidate_by_delta:
            reason = "delta_ppm"
        elif candidate_by_absolute:
            reason = "absolute_ppm"
        else:
            reason = "below_threshold"
        log(
            "GAS",
            "gas candidate decision: "
            f"side={side}, baseline={baseline.avg_ppm:.2f}, inspection={inspection.avg_ppm:.2f}, "
            f"delta={delta_ppm:.2f}, candidate={gas_candidate}, reason={reason}",
        )
        if gas_candidate:
            log("GAS", f"high gas zone candidate detected at opening={opening_id}")
        else:
            log("GAS", f"no significant gas increase at opening={opening_id}")

        event = build_body_inspection_event(
            inspection_index=len(events) + 1,
            side=side,
            opening_id=opening_id,
            step_index=step_index,
            baseline=baseline,
            inspection=inspection,
            delta_ppm=delta_ppm,
            gas_candidate=gas_candidate,
            candidate_reason=reason,
            entry_anchor_position=entry_anchor_position,
            enter_target_distance_m=args.inspection_enter_distance,
            enter_result=enter_result,
            exit_tolerance_m=args.inspection_exit_tolerance,
        )
        events.append(event)
        write_inspection_events(args.inspection_events_output, payload)

        exit_result = await run_distance_based_lateral_body_motion(
            drone,
            rclpy,
            monitor,
            args,
            velocity_type,
            f"exit {side} opening",
            -enter_right_speed,
            entry_anchor_position,
            None,
            args.inspection_exit_tolerance,
            args.inspection_exit_timeout_seconds,
        )
        event["exit_final_distance_m"] = exit_result["final_anchor_distance_m"]
        event["exit_elapsed_seconds"] = exit_result["elapsed_seconds"]
        write_inspection_events(args.inspection_events_output, payload)
        log("INSPECT", f"returned from {side} inspection; continuing corridor-follow")

    async def try_inspect_opening(side: str, step_index: int) -> None:
        nonlocal last_inspection_step
        if len(events) >= max_inspections:
            log("INSPECT", "max inspections reached; continuing corridor-follow without inspection")
            return
        if step_index - last_inspection_step < cooldown_steps:
            log("INSPECT", f"skip opening due to cooldown: step={step_index + 1}, last={last_inspection_step + 1}")
            return
        if not await run_opening_alignment_gate(drone, rclpy, monitor, args, velocity_type, side):
            log("ALIGN", f"{side} opening skipped after alignment gate")
            return
        await inspect_opening(side, step_index)

    def reject_opening_candidate(reason: str) -> None:
        nonlocal active_candidate
        if active_candidate is None:
            return
        log(
            "OPENING",
            f"candidate rejected reason={reason} side={active_candidate.active_side} "
            f"frames={active_candidate.frames_seen} best={active_candidate.best_side_avg:.2f}",
        )
        active_candidate = None

    async def update_opening_candidate(opening_side: str | None, step_index: int, phase: str) -> bool:
        nonlocal active_candidate
        if len(events) >= max_inspections:
            active_candidate = None
            return False

        scan = monitor.decision_snapshot()
        side_to_track = opening_side or (active_candidate.active_side if active_candidate is not None else None)
        if side_to_track is None:
            return False

        is_open, reason, metrics = side_decision_diagnostics(scan, side_to_track, args)
        if not is_open:
            if active_candidate is not None:
                reject_opening_candidate("lost_opening")
            elif opening_side is not None:
                log("OPENING", f"candidate rejected reason=lost_opening {metrics}")
            return False

        front_clear, front_distance = front_clearance_for_candidate(monitor, args)
        if args.opening_require_front_clear and not front_clear:
            if active_candidate is not None:
                reject_opening_candidate("front_not_clear")
            else:
                front_text = "unknown" if front_distance is None else f"{front_distance:.2f}"
                log(
                    "OPENING",
                    f"candidate rejected reason=front_not_clear side={side_to_track} front={front_text}",
                )
            return False

        current_position = await read_position_ned_quiet(drone)
        side_avg = side_avg_for_snapshot(scan, side_to_track)
        side_min = side_min_for_snapshot(scan, side_to_track)

        if active_candidate is None or active_candidate.active_side != side_to_track:
            if active_candidate is not None:
                reject_opening_candidate("side_changed")
            active_candidate = OpeningCandidateState(
                active_side=side_to_track,
                start_step=step_index,
                start_position=current_position,
                best_position=current_position,
                best_side_avg=side_avg,
                frames_seen=1,
                last_seen_step=step_index,
                best_front_distance=front_distance,
            )
            log(
                "OPENING",
                f"candidate started side={side_to_track} step={step_index + 1} phase={phase} "
                f"side_avg={side_avg:.2f} side_min={side_min:.2f} reason={reason}",
            )
            return False

        active_candidate.frames_seen += 1
        active_candidate.last_seen_step = step_index
        if side_avg > active_candidate.best_side_avg:
            active_candidate.best_side_avg = side_avg
            active_candidate.best_position = current_position
            active_candidate.best_front_distance = front_distance

        progress = horizontal_distance(current_position, active_candidate.start_position)
        progress_text = "unknown" if progress is None else f"{progress:.2f}"
        front_text = "unknown" if front_distance is None else f"{front_distance:.2f}"
        log(
            "OPENING",
            f"candidate update side={side_to_track} phase={phase} side_avg={side_avg:.2f} "
            f"side_min={side_min:.2f} best={active_candidate.best_side_avg:.2f} "
            f"frames={active_candidate.frames_seen} progress={progress_text} front={front_text} "
            f"valid={scan.left_valid_count if side_to_track == 'left' else scan.right_valid_count} "
            f"inf={scan.left_inf_count if side_to_track == 'left' else scan.right_inf_count}",
        )

        min_frames = max(1, args.opening_min_persistence_frames)
        min_progress = max(0.0, args.opening_min_forward_progress)
        peak_drop = max(0.0, args.opening_peak_drop_distance)
        if active_candidate.frames_seen < min_frames:
            log(
                "OPENING",
                f"candidate waiting reason=insufficient_persistence "
                f"frames={active_candidate.frames_seen}/{min_frames}",
            )
            return False
        if progress is None or progress < min_progress:
            log(
                "OPENING",
                f"candidate waiting reason=insufficient_progress progress={progress_text}/{min_progress:.2f}",
            )
            return False
        if side_avg < active_candidate.best_side_avg - peak_drop:
            reject_opening_candidate("lost_peak")
            return False

        side_for_inspection = active_candidate.active_side
        log(
            "OPENING",
            f"candidate mature/confirmed side={side_for_inspection} "
            f"frames={active_candidate.frames_seen} progress={progress_text} "
            f"best={active_candidate.best_side_avg:.2f}",
        )
        log("OPENING", f"inspection allowed at best candidate side={side_for_inspection}")
        active_candidate = None
        await try_inspect_opening(side_for_inspection, step_index)
        return True

    start_position = await read_position_ned(drone, "inspection start")
    await warmup_offboard_body(drone, rclpy, monitor, velocity_type, args)
    log("INSPECT", "starting offboard mode")
    await drone.offboard.start()

    try:
        for step_index in range(step_count):
            tag = f"INSPECT MOVE {step_index + 1}/{step_count}"
            rclpy.spin_once(monitor.node, timeout_sec=0.05)
            decision = log_passive_corridor_decision(monitor, memory, args, f"step={step_index + 1}/{step_count} pre")
            opening_side = opening_side_for_decision(decision)
            if opening_side is not None or active_candidate is not None:
                await update_opening_candidate(opening_side, step_index, "pre")
                if len(events) >= max_inspections:
                    log("INSPECT", "max inspections reached; remaining steps will only follow corridor")
            if not await front_motion_allowed_with_retry(drone, rclpy, monitor, args, velocity_type):
                await send_zero_body_velocity(drone, velocity_type)
                break

            log(tag, f"body velocity: forward={forward_speed:.2f} m/s")
            step_started_at = time.monotonic()
            next_decision_log_at = step_started_at
            while time.monotonic() - step_started_at < step_duration_sec:
                rclpy.spin_once(monitor.node, timeout_sec=0.0)
                now = time.monotonic()
                if now >= next_decision_log_at:
                    decision = log_passive_corridor_decision(
                        monitor,
                        memory,
                        args,
                        f"step={step_index + 1}/{step_count} moving",
                    )
                    next_decision_log_at = now + decision_log_interval_sec
                    opening_side = opening_side_for_decision(decision)
                    if (opening_side is not None or active_candidate is not None) and len(events) < max_inspections:
                        inspection_started = await update_opening_candidate(opening_side, step_index, "moving")
                        if inspection_started:
                            break
                if not await front_motion_allowed_with_retry(drone, rclpy, monitor, args, velocity_type):
                    await send_zero_body_velocity(drone, velocity_type)
                    return payload
                command_right_speed = compute_corridor_centering_command(monitor, args)
                command_down_speed = (
                    await compute_altitude_hold_command(drone, args)
                    if args.enable_altitude_hold
                    else 0.0
                )
                await drone.offboard.set_velocity_body(
                    velocity_type(forward_speed, command_right_speed, command_down_speed, 0.0)
                )
                await asyncio.sleep(interval_sec)

            await send_zero_body_velocity(drone, velocity_type)
            end_position = await read_position_ned(drone, f"after inspection step {step_index + 1}")
            log_position_delta(f"inspection step {step_index + 1}", end_position, start_position)
            if pause_sec > 0.0:
                log(tag, f"pausing for {pause_sec:.1f}s")
                pause_started_at = time.monotonic()
                while time.monotonic() - pause_started_at < pause_sec:
                    rclpy.spin_once(monitor.node, timeout_sec=0.02)
                    await send_zero_body_velocity(drone, velocity_type)
                    await asyncio.sleep(0.05)
    finally:
        try:
            await send_zero_body_velocity(drone, velocity_type)
        except Exception as exc:
            log("INSPECT", f"body zero velocity before offboard stop failed: {exc}")
        try:
            log("INSPECT", "stopping offboard mode")
            await drone.offboard.stop()
        except Exception as exc:
            log("INSPECT", f"offboard stop failed, continuing to land: {exc}")

    write_inspection_events(args.inspection_events_output, payload)
    log("INSPECT", f"inspection events written: {args.inspection_events_output}")
    return payload


async def stream_zero_hover(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    velocity_type: object,
    yaw_deg: float,
    duration_sec: float,
    rate_hz: float,
) -> tuple[float, float] | None:
    duration_sec = max(0.0, duration_sec)
    rate_hz = max(1.0, rate_hz)
    interval_sec = 1.0 / rate_hz
    if duration_sec == 0.0:
        return None

    start_position = await read_position_ned(drone, "zero-hover start")
    started_at = time.monotonic()
    while time.monotonic() - started_at < duration_sec:
        rclpy.spin_once(monitor.node, timeout_sec=0.0)
        await send_zero_velocity(drone, velocity_type, yaw_deg)
        await asyncio.sleep(interval_sec)

    end_position = await read_position_ned(drone, "zero-hover end")
    if start_position is None or end_position is None:
        log("CALIBRATE", "zero-hover drift unavailable")
        return None

    delta_north, delta_east, delta_down = position_delta(end_position, start_position)
    drift = horizontal_magnitude(delta_north, delta_east)
    log(
        "CALIBRATE",
        f"zero-hover drift: north={delta_north:.2f} m, east={delta_east:.2f} m, "
        f"down={delta_down:.2f} m, horizontal={drift:.2f} m",
    )
    return delta_north, delta_east


async def run_axis_calibration_pulse(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    args: argparse.Namespace,
    velocity_type: object,
    yaw_deg: float,
    label: str,
    north_speed: float,
    east_speed: float,
) -> tuple[float, float] | None:
    duration_sec = max(0.0, args.axis_calibration_duration)
    rate_hz = max(1.0, args.move_rate_hz)
    interval_sec = 1.0 / rate_hz
    if duration_sec == 0.0:
        log("CALIBRATE", f"{label} pulse skipped because duration is zero")
        return None

    start_position = await read_position_ned(drone, f"{label} pulse start")
    log(
        "CALIBRATE",
        f"{label} pulse: north={north_speed:.2f} m/s, east={east_speed:.2f} m/s, duration={duration_sec:.1f}s",
    )

    started_at = time.monotonic()
    next_front_log_at = started_at
    while time.monotonic() - started_at < duration_sec:
        rclpy.spin_once(monitor.node, timeout_sec=0.0)
        now = time.monotonic()
        if now >= next_front_log_at:
            front_min = front_sector_min_distance(monitor.latest_front, args.front_sector_deg)
            front_text = "unavailable" if front_min is None else f"{front_min:.2f} m"
            log("CALIBRATE", f"{label} front_sector_min={front_text}")
            next_front_log_at = now + 0.5
        await drone.offboard.set_velocity_ned(velocity_type(north_speed, east_speed, 0.0, yaw_deg))
        await asyncio.sleep(interval_sec)

    await send_zero_velocity(drone, velocity_type, yaw_deg)
    await asyncio.sleep(0.2)
    end_position = await read_position_ned(drone, f"{label} pulse end")
    if start_position is None or end_position is None:
        log("CALIBRATE", f"{label} pulse displacement unavailable")
        return None

    delta_north, delta_east, delta_down = position_delta(end_position, start_position)
    log(
        "CALIBRATE",
        f"{label} pulse displacement: north={delta_north:.2f} m, east={delta_east:.2f} m, down={delta_down:.2f} m",
    )
    return delta_north, delta_east


async def run_body_axis_calibration_pulse(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    args: argparse.Namespace,
    velocity_type: object,
) -> tuple[float, float] | None:
    duration_sec = max(0.0, args.axis_calibration_duration)
    rate_hz = max(1.0, args.move_rate_hz)
    interval_sec = 1.0 / rate_hz
    forward_speed = float(args.body_forward_speed)
    right_speed = float(args.body_right_speed)
    down_speed = float(args.body_down_speed)
    yawspeed = float(args.body_yawspeed)
    if duration_sec == 0.0:
        log("CALIBRATE", "body-forward pulse skipped because duration is zero")
        return None

    start_position = await read_position_ned(drone, "body-forward pulse start")
    log(
        "CALIBRATE",
        "body-forward pulse: "
        f"forward={forward_speed:.2f} m/s, right={right_speed:.2f} m/s, "
        f"down={down_speed:.2f} m/s, yawspeed={yawspeed:.2f} deg/s, duration={duration_sec:.1f}s",
    )

    started_at = time.monotonic()
    next_front_log_at = started_at
    while time.monotonic() - started_at < duration_sec:
        rclpy.spin_once(monitor.node, timeout_sec=0.0)
        now = time.monotonic()
        if now >= next_front_log_at:
            front_min = front_sector_min_distance(monitor.latest_front, args.front_sector_deg)
            front_text = "unavailable" if front_min is None else f"{front_min:.2f} m"
            log("CALIBRATE", f"body-forward front_sector_min={front_text}")
            next_front_log_at = now + 0.5
        await drone.offboard.set_velocity_body(
            velocity_type(forward_speed, right_speed, down_speed, yawspeed)
        )
        await asyncio.sleep(interval_sec)

    await send_zero_body_velocity(drone, velocity_type)
    await asyncio.sleep(0.2)
    end_position = await read_position_ned(drone, "body-forward pulse end")
    if start_position is None or end_position is None:
        log("CALIBRATE", "body-forward pulse displacement unavailable")
        return None

    delta_north, delta_east, delta_down = position_delta(end_position, start_position)
    log(
        "CALIBRATE",
        f"body-forward pulse displacement: north={delta_north:.2f} m, east={delta_east:.2f} m, down={delta_down:.2f} m",
    )
    return delta_north, delta_east


def recommend_corridor_axis(
    north_result: tuple[float, float] | None,
    east_result: tuple[float, float] | None,
) -> str:
    if north_result is None or east_result is None:
        return "unknown"

    north_magnitude = horizontal_magnitude(*north_result)
    east_magnitude = horizontal_magnitude(*east_result)
    if north_magnitude < 0.05 and east_magnitude < 0.05:
        return "unknown"
    if east_magnitude > north_magnitude * 1.25:
        return "east"
    if north_magnitude > east_magnitude * 1.25:
        return "north"
    return "unknown"


async def run_axis_calibration_steps(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    args: argparse.Namespace,
    ned_velocity_type: object,
    body_velocity_type: object,
    yaw_deg: float,
) -> None:
    speed = max(0.0, args.axis_calibration_speed)
    rate_hz = max(1.0, args.move_rate_hz)

    await warmup_offboard_ned(drone, rclpy, monitor, ned_velocity_type, yaw_deg, args)
    log("CALIBRATE", "starting offboard mode")
    await drone.offboard.start()

    try:
        zero_drift = await stream_zero_hover(
            drone,
            rclpy,
            monitor,
            ned_velocity_type,
            yaw_deg,
            args.offboard_zero_hover_seconds,
            rate_hz,
        )
        north_result = await run_axis_calibration_pulse(
            drone,
            rclpy,
            monitor,
            args,
            ned_velocity_type,
            yaw_deg,
            "north",
            speed,
            0.0,
        )
        east_result = await run_axis_calibration_pulse(
            drone,
            rclpy,
            monitor,
            args,
            ned_velocity_type,
            yaw_deg,
            "east",
            0.0,
            speed,
        )
        body_result = await run_body_axis_calibration_pulse(
            drone,
            rclpy,
            monitor,
            args,
            body_velocity_type,
        )

        if zero_drift is not None:
            log("CALIBRATE", f"zero-hover summary: north={zero_drift[0]:.2f} m, east={zero_drift[1]:.2f} m")
        if north_result is not None:
            log("CALIBRATE", f"north pulse summary: north={north_result[0]:.2f} m, east={north_result[1]:.2f} m")
        if east_result is not None:
            log("CALIBRATE", f"east pulse summary: north={east_result[0]:.2f} m, east={east_result[1]:.2f} m")
        if body_result is not None:
            log("CALIBRATE", f"body-forward pulse summary: north={body_result[0]:.2f} m, east={body_result[1]:.2f} m")
        log("CALIBRATE", f"recommended_corridor_axis={recommend_corridor_axis(north_result, east_result)}")
    finally:
        try:
            await send_zero_velocity(drone, ned_velocity_type, yaw_deg)
        except Exception as exc:
            log("CALIBRATE", f"zero velocity before offboard stop failed: {exc}")
        try:
            log("CALIBRATE", "stopping offboard mode")
            await drone.offboard.stop()
        except Exception as exc:
            log("CALIBRATE", f"offboard stop failed, continuing to land: {exc}")


def is_grpc_disconnect_error(exc: Exception) -> bool:
    message = str(exc).lower()
    return any(
        marker in message
        for marker in (
            "grpc",
            "channel",
            "connection",
            "socket",
            "unavailable",
            "connection reset",
            "broken pipe",
        )
    )


async def run_takeoff_land_check(args: argparse.Namespace) -> int:
    system_type = import_mavsdk_system()
    if system_type is None:
        return 1

    rclpy = None
    monitor = None
    if not args.skip_scan_ready_check:
        monitor_bundle = create_scan_monitor(args)
        if monitor_bundle is None:
            return 1
        rclpy, monitor = monitor_bundle

    seed = args.seed if args.seed is not None else 0
    memory = MissionMemory(
        visited_openings=set(),
        skipped_openings=set(),
        bypass_attempts=0,
        corridor_x=0.0,
        seed=seed,
    )
    drone = system_type()
    land_requested = False

    log("CONFIG", "opening-based takeoff/land scan readiness check")
    log("CONFIG", f"system_address={args.system_address}")
    log("CONFIG", f"takeoff_altitude={args.takeoff_altitude:.1f} m")
    log("CONFIG", f"takeoff_altitude_tolerance={args.takeoff_altitude_tolerance:.2f} m")
    log("CONFIG", f"min_takeoff_confirm_altitude={args.min_takeoff_confirm_altitude:.2f} m")
    log("CONFIG", f"post_takeoff_settle_seconds={max(0.0, args.post_takeoff_settle_seconds):.1f}")
    log("CONFIG", f"hover_seconds={max(0.0, args.hover_seconds):.1f}")
    log("CONFIG", f"pre_land_settle_seconds={max(0.0, args.pre_land_settle_seconds):.1f}")
    log("CONFIG", f"scan_ready_check={not args.skip_scan_ready_check}")
    log("CONFIG", f"front_scan_topic={args.front_scan_topic}")
    log("CONFIG", f"left_scan_topic={args.left_scan_topic}")
    log("CONFIG", f"right_scan_topic={args.right_scan_topic}")

    try:
        if monitor is not None and rclpy is not None:
            scan_ready = await wait_for_scan_readiness(rclpy, monitor, args)
            if not scan_ready:
                log("FINISH", "aborting before MAVSDK connect because scan readiness failed")
                return 1

        log("CONNECT", f"connecting to MAVSDK system at {args.system_address}")
        await asyncio.wait_for(drone.connect(system_address=args.system_address), timeout=args.connection_timeout)
        await wait_for_mavsdk_connection(drone, args.connection_timeout)
        await wait_for_mavsdk_health(drone, args.connection_timeout)

        log("TAKEOFF", f"setting takeoff altitude to {args.takeoff_altitude:.1f} m")
        await drone.action.set_takeoff_altitude(max(0.1, args.takeoff_altitude))
        log("TAKEOFF", "arming")
        await drone.action.arm()
        log("TAKEOFF", "takeoff command sent")
        await drone.action.takeoff()

        altitude_result = await wait_until_takeoff_altitude(
            drone,
            target_altitude_m=args.takeoff_altitude,
            timeout_sec=args.takeoff_timeout,
            tolerance_m=args.takeoff_altitude_tolerance,
            min_confirm_altitude_m=args.min_takeoff_confirm_altitude,
        )
        should_hover = altitude_result.confirmed or altitude_result.safe_hover_altitude
        if altitude_result.confirmed:
            log(
                "TAKEOFF",
                f"takeoff altitude confirmed at {altitude_result.last_altitude_m:.2f} m",
            )
        elif altitude_result.safe_hover_altitude:
            log(
                "TAKEOFF",
                "target altitude was not fully confirmed, "
                f"but safe hover altitude was reached at {altitude_result.last_altitude_m:.2f} m",
            )
        else:
            log(
                "TAKEOFF",
                "takeoff altitude was not confirmed and safe hover altitude was not reached; "
                f"last_altitude={altitude_result.last_altitude_m:.2f} m",
            )

        if should_hover:
            settle_seconds = max(0.0, args.post_takeoff_settle_seconds)
            if settle_seconds > 0.0:
                log("TAKEOFF", f"settling after takeoff for {settle_seconds:.1f}s")
                await asyncio.sleep(settle_seconds)

        if monitor is not None and rclpy is not None and should_hover:
            await log_airborne_scan_checks(rclpy, monitor, args, memory, args.hover_seconds)
        elif should_hover:
            hover_seconds = max(0.0, args.hover_seconds)
            log("TAKEOFF", f"hovering without scan checks for {hover_seconds:.1f}s")
            if hover_seconds > 0.0:
                await asyncio.sleep(hover_seconds)
        else:
            log("TAKEOFF", "skipping hover scan checks and landing")

        pre_land_settle_seconds = max(0.0, args.pre_land_settle_seconds)
        if pre_land_settle_seconds > 0.0:
            log("LAND", f"settling before land for {pre_land_settle_seconds:.1f}s")
            await asyncio.sleep(pre_land_settle_seconds)

        land_requested = True
        log("LAND", "land command sent")
        await drone.action.land()
        log("FINISH", "takeoff/land scan readiness check complete")
        return 0
    except asyncio.TimeoutError:
        log("FINISH", "timed out while waiting for PX4 connection or vehicle health")
        if not land_requested:
            await try_safety_land(drone)
        return 1
    except KeyboardInterrupt:
        log("FINISH", "interrupted by user")
        if not land_requested:
            await try_safety_land(drone)
        return 130
    except Exception as exc:
        if land_requested:
            if is_grpc_disconnect_error(exc):
                log("LAND", f"connection closed after land command; exiting calmly: {exc}")
                return 0
            log("LAND", f"land command was already sent; not retrying safety land: {exc}")
            return 1

        log("FINISH", f"mission failed before land command: {exc}")
        await try_safety_land(drone)
        return 1
    finally:
        close_scan_monitor(rclpy, monitor)


async def run_corridor_follow_check(args: argparse.Namespace) -> int:
    system_type = import_mavsdk_system()
    velocity_type = import_velocity_body_yawspeed()
    if system_type is None or velocity_type is None:
        return 1

    monitor_bundle = create_scan_monitor(args)
    if monitor_bundle is None:
        return 1
    rclpy, monitor = monitor_bundle

    drone = system_type()
    land_requested = False
    offboard_started = False

    log("CONFIG", "opening-based corridor follow check")
    log("CONFIG", f"system_address={args.system_address}")
    log("CONFIG", f"takeoff_altitude={args.takeoff_altitude:.1f} m")
    log("CONFIG", f"takeoff_altitude_tolerance={args.takeoff_altitude_tolerance:.2f} m")
    log("CONFIG", f"min_takeoff_confirm_altitude={args.min_takeoff_confirm_altitude:.2f} m")
    log("CONFIG", f"post_takeoff_settle_seconds={max(0.0, args.post_takeoff_settle_seconds):.1f}")
    log("CONFIG", f"pre_land_settle_seconds={max(0.0, args.pre_land_settle_seconds):.1f}")
    log("CONFIG", f"corridor_step_count={max(0, args.corridor_step_count)}")
    log("CONFIG", f"corridor_step_duration_seconds={max(0.0, args.corridor_step_duration_seconds):.1f}")
    log("CONFIG", f"body_forward_speed={args.body_forward_speed:.2f} m/s")
    log("CONFIG", f"body_right_speed={args.body_right_speed:.2f} m/s")
    log("CONFIG", f"body_down_speed={args.body_down_speed:.2f} m/s")
    log("CONFIG", f"body_yawspeed={args.body_yawspeed:.2f} deg/s")
    log("CONFIG", f"front_stop_distance={args.front_stop_distance:.2f} m")
    log("CONFIG", f"front_clear_distance={args.front_clear_distance:.2f} m")
    log("CONFIG", f"front_sector_deg={args.front_sector_deg:.1f}")
    log("CONFIG", f"front_scan_topic={args.front_scan_topic}")

    try:
        scan_ready = await wait_for_scan_readiness(rclpy, monitor, args)
        if not scan_ready:
            log("FINISH", "aborting before MAVSDK connect because scan readiness failed")
            return 1

        log("CONNECT", f"connecting to MAVSDK system at {args.system_address}")
        await asyncio.wait_for(drone.connect(system_address=args.system_address), timeout=args.connection_timeout)
        await wait_for_mavsdk_connection(drone, args.connection_timeout)
        await wait_for_mavsdk_health(drone, args.connection_timeout)

        log("TAKEOFF", f"setting takeoff altitude to {args.takeoff_altitude:.1f} m")
        await drone.action.set_takeoff_altitude(max(0.1, args.takeoff_altitude))
        log("TAKEOFF", "arming")
        await drone.action.arm()
        log("TAKEOFF", "takeoff command sent")
        await drone.action.takeoff()

        altitude_result = await wait_until_takeoff_altitude(
            drone,
            target_altitude_m=args.takeoff_altitude,
            timeout_sec=args.takeoff_timeout,
            tolerance_m=args.takeoff_altitude_tolerance,
            min_confirm_altitude_m=args.min_takeoff_confirm_altitude,
        )
        should_move = altitude_result.confirmed or altitude_result.safe_hover_altitude
        if altitude_result.confirmed:
            log("TAKEOFF", f"takeoff altitude confirmed at {altitude_result.last_altitude_m:.2f} m")
        elif altitude_result.safe_hover_altitude:
            log(
                "TAKEOFF",
                "target altitude was not fully confirmed, "
                f"but safe hover altitude was reached at {altitude_result.last_altitude_m:.2f} m",
            )
        else:
            log(
                "TAKEOFF",
                "takeoff altitude was not confirmed and safe hover altitude was not reached; "
                f"last_altitude={altitude_result.last_altitude_m:.2f} m",
            )

        if should_move:
            settle_seconds = max(0.0, args.post_takeoff_settle_seconds)
            if settle_seconds > 0.0:
                log("TAKEOFF", f"settling after takeoff for {settle_seconds:.1f}s")
                settle_started_at = time.monotonic()
                while time.monotonic() - settle_started_at < settle_seconds:
                    rclpy.spin_once(monitor.node, timeout_sec=0.02)
                    await asyncio.sleep(0.05)

            yaw_deg = await read_initial_yaw_deg(drone)
            log("MOVE", f"front safety is aligned with body forward direction; initial_yaw={yaw_deg:.1f} deg")
            offboard_started = True
            await run_body_corridor_follow_steps(drone, rclpy, monitor, args, velocity_type)
            offboard_started = False
        else:
            log("MOVE", "skipping corridor movement and landing")

        pre_land_settle_seconds = max(0.0, args.pre_land_settle_seconds)
        if pre_land_settle_seconds > 0.0:
            log("LAND", f"settling before land for {pre_land_settle_seconds:.1f}s")
            settle_started_at = time.monotonic()
            while time.monotonic() - settle_started_at < pre_land_settle_seconds:
                rclpy.spin_once(monitor.node, timeout_sec=0.02)
                await asyncio.sleep(0.05)

        land_requested = True
        log("LAND", "land command sent")
        await drone.action.land()
        log("FINISH", "corridor follow check complete")
        return 0
    except asyncio.TimeoutError:
        log("FINISH", "timed out while waiting for PX4 connection or vehicle health")
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception as exc:
                log("MOVE", f"offboard stop during timeout cleanup failed: {exc}")
        if not land_requested:
            await try_safety_land(drone)
        return 1
    except KeyboardInterrupt:
        log("FINISH", "interrupted by user")
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception as exc:
                log("MOVE", f"offboard stop during interrupt cleanup failed: {exc}")
        if not land_requested:
            await try_safety_land(drone)
        return 130
    except Exception as exc:
        if land_requested:
            if is_grpc_disconnect_error(exc):
                log("LAND", f"connection closed after land command; exiting calmly: {exc}")
                return 0
            log("LAND", f"land command was already sent; not retrying safety land: {exc}")
            return 1

        log("FINISH", f"mission failed before land command: {exc}")
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception as stop_exc:
                log("MOVE", f"offboard stop during error cleanup failed: {stop_exc}")
        await try_safety_land(drone)
        return 1
    finally:
        close_scan_monitor(rclpy, monitor)


async def run_room_inspection_check(args: argparse.Namespace) -> int:
    system_type = import_mavsdk_system()
    velocity_type = import_velocity_body_yawspeed()
    gas_model = import_gas_model()
    if system_type is None or velocity_type is None or gas_model is None:
        return 1
    compute_ppm, resolve_scenario, possible_gas_zones = gas_model

    monitor_bundle = create_scan_monitor(args)
    if monitor_bundle is None:
        return 1
    rclpy, monitor = monitor_bundle

    rng = random.Random(args.gas_seed)
    try:
        scenario, active_sources = resolve_scenario(args.gas_scenario, rng)
    except Exception as exc:
        print(f"Could not resolve gas scenario {args.gas_scenario!r}: {exc}")
        close_scan_monitor(rclpy, monitor)
        return 2

    drone = system_type()
    land_requested = False
    offboard_started = False

    log("CONFIG", "opening-based room inspection check")
    log("CONFIG", f"system_address={args.system_address}")
    log("CONFIG", f"takeoff_altitude={args.takeoff_altitude:.1f} m")
    log("CONFIG", f"min_takeoff_confirm_altitude={args.min_takeoff_confirm_altitude:.2f} m")
    log("CONFIG", f"corridor_step_count={max(0, args.corridor_step_count)}")
    log("CONFIG", f"body_forward_speed={args.body_forward_speed:.2f} m/s")
    log("CONFIG", f"max_inspections={max(0, args.max_inspections)}")
    log("CONFIG", f"inspection_side_speed={abs(float(args.inspection_side_speed)):.2f} m/s")
    log("CONFIG", f"inspection_events_output={args.inspection_events_output}")
    log("GAS", f"gas_scenario={args.gas_scenario}, resolved={scenario}, gas_seed={args.gas_seed}")

    try:
        scan_ready = await wait_for_scan_readiness(rclpy, monitor, args)
        if not scan_ready:
            log("FINISH", "aborting before MAVSDK connect because scan readiness failed")
            return 1

        log("CONNECT", f"connecting to MAVSDK system at {args.system_address}")
        await asyncio.wait_for(drone.connect(system_address=args.system_address), timeout=args.connection_timeout)
        await wait_for_mavsdk_connection(drone, args.connection_timeout)
        await wait_for_mavsdk_health(drone, args.connection_timeout)

        log("TAKEOFF", f"setting takeoff altitude to {args.takeoff_altitude:.1f} m")
        await drone.action.set_takeoff_altitude(max(0.1, args.takeoff_altitude))
        log("TAKEOFF", "arming")
        await drone.action.arm()
        log("TAKEOFF", "takeoff command sent")
        await drone.action.takeoff()

        altitude_result = await wait_until_takeoff_altitude(
            drone,
            target_altitude_m=args.takeoff_altitude,
            timeout_sec=args.takeoff_timeout,
            tolerance_m=args.takeoff_altitude_tolerance,
            min_confirm_altitude_m=args.min_takeoff_confirm_altitude,
        )
        should_inspect = altitude_result.confirmed or altitude_result.safe_hover_altitude
        if altitude_result.confirmed:
            log("TAKEOFF", f"takeoff altitude confirmed at {altitude_result.last_altitude_m:.2f} m")
        elif altitude_result.safe_hover_altitude:
            log(
                "TAKEOFF",
                "target altitude was not fully confirmed, "
                f"but safe hover altitude was reached at {altitude_result.last_altitude_m:.2f} m",
            )
        else:
            log(
                "TAKEOFF",
                "takeoff altitude was not confirmed and safe hover altitude was not reached; "
                f"last_altitude={altitude_result.last_altitude_m:.2f} m",
            )

        if should_inspect:
            settle_seconds = max(0.0, args.post_takeoff_settle_seconds)
            if settle_seconds > 0.0:
                log("TAKEOFF", f"settling after takeoff for {settle_seconds:.1f}s")
                settle_started_at = time.monotonic()
                while time.monotonic() - settle_started_at < settle_seconds:
                    rclpy.spin_once(monitor.node, timeout_sec=0.02)
                    await asyncio.sleep(0.05)

            yaw_deg = await read_initial_yaw_deg(drone)
            log("INSPECT", f"front safety is aligned with body forward direction; initial_yaw={yaw_deg:.1f} deg")
            offboard_started = True
            await run_room_inspection_steps(
                drone,
                rclpy,
                monitor,
                args,
                velocity_type,
                compute_ppm,
                active_sources,
                scenario,
                possible_gas_zones,
            )
            offboard_started = False
        else:
            log("INSPECT", "skipping room inspection and landing")

        pre_land_settle_seconds = max(0.0, args.pre_land_settle_seconds)
        if pre_land_settle_seconds > 0.0:
            log("LAND", f"settling before land for {pre_land_settle_seconds:.1f}s")
            settle_started_at = time.monotonic()
            while time.monotonic() - settle_started_at < pre_land_settle_seconds:
                rclpy.spin_once(monitor.node, timeout_sec=0.02)
                await asyncio.sleep(0.05)

        land_requested = True
        log("LAND", "land command sent")
        await drone.action.land()
        log("FINISH", "room inspection check complete")
        return 0
    except asyncio.TimeoutError:
        log("FINISH", "timed out while waiting for PX4 connection or vehicle health")
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception as exc:
                log("INSPECT", f"offboard stop during timeout cleanup failed: {exc}")
        if not land_requested:
            await try_safety_land(drone)
        return 1
    except KeyboardInterrupt:
        log("FINISH", "interrupted by user")
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception as exc:
                log("INSPECT", f"offboard stop during interrupt cleanup failed: {exc}")
        if not land_requested:
            await try_safety_land(drone)
        return 130
    except Exception as exc:
        if land_requested:
            if is_grpc_disconnect_error(exc):
                log("LAND", f"connection closed after land command; exiting calmly: {exc}")
                return 0
            log("LAND", f"land command was already sent; not retrying safety land: {exc}")
            return 1

        log("FINISH", f"room inspection failed before land command: {exc}")
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception as stop_exc:
                log("INSPECT", f"offboard stop during error cleanup failed: {stop_exc}")
        await try_safety_land(drone)
        return 1
    finally:
        close_scan_monitor(rclpy, monitor)


async def run_position_room_inspection_steps(
    drone: object,
    rclpy: object,
    monitor: LaserScanMonitor,
    args: argparse.Namespace,
    position_type: object,
    compute_ppm: object,
    active_sources: list[dict[str, Any]],
    scenario: str,
    possible_gas_zones: dict[str, tuple[float, float]],
) -> dict[str, Any]:
    step_count = max(0, args.position_step_count)
    forward_step = max(0.0, args.position_forward_step)
    hold_sec = max(0.0, args.position_hold_seconds)
    down_m = -max(0.1, args.position_altitude)
    yaw_deg = float(args.position_yaw)
    room_entry_distance = max(0.0, args.position_room_entry_distance)
    room_entry_hold_sec = max(0.0, args.position_room_entry_hold_seconds)
    max_inspections = max(0, args.max_inspections)
    rng = random.Random(args.gas_seed)
    memory = MissionMemory(
        visited_openings=set(),
        skipped_openings=set(),
        bypass_attempts=0,
        corridor_x=0.0,
        seed=args.seed if args.seed is not None else 0,
    )
    current_north = 0.0
    current_east = 0.0
    active_candidate: OpeningCandidateState | None = None
    events: list[dict[str, Any]] = []
    payload: dict[str, Any] = {
        "requested_scenario": args.gas_scenario,
        "scenario": scenario,
        "gas_seed": args.gas_seed,
        "active_sources": active_sources,
        "possible_gas_zones": {name: {"x": xy[0], "y": xy[1]} for name, xy in possible_gas_zones.items()},
        "events": events,
    }

    log("POS", "position room inspection uses PositionNedYaw setpoint steps")
    log("POS", f"step_count={step_count}")
    log("POS", f"forward_step={forward_step:.2f} m")
    log("POS", f"hold_seconds={hold_sec:.1f}")
    log("POS", f"altitude={-down_m:.2f} m")
    log("POS", f"yaw={yaw_deg:.1f} deg")
    log("POS", f"room_entry_distance={room_entry_distance:.2f} m")
    log("POS", f"max_inspections={max_inspections}")
    log("CAPTURE", f"no_backtrack_enabled={args.enable_no_backtrack_door_capture}")
    log("ROOM", f"sensor_traversal_enabled={args.enable_sensor_room_traversal}")
    log("ROOM", f"room_facing_yaw_enabled={args.enable_room_facing_yaw_entry}")

    def reject_candidate(reason: str) -> None:
        nonlocal active_candidate
        if active_candidate is None:
            return
        log(
            "OPENING",
            f"position candidate rejected reason={reason} side={active_candidate.active_side} "
            f"frames={active_candidate.frames_seen} best={active_candidate.best_side_avg:.2f}",
        )
        active_candidate = None

    def capture_candidate_side(scan: ScanSnapshot) -> str | None:
        left_open, _left_reason, _left_metrics = side_decision_diagnostics(scan, "left", args)
        right_open, _right_reason, _right_metrics = side_decision_diagnostics(scan, "right", args)
        if left_open and right_open:
            return "left" if scan.left_avg >= scan.right_avg else "right"
        if left_open:
            return "left"
        if right_open:
            return "right"
        return None

    async def update_candidate(step_index: int, phase: str) -> PositionOpeningAnchor | None:
        nonlocal active_candidate
        scan = monitor.decision_snapshot()
        decision = decide_corridor_action(scan, memory, args)
        log_scan_monitor_snapshot(scan)
        log("DECIDE", f"position step={step_index + 1}/{step_count} {phase} corridor_action={decision.value}")
        opening_side = opening_side_for_decision(decision)
        side_to_track = opening_side or (active_candidate.active_side if active_candidate is not None else None)
        if side_to_track is None:
            return None

        is_open, reason, metrics = side_decision_diagnostics(scan, side_to_track, args)
        if not is_open:
            reject_candidate("lost_opening")
            return None

        front_clear, front_distance = front_clearance_for_candidate(monitor, args)
        if args.opening_require_front_clear and not front_clear:
            reject_candidate("front_not_clear")
            return None

        current_position = await read_position_ned_quiet(drone)
        side_avg = side_avg_for_snapshot(scan, side_to_track)
        side_min = side_min_for_snapshot(scan, side_to_track)
        if active_candidate is None or active_candidate.active_side != side_to_track:
            if active_candidate is not None:
                reject_candidate("side_changed")
            active_candidate = OpeningCandidateState(
                active_side=side_to_track,
                start_step=step_index,
                start_position=current_position,
                best_position=current_position,
                best_side_avg=side_avg,
                frames_seen=1,
                last_seen_step=step_index,
                best_front_distance=front_distance,
            )
            log(
                "OPENING",
                f"position candidate started side={side_to_track} step={step_index + 1} "
                f"phase={phase} side_avg={side_avg:.2f} side_min={side_min:.2f} reason={reason}",
            )
            return None

        active_candidate.frames_seen += 1
        active_candidate.last_seen_step = step_index
        if side_avg > active_candidate.best_side_avg:
            active_candidate.best_side_avg = side_avg
            active_candidate.best_position = current_position
            active_candidate.best_front_distance = front_distance

        progress = horizontal_distance(current_position, active_candidate.start_position)
        progress_text = "unknown" if progress is None else f"{progress:.2f}"
        front_text = "unknown" if front_distance is None else f"{front_distance:.2f}"
        log(
            "OPENING",
            f"position candidate update side={side_to_track} phase={phase} side_avg={side_avg:.2f} "
            f"side_min={side_min:.2f} best={active_candidate.best_side_avg:.2f} "
            f"frames={active_candidate.frames_seen} progress={progress_text} front={front_text}",
        )

        min_frames = max(1, args.opening_min_persistence_frames)
        min_progress = max(0.0, args.opening_min_forward_progress)
        peak_drop = max(0.0, args.opening_peak_drop_distance)
        if active_candidate.frames_seen < min_frames:
            return None
        if progress is None or progress < min_progress:
            return None
        if side_avg < active_candidate.best_side_avg - peak_drop:
            reject_candidate("lost_peak")
            return None

        candidate = active_candidate
        side_for_inspection = candidate.active_side
        anchor_east = (
            float(candidate.start_position.east_m)
            if candidate.start_position is not None
            else current_east
        )
        anchor = PositionOpeningAnchor(
            side=side_for_inspection,
            start_step=candidate.start_step,
            mature_step=step_index,
            anchor_north=current_north,
            anchor_east=anchor_east,
            anchor_source="candidate_start",
            start_position=candidate.start_position,
            mature_position=current_position,
            best_position=candidate.best_position,
            best_side_avg=candidate.best_side_avg,
            frames_seen=candidate.frames_seen,
        )
        log(
            "OPENING",
            f"position candidate mature/confirmed side={side_for_inspection} "
            f"frames={candidate.frames_seen} progress={progress_text} best={candidate.best_side_avg:.2f} "
            f"anchor_east={anchor_east:.2f}",
        )
        active_candidate = None
        return anchor

    async def try_no_backtrack_capture(step_index: int, side: str) -> PositionOpeningAnchor | None:
        nonlocal current_east
        confirm_target = max(1, args.door_capture_confirm_frames)
        hold_sec_capture = max(0.0, args.door_capture_hold_seconds)
        crawl_step = max(0.0, args.door_capture_crawl_step)
        max_crawl_steps = max(0, args.door_capture_max_crawl_steps)
        confirmed_frames = 0
        best_position: object | None = None
        best_side_avg = -math.inf
        started_position = await read_position_ned_quiet(drone)

        log(
            "CAPTURE",
            f"started side={side} step={step_index + 1} "
            f"confirm_frames={confirm_target} crawl_step={crawl_step:.2f} max_crawl_steps={max_crawl_steps}",
        )

        for crawl_index in range(max_crawl_steps + 1):
            await goto_position_ned(
                drone,
                rclpy,
                monitor,
                position_type,
                current_north,
                current_east,
                down_m,
                yaw_deg,
                hold_sec_capture,
                f"capture hold {side} {crawl_index + 1}/{max_crawl_steps + 1}",
            )
            rclpy.spin_once(monitor.node, timeout_sec=0.05)
            scan = monitor.decision_snapshot()
            is_open, reason, metrics = side_decision_diagnostics(scan, side, args)
            front_clear, front_distance = front_clearance_for_candidate(monitor, args)
            side_avg = side_avg_for_snapshot(scan, side)
            side_min = side_min_for_snapshot(scan, side)
            current_position = await read_position_ned_quiet(drone)
            if side_avg > best_side_avg:
                best_side_avg = side_avg
                best_position = current_position
            front_text = "unknown" if front_distance is None else f"{front_distance:.2f}"
            log(
                "CAPTURE",
                f"confirm frame={confirmed_frames + 1}/{confirm_target} side={side} "
                f"open={is_open} reason={reason} side_avg={side_avg:.2f} side_min={side_min:.2f} front={front_text}",
            )
            log("CAPTURE", metrics)

            if args.opening_require_front_clear and not front_clear:
                log("CAPTURE", "rejected reason=front_not_clear")
                return None
            if is_open:
                confirmed_frames += 1
                if confirmed_frames >= confirm_target:
                    anchor_position = current_position
                    anchor_east = current_east
                    if anchor_position is not None:
                        anchor_east = float(anchor_position.east_m)
                    log(
                        "CAPTURE",
                        f"confirmed no-backtrack anchor side={side} "
                        f"N={current_north:.2f}, E={anchor_east:.2f}, frames={confirmed_frames}",
                    )
                    return PositionOpeningAnchor(
                        side=side,
                        start_step=step_index,
                        mature_step=step_index,
                        anchor_north=current_north,
                        anchor_east=anchor_east,
                        anchor_source="door_capture_current",
                        start_position=started_position,
                        mature_position=anchor_position,
                        best_position=best_position,
                        best_side_avg=best_side_avg,
                        frames_seen=confirmed_frames,
                    )
            else:
                log("CAPTURE", f"rejected reason={reason}")
                return None

            if crawl_index < max_crawl_steps and crawl_step > 0.0:
                current_east += crawl_step
                log("CAPTURE", f"crawl step={crawl_index + 1}/{max_crawl_steps}, next_east={current_east:.2f}")

        log("CAPTURE", "rejected reason=insufficient_confirmation")
        return None

    async def run_sensor_room_traversal(side: str, anchor_north: float, anchor_east: float) -> dict[str, Any]:
        nonlocal current_north, current_east
        stop_distance = max(0.0, args.room_traverse_stop_distance)
        step_distance = max(0.0, args.room_traverse_step_distance)
        max_distance = max(0.0, args.room_traverse_max_distance)
        hold_seconds = max(0.0, args.room_traverse_hold_seconds)
        direction = 1.0 if side == "left" else -1.0
        direction_scan = "left_decision_scan" if side == "left" else "right_decision_scan"
        traveled = 0.0
        step_index = 0
        stop_reason = "max_distance"
        final_side_min = math.inf
        final_side_avg = math.inf

        log(
            "ROOM",
            f"traversal enabled side={side} scan={direction_scan} "
            f"step={step_distance:.2f} max={max_distance:.2f} stop={stop_distance:.2f}",
        )

        while traveled < max_distance:
            if front_motion_status(monitor, args) == "blocked":
                stop_reason = "front_blocked"
                log("ROOM", "stop reason=front_blocked")
                break

            next_step = min(step_distance, max_distance - traveled)
            if next_step <= 0.0:
                stop_reason = "max_distance"
                break

            target_north = anchor_north + direction * (traveled + next_step)
            await goto_position_ned(
                drone,
                rclpy,
                monitor,
                position_type,
                target_north,
                anchor_east,
                down_m,
                yaw_deg,
                hold_seconds,
                f"room traverse {side} step {step_index + 1}",
            )
            current_north = target_north
            current_east = anchor_east
            traveled += next_step
            step_index += 1

            rclpy.spin_once(monitor.node, timeout_sec=0.05)
            scan = monitor.decision_snapshot()
            final_side_min = side_min_for_snapshot(scan, side)
            final_side_avg = side_avg_for_snapshot(scan, side)
            log(
                "ROOM",
                f"step={step_index} target_north={target_north:.2f} traveled={traveled:.2f} "
                f"side_min={format_distance(final_side_min)} side_avg={format_distance(final_side_avg)}",
            )
            if final_side_min <= stop_distance:
                stop_reason = "side_stop_distance"
                log("ROOM", f"stop reason=side_stop_distance side_min={final_side_min:.2f}")
                break

        if traveled >= max_distance and stop_reason == "max_distance":
            log("ROOM", f"stop reason=max_distance traveled={traveled:.2f}")

        return {
            "mode": "sensor_incremental",
            "stop_distance_m": round(stop_distance, 3),
            "step_distance_m": round(step_distance, 3),
            "max_distance_m": round(max_distance, 3),
            "actual_distance_m": round(traveled, 3),
            "stop_reason": stop_reason,
            "direction_scan": direction_scan,
            "final_side_min_m": None if math.isinf(final_side_min) else round(final_side_min, 3),
            "final_side_avg_m": None if math.isinf(final_side_avg) else round(final_side_avg, 3),
            "depth_estimate_m": round(traveled, 3),
            "width_estimate_m": None if math.isinf(final_side_avg) else round(final_side_avg, 3),
        }

    def room_front_min_distance() -> float | None:
        for msg in (monitor.latest_front, monitor.latest_front_decision):
            stats = front_sector_stats(msg, args.front_sector_deg)
            if stats.sample_count == 0:
                continue
            if stats.valid_count < max(1, args.min_valid_samples):
                continue
            if stats.valid_ratio < max(0.0, min(1.0, args.min_valid_ratio)):
                continue
            if stats.min_finite_distance is None:
                return math.inf
            return stats.min_finite_distance
        return None

    async def stabilize_room_facing_yaw(
        north_m: float,
        east_m: float,
        from_yaw: float,
        to_yaw: float,
        label: str,
    ) -> None:
        hold_before = max(0.0, args.room_facing_yaw_hold_before_seconds)
        hold_after = max(0.0, args.room_facing_yaw_hold_after_seconds)
        repeat_count = max(1, args.room_facing_yaw_settle_repeat_count)
        repeat_interval = max(0.0, args.room_facing_yaw_settle_repeat_interval)
        interpolation_step = max(0.0, args.room_facing_yaw_interpolation_step_deg)
        interpolation_hold = max(0.0, args.room_facing_yaw_interpolation_hold_seconds)
        from_yaw = normalize_yaw_deg(from_yaw)
        to_yaw = normalize_yaw_deg(to_yaw)
        yaw_targets = interpolated_yaw_targets(from_yaw, to_yaw, interpolation_step)
        log(
            "ROOM",
            f"yaw stabilize {label}: N={north_m:.2f}, E={east_m:.2f}, "
            f"from={from_yaw:.1f}, to={to_yaw:.1f}, "
            f"interp_step={interpolation_step:.1f}, targets={len(yaw_targets)}, "
            f"repeats={repeat_count}, interval={repeat_interval:.2f}s",
        )
        await goto_position_ned(
            drone,
            rclpy,
            monitor,
            position_type,
            north_m,
            east_m,
            down_m,
            from_yaw,
            hold_before,
            f"{label} yaw pre-hold",
        )
        for target_index, yaw_target in enumerate(yaw_targets):
            log(
                "ROOM",
                f"yaw interpolate {label} target {target_index + 1}/{len(yaw_targets)} "
                f"yaw={yaw_target:.1f}",
            )
            await drone.offboard.set_position_ned(position_type(north_m, east_m, down_m, yaw_target))
            rclpy.spin_once(monitor.node, timeout_sec=0.02)
            await asyncio.sleep(interpolation_hold)
        for repeat_index in range(repeat_count):
            log("ROOM", f"yaw stabilize {label} setpoint {repeat_index + 1}/{repeat_count}")
            await drone.offboard.set_position_ned(position_type(north_m, east_m, down_m, to_yaw))
            rclpy.spin_once(monitor.node, timeout_sec=0.02)
            await asyncio.sleep(repeat_interval)
        await goto_position_ned(
            drone,
            rclpy,
            monitor,
            position_type,
            north_m,
            east_m,
            down_m,
            to_yaw,
            hold_after,
            f"{label} yaw post-hold",
        )

    async def run_room_facing_yaw_entry(side: str, anchor_north: float, anchor_east: float) -> dict[str, Any]:
        nonlocal current_north, current_east
        step_distance = max(0.0, args.room_facing_step_distance)
        max_distance = max(0.0, args.room_facing_max_distance)
        stop_distance = max(0.0, args.room_facing_front_stop_distance)
        hold_seconds = max(0.0, args.room_facing_step_hold_seconds)
        door_forward_offset = max(0.0, args.room_facing_door_forward_offset)
        corridor_yaw = normalize_yaw_deg(yaw_deg)
        room_yaw = normalize_yaw_deg(corridor_yaw - 90.0 if side == "left" else corridor_yaw + 90.0)
        corridor_forward_north, corridor_forward_east = ned_forward_delta(corridor_yaw, 1.0)
        forward_north, forward_east = ned_forward_delta(room_yaw, 1.0)
        entry_anchor_north = anchor_north
        entry_anchor_east = anchor_east
        traveled = 0.0
        step_index = 0
        stop_reason = "max_distance_safety_limit"
        final_front_min: float | None = None
        post_yaw_total_offset = 0.0
        post_yaw_steps = 0
        post_yaw_front_min: float | None = None

        log(
            "ROOM",
            f"room-facing yaw entry side={side} room_yaw={room_yaw:.1f} "
            f"step={step_distance:.2f} max={max_distance:.2f} front_stop={stop_distance:.2f} "
            f"door_offset={door_forward_offset:.2f}",
        )

        if door_forward_offset > 0.0:
            entry_anchor_north = anchor_north + corridor_forward_north * door_forward_offset
            entry_anchor_east = anchor_east + corridor_forward_east * door_forward_offset
            await goto_position_ned(
                drone,
                rclpy,
                monitor,
                position_type,
                entry_anchor_north,
                entry_anchor_east,
                down_m,
                corridor_yaw,
                hold_seconds,
                f"room-facing door forward offset {side}",
            )
            current_north = entry_anchor_north
            current_east = entry_anchor_east

        rclpy.spin_once(monitor.node, timeout_sec=0.05)
        pre_scan = monitor.decision_snapshot()
        pre_front_min = room_front_min_distance()
        pre_front_clear, pre_front_distance = front_clearance_for_candidate(monitor, args)
        pre_side_open, pre_side_reason, pre_side_metrics = side_decision_diagnostics(pre_scan, side, args)
        pre_side_min = side_min_for_snapshot(pre_scan, side)
        pre_side_avg = side_avg_for_snapshot(pre_scan, side)
        pre_front_text = "unavailable" if pre_front_min is None else format_distance(pre_front_min)
        pre_front_clear_text = "unknown" if pre_front_distance is None else format_distance(pre_front_distance)
        log(
            "ROOM",
            f"pre-entry front_min={pre_front_text} corridor_clear={pre_front_clear} "
            f"corridor_front={pre_front_clear_text}",
        )
        log(
            "ROOM",
            f"pre-entry {side}_min={format_distance(pre_side_min)} "
            f"{side}_avg={format_distance(pre_side_avg)} open={pre_side_open} reason={pre_side_reason}",
        )
        log("ROOM", f"pre-entry metrics: {pre_side_metrics}")

        if pre_front_min is None or not pre_front_clear or not pre_side_open:
            stop_reason = "pre_entry_clearance_failed"
            log("ROOM", f"room-facing entry aborted reason={stop_reason}")
            return {
                "mode": "room_facing_yaw",
                "stop_distance_m": round(stop_distance, 3),
                "step_distance_m": round(step_distance, 3),
                "max_distance_m": round(max_distance, 3),
                "actual_distance_m": 0.0,
                "stop_reason": stop_reason,
                "direction_scan": "front_scan",
                "final_side_min_m": None,
                "final_side_avg_m": None,
                "depth_estimate_m": 0.0,
                "width_estimate_m": None,
                "room_facing_yaw_deg": round(room_yaw, 3),
                "room_facing_final_front_min_m": (
                    None if pre_front_min is None or math.isinf(pre_front_min) else round(pre_front_min, 3)
                ),
                "room_facing_exit_steps": None,
                "room_facing_exit_actual_distance_m": None,
                "room_facing_aborted": True,
                "room_facing_door_forward_offset_m": round(door_forward_offset, 3),
                "room_facing_entry_anchor_north": round(entry_anchor_north, 3),
                "room_facing_entry_anchor_east": round(entry_anchor_east, 3),
            }

        await stabilize_room_facing_yaw(
            entry_anchor_north,
            entry_anchor_east,
            corridor_yaw,
            room_yaw,
            f"turn toward {side} room",
        )
        current_north = entry_anchor_north
        current_east = entry_anchor_east

        if args.enable_room_facing_post_yaw_realign:
            min_post_yaw_clearance = max(0.0, args.room_facing_post_yaw_min_front_clearance)
            offset_step = max(0.0, args.room_facing_post_yaw_forward_offset_step)
            max_offset = max(0.0, args.room_facing_post_yaw_max_forward_offset)
            log(
                "ROOM",
                f"post-yaw realign enabled min_front={min_post_yaw_clearance:.2f} "
                f"step={offset_step:.2f} max_offset={max_offset:.2f}",
            )
            while True:
                rclpy.spin_once(monitor.node, timeout_sec=0.05)
                post_yaw_front_min = room_front_min_distance()
                front_text = "unavailable" if post_yaw_front_min is None else format_distance(post_yaw_front_min)
                log("ROOM", f"post-yaw front_min={front_text} offset={post_yaw_total_offset:.2f}")
                if post_yaw_front_min is not None and post_yaw_front_min >= min_post_yaw_clearance:
                    log("ROOM", "post-yaw clearance accepted")
                    break
                if offset_step <= 0.0 or post_yaw_total_offset + offset_step > max_offset + 1e-9:
                    stop_reason = "post_yaw_clearance_failed"
                    log("ROOM", f"post-yaw clearance rejected reason={stop_reason}")
                    return {
                        "mode": "room_facing_yaw",
                        "stop_distance_m": round(stop_distance, 3),
                        "step_distance_m": round(step_distance, 3),
                        "max_distance_m": round(max_distance, 3),
                        "actual_distance_m": 0.0,
                        "stop_reason": stop_reason,
                        "direction_scan": "front_scan",
                        "final_side_min_m": None,
                        "final_side_avg_m": None,
                        "depth_estimate_m": 0.0,
                        "width_estimate_m": None,
                        "room_facing_yaw_deg": round(room_yaw, 3),
                        "room_facing_final_front_min_m": (
                            None
                            if post_yaw_front_min is None or math.isinf(post_yaw_front_min)
                            else round(post_yaw_front_min, 3)
                        ),
                        "room_facing_exit_steps": None,
                        "room_facing_exit_actual_distance_m": None,
                        "room_facing_aborted": True,
                        "room_facing_door_forward_offset_m": round(door_forward_offset, 3),
                        "room_facing_entry_anchor_north": round(entry_anchor_north, 3),
                        "room_facing_entry_anchor_east": round(entry_anchor_east, 3),
                        "room_facing_post_yaw_realign_enabled": True,
                        "room_facing_post_yaw_total_offset_m": round(post_yaw_total_offset, 3),
                        "room_facing_post_yaw_front_min_m": (
                            None
                            if post_yaw_front_min is None or math.isinf(post_yaw_front_min)
                            else round(post_yaw_front_min, 3)
                        ),
                        "room_facing_post_yaw_realign_steps": post_yaw_steps,
                    }

                post_yaw_total_offset += offset_step
                post_yaw_steps += 1
                entry_anchor_north += corridor_forward_north * offset_step
                entry_anchor_east += corridor_forward_east * offset_step
                log(
                    "ROOM",
                    f"post-yaw realign offset step={post_yaw_steps} total={post_yaw_total_offset:.2f} "
                    f"target_N={entry_anchor_north:.2f} target_E={entry_anchor_east:.2f}",
                )
                await goto_position_ned(
                    drone,
                    rclpy,
                    monitor,
                    position_type,
                    entry_anchor_north,
                    entry_anchor_east,
                    down_m,
                    room_yaw,
                    hold_seconds,
                    f"post-yaw realign {side} step {post_yaw_steps}",
                )
                await stabilize_room_facing_yaw(
                    entry_anchor_north,
                    entry_anchor_east,
                    room_yaw,
                    room_yaw,
                    f"post-yaw realign hold {side}",
                )
                current_north = entry_anchor_north
                current_east = entry_anchor_east

        while traveled < max_distance:
            rclpy.spin_once(monitor.node, timeout_sec=0.05)
            front_min = room_front_min_distance()
            final_front_min = front_min
            if front_min is not None and front_min <= stop_distance:
                stop_reason = "front_stop_distance"
                log("ROOM", f"stop before step reason=front_stop_distance front_min={front_min:.2f}")
                break

            safe_step_distance = min(step_distance, 0.25) if traveled < 1.0 else step_distance
            next_step = min(safe_step_distance, max_distance - traveled)
            if next_step <= 0.0:
                break

            target_north = entry_anchor_north + forward_north * (traveled + next_step)
            target_east = entry_anchor_east + forward_east * (traveled + next_step)
            await goto_position_ned(
                drone,
                rclpy,
                monitor,
                position_type,
                target_north,
                target_east,
                down_m,
                room_yaw,
                hold_seconds,
                f"room-facing enter {side} step {step_index + 1}",
            )
            current_north = target_north
            current_east = target_east
            traveled += next_step
            step_index += 1

            rclpy.spin_once(monitor.node, timeout_sec=0.05)
            front_min = room_front_min_distance()
            final_front_min = front_min
            front_text = "unavailable" if front_min is None else format_distance(front_min)
            log(
                "ROOM",
                f"room-facing step={step_index} traveled={traveled:.2f} "
                f"front_min={front_text}",
            )
            if front_min is not None and front_min <= stop_distance:
                stop_reason = "front_stop_distance"
                log("ROOM", f"stop reason=front_stop_distance front_min={front_min:.2f}")
                break

        if traveled >= max_distance and stop_reason == "max_distance_safety_limit":
            log("ROOM", f"stop reason=max_distance_safety_limit traveled={traveled:.2f}")

        final_front = None if final_front_min is None or math.isinf(final_front_min) else round(final_front_min, 3)
        return {
            "mode": "room_facing_yaw",
            "stop_distance_m": round(stop_distance, 3),
            "step_distance_m": round(step_distance, 3),
            "max_distance_m": round(max_distance, 3),
            "actual_distance_m": round(traveled, 3),
            "stop_reason": stop_reason,
            "direction_scan": "front_scan",
            "final_side_min_m": None,
            "final_side_avg_m": None,
            "depth_estimate_m": round(traveled, 3),
            "width_estimate_m": None,
            "room_facing_yaw_deg": round(room_yaw, 3),
            "room_facing_final_front_min_m": final_front,
            "room_facing_exit_steps": None,
            "room_facing_exit_actual_distance_m": None,
            "room_facing_aborted": False,
            "room_facing_door_forward_offset_m": round(door_forward_offset, 3),
            "room_facing_entry_anchor_north": round(entry_anchor_north, 3),
            "room_facing_entry_anchor_east": round(entry_anchor_east, 3),
            "room_facing_post_yaw_realign_enabled": bool(args.enable_room_facing_post_yaw_realign),
            "room_facing_post_yaw_total_offset_m": round(post_yaw_total_offset, 3),
            "room_facing_post_yaw_front_min_m": (
                None
                if post_yaw_front_min is None or math.isinf(post_yaw_front_min)
                else round(post_yaw_front_min, 3)
            ),
            "room_facing_post_yaw_realign_steps": post_yaw_steps,
        }

    async def exit_room_facing_yaw(
        side: str,
        anchor_north: float,
        anchor_east: float,
        room_yaw: float,
        traveled: float,
    ) -> dict[str, Any]:
        nonlocal current_north, current_east
        exit_step_distance = max(0.01, args.room_facing_exit_step_distance)
        hold_seconds = max(0.0, args.room_facing_step_hold_seconds)
        corridor_yaw = normalize_yaw_deg(yaw_deg)
        forward_north, forward_east = ned_forward_delta(room_yaw, 1.0)
        exit_steps = 0
        remaining = max(0.0, traveled)

        log("ROOM", f"room-facing exit to anchor, distance={traveled:.2f}, step={exit_step_distance:.2f}")
        while remaining > 1e-6:
            safe_exit_step = min(exit_step_distance, 0.25) if remaining <= 1.0 else exit_step_distance
            next_exit = min(safe_exit_step, remaining)
            remaining -= next_exit
            target_north = anchor_north + forward_north * remaining
            target_east = anchor_east + forward_east * remaining
            exit_steps += 1
            await goto_position_ned(
                drone,
                rclpy,
                monitor,
                position_type,
                target_north,
                target_east,
                down_m,
                room_yaw,
                hold_seconds,
                f"room-facing exit {side} step {exit_steps}",
            )
            current_north = target_north
            current_east = target_east

        await stabilize_room_facing_yaw(
            anchor_north,
            anchor_east,
            room_yaw,
            corridor_yaw,
            "turn back to corridor yaw",
        )
        current_north = anchor_north
        current_east = anchor_east
        return {
            "exit_steps": exit_steps,
            "exit_actual_distance_m": round(traveled, 3),
            "exit_final_distance_m": 0.0,
        }

    async def inspect_position_opening(anchor: PositionOpeningAnchor) -> None:
        nonlocal current_north, current_east
        if len(events) >= max_inspections:
            return

        side = anchor.side
        anchor_north = anchor.anchor_north
        anchor_east = anchor.anchor_east
        if anchor.anchor_source == "door_capture_current":
            log(
                "POS",
                f"using no-backtrack {side} capture anchor before entry: "
                f"N={anchor_north:.2f}, E={anchor_east:.2f}, step={anchor.mature_step + 1}",
            )
            current_north = anchor_north
            current_east = anchor_east
        else:
            log(
                "POS",
                f"aligning to {side} candidate anchor before entry: "
                f"N={anchor_north:.2f}, E={anchor_east:.2f}, start_step={anchor.start_step + 1}, mature_step={anchor.mature_step + 1}",
            )
            await goto_position_ned(
                drone,
                rclpy,
                monitor,
                position_type,
                anchor_north,
                anchor_east,
                down_m,
                yaw_deg,
                hold_sec,
                f"align {side} opening anchor",
            )
            current_north = anchor_north
            current_east = anchor_east
        opening_id = f"{side}@north={anchor_north:.1f},east={anchor_east:.1f}"
        baseline = await sample_gas_at_position(
            drone,
            rclpy,
            monitor,
            position_type,
            args,
            "baseline",
            args.baseline_sample_seconds,
            anchor_north,
            anchor_east,
            down_m,
            yaw_deg,
            compute_ppm,
            active_sources,
            rng,
        )

        if args.enable_room_facing_yaw_entry:
            traversal = await run_room_facing_yaw_entry(side, anchor_north, anchor_east)
            entry_north = current_north
        elif args.enable_sensor_room_traversal:
            traversal = await run_sensor_room_traversal(side, anchor_north, anchor_east)
            entry_north = current_north
        else:
            entry_north = anchor_north + room_entry_distance if side == "left" else anchor_north - room_entry_distance
            log("POS", f"entering {side} room by position step: target_north={entry_north:.2f}, east={anchor_east:.2f}")
            await goto_position_ned(
                drone,
                rclpy,
                monitor,
                position_type,
                entry_north,
                anchor_east,
                down_m,
                yaw_deg,
                room_entry_hold_sec,
                f"enter {side} opening",
            )
            current_north = entry_north
            current_east = anchor_east
            traversal = {
                "mode": "fixed_distance",
                "stop_distance_m": None,
                "step_distance_m": None,
                "max_distance_m": round(room_entry_distance, 3),
                "actual_distance_m": round(abs(entry_north - anchor_north), 3),
                "stop_reason": "target_distance",
                "direction_scan": "left_decision_scan" if side == "left" else "right_decision_scan",
                "final_side_min_m": None,
                "final_side_avg_m": None,
                "depth_estimate_m": round(abs(entry_north - anchor_north), 3),
                "width_estimate_m": None,
            }

        if traversal.get("room_facing_aborted"):
            event = build_position_abort_event(
                inspection_index=len(events) + 1,
                side=side,
                opening_id=opening_id,
                anchor=anchor,
                anchor_north=anchor_north,
                anchor_east=anchor_east,
                down_m=down_m,
                room_entry_distance=room_entry_distance,
                baseline=baseline,
                traversal=traversal,
                args=args,
            )
            events.append(event)
            write_inspection_events(args.inspection_events_output, payload)
            log("ROOM", "room-facing inspection aborted before entry; continuing corridor flow")
            return

        inspection_yaw = float(traversal.get("room_facing_yaw_deg", yaw_deg) or yaw_deg)
        inspection = await sample_gas_at_position(
            drone,
            rclpy,
            monitor,
            position_type,
            args,
            "inspection",
            args.inspection_sample_seconds,
            current_north,
            current_east,
            down_m,
            inspection_yaw,
            compute_ppm,
            active_sources,
            rng,
        )
        delta_ppm = inspection.avg_ppm - baseline.avg_ppm
        candidate_by_delta = delta_ppm >= max(0.0, args.gas_delta_threshold)
        candidate_by_absolute = inspection.avg_ppm >= max(0.0, args.gas_absolute_threshold)
        gas_candidate = candidate_by_delta or candidate_by_absolute
        reason = "delta_ppm" if candidate_by_delta else "absolute_ppm" if candidate_by_absolute else "below_threshold"
        log(
            "GAS",
            "position gas candidate decision: "
            f"side={side}, baseline={baseline.avg_ppm:.2f}, inspection={inspection.avg_ppm:.2f}, "
            f"delta={delta_ppm:.2f}, candidate={gas_candidate}, reason={reason}",
        )

        event = build_position_inspection_event(
            inspection_index=len(events) + 1,
            side=side,
            opening_id=opening_id,
            anchor=anchor,
            anchor_north=anchor_north,
            anchor_east=anchor_east,
            down_m=down_m,
            room_entry_distance=room_entry_distance,
            entry_north=entry_north,
            baseline=baseline,
            inspection=inspection,
            delta_ppm=delta_ppm,
            gas_candidate=gas_candidate,
            candidate_reason=reason,
            traversal=traversal,
            args=args,
        )
        events.append(event)
        write_inspection_events(args.inspection_events_output, payload)

        if traversal["mode"] == "room_facing_yaw":
            exit_summary = await exit_room_facing_yaw(
                side,
                float(traversal.get("room_facing_entry_anchor_north", anchor_north)),
                float(traversal.get("room_facing_entry_anchor_east", anchor_east)),
                float(traversal["room_facing_yaw_deg"]),
                float(traversal["actual_distance_m"]),
            )
            event["room_facing_exit_steps"] = exit_summary["exit_steps"]
            event["room_facing_exit_actual_distance_m"] = exit_summary["exit_actual_distance_m"]
            event["exit_final_distance_m"] = exit_summary["exit_final_distance_m"]
        else:
            log("POS", f"exiting {side} room back to corridor anchor")
            await goto_position_ned(
                drone,
                rclpy,
                monitor,
                position_type,
                anchor_north,
                anchor_east,
                down_m,
                yaw_deg,
                max(2.0, room_entry_hold_sec),
                f"exit {side} opening",
            )
            current_north = anchor_north
            current_east = anchor_east
            event["exit_final_distance_m"] = 0.0
        write_inspection_events(args.inspection_events_output, payload)

    await goto_position_ned(drone, rclpy, monitor, position_type, current_north, current_east, down_m, yaw_deg, 5.0, "initial hover")
    for step_index in range(step_count):
        rclpy.spin_once(monitor.node, timeout_sec=0.05)
        if front_motion_status(monitor, args) == "blocked":
            log("SAFETY", "position mode front blocked before next corridor step")
            break

        current_east += forward_step
        await goto_position_ned(
            drone,
            rclpy,
            monitor,
            position_type,
            current_north,
            current_east,
            down_m,
            yaw_deg,
            hold_sec,
            f"corridor step {step_index + 1}/{step_count}",
        )
        if args.enable_no_backtrack_door_capture and len(events) < max_inspections:
            scan = monitor.decision_snapshot()
            opening_side = capture_candidate_side(scan)
            if opening_side is not None:
                log(
                    "CAPTURE",
                    f"candidate side={opening_side} detected after corridor step {step_index + 1}; "
                    "pausing normal stepping for door capture",
                )
                anchor = await try_no_backtrack_capture(step_index, opening_side)
                active_candidate = None
                if anchor is not None:
                    await inspect_position_opening(anchor)
                if len(events) >= max_inspections:
                    log("POS", "max inspections reached; remaining steps only follow position corridor")
                continue

        anchor = await update_candidate(step_index, "post-step")
        if anchor is not None and len(events) < max_inspections:
            await inspect_position_opening(anchor)
        if len(events) >= max_inspections:
            log("POS", "max inspections reached; remaining steps only follow position corridor")

    write_inspection_events(args.inspection_events_output, payload)
    log("POS", f"position inspection events written: {args.inspection_events_output}")
    return payload


async def run_position_room_inspection_check(args: argparse.Namespace) -> int:
    system_type = import_mavsdk_system()
    position_type = import_position_ned_yaw()
    gas_model = import_gas_model()
    if system_type is None or position_type is None or gas_model is None:
        return 1
    compute_ppm, resolve_scenario, possible_gas_zones = gas_model

    monitor_bundle = create_scan_monitor(args)
    if monitor_bundle is None:
        return 1
    rclpy, monitor = monitor_bundle

    rng = random.Random(args.gas_seed)
    try:
        scenario, active_sources = resolve_scenario(args.gas_scenario, rng)
    except Exception as exc:
        print(f"Could not resolve gas scenario {args.gas_scenario!r}: {exc}")
        close_scan_monitor(rclpy, monitor)
        return 2

    drone = system_type()
    land_requested = False
    offboard_started = False
    current_north = 0.0
    current_east = 0.0
    down_m = -max(0.1, args.position_altitude)
    yaw_deg = float(args.position_yaw)

    log("CONFIG", "opening-based position room inspection check")
    log("CONFIG", f"system_address={args.system_address}")
    log("CONFIG", f"position_step_count={max(0, args.position_step_count)}")
    log("CONFIG", f"position_forward_step={max(0.0, args.position_forward_step):.2f} m")
    log("CONFIG", f"position_altitude={-down_m:.2f} m")
    log("CONFIG", f"position_yaw={yaw_deg:.1f} deg")
    log("CONFIG", f"position_room_entry_distance={max(0.0, args.position_room_entry_distance):.2f} m")
    log("GAS", f"gas_scenario={args.gas_scenario}, resolved={scenario}, gas_seed={args.gas_seed}")

    try:
        scan_ready = await wait_for_scan_readiness(rclpy, monitor, args)
        if not scan_ready:
            log("FINISH", "aborting before MAVSDK connect because scan readiness failed")
            return 1

        log("CONNECT", f"connecting to MAVSDK system at {args.system_address}")
        await asyncio.wait_for(drone.connect(system_address=args.system_address), timeout=args.connection_timeout)
        await wait_for_mavsdk_connection(drone, args.connection_timeout)
        await wait_for_mavsdk_health(drone, args.connection_timeout)

        log("POS", "sending initial position setpoint before arming")
        await drone.offboard.set_position_ned(position_type(current_north, current_east, down_m, yaw_deg))
        log("TAKEOFF", "arming")
        await drone.action.arm()
        log("POS", "starting offboard position mode")
        await drone.offboard.start()
        offboard_started = True

        await run_position_room_inspection_steps(
            drone,
            rclpy,
            monitor,
            args,
            position_type,
            compute_ppm,
            active_sources,
            scenario,
            possible_gas_zones,
        )
        final_position = await read_position_ned_quiet(drone)
        if final_position is not None:
            current_north = float(final_position.north_m)
            current_east = float(final_position.east_m)
        await soft_land_position_mode(drone, rclpy, monitor, position_type, current_north, current_east, yaw_deg)
        log("POS", "stopping offboard mode")
        await drone.offboard.stop()
        offboard_started = False
        land_requested = True
        log("LAND", "land command sent")
        await drone.action.land()
        log("FINISH", "position room inspection check complete")
        return 0
    except asyncio.TimeoutError:
        log("FINISH", "timed out while waiting for PX4 connection or vehicle health")
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception as exc:
                log("POS", f"offboard stop during timeout cleanup failed: {exc}")
        if not land_requested:
            await try_safety_land(drone)
        return 1
    except KeyboardInterrupt:
        log("FINISH", "interrupted by user")
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception as exc:
                log("POS", f"offboard stop during interrupt cleanup failed: {exc}")
        if not land_requested:
            await try_safety_land(drone)
        return 130
    except Exception as exc:
        log("FINISH", f"position room inspection failed before land command: {exc}")
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception as stop_exc:
                log("POS", f"offboard stop during error cleanup failed: {stop_exc}")
        if not land_requested:
            await try_safety_land(drone)
        return 1
    finally:
        close_scan_monitor(rclpy, monitor)


async def run_position_side_sign_check(args: argparse.Namespace) -> int:
    system_type = import_mavsdk_system()
    position_type = import_position_ned_yaw()
    if system_type is None or position_type is None:
        return 1

    monitor_bundle = create_scan_monitor(args)
    if monitor_bundle is None:
        return 1
    rclpy, monitor = monitor_bundle

    drone = system_type()
    land_requested = False
    offboard_started = False
    center_north = 0.0
    center_east = 0.0
    down_m = -max(0.1, args.position_altitude)
    yaw_deg = float(args.position_yaw)
    lateral_distance = max(0.0, args.position_room_entry_distance)
    hold_sec = max(0.0, args.position_room_entry_hold_seconds)

    log("CONFIG", "position side sign diagnostic check")
    log("CONFIG", f"system_address={args.system_address}")
    log("CONFIG", f"position_altitude={-down_m:.2f} m")
    log("CONFIG", f"position_yaw={yaw_deg:.1f} deg")
    log("CONFIG", f"lateral_test_distance={lateral_distance:.2f} m")
    log("CONFIG", f"hold_seconds={hold_sec:.1f}")

    try:
        scan_ready = await wait_for_scan_readiness(rclpy, monitor, args)
        if not scan_ready:
            log("FINISH", "aborting before MAVSDK connect because scan readiness failed")
            return 1

        log("CONNECT", f"connecting to MAVSDK system at {args.system_address}")
        await asyncio.wait_for(drone.connect(system_address=args.system_address), timeout=args.connection_timeout)
        await wait_for_mavsdk_connection(drone, args.connection_timeout)
        await wait_for_mavsdk_health(drone, args.connection_timeout)

        log("POS", "sending initial position setpoint before arming")
        await drone.offboard.set_position_ned(position_type(center_north, center_east, down_m, yaw_deg))
        log("TAKEOFF", "arming")
        await drone.action.arm()
        log("POS", "starting offboard position mode")
        await drone.offboard.start()
        offboard_started = True

        await goto_position_ned(
            drone,
            rclpy,
            monitor,
            position_type,
            center_north,
            center_east,
            down_m,
            yaw_deg,
            4.0,
            "center hover",
        )
        log_position_decision_scan(monitor, args, "center hover")

        test_points = (
            ("test_left", center_north + lateral_distance, center_east),
            ("return_center_after_left", center_north, center_east),
            ("test_right", center_north - lateral_distance, center_east),
            ("return_center_after_right", center_north, center_east),
        )
        for label, target_north, target_east in test_points:
            await goto_position_ned(
                drone,
                rclpy,
                monitor,
                position_type,
                target_north,
                target_east,
                down_m,
                yaw_deg,
                hold_sec,
                label,
            )
            log_position_decision_scan(monitor, args, label)

        final_position = await read_position_ned_quiet(drone)
        land_north = center_north if final_position is None else float(final_position.north_m)
        land_east = center_east if final_position is None else float(final_position.east_m)
        await soft_land_position_mode(drone, rclpy, monitor, position_type, land_north, land_east, yaw_deg)
        log("POS", "stopping offboard mode")
        await drone.offboard.stop()
        offboard_started = False
        land_requested = True
        log("LAND", "land command sent")
        await drone.action.land()
        log("FINISH", "position side sign diagnostic complete")
        return 0
    except asyncio.TimeoutError:
        log("FINISH", "timed out while waiting for PX4 connection or vehicle health")
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception as exc:
                log("POS", f"offboard stop during timeout cleanup failed: {exc}")
        if not land_requested:
            await try_safety_land(drone)
        return 1
    except KeyboardInterrupt:
        log("FINISH", "interrupted by user")
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception as exc:
                log("POS", f"offboard stop during interrupt cleanup failed: {exc}")
        if not land_requested:
            await try_safety_land(drone)
        return 130
    except Exception as exc:
        log("FINISH", f"position side sign diagnostic failed before land command: {exc}")
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception as stop_exc:
                log("POS", f"offboard stop during error cleanup failed: {stop_exc}")
        if not land_requested:
            await try_safety_land(drone)
        return 1
    finally:
        close_scan_monitor(rclpy, monitor)


async def run_axis_calibration_check(args: argparse.Namespace) -> int:
    system_type = import_mavsdk_system()
    ned_velocity_type = import_velocity_ned_yaw()
    body_velocity_type = import_velocity_body_yawspeed()
    if system_type is None or ned_velocity_type is None or body_velocity_type is None:
        return 1

    monitor_bundle = create_scan_monitor(args)
    if monitor_bundle is None:
        return 1
    rclpy, monitor = monitor_bundle

    drone = system_type()
    land_requested = False
    offboard_started = False

    log("CONFIG", "opening-based axis calibration check")
    log("CONFIG", f"system_address={args.system_address}")
    log("CONFIG", f"takeoff_altitude={args.takeoff_altitude:.1f} m")
    log("CONFIG", f"takeoff_altitude_tolerance={args.takeoff_altitude_tolerance:.2f} m")
    log("CONFIG", f"min_takeoff_confirm_altitude={args.min_takeoff_confirm_altitude:.2f} m")
    log("CONFIG", f"post_takeoff_settle_seconds={max(0.0, args.post_takeoff_settle_seconds):.1f}")
    log("CONFIG", f"offboard_zero_hover_seconds={max(0.0, args.offboard_zero_hover_seconds):.1f}")
    log("CONFIG", f"axis_calibration_speed={max(0.0, args.axis_calibration_speed):.2f} m/s")
    log("CONFIG", f"axis_calibration_duration={max(0.0, args.axis_calibration_duration):.1f}")
    log("CONFIG", f"body_forward_speed={args.body_forward_speed:.2f} m/s")
    log("CONFIG", f"offboard_warmup_seconds={max(0.0, args.offboard_warmup_seconds):.1f}")
    log("CONFIG", f"move_rate_hz={max(1.0, args.move_rate_hz):.1f}")
    log("CONFIG", f"front_sector_deg={args.front_sector_deg:.1f}")
    log("CONFIG", f"front_scan_topic={args.front_scan_topic}")
    log("CONFIG", "this mode diagnoses axes only; it does not perform corridor following")

    try:
        scan_ready = await wait_for_scan_readiness(rclpy, monitor, args)
        if not scan_ready:
            log("FINISH", "aborting before MAVSDK connect because scan readiness failed")
            return 1

        log("CONNECT", f"connecting to MAVSDK system at {args.system_address}")
        await asyncio.wait_for(drone.connect(system_address=args.system_address), timeout=args.connection_timeout)
        await wait_for_mavsdk_connection(drone, args.connection_timeout)
        await wait_for_mavsdk_health(drone, args.connection_timeout)

        log("TAKEOFF", f"setting takeoff altitude to {args.takeoff_altitude:.1f} m")
        await drone.action.set_takeoff_altitude(max(0.1, args.takeoff_altitude))
        log("TAKEOFF", "arming")
        await drone.action.arm()
        log("TAKEOFF", "takeoff command sent")
        await drone.action.takeoff()

        altitude_result = await wait_until_takeoff_altitude(
            drone,
            target_altitude_m=args.takeoff_altitude,
            timeout_sec=args.takeoff_timeout,
            tolerance_m=args.takeoff_altitude_tolerance,
            min_confirm_altitude_m=args.min_takeoff_confirm_altitude,
        )
        should_calibrate = altitude_result.confirmed or altitude_result.safe_hover_altitude
        if altitude_result.confirmed:
            log("TAKEOFF", f"takeoff altitude confirmed at {altitude_result.last_altitude_m:.2f} m")
        elif altitude_result.safe_hover_altitude:
            log(
                "TAKEOFF",
                "target altitude was not fully confirmed, "
                f"but safe hover altitude was reached at {altitude_result.last_altitude_m:.2f} m",
            )
        else:
            log(
                "TAKEOFF",
                "takeoff altitude was not confirmed and safe hover altitude was not reached; "
                f"last_altitude={altitude_result.last_altitude_m:.2f} m",
            )

        if should_calibrate:
            settle_seconds = max(0.0, args.post_takeoff_settle_seconds)
            if settle_seconds > 0.0:
                log("TAKEOFF", f"settling after takeoff for {settle_seconds:.1f}s")
                settle_started_at = time.monotonic()
                while time.monotonic() - settle_started_at < settle_seconds:
                    rclpy.spin_once(monitor.node, timeout_sec=0.02)
                    await asyncio.sleep(0.05)

            yaw_deg = await read_initial_yaw_deg(drone)
            log("CALIBRATE", f"initial_yaw={yaw_deg:.1f} deg")
            offboard_started = True
            await run_axis_calibration_steps(
                drone,
                rclpy,
                monitor,
                args,
                ned_velocity_type,
                body_velocity_type,
                yaw_deg,
            )
            offboard_started = False
        else:
            log("CALIBRATE", "skipping axis calibration and landing")

        pre_land_settle_seconds = max(0.0, args.pre_land_settle_seconds)
        if pre_land_settle_seconds > 0.0:
            log("LAND", f"settling before land for {pre_land_settle_seconds:.1f}s")
            settle_started_at = time.monotonic()
            while time.monotonic() - settle_started_at < pre_land_settle_seconds:
                rclpy.spin_once(monitor.node, timeout_sec=0.02)
                await asyncio.sleep(0.05)

        land_requested = True
        log("LAND", "land command sent")
        await drone.action.land()
        log("FINISH", "axis calibration check complete")
        return 0
    except asyncio.TimeoutError:
        log("FINISH", "timed out while waiting for PX4 connection or vehicle health")
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception as exc:
                log("CALIBRATE", f"offboard stop during timeout cleanup failed: {exc}")
        if not land_requested:
            await try_safety_land(drone)
        return 1
    except KeyboardInterrupt:
        log("FINISH", "interrupted by user")
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception as exc:
                log("CALIBRATE", f"offboard stop during interrupt cleanup failed: {exc}")
        if not land_requested:
            await try_safety_land(drone)
        return 130
    except Exception as exc:
        if land_requested:
            if is_grpc_disconnect_error(exc):
                log("LAND", f"connection closed after land command; exiting calmly: {exc}")
                return 0
            log("LAND", f"land command was already sent; not retrying safety land: {exc}")
            return 1

        log("FINISH", f"axis calibration failed before land command: {exc}")
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception as stop_exc:
                log("CALIBRATE", f"offboard stop during error cleanup failed: {stop_exc}")
        await try_safety_land(drone)
        return 1
    finally:
        close_scan_monitor(rclpy, monitor)


def candidate_near_position(candidate: OpeningCandidate, corridor_x: float, step_size: float) -> bool:
    return abs(candidate.corridor_x - corridor_x) <= max(0.25, step_size / 2.0)


def decide_probe(candidate: OpeningCandidate, rng: random.Random, verbose: bool) -> tuple[bool, float]:
    jitter = rng.random()
    decision_score = 0.65 * candidate.opening_score + 0.35 * jitter
    should_probe = decision_score >= 0.55
    if verbose:
        log(
            "DECIDE",
            f"score details: base={candidate.opening_score:.2f} jitter={jitter:.3f} total={decision_score:.3f}",
        )
    return should_probe, decision_score


def run_dry_run(args: argparse.Namespace) -> int:
    seed = args.seed if args.seed is not None else random.SystemRandom().randint(1, 999999)
    rng = random.Random(seed)
    seed_text = str(seed)
    summary = DryRunSummary()
    detected_names: set[str] = set()

    max_openings = max(0, args.max_openings)
    max_corridor_x = max(0.0, args.max_corridor_x)
    step_size = max(0.1, args.corridor_step)

    config_summary(args, seed_text)

    if args.dry_run_scenario is not None:
        return run_dry_run_scenario(args, seed)

    corridor_x = 0.0
    while corridor_x <= max_corridor_x and summary.probed < max_openings:
        summary.corridor_steps += 1
        log("STATE", f"{MissionState.FOLLOW_CORRIDOR.value}: x={corridor_x:.1f}")

        for candidate in OPENING_CANDIDATES:
            if candidate.name in detected_names:
                continue
            if not candidate_near_position(candidate, corridor_x, step_size):
                continue

            detected_names.add(candidate.name)
            summary.detected += 1
            log(
                "DETECT",
                f"candidate opening: name={candidate.name}, side={candidate.side}, corridor_x={candidate.corridor_x:.1f}",
            )

            should_probe, decision_score = decide_probe(candidate, rng, args.verbose)
            if should_probe and summary.probed < max_openings:
                summary.probed += 1
                log(
                    "DECIDE",
                    f"probe: name={candidate.name}, side={candidate.side}, score={decision_score:.3f}",
                )
                log("PROBE", f"simulated: entering {candidate.side} opening near x={candidate.corridor_x:.1f}")
                log("RETURN", f"simulated: returning to corridor anchor x={candidate.corridor_x:.1f}")
            else:
                summary.skipped += 1
                log(
                    "DECIDE",
                    f"skip: name={candidate.name}, side={candidate.side}, score={decision_score:.3f}",
                )

        corridor_x += step_size

    if args.return_home:
        log("RETURN", "simulated: returning to START / SAFE_EXIT")

    log(
        "FINISH",
        "summary: "
        f"corridor_steps={summary.corridor_steps}, "
        f"detected={summary.detected}, "
        f"probed={summary.probed}, "
        f"skipped={summary.skipped}, "
        f"seed={seed_text}",
    )
    return 0


def main() -> int:
    args = parse_args()
    if args.scan_monitor:
        return run_scan_monitor(args)
    if args.takeoff_land_check:
        return asyncio.run(run_takeoff_land_check(args))
    if args.corridor_follow_check:
        return asyncio.run(run_corridor_follow_check(args))
    if args.room_inspection_check:
        return asyncio.run(run_room_inspection_check(args))
    if args.position_room_inspection_check:
        return asyncio.run(run_position_room_inspection_check(args))
    if args.position_side_sign_check:
        return asyncio.run(run_position_side_sign_check(args))
    if args.axis_calibration_check:
        return asyncio.run(run_axis_calibration_check(args))
    if not args.dry_run:
        print(
            "Use --dry-run for pure Python simulation, --scan-monitor for ROS2 LaserScan monitoring, "
            "--takeoff-land-check for MAVSDK takeoff/land validation, "
            "--corridor-follow-check for low-speed front-safe corridor movement, "
            "--room-inspection-check for opening inspection with simulated gas candidate logging, "
            "--position-room-inspection-check for position-setpoint opening inspection, "
            "--position-side-sign-check for position left/right sign diagnosis, "
            "or --axis-calibration-check for offboard axis diagnosis."
        )
        return 2
    return run_dry_run(args)


if __name__ == "__main__":
    raise SystemExit(main())
