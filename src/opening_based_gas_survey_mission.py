#!/usr/bin/env python3
"""Opening-based gas survey mission prototype.

Dry-run and scan-monitor modes never command a drone. MAVSDK modes import
dependencies lazily and verify scan readiness before any flight command.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import random
import time
from dataclasses import dataclass
from enum import Enum

DEFAULT_SYSTEM_ADDRESS = "udpin://0.0.0.0:14540"


class MissionState(str, Enum):
    FOLLOW_CORRIDOR = "FOLLOW_CORRIDOR"
    DETECT_OPENING = "DETECT_OPENING"
    DECIDE_OPENING = "DECIDE_OPENING"
    PROBE_OPENING = "PROBE_OPENING"
    RETURN_TO_CORRIDOR = "RETURN_TO_CORRIDOR"
    FINISH = "FINISH"


class Decision(str, Enum):
    FOLLOW_FORWARD = "FOLLOW_FORWARD"
    DETECT_LEFT_OPENING = "DETECT_LEFT_OPENING"
    DETECT_RIGHT_OPENING = "DETECT_RIGHT_OPENING"
    BYPASS_LEFT = "BYPASS_LEFT"
    BYPASS_RIGHT = "BYPASS_RIGHT"
    BLOCKED = "BLOCKED"
    NARROW_FORWARD = "NARROW_FORWARD"
    PROBE_OPENING = "PROBE_OPENING"
    SKIP_OPENING = "SKIP_OPENING"


@dataclass(frozen=True)
class ScanStats:
    min_distance: float
    avg_distance: float
    sample_count: int
    valid_count: int
    finite_count: int
    inf_count: int
    valid_ratio: float
    ready: bool


@dataclass(frozen=True)
class ScanSnapshot:
    front_min: float
    left_min: float
    right_min: float
    left_avg: float
    right_avg: float
    front_ready: bool = True
    left_ready: bool = True
    right_ready: bool = True
    front_valid_count: int = 15
    left_valid_count: int = 15
    right_valid_count: int = 15
    front_finite_count: int = 15
    left_finite_count: int = 15
    right_finite_count: int = 15
    front_inf_count: int = 0
    left_inf_count: int = 0
    right_inf_count: int = 0
    front_valid_ratio: float = 1.0
    left_valid_ratio: float = 1.0
    right_valid_ratio: float = 1.0


@dataclass(frozen=True)
class OpeningCandidate:
    name: str
    corridor_x: float
    side: str
    opening_score: float


@dataclass
class DryRunSummary:
    detected: int = 0
    probed: int = 0
    skipped: int = 0
    corridor_steps: int = 0


@dataclass
class MissionMemory:
    visited_openings: set[str]
    skipped_openings: set[str]
    bypass_attempts: int
    corridor_x: float
    seed: int
    left_open_frames: int = 0
    right_open_frames: int = 0


@dataclass(frozen=True)
class TakeoffAltitudeResult:
    confirmed: bool
    safe_hover_altitude: bool
    last_altitude_m: float


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
    parser.add_argument("--min-valid-samples", type=int, default=5)
    parser.add_argument("--min-valid-ratio", type=float, default=0.35)
    parser.add_argument("--opening-confirm-frames", type=int, default=2)
    parser.add_argument("--front-scan-topic", default="/drone/front_scan")
    parser.add_argument("--left-scan-topic", default="/drone/left_scan")
    parser.add_argument("--right-scan-topic", default="/drone/right_scan")
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
    parser.add_argument("--move-rate-hz", type=float, default=10.0)
    parser.add_argument("--pause-between-steps", type=float, default=0.75)
    parser.add_argument("--front-sector-deg", type=float, default=35.0)
    parser.add_argument("--axis-calibration-speed", type=float, default=0.15)
    parser.add_argument("--axis-calibration-duration", type=float, default=1.5)
    parser.add_argument("--offboard-zero-hover-seconds", type=float, default=2.0)
    parser.add_argument("--offboard-warmup-seconds", type=float, default=1.5)
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


def build_scan_snapshot(
    front_min: float,
    left_min: float,
    right_min: float,
    left_avg: float,
    right_avg: float,
    *,
    front_ready: bool = True,
    left_ready: bool = True,
    right_ready: bool = True,
    front_valid_count: int = 15,
    left_valid_count: int = 15,
    right_valid_count: int = 15,
    front_finite_count: int = 15,
    left_finite_count: int = 15,
    right_finite_count: int = 15,
    front_inf_count: int = 0,
    left_inf_count: int = 0,
    right_inf_count: int = 0,
    front_valid_ratio: float = 1.0,
    left_valid_ratio: float = 1.0,
    right_valid_ratio: float = 1.0,
) -> ScanSnapshot:
    return ScanSnapshot(
        front_min=front_min,
        left_min=left_min,
        right_min=right_min,
        left_avg=left_avg,
        right_avg=right_avg,
        front_ready=front_ready,
        left_ready=left_ready,
        right_ready=right_ready,
        front_valid_count=front_valid_count,
        left_valid_count=left_valid_count,
        right_valid_count=right_valid_count,
        front_finite_count=front_finite_count,
        left_finite_count=left_finite_count,
        right_finite_count=right_finite_count,
        front_inf_count=front_inf_count,
        left_inf_count=left_inf_count,
        right_inf_count=right_inf_count,
        front_valid_ratio=front_valid_ratio,
        left_valid_ratio=left_valid_ratio,
        right_valid_ratio=right_valid_ratio,
    )


def mock_scan_for_scenario(scenario: str) -> ScanSnapshot:
    scans = {
        "clear_corridor": build_scan_snapshot(3.6, 0.8, 0.9, 1.0, 1.1),
        "normal_corridor_side_distance": build_scan_snapshot(2.76, 1.70, 1.70, 1.82, 1.82),
        "left_opening": build_scan_snapshot(3.1, 2.4, 0.8, 2.8, 1.0),
        "right_opening": build_scan_snapshot(3.0, 0.9, 2.5, 1.1, 2.9),
        "front_blocked_left_open": build_scan_snapshot(0.7, 2.2, 0.7, 2.5, 0.9),
        "front_blocked_right_open": build_scan_snapshot(0.7, 0.8, 2.3, 0.9, 2.6),
        "front_blocked_both_blocked": build_scan_snapshot(0.6, 0.7, 0.8, 0.8, 0.9),
        "narrow_passage": build_scan_snapshot(1.5, 0.5, 0.6, 0.7, 0.8),
        "missing_side_scans": build_scan_snapshot(
            3.0,
            float("inf"),
            float("inf"),
            float("inf"),
            float("inf"),
            left_ready=False,
            right_ready=False,
            left_valid_count=0,
            right_valid_count=0,
            left_finite_count=0,
            right_finite_count=0,
            left_valid_ratio=0.0,
            right_valid_ratio=0.0,
        ),
        "all_inf_side_after_ready": build_scan_snapshot(
            3.0,
            float("inf"),
            float("inf"),
            float("inf"),
            float("inf"),
            left_finite_count=0,
            right_finite_count=0,
            left_inf_count=15,
            right_inf_count=15,
        ),
    }
    return scans[scenario]


def log_scan(scan: ScanSnapshot) -> None:
    log(
        "SCAN",
        "front_min={} m, left_min={} m, right_min={} m, left_avg={} m, right_avg={} m".format(
            format_distance(scan.front_min),
            format_distance(scan.left_min),
            format_distance(scan.right_min),
            format_distance(scan.left_avg),
            format_distance(scan.right_avg),
        ),
    )


def action_message(decision: Decision) -> str:
    if decision == Decision.DETECT_LEFT_OPENING:
        return "simulated left opening candidate"
    if decision == Decision.DETECT_RIGHT_OPENING:
        return "simulated right opening candidate"
    if decision == Decision.BYPASS_LEFT:
        return "simulated bypass_left"
    if decision == Decision.BYPASS_RIGHT:
        return "simulated bypass_right"
    if decision == Decision.BLOCKED:
        return "simulated stop: corridor blocked"
    if decision == Decision.NARROW_FORWARD:
        return "simulated slow forward through narrow passage"
    return "simulated follow forward"


def scan_stats(msg: object | None) -> ScanStats:
    if msg is None:
        return ScanStats(
            min_distance=float("inf"),
            avg_distance=float("inf"),
            sample_count=0,
            valid_count=0,
            finite_count=0,
            inf_count=0,
            valid_ratio=0.0,
            ready=False,
        )

    finite_ranges: list[float] = []
    inf_count = 0
    range_min = float(getattr(msg, "range_min", 0.0))
    range_max = float(getattr(msg, "range_max", 0.0))
    ranges = list(getattr(msg, "ranges", []))
    for distance in ranges:
        value = float(distance)
        if math.isinf(value):
            inf_count += 1
            continue
        if math.isnan(value) or value <= 0.0:
            continue
        if range_min > 0.0 and value < range_min:
            continue
        if range_max > 0.0 and value > range_max:
            continue
        finite_ranges.append(value)

    finite_count = len(finite_ranges)
    valid_count = finite_count + inf_count
    sample_count = len(ranges)
    valid_ratio = valid_count / sample_count if sample_count else 0.0
    min_distance = min(finite_ranges) if finite_ranges else float("inf")
    avg_distance = sum(finite_ranges) / finite_count if finite_count else float("inf")
    return ScanStats(
        min_distance=min_distance,
        avg_distance=avg_distance,
        sample_count=sample_count,
        valid_count=valid_count,
        finite_count=finite_count,
        inf_count=inf_count,
        valid_ratio=valid_ratio,
        ready=True,
    )


def format_distance(value: float) -> str:
    return "inf" if math.isinf(value) else f"{value:.2f}"


def log_scan_monitor_snapshot(scan: ScanSnapshot) -> None:
    log(
        "SCAN",
        (
            "front_min={} left_min={} right_min={} left_avg={} right_avg={} "
            "ready(front/left/right)={}/{}/{} valid(front/left/right)={}/{}/{} "
            "ratio(left/right)={:.2f}/{:.2f}"
        ).format(
            format_distance(scan.front_min),
            format_distance(scan.left_min),
            format_distance(scan.right_min),
            format_distance(scan.left_avg),
            format_distance(scan.right_avg),
            scan.front_ready,
            scan.left_ready,
            scan.right_ready,
            scan.front_valid_count,
            scan.left_valid_count,
            scan.right_valid_count,
            scan.left_valid_ratio,
            scan.right_valid_ratio,
        ),
    )


class LaserScanMonitor:
    def __init__(self, rclpy: object, laser_scan_type: object, args: argparse.Namespace) -> None:
        self.rclpy = rclpy
        self.latest_front: object | None = None
        self.latest_left: object | None = None
        self.latest_right: object | None = None
        self.node = self.rclpy.create_node("opening_based_scan_monitor")
        self.node.create_subscription(laser_scan_type, args.front_scan_topic, self._front_callback, 10)
        self.node.create_subscription(laser_scan_type, args.left_scan_topic, self._left_callback, 10)
        self.node.create_subscription(laser_scan_type, args.right_scan_topic, self._right_callback, 10)

    def _front_callback(self, msg: object) -> None:
        self.latest_front = msg

    def _left_callback(self, msg: object) -> None:
        self.latest_left = msg

    def _right_callback(self, msg: object) -> None:
        self.latest_right = msg

    def has_all_messages(self) -> bool:
        return self.latest_front is not None and self.latest_left is not None and self.latest_right is not None

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

    def close(self) -> None:
        self.node.destroy_node()


def decide_corridor_action(scan: ScanSnapshot, memory: MissionMemory, args: argparse.Namespace) -> Decision:
    front_stop = max(0.0, args.front_stop_distance)
    side_stop = max(0.0, args.side_stop_distance)
    narrow_side_threshold = 0.8

    if scan.front_min < front_stop:
        left_open = side_open_for_corridor_detection(scan, "left", args)
        right_open = side_open_for_corridor_detection(scan, "right", args)
        if left_open and right_open:
            return Decision.BYPASS_LEFT if scan.left_avg >= scan.right_avg else Decision.BYPASS_RIGHT
        if left_open:
            return Decision.BYPASS_LEFT
        if right_open:
            return Decision.BYPASS_RIGHT
        return Decision.BLOCKED

    if scan.left_min < narrow_side_threshold and scan.right_min < narrow_side_threshold:
        return Decision.NARROW_FORWARD

    left_open = side_open_for_corridor_detection(scan, "left", args)
    right_open = side_open_for_corridor_detection(scan, "right", args)
    memory.left_open_frames = memory.left_open_frames + 1 if left_open else 0
    memory.right_open_frames = memory.right_open_frames + 1 if right_open else 0
    confirm_frames = max(1, args.opening_confirm_frames)

    if memory.left_open_frames >= confirm_frames:
        return Decision.DETECT_LEFT_OPENING
    if memory.right_open_frames >= confirm_frames:
        return Decision.DETECT_RIGHT_OPENING

    if scan.left_min < side_stop and scan.right_min < side_stop:
        return Decision.NARROW_FORWARD

    return Decision.FOLLOW_FORWARD


def decide_opening_probe(scan: ScanSnapshot, side: str, memory: MissionMemory, args: argparse.Namespace) -> Decision:
    opening_id = f"{side}@{memory.corridor_x:.1f}"
    if opening_id in memory.visited_openings or opening_id in memory.skipped_openings:
        return Decision.SKIP_OPENING

    return Decision.PROBE_OPENING if side_open_for_probe_confirm(scan, side, args) else Decision.SKIP_OPENING


def side_ready_for_decision(scan: ScanSnapshot, side: str, args: argparse.Namespace) -> bool:
    min_valid_samples = max(1, args.min_valid_samples)
    min_valid_ratio = max(0.0, min(1.0, args.min_valid_ratio))
    if side == "left":
        return scan.left_ready and scan.left_valid_count >= min_valid_samples and scan.left_valid_ratio >= min_valid_ratio
    if side == "right":
        return scan.right_ready and scan.right_valid_count >= min_valid_samples and scan.right_valid_ratio >= min_valid_ratio
    return False


def side_open_for_corridor_detection(scan: ScanSnapshot, side: str, args: argparse.Namespace) -> bool:
    if not side_ready_for_decision(scan, side, args):
        return False

    side_open = max(0.0, args.side_open_distance)

    if side == "left":
        if scan.left_finite_count == 0 and scan.left_inf_count >= max(1, args.min_valid_samples):
            return True
        return scan.left_avg >= side_open
    if side == "right":
        if scan.right_finite_count == 0 and scan.right_inf_count >= max(1, args.min_valid_samples):
            return True
        return scan.right_avg >= side_open
    return False


def side_open_for_probe_confirm(scan: ScanSnapshot, side: str, args: argparse.Namespace) -> bool:
    if side_open_for_corridor_detection(scan, side, args):
        return True
    if not side_ready_for_decision(scan, side, args):
        return False

    side_confirm = max(0.0, args.side_confirm_distance)
    if side == "left":
        return scan.left_min >= side_confirm and scan.left_avg >= side_confirm
    if side == "right":
        return scan.right_min >= side_confirm and scan.right_avg >= side_confirm
    return False


def scan_ready_for_mission(scan: ScanSnapshot, args: argparse.Namespace) -> bool:
    min_valid_samples = max(1, args.min_valid_samples)
    min_valid_ratio = max(0.0, min(1.0, args.min_valid_ratio))
    return (
        scan.front_ready
        and scan.left_ready
        and scan.right_ready
        and scan.front_valid_count >= min_valid_samples
        and scan.left_valid_count >= min_valid_samples
        and scan.right_valid_count >= min_valid_samples
        and scan.front_valid_ratio >= min_valid_ratio
        and scan.left_valid_ratio >= min_valid_ratio
        and scan.right_valid_ratio >= min_valid_ratio
    )


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
                    f"right_ready={monitor.latest_right is not None}",
                )
                log("ACTION", "simulated wait: scan data not ready")
                next_log_at = now + log_interval_sec
                continue
            scan = monitor.snapshot()
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
        scan = monitor.snapshot()
        if monitor.has_all_messages() and scan_ready_for_mission(scan, args):
            log_scan_monitor_snapshot(scan)
            log("SCAN", "front/left/right scan readiness confirmed")
            return True

        now = time.monotonic()
        if now >= next_log_at:
            log(
                "SCAN",
                "waiting for scan readiness: "
                f"front_ready={monitor.latest_front is not None} "
                f"left_ready={monitor.latest_left is not None} "
                f"right_ready={monitor.latest_right is not None}",
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

        scan = monitor.snapshot()
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
    if msg is None:
        return None

    sector_half_rad = math.radians(max(0.0, sector_deg) / 2.0)
    angle = float(getattr(msg, "angle_min", 0.0))
    angle_increment = float(getattr(msg, "angle_increment", 0.0))
    valid_ranges: list[float] = []
    for distance in getattr(msg, "ranges", []):
        if abs(angle) <= sector_half_rad and valid_laser_range(float(distance), msg):
            valid_ranges.append(float(distance))
        angle += angle_increment

    return min(valid_ranges) if valid_ranges else None


def front_motion_allowed(monitor: LaserScanMonitor, args: argparse.Namespace) -> bool:
    front_min = front_sector_min_distance(monitor.latest_front, args.front_sector_deg)
    if front_min is None:
        log("SAFETY", "front sector unavailable; stopping corridor motion")
        return False

    log("SAFETY", f"front_sector_min={front_min:.2f} m, sector={max(0.0, args.front_sector_deg):.1f} deg")
    if front_min < max(0.0, args.front_stop_distance):
        log("SAFETY", "front obstacle detected; stopping corridor motion")
        return False

    if front_min >= max(args.front_stop_distance, args.front_clear_distance):
        log("SAFETY", "front sector clear")
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


async def send_zero_velocity(drone: object, velocity_type: object, yaw_deg: float) -> None:
    await drone.offboard.set_velocity_ned(velocity_type(0.0, 0.0, 0.0, yaw_deg))


async def send_zero_body_velocity(drone: object, velocity_type: object) -> None:
    await drone.offboard.set_velocity_body(velocity_type(0.0, 0.0, 0.0, 0.0))


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
            log_passive_corridor_decision(monitor, memory, args, f"step={step_index + 1}/{step_count} pre")
            if not front_motion_allowed(monitor, args):
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
                    log_passive_corridor_decision(
                        monitor,
                        memory,
                        args,
                        f"step={step_index + 1}/{step_count} moving",
                    )
                    next_decision_log_at = now + decision_log_interval_sec
                if not front_motion_allowed(monitor, args):
                    await send_zero_body_velocity(drone, velocity_type)
                    return
                await drone.offboard.set_velocity_body(
                    velocity_type(forward_speed, right_speed, down_speed, yawspeed)
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
                        log_passive_corridor_decision(
                            monitor,
                            memory,
                            args,
                            f"step={step_index + 1}/{step_count} pause",
                        )
                        next_decision_log_at = now + decision_log_interval_sec
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
    if args.axis_calibration_check:
        return asyncio.run(run_axis_calibration_check(args))
    if not args.dry_run:
        print(
            "Use --dry-run for pure Python simulation, --scan-monitor for ROS2 LaserScan monitoring, "
            "--takeoff-land-check for MAVSDK takeoff/land validation, "
            "--corridor-follow-check for low-speed front-safe corridor movement, "
            "or --axis-calibration-check for offboard axis diagnosis."
        )
        return 2
    return run_dry_run(args)


if __name__ == "__main__":
    raise SystemExit(main())
