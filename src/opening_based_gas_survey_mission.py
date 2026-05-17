#!/usr/bin/env python3
"""Dry-run skeleton for an opening-based gas survey mission.

This phase is intentionally simulation-only. It does not import MAVSDK, ROS2, or
sensor message types, and it never commands a drone.
"""

from __future__ import annotations

import argparse
import math
import random
import time
from dataclasses import dataclass
from enum import Enum


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
    if not args.dry_run:
        print("Use --dry-run for pure Python simulation or --scan-monitor for ROS2 LaserScan monitoring.")
        return 2
    return run_dry_run(args)


if __name__ == "__main__":
    raise SystemExit(main())
