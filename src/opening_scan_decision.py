#!/usr/bin/env python3
"""Scan snapshot and opening-decision helpers."""

from __future__ import annotations

import math
from typing import Any

from opening_mission_types import Decision, MissionMemory, ScanSnapshot, ScanStats


def log(tag: str, message: str) -> None:
    print(f"[{tag}] {message}", flush=True)


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


def decide_corridor_action(scan: ScanSnapshot, memory: MissionMemory, args: Any) -> Decision:
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


def decide_opening_probe(scan: ScanSnapshot, side: str, memory: MissionMemory, args: Any) -> Decision:
    opening_id = f"{side}@{memory.corridor_x:.1f}"
    if opening_id in memory.visited_openings or opening_id in memory.skipped_openings:
        return Decision.SKIP_OPENING

    return Decision.PROBE_OPENING if side_open_for_probe_confirm(scan, side, args) else Decision.SKIP_OPENING


def side_ready_for_decision(scan: ScanSnapshot, side: str, args: Any) -> bool:
    min_valid_samples = max(1, args.min_valid_samples)
    min_valid_ratio = max(0.0, min(1.0, args.min_valid_ratio))
    if side == "left":
        return scan.left_ready and scan.left_valid_count >= min_valid_samples and scan.left_valid_ratio >= min_valid_ratio
    if side == "right":
        return scan.right_ready and scan.right_valid_count >= min_valid_samples and scan.right_valid_ratio >= min_valid_ratio
    return False


def side_open_for_corridor_detection(scan: ScanSnapshot, side: str, args: Any) -> bool:
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


def side_decision_diagnostics(scan: ScanSnapshot, side: str, args: Any) -> tuple[bool, str, str]:
    min_valid_samples = max(1, args.min_valid_samples)
    min_valid_ratio = max(0.0, min(1.0, args.min_valid_ratio))
    side_open = max(0.0, args.side_open_distance)

    if side == "left":
        ready = scan.left_ready
        min_distance = scan.left_min
        avg_distance = scan.left_avg
        valid_count = scan.left_valid_count
        finite_count = scan.left_finite_count
        inf_count = scan.left_inf_count
        valid_ratio = scan.left_valid_ratio
    elif side == "right":
        ready = scan.right_ready
        min_distance = scan.right_min
        avg_distance = scan.right_avg
        valid_count = scan.right_valid_count
        finite_count = scan.right_finite_count
        inf_count = scan.right_inf_count
        valid_ratio = scan.right_valid_ratio
    else:
        return False, "invalid_side", f"side={side}"

    metrics = (
        f"side={side} min={min_distance:.2f} avg={avg_distance:.2f} "
        f"valid={valid_count} finite={finite_count} inf={inf_count} "
        f"ratio={valid_ratio:.2f} threshold={side_open:.2f}"
    )

    if not ready:
        return False, "not_ready", metrics
    if valid_count < min_valid_samples:
        return False, "valid_count_low", f"{metrics} min_valid_samples={min_valid_samples}"
    if valid_ratio < min_valid_ratio:
        return False, "valid_ratio_low", f"{metrics} min_valid_ratio={min_valid_ratio:.2f}"
    if finite_count == 0 and inf_count >= min_valid_samples:
        return True, "all_inf_clear", metrics
    if avg_distance >= side_open:
        return True, "avg_above_threshold", metrics
    return False, "avg_below_threshold", metrics


def side_avg_for_snapshot(scan: ScanSnapshot, side: str) -> float:
    if side == "left":
        return scan.left_avg
    if side == "right":
        return scan.right_avg
    return math.inf


def side_min_for_snapshot(scan: ScanSnapshot, side: str) -> float:
    if side == "left":
        return scan.left_min
    if side == "right":
        return scan.right_min
    return math.inf


def side_open_for_probe_confirm(scan: ScanSnapshot, side: str, args: Any) -> bool:
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


def scan_ready_for_mission(scan: ScanSnapshot, args: Any) -> bool:
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
