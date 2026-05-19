#!/usr/bin/env python3
"""Shared data types for opening-based mission scripts."""

from __future__ import annotations

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
class FrontSectorStats:
    sample_count: int
    valid_count: int
    finite_count: int
    inf_count: int
    valid_ratio: float
    min_finite_distance: float | None


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


@dataclass
class OpeningCandidateState:
    active_side: str
    start_step: int
    start_position: object | None
    best_position: object | None
    best_side_avg: float
    frames_seen: int
    last_seen_step: int
    best_front_distance: float | None


@dataclass(frozen=True)
class PositionOpeningAnchor:
    side: str
    start_step: int
    mature_step: int
    anchor_north: float
    anchor_east: float
    start_position: object | None
    mature_position: object | None
    best_position: object | None
    best_side_avg: float
    frames_seen: int


@dataclass(frozen=True)
class TakeoffAltitudeResult:
    confirmed: bool
    safe_hover_altitude: bool
    last_altitude_m: float


@dataclass(frozen=True)
class GasSampleSummary:
    label: str
    sample_count: int
    avg_ppm: float
    max_ppm: float
    position: dict[str, float] | None
