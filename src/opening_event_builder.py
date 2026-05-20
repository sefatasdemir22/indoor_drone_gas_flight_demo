"""Pure event JSON builders for the opening-based gas survey mission."""

from __future__ import annotations

from typing import Any


def round_float_or_none(value: object, digits: int = 3) -> float | None:
    if value is None:
        return None
    return round(float(value), digits)


def _position_as_event_dict(position: object | None) -> dict[str, float] | None:
    if position is None:
        return None
    return {
        "north": round(float(position.north_m), 3),
        "east": round(float(position.east_m), 3),
        "altitude": round(max(0.0, -float(position.down_m)), 3),
    }


def _anchor_position(north_m: float, east_m: float, down_m: float) -> dict[str, float]:
    return {"north": round(north_m, 3), "east": round(east_m, 3), "altitude": -down_m}


def room_facing_entry_anchor_position_from_traversal(
    traversal: dict[str, Any],
    down_m: float,
    *,
    include_when_missing: bool = False,
) -> dict[str, float | None] | None:
    if not include_when_missing and traversal.get("room_facing_entry_anchor_north") is None:
        return None
    return {
        "north": traversal.get("room_facing_entry_anchor_north"),
        "east": traversal.get("room_facing_entry_anchor_east"),
        "altitude": -down_m,
    }


def _room_facing_config_fields(args: Any) -> dict[str, Any]:
    return {
        "room_facing_yaw_hold_before_seconds": round(
            max(0.0, args.room_facing_yaw_hold_before_seconds),
            3,
        ),
        "room_facing_yaw_hold_after_seconds": round(
            max(0.0, args.room_facing_yaw_hold_after_seconds),
            3,
        ),
        "room_facing_yaw_settle_repeat_count": max(1, args.room_facing_yaw_settle_repeat_count),
        "room_facing_yaw_interpolation_step_deg": round(
            max(0.0, args.room_facing_yaw_interpolation_step_deg),
            3,
        ),
        "room_facing_yaw_interpolation_hold_seconds": round(
            max(0.0, args.room_facing_yaw_interpolation_hold_seconds),
            3,
        ),
    }


def _traversal_base_event_fields(traversal: dict[str, Any]) -> dict[str, Any]:
    return {
        "room_traversal_mode": traversal["mode"],
        "room_traverse_stop_distance_m": traversal["stop_distance_m"],
        "room_traverse_step_distance_m": traversal["step_distance_m"],
        "room_traverse_max_distance_m": traversal["max_distance_m"],
        "room_traverse_actual_distance_m": traversal["actual_distance_m"],
        "room_traverse_stop_reason": traversal["stop_reason"],
        "room_direction_scan": traversal["direction_scan"],
        "room_final_side_min_m": traversal["final_side_min_m"],
        "room_final_side_avg_m": traversal["final_side_avg_m"],
        "room_depth_estimate_m": traversal["depth_estimate_m"],
        "room_width_estimate_m": traversal["width_estimate_m"],
        "room_facing_yaw_deg": traversal.get("room_facing_yaw_deg"),
        "room_facing_final_front_min_m": traversal.get("room_facing_final_front_min_m"),
        "room_facing_exit_steps": traversal.get("room_facing_exit_steps"),
        "room_facing_exit_actual_distance_m": traversal.get("room_facing_exit_actual_distance_m"),
        "room_facing_door_forward_offset_m": traversal.get("room_facing_door_forward_offset_m"),
    }


def _room_facing_post_yaw_fields(traversal: dict[str, Any]) -> dict[str, Any]:
    return {
        "room_facing_post_yaw_realign_enabled": traversal.get("room_facing_post_yaw_realign_enabled"),
        "room_facing_post_yaw_total_offset_m": traversal.get("room_facing_post_yaw_total_offset_m"),
        "room_facing_post_yaw_front_min_m": traversal.get("room_facing_post_yaw_front_min_m"),
        "room_facing_post_yaw_realign_steps": traversal.get("room_facing_post_yaw_realign_steps"),
    }


def _position_candidate_step_fields(anchor: Any) -> dict[str, Any]:
    return {
        "candidate_start_step": anchor.start_step + 1,
        "candidate_mature_step": anchor.mature_step + 1,
    }


def _position_candidate_detail_fields(anchor: Any) -> dict[str, Any]:
    return {
        "candidate_start_position": _position_as_event_dict(anchor.start_position),
        "candidate_mature_position": _position_as_event_dict(anchor.mature_position),
        "candidate_best_position": _position_as_event_dict(anchor.best_position),
        "candidate_best_side_avg": round(anchor.best_side_avg, 3),
        "candidate_frames_seen": anchor.frames_seen,
    }


def build_body_inspection_event(
    *,
    inspection_index: int,
    side: str,
    opening_id: str,
    step_index: int,
    baseline: Any,
    inspection: Any,
    delta_ppm: float,
    gas_candidate: bool,
    candidate_reason: str,
    entry_anchor_position: object | None,
    enter_target_distance_m: float,
    enter_result: dict[str, Any],
    exit_tolerance_m: float,
) -> dict[str, Any]:
    return {
        "inspection_index": inspection_index,
        "side": side,
        "opening_id": opening_id,
        "step_index": step_index + 1,
        "baseline_avg_ppm": round(baseline.avg_ppm, 3),
        "inspection_avg_ppm": round(inspection.avg_ppm, 3),
        "delta_ppm": round(delta_ppm, 3),
        "gas_candidate": gas_candidate,
        "candidate_reason": candidate_reason,
        "baseline_sample_count": baseline.sample_count,
        "inspection_sample_count": inspection.sample_count,
        "baseline_position": baseline.position,
        "inspection_position": inspection.position,
        "entry_anchor_position": _position_as_event_dict(entry_anchor_position),
        "enter_target_distance_m": round(max(0.0, enter_target_distance_m), 3),
        "enter_actual_distance_m": enter_result["actual_distance_m"],
        "enter_elapsed_seconds": enter_result["elapsed_seconds"],
        "exit_tolerance_m": round(max(0.0, exit_tolerance_m), 3),
        "exit_final_distance_m": None,
        "exit_elapsed_seconds": None,
    }


def build_position_abort_event(
    *,
    inspection_index: int,
    side: str,
    opening_id: str,
    anchor: Any,
    anchor_north: float,
    anchor_east: float,
    down_m: float,
    room_entry_distance: float,
    baseline: Any,
    traversal: dict[str, Any],
    args: Any,
) -> dict[str, Any]:
    event = {
        "inspection_index": inspection_index,
        "side": side,
        "opening_id": opening_id,
        "step_index": anchor.mature_step + 1,
        **_position_candidate_step_fields(anchor),
        "baseline_avg_ppm": round(baseline.avg_ppm, 3),
        "inspection_avg_ppm": None,
        "delta_ppm": None,
        "gas_candidate": False,
        "candidate_reason": traversal["stop_reason"],
        "baseline_sample_count": baseline.sample_count,
        "inspection_sample_count": 0,
        "baseline_position": baseline.position,
        "inspection_position": None,
        "entry_anchor_position": _anchor_position(anchor_north, anchor_east, down_m),
        "entry_anchor_source": anchor.anchor_source,
        **_position_candidate_detail_fields(anchor),
        "enter_target_distance_m": round(room_entry_distance, 3),
        "enter_actual_distance_m": 0.0,
        **_traversal_base_event_fields(traversal),
        **_room_facing_config_fields(args),
        **_room_facing_post_yaw_fields(traversal),
        "room_facing_entry_anchor_position": room_facing_entry_anchor_position_from_traversal(
            traversal,
            down_m,
            include_when_missing=True,
        ),
        "exit_final_distance_m": None,
    }
    return event


def build_position_inspection_event(
    *,
    inspection_index: int,
    side: str,
    opening_id: str,
    anchor: Any,
    anchor_north: float,
    anchor_east: float,
    down_m: float,
    room_entry_distance: float,
    entry_north: float,
    baseline: Any,
    inspection: Any,
    delta_ppm: float,
    gas_candidate: bool,
    candidate_reason: str,
    traversal: dict[str, Any],
    args: Any,
) -> dict[str, Any]:
    event = {
        "inspection_index": inspection_index,
        "side": side,
        "opening_id": opening_id,
        "step_index": anchor.mature_step + 1,
        **_position_candidate_step_fields(anchor),
        "baseline_avg_ppm": round(baseline.avg_ppm, 3),
        "inspection_avg_ppm": round(inspection.avg_ppm, 3),
        "delta_ppm": round(delta_ppm, 3),
        "gas_candidate": gas_candidate,
        "candidate_reason": candidate_reason,
        "baseline_sample_count": baseline.sample_count,
        "inspection_sample_count": inspection.sample_count,
        "baseline_position": baseline.position,
        "inspection_position": inspection.position,
        "entry_anchor_position": _anchor_position(anchor_north, anchor_east, down_m),
        "entry_anchor_source": anchor.anchor_source,
        **_position_candidate_detail_fields(anchor),
        "enter_target_distance_m": round(room_entry_distance, 3),
        "enter_actual_distance_m": (
            traversal["actual_distance_m"]
            if traversal["mode"] == "room_facing_yaw"
            else round(abs(entry_north - anchor_north), 3)
        ),
        **_traversal_base_event_fields(traversal),
        **_room_facing_config_fields(args),
        **_room_facing_post_yaw_fields(traversal),
        "room_facing_entry_anchor_position": room_facing_entry_anchor_position_from_traversal(
            traversal,
            down_m,
        ),
        "exit_final_distance_m": None,
    }
    return event
