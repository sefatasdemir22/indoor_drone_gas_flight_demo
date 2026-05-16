#!/usr/bin/env python3
"""Mission finite-state-machine for the indoor drone demo.

The default sim mode does not connect to PX4, MAVSDK, ROS2, or a real drone.
The mavsdk mode only verifies PX4 SITL connection and basic takeoff/land; it
does not yet fly the room-scanning waypoint mission.
"""

from __future__ import annotations

import argparse
import asyncio
import time
from dataclasses import dataclass
from enum import Enum


class MissionState(str, Enum):
    TAKEOFF = "TAKEOFF"
    ENTER_ENVIRONMENT = "ENTER_ENVIRONMENT"
    EXPLORE_CORRIDOR = "EXPLORE_CORRIDOR"
    ENTER_ROOM = "ENTER_ROOM"
    SAMPLE_GAS = "SAMPLE_GAS"
    MAP_UPDATE = "MAP_UPDATE"
    MOVE = "MOVE"
    RETURN_TO_SAFE_EXIT = "RETURN_TO_SAFE_EXIT"
    LAND = "LAND"
    FINISH = "FINISH"


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    z: float
    sample: bool = False
    room: bool = False


MISSION_WAYPOINTS: list[Waypoint] = [
    Waypoint("START_SAFE_EXIT", 0.0, 0.0, 1.5),
    Waypoint("CORRIDOR_ENTRY", 2.0, 0.0, 1.5, sample=True),
    Waypoint("LEFT_ROOM", 5.0, 4.0, 1.5, sample=True, room=True),
    Waypoint("CORRIDOR_MID", 7.0, 0.0, 1.5, sample=True),
    Waypoint("RIGHT_ROOM", 9.0, -4.0, 1.5, sample=True, room=True),
    Waypoint("FORWARD_ROOM", 13.0, 3.0, 1.5, sample=True, room=True),
    Waypoint("RETURN_CORRIDOR", 7.0, 0.0, 1.5, sample=True),
    Waypoint("START_SAFE_EXIT_RETURN", 0.0, 0.0, 1.5),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the simulated mission FSM.")
    parser.add_argument(
        "--mode",
        default="sim",
        choices=["sim", "mavsdk"],
        help="Mission backend. sim logs the FSM; mavsdk runs a basic PX4 SITL takeoff/land check.",
    )
    parser.add_argument(
        "--sleep",
        type=float,
        default=0.25,
        help="Delay in seconds between simulated state transitions.",
    )
    parser.add_argument("--system-address", default="udp://:14540")
    parser.add_argument("--takeoff-altitude", type=float, default=2.0)
    parser.add_argument("--takeoff-timeout", type=float, default=25.0)
    parser.add_argument("--takeoff-altitude-tolerance", type=float, default=0.5)
    parser.add_argument("--hover-seconds", type=float, default=5.0)
    parser.add_argument("--connection-timeout", type=float, default=30.0)
    parser.add_argument(
        "--enable-short-move",
        action="store_true",
        help="After takeoff, run a short low-speed offboard forward movement before landing.",
    )
    parser.add_argument("--move-forward-seconds", type=float, default=4.0)
    parser.add_argument("--move-forward-speed", type=float, default=0.5)
    parser.add_argument("--move-rate-hz", type=float, default=10.0)
    return parser.parse_args()


def log_state(state: MissionState, message: str) -> None:
    print(f"[{state.value}] {message}", flush=True)


def log_waypoint(prefix: str, waypoint: Waypoint) -> None:
    print(f"  {prefix}: {waypoint.name} -> x={waypoint.x:.1f}, y={waypoint.y:.1f}, z={waypoint.z:.1f}")


def pause(seconds: float) -> None:
    if seconds > 0:
        time.sleep(seconds)


def sample_and_update(waypoint: Waypoint, sleep_seconds: float) -> None:
    log_state(MissionState.SAMPLE_GAS, f"gas sample requested at {waypoint.name}")
    log_waypoint("sample position", waypoint)
    pause(sleep_seconds)

    log_state(MissionState.MAP_UPDATE, f"map update triggered after sampling {waypoint.name}")
    pause(sleep_seconds)


def run_simulated_mission(sleep_seconds: float) -> None:
    print("Mission manager mode: sim")
    print("No PX4, MAVSDK, ROS2, or real drone commands are used in this mode.")
    print()

    start = MISSION_WAYPOINTS[0]
    corridor_entry = MISSION_WAYPOINTS[1]
    return_corridor = MISSION_WAYPOINTS[-2]
    safe_exit_return = MISSION_WAYPOINTS[-1]

    log_state(MissionState.TAKEOFF, "arming/checks would happen here in future MAVSDK mode")
    log_waypoint("takeoff target", start)
    pause(sleep_seconds)

    log_state(MissionState.ENTER_ENVIRONMENT, "entering the indoor environment from START / SAFE_EXIT")
    log_waypoint("target", corridor_entry)
    pause(sleep_seconds)
    sample_and_update(corridor_entry, sleep_seconds)

    for waypoint in MISSION_WAYPOINTS[2:6]:
        log_state(MissionState.EXPLORE_CORRIDOR, f"navigating corridor toward {waypoint.name}")
        log_waypoint("target", waypoint)
        pause(sleep_seconds)

        if waypoint.room:
            log_state(MissionState.ENTER_ROOM, f"entering room/area: {waypoint.name}")
            log_waypoint("room scan target", waypoint)
            pause(sleep_seconds)

        if waypoint.sample:
            sample_and_update(waypoint, sleep_seconds)

    log_state(MissionState.EXPLORE_CORRIDOR, "returning to the main corridor after room visits")
    log_waypoint("target", return_corridor)
    pause(sleep_seconds)
    sample_and_update(return_corridor, sleep_seconds)

    log_state(MissionState.RETURN_TO_SAFE_EXIT, "return route: RETURN_CORRIDOR -> START_SAFE_EXIT_RETURN")
    log_waypoint("return start", return_corridor)
    log_waypoint("return target", safe_exit_return)
    pause(sleep_seconds)

    log_state(MissionState.LAND, "landing at START / SAFE_EXIT; no separate corridor-end landing area is used")
    log_waypoint("landing position", safe_exit_return)
    pause(sleep_seconds)

    log_state(MissionState.FINISH, "simulated mission complete")


async def wait_for_mavsdk_connection(drone: object, timeout_sec: float) -> None:
    log_state(MissionState.TAKEOFF, f"waiting for PX4 connection, timeout={timeout_sec:.1f}s")

    async def _wait() -> None:
        async for state in drone.core.connection_state():
            print(f"  connection_state: is_connected={state.is_connected}")
            if state.is_connected:
                return

    await asyncio.wait_for(_wait(), timeout=timeout_sec)
    log_state(MissionState.TAKEOFF, "PX4 connection established")


async def wait_for_mavsdk_health(drone: object, timeout_sec: float) -> None:
    log_state(MissionState.TAKEOFF, f"waiting for vehicle health, timeout={timeout_sec:.1f}s")

    async def _wait() -> None:
        async for health in drone.telemetry.health():
            print(
                "  health: "
                f"global={health.is_global_position_ok}, "
                f"local={health.is_local_position_ok}, "
                f"home={health.is_home_position_ok}"
            )
            if health.is_global_position_ok and health.is_local_position_ok and health.is_home_position_ok:
                return

    await asyncio.wait_for(_wait(), timeout=timeout_sec)
    log_state(MissionState.TAKEOFF, "vehicle health checks passed")


async def try_mavsdk_land(drone: object) -> None:
    try:
        log_state(MissionState.LAND, "attempting safety land")
        await drone.action.land()
    except Exception as exc:
        log_state(MissionState.LAND, f"safety land attempt failed: {exc}")


async def wait_until_takeoff_altitude(
    drone: object,
    target_altitude_m: float,
    timeout_sec: float = 20.0,
    tolerance_m: float = 0.5,
) -> bool:
    target_altitude_m = max(0.0, target_altitude_m)
    tolerance_m = max(0.05, tolerance_m)
    log_state(
        MissionState.TAKEOFF,
        f"waiting until takeoff altitude is reached: target={target_altitude_m:.1f} m, tolerance={tolerance_m:.1f} m, timeout={timeout_sec:.1f}s",
    )

    async def _wait() -> bool:
        last_altitude_log_time = 0.0
        async for position_velocity in drone.telemetry.position_velocity_ned():
            position = position_velocity.position
            altitude_estimate = abs(position.down_m)
            now = time.monotonic()
            if now - last_altitude_log_time >= 1.0:
                log_state(MissionState.TAKEOFF, f"current altitude estimate: {altitude_estimate:.2f} m")
                last_altitude_log_time = now
            if abs(altitude_estimate - target_altitude_m) < tolerance_m:
                return True
            if position.down_m <= -(target_altitude_m - tolerance_m):
                return True
        return False

    try:
        reached = await asyncio.wait_for(_wait(), timeout=timeout_sec)
    except asyncio.TimeoutError:
        log_state(MissionState.TAKEOFF, "takeoff altitude wait timed out")
        return False
    except Exception as exc:
        log_state(MissionState.TAKEOFF, f"takeoff altitude wait failed: {exc}")
        return False

    if reached:
        log_state(MissionState.TAKEOFF, "takeoff altitude reached")
        return True
    return False


async def read_local_position_ned(drone: object, label: str) -> object | None:
    try:
        position_velocity = await asyncio.wait_for(anext(drone.telemetry.position_velocity_ned()), timeout=2.0)
    except Exception as exc:
        log_state(MissionState.MOVE, f"could not read {label} local position: {exc}")
        return None

    position = position_velocity.position
    log_state(
        MissionState.MOVE,
        f"{label} local position NED: north={position.north_m:.2f} m, east={position.east_m:.2f} m, down={position.down_m:.2f} m",
    )
    return position


async def run_short_forward_move(drone: object, args: argparse.Namespace, velocity_type: object) -> None:
    speed = max(0.0, args.move_forward_speed)
    duration = max(0.0, args.move_forward_seconds)
    rate_hz = max(1.0, args.move_rate_hz)
    interval = 1.0 / rate_hz
    setpoint_count = 0

    log_state(MissionState.MOVE, "starting short forward move")
    if args.move_forward_speed < 0:
        log_state(MissionState.MOVE, f"negative speed requested; clamped to {speed:.2f} m/s")
    if args.move_rate_hz < 1.0:
        log_state(MissionState.MOVE, f"move rate too low; clamped to {rate_hz:.1f} Hz")
    if duration < 1.0:
        log_state(MissionState.MOVE, f"move duration is very short: {duration:.2f} s")

    start_position = await read_local_position_ned(drone, "start")

    log_state(MissionState.MOVE, "sending initial zero velocity setpoint")
    await drone.offboard.set_velocity_ned(velocity_type(0.0, 0.0, 0.0, 0.0))

    log_state(MissionState.MOVE, "starting offboard mode")
    await drone.offboard.start()

    log_state(
        MissionState.MOVE,
        f"velocity command loop: north={speed:.2f} m/s, east=0.00 m/s, down=0.00 m/s, duration={duration:.1f}s, rate={rate_hz:.1f}Hz",
    )
    loop_start = asyncio.get_running_loop().time()
    while asyncio.get_running_loop().time() - loop_start < duration:
        await drone.offboard.set_velocity_ned(velocity_type(speed, 0.0, 0.0, 0.0))
        setpoint_count += 1
        await asyncio.sleep(interval)

    log_state(MissionState.MOVE, "stopping movement with zero velocity")
    for _ in range(5):
        await drone.offboard.set_velocity_ned(velocity_type(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(interval)
    log_state(MissionState.MOVE, f"sent {setpoint_count} forward velocity setpoints")

    end_position = await read_local_position_ned(drone, "end")
    if start_position is not None and end_position is not None:
        delta_north = end_position.north_m - start_position.north_m
        delta_east = end_position.east_m - start_position.east_m
        log_state(MissionState.MOVE, f"estimated local displacement: north={delta_north:.2f} m, east={delta_east:.2f} m")

    try:
        log_state(MissionState.MOVE, "stopping offboard mode")
        await drone.offboard.stop()
    except Exception as exc:
        log_state(MissionState.MOVE, f"offboard stop failed, continuing to land: {exc}")


async def run_mavsdk_takeoff_land(args: argparse.Namespace) -> int:
    try:
        from mavsdk import System
        from mavsdk.offboard import VelocityNedYaw
    except ImportError as exc:
        print(f"MAVSDK import failed: {exc}")
        print("Install MAVSDK Python or use --mode sim.")
        return 1

    drone = System()

    print("Mission manager mode: mavsdk")
    print("This mode does not run the room waypoint mission yet.")
    print("It only connects to PX4 SITL and performs takeoff/hover/land.")
    print(f"system_address={args.system_address}")
    print(f"takeoff_altitude={args.takeoff_altitude:.1f} m")
    print(f"takeoff_timeout={args.takeoff_timeout:.1f} s")
    print(f"takeoff_altitude_tolerance={args.takeoff_altitude_tolerance:.1f} m")
    print(f"hover_seconds={args.hover_seconds:.1f} s")
    print(f"short_move_enabled={args.enable_short_move}")
    if args.enable_short_move:
        print(f"move_forward_seconds={args.move_forward_seconds:.1f} s")
        print(f"move_forward_speed={args.move_forward_speed:.2f} m/s")
        print(f"move_rate_hz={args.move_rate_hz:.1f} Hz")
    print()

    try:
        log_state(MissionState.TAKEOFF, "creating MAVSDK connection")
        await asyncio.wait_for(drone.connect(system_address=args.system_address), timeout=args.connection_timeout)
        await wait_for_mavsdk_connection(drone, args.connection_timeout)
        await wait_for_mavsdk_health(drone, args.connection_timeout)

        log_state(MissionState.TAKEOFF, f"setting takeoff altitude to {args.takeoff_altitude:.1f} m")
        await drone.action.set_takeoff_altitude(args.takeoff_altitude)

        log_state(MissionState.TAKEOFF, "arming")
        await drone.action.arm()

        log_state(MissionState.TAKEOFF, "takeoff command sent")
        await drone.action.takeoff()

        takeoff_reached = await wait_until_takeoff_altitude(
            drone,
            target_altitude_m=args.takeoff_altitude,
            timeout_sec=args.takeoff_timeout,
            tolerance_m=args.takeoff_altitude_tolerance,
        )

        if takeoff_reached:
            log_state(MissionState.TAKEOFF, f"hovering for {args.hover_seconds:.1f} seconds")
            await asyncio.sleep(args.hover_seconds)
        else:
            log_state(MissionState.TAKEOFF, "takeoff altitude was not reached; skipping hover/move and landing")

        if args.enable_short_move and takeoff_reached:
            await run_short_forward_move(drone, args, VelocityNedYaw)
        elif args.enable_short_move:
            log_state(MissionState.MOVE, "short move skipped because takeoff altitude was not reached")

        log_state(MissionState.LAND, "land command sent")
        await drone.action.land()

        log_state(MissionState.FINISH, "MAVSDK takeoff/land mission complete")
        return 0

    except asyncio.TimeoutError:
        print("Timed out while waiting for PX4 connection or vehicle health.")
        print("Make sure PX4 SITL is running and MAVLink is available on udp://:14540.")
        await try_mavsdk_land(drone)
        return 1
    except Exception as exc:
        print(f"MAVSDK mission failed: {exc}")
        await try_mavsdk_land(drone)
        return 1


def main() -> int:
    args = parse_args()
    if args.mode == "sim":
        run_simulated_mission(args.sleep)
        return 0
    return asyncio.run(run_mavsdk_takeoff_land(args))


if __name__ == "__main__":
    raise SystemExit(main())
