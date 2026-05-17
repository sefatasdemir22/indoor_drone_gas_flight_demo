#!/usr/bin/env python3
"""Step-based MAVSDK corridor movement prototype.

This script commands only simple NED velocity steps. It does not read gas
values and does not use lidar/range data in the control loop.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import time


DEFAULT_SYSTEM_ADDRESS = "udpin://0.0.0.0:14540"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a simple safe corridor MAVSDK mission.")
    parser.add_argument(
        "--system-address",
        default=DEFAULT_SYSTEM_ADDRESS,
        help=f"MAVSDK system address. Default: {DEFAULT_SYSTEM_ADDRESS}",
    )
    parser.add_argument("--connection-timeout", type=float, default=30.0)
    parser.add_argument("--takeoff-altitude", type=float, default=1.2)
    parser.add_argument("--takeoff-timeout", type=float, default=25.0)
    parser.add_argument("--takeoff-altitude-tolerance", type=float, default=0.35)
    parser.add_argument("--step-count", type=int, default=6)
    parser.add_argument("--step-duration-seconds", type=float, default=2.5)
    parser.add_argument("--step-rate-hz", type=float, default=10.0)
    parser.add_argument("--north-speed", type=float, default=0.0)
    parser.add_argument("--east-speed", type=float, default=0.3)
    parser.add_argument("--pause-between-steps", type=float, default=1.0)
    parser.add_argument("--enable-front-safety", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--disable-front-safety", action="store_true")
    parser.add_argument("--front-scan-topic", default="/drone/front_scan")
    parser.add_argument("--front-stop-distance", type=float, default=1.0)
    parser.add_argument("--front-clear-distance", type=float, default=1.3)
    parser.add_argument("--front-sector-deg", type=float, default=50.0)
    return parser.parse_args()


def log(tag: str, message: str) -> None:
    print(f"[{tag}] {message}", flush=True)


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


class FrontSafetyMonitor:
    def __init__(
        self,
        topic: str,
        stop_distance_m: float,
        clear_distance_m: float,
        sector_deg: float,
    ) -> None:
        import rclpy
        from sensor_msgs.msg import LaserScan

        self.rclpy = rclpy
        self.stop_distance_m = max(0.0, stop_distance_m)
        self.clear_distance_m = max(self.stop_distance_m, clear_distance_m)
        self.sector_half_rad = math.radians(max(0.0, sector_deg) / 2.0)
        self.front_min_distance_m: float | None = None
        self._owns_rclpy = False

        if not self.rclpy.ok():
            self.rclpy.init()
            self._owns_rclpy = True

        self.node = self.rclpy.create_node("safe_corridor_front_safety")
        self.node.create_subscription(LaserScan, topic, self._on_scan, 10)
        log(
            "SAFETY",
            f"front safety enabled: topic={topic}, stop={self.stop_distance_m:.2f} m, clear={self.clear_distance_m:.2f} m, sector={sector_deg:.1f} deg",
        )

    def _on_scan(self, msg: object) -> None:
        valid_ranges: list[float] = []
        angle = msg.angle_min
        for distance in msg.ranges:
            if abs(angle) <= self.sector_half_rad and self._is_valid_range(distance, msg):
                valid_ranges.append(float(distance))
            angle += msg.angle_increment

        self.front_min_distance_m = min(valid_ranges) if valid_ranges else None

    @staticmethod
    def _is_valid_range(distance: float, msg: object) -> bool:
        if not math.isfinite(distance) or distance <= 0.0:
            return False
        if msg.range_min > 0.0 and distance < msg.range_min:
            return False
        if msg.range_max > 0.0 and distance > msg.range_max:
            return False
        return True

    def update(self, timeout_sec: float = 0.1) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def is_step_allowed(self) -> bool:
        self.update()
        if self.front_min_distance_m is None:
            log("SAFETY", "front_min=unavailable")
            return True

        log("SAFETY", f"front_min={self.front_min_distance_m:.2f} m")
        if self.front_min_distance_m < self.stop_distance_m:
            log("SAFETY", "front obstacle detected; stopping mission steps")
            return False

        if self.front_min_distance_m >= self.clear_distance_m:
            log("SAFETY", "front clear")
        return True

    def close(self) -> None:
        self.node.destroy_node()
        if self._owns_rclpy:
            self.rclpy.shutdown()


async def wait_for_connection(drone: object, timeout_sec: float) -> None:
    log("CONNECT", f"waiting for PX4 connection, timeout={timeout_sec:.1f}s")

    async def _wait() -> None:
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    await asyncio.wait_for(_wait(), timeout=timeout_sec)
    log("CONNECT", "PX4 connection established")


async def wait_for_health(drone: object, timeout_sec: float) -> None:
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


async def read_position_ned(drone: object, timeout_sec: float = 2.0) -> object | None:
    try:
        position_velocity = await asyncio.wait_for(anext(drone.telemetry.position_velocity_ned()), timeout=timeout_sec)
    except Exception as exc:
        log("STEP", f"could not read local position NED: {exc}")
        return None
    return position_velocity.position


async def read_initial_yaw_deg(drone: object) -> float:
    try:
        attitude = await asyncio.wait_for(anext(drone.telemetry.attitude_euler()), timeout=2.0)
    except Exception as exc:
        log("TAKEOFF", f"could not read initial yaw; using 0.0 deg fallback: {exc}")
        return 0.0

    yaw_deg = float(attitude.yaw_deg)
    log("TAKEOFF", f"initial yaw={yaw_deg:.1f} deg")
    return yaw_deg


async def wait_until_takeoff_altitude(
    drone: object,
    target_altitude_m: float,
    timeout_sec: float,
    tolerance_m: float,
) -> bool:
    target_altitude_m = max(0.0, target_altitude_m)
    tolerance_m = max(0.05, tolerance_m)
    log(
        "TAKEOFF",
        f"waiting for altitude target={target_altitude_m:.1f} m, tolerance={tolerance_m:.2f} m",
    )

    async def _wait() -> bool:
        last_log_time = 0.0
        async for position_velocity in drone.telemetry.position_velocity_ned():
            altitude_estimate = abs(position_velocity.position.down_m)
            now = time.monotonic()
            if now - last_log_time >= 1.0:
                log("TAKEOFF", f"current altitude estimate: {altitude_estimate:.2f} m")
                last_log_time = now
            if position_velocity.position.down_m <= -(target_altitude_m - tolerance_m):
                return True
        return False

    try:
        return await asyncio.wait_for(_wait(), timeout=timeout_sec)
    except asyncio.TimeoutError:
        log("TAKEOFF", "takeoff altitude wait timed out")
        return False
    except Exception as exc:
        log("TAKEOFF", f"takeoff altitude wait failed: {exc}")
        return False


async def try_safety_land(drone: object) -> None:
    try:
        log("LAND", "attempting safety land")
        await drone.action.land()
    except Exception as exc:
        log("LAND", f"safety land attempt failed: {exc}")


def log_step_position(tag: str, position: object | None, start_position: object | None) -> None:
    if position is None:
        log(tag, "position unavailable")
        return

    log(
        tag,
        f"current north/east/down: {position.north_m:.2f}, {position.east_m:.2f}, {position.down_m:.2f} m",
    )
    if start_position is None:
        log(tag, "estimated displacement from start unavailable")
        return

    delta_north = position.north_m - start_position.north_m
    delta_east = position.east_m - start_position.east_m
    delta_down = position.down_m - start_position.down_m
    log(
        tag,
        f"estimated displacement from start: north={delta_north:.2f} m, east={delta_east:.2f} m, down={delta_down:.2f} m",
    )


async def run_step_mission(
    drone: object,
    args: argparse.Namespace,
    velocity_type: object,
    yaw_deg: float,
    front_safety: FrontSafetyMonitor | None,
) -> None:
    step_count = max(0, args.step_count)
    duration = max(0.0, args.step_duration_seconds)
    rate_hz = max(1.0, args.step_rate_hz)
    pause_seconds = max(0.0, args.pause_between_steps)
    interval = 1.0 / rate_hz

    start_position = await read_position_ned(drone)
    if start_position is not None:
        log(
            "STEP",
            f"start north/east/down: {start_position.north_m:.2f}, {start_position.east_m:.2f}, {start_position.down_m:.2f} m",
        )

    if step_count == 0:
        log("STEP", "step_count is 0; skipping movement")
        return

    await drone.offboard.set_velocity_ned(velocity_type(0.0, 0.0, 0.0, yaw_deg))
    await drone.offboard.start()

    for step_index in range(step_count):
        tag = f"STEP {step_index + 1}/{step_count}"
        if front_safety is not None and not front_safety.is_step_allowed():
            await drone.offboard.set_velocity_ned(velocity_type(0.0, 0.0, 0.0, yaw_deg))
            break

        log(
            tag,
            f"NED velocity command: north={args.north_speed:.2f} m/s, east={args.east_speed:.2f} m/s, yaw={yaw_deg:.1f} deg",
        )

        step_start = asyncio.get_running_loop().time()
        while asyncio.get_running_loop().time() - step_start < duration:
            await drone.offboard.set_velocity_ned(velocity_type(args.north_speed, args.east_speed, 0.0, yaw_deg))
            await asyncio.sleep(interval)

        await drone.offboard.set_velocity_ned(velocity_type(0.0, 0.0, 0.0, yaw_deg))

        position = await read_position_ned(drone)
        log_step_position(tag, position, start_position)

        if pause_seconds > 0:
            log(tag, f"pausing for {pause_seconds:.1f}s")
            await asyncio.sleep(pause_seconds)

    log("STEP", "stopping offboard mode")
    try:
        await drone.offboard.stop()
    except Exception as exc:
        log("STEP", f"offboard stop failed, continuing to land: {exc}")


async def run_mission(args: argparse.Namespace) -> int:
    try:
        from mavsdk import System
        from mavsdk.offboard import VelocityNedYaw
    except ImportError as exc:
        print(f"MAVSDK import failed: {exc}")
        print("Install MAVSDK Python before running this mission.")
        return 1

    drone = System()
    land_requested = False
    front_safety = None
    front_safety_enabled = not args.disable_front_safety or args.enable_front_safety

    print("Safe corridor mission")
    print(f"system_address={args.system_address}")
    print(f"takeoff_altitude={args.takeoff_altitude:.1f} m")
    print(f"step_count={args.step_count}")
    print(f"step_duration_seconds={args.step_duration_seconds:.1f}")
    print(f"north_speed={args.north_speed:.2f} m/s")
    print(f"east_speed={args.east_speed:.2f} m/s")
    print(f"front_safety_enabled={front_safety_enabled}")
    print(f"front_stop_distance={args.front_stop_distance:.2f} m")
    print()

    try:
        if front_safety_enabled:
            front_safety = FrontSafetyMonitor(
                topic=args.front_scan_topic,
                stop_distance_m=args.front_stop_distance,
                clear_distance_m=args.front_clear_distance,
                sector_deg=args.front_sector_deg,
            )

        log("CONNECT", "creating MAVSDK connection")
        await asyncio.wait_for(drone.connect(system_address=args.system_address), timeout=args.connection_timeout)
        await wait_for_connection(drone, args.connection_timeout)
        await wait_for_health(drone, args.connection_timeout)

        log("TAKEOFF", f"setting takeoff altitude to {args.takeoff_altitude:.1f} m")
        await drone.action.set_takeoff_altitude(args.takeoff_altitude)
        log("TAKEOFF", "arming")
        await drone.action.arm()
        log("TAKEOFF", "takeoff command sent")
        await drone.action.takeoff()

        takeoff_reached = await wait_until_takeoff_altitude(
            drone,
            target_altitude_m=args.takeoff_altitude,
            timeout_sec=args.takeoff_timeout,
            tolerance_m=args.takeoff_altitude_tolerance,
        )
        if not takeoff_reached:
            log("TAKEOFF", "target altitude not reached; skipping step mission and landing")
        else:
            log("TAKEOFF", "takeoff altitude reached")
            yaw_deg = await read_initial_yaw_deg(drone)
            await run_step_mission(drone, args, VelocityNedYaw, yaw_deg, front_safety)

        log("LAND", "sending land command")
        land_requested = True
        await drone.action.land()
        log("FINISH", "safe corridor mission complete")
        return 0

    except asyncio.TimeoutError:
        if land_requested:
            log("LAND", "MAVSDK timed out after land was requested; exiting without safety-land retry")
            log("FINISH", "exiting after land request")
            return 0

        log("CONNECT", f"timed out; check PX4 SITL and MAVLink at {args.system_address}")
        await try_safety_land(drone)
        return 1
    except Exception as exc:
        if land_requested:
            if is_grpc_disconnect_error(exc):
                log("LAND", f"MAVSDK connection closed after land was requested; exiting calmly: {exc}")
            else:
                log("LAND", f"land request returned an error after it was issued; exiting calmly: {exc}")
            log("FINISH", "exiting after land request")
            return 0

        log("FINISH", f"mission failed before land request: {exc}")
        await try_safety_land(drone)
        return 1
    finally:
        if front_safety is not None:
            front_safety.close()


def main() -> int:
    args = parse_args()
    return asyncio.run(run_mission(args))


if __name__ == "__main__":
    raise SystemExit(main())
