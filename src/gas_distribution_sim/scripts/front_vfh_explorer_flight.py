#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import math
import threading

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


ALTITUDE = -1.5
BASE_YAW = 90.0
EMERGENCY_DIST = 0.7
MIN_CLEARANCE = 2.0
FRONT_THRESHOLD = 2.0
ROTATE_ONLY_FRONT_MIN = 2.0
ROTATE_ONLY_WAIT_SEC = 1.5
STRAIGHT_CLEAR_FRONT_MIN = 3.0
BASE_YAW_CORRECTION_STEP_DEG = 8.0
NEAR_OBSTACLE_DIST = 1.0
RECOVERY_STEP = 0.25
RECOVERY_WAIT_SEC = 1.0
SIDE_ESCAPE_MIN_STEP = 0.35
SIDE_ESCAPE_MAX_STEP = 1.20
SIDE_ESCAPE_CLEARANCE = 1.8
SIDE_ESCAPE_MARGIN = 1.2
RECOVERY_TARGET_FRONT_MIN = 2.2
MAX_ROTATE_ONLY_ATTEMPTS = 2
ESCAPE_WAIT_SEC = 1.5
OPEN_VOID_LIMIT = 5.0
SATURATION_RANGE = 3.5
MAX_SCAN_ANGLE_DEG = 45.0
MAX_TURN_PER_STEP_DEG = 25.0
FORWARD_STEP = 0.7
NARROW_STEP = 0.35
MAX_STEPS = 24
WINDOW_RADIUS = 4
MOVE_WAIT_SEC = 2.0
RETURN_WAIT_SEC = 2.0
MIN_ACCEPTABLE_SCORE = 4.0

FRONT_SCAN_TOPIC = '/drone/front_scan'
LEFT_SCAN_TOPIC = '/drone/left_scan'
RIGHT_SCAN_TOPIC = '/drone/right_scan'


class ScanMonitor(Node):
    def __init__(self):
        super().__init__('front_vfh_scan_monitor')
        self.front_scan = None
        self.left_scan = None
        self.right_scan = None
        self.create_subscription(LaserScan, FRONT_SCAN_TOPIC, self.front_scan_callback, 10)
        self.create_subscription(LaserScan, LEFT_SCAN_TOPIC, self.left_scan_callback, 10)
        self.create_subscription(LaserScan, RIGHT_SCAN_TOPIC, self.right_scan_callback, 10)
        self.get_logger().info(f'Ön laser topic dinleniyor: {FRONT_SCAN_TOPIC}')
        self.get_logger().info(f'Sol laser topic dinleniyor: {LEFT_SCAN_TOPIC}')
        self.get_logger().info(f'Sağ laser topic dinleniyor: {RIGHT_SCAN_TOPIC}')

    def front_scan_callback(self, msg):
        self.front_scan = msg

    def left_scan_callback(self, msg):
        self.left_scan = msg

    def right_scan_callback(self, msg):
        self.right_scan = msg


def start_scan_monitor():
    rclpy.init(args=None)
    monitor = ScanMonitor()
    spin_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    spin_thread.start()
    return monitor


async def wait_until_connected(drone):
    print("Bağlantı bekleniyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Bağlandı.")
            return


async def wait_until_ready(drone):
    print("Local position bekleniyor...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("Hazır.")
            return


async def wait_for_scan(scan_monitor):
    print("Front scan verisi bekleniyor...")
    for _ in range(30):
        if scan_monitor.front_scan is not None:
            print("Front scan hazır.")
            return True
        await asyncio.sleep(0.5)

    print("Front scan verisi gelmedi.")
    return False


async def goto(drone, north, east, down=ALTITUDE, yaw_deg=BASE_YAW, wait_sec=5):
    print(f"Setpoint -> N:{north:.2f} E:{east:.2f} D:{down:.2f} Yaw:{yaw_deg:.1f}")
    await drone.offboard.set_position_ned(
        PositionNedYaw(north, east, down, yaw_deg)
    )
    await asyncio.sleep(wait_sec)


def normalize_yaw(yaw_deg):
    return yaw_deg % 360.0


def angle_diff_deg(target_yaw, current_yaw):
    return (target_yaw - current_yaw + 180.0) % 360.0 - 180.0


def base_yaw_corrected(current_yaw):
    diff = angle_diff_deg(BASE_YAW, current_yaw)
    correction = clamp(
        diff,
        -BASE_YAW_CORRECTION_STEP_DEG,
        BASE_YAW_CORRECTION_STEP_DEG
    )
    return normalize_yaw(current_yaw + correction)


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def side_escape_step(side_min):
    usable = side_min - SIDE_ESCAPE_MARGIN
    return clamp(usable, SIDE_ESCAPE_MIN_STEP, SIDE_ESCAPE_MAX_STEP)


def projected_step(current_north, current_east, yaw_deg, distance_m):
    yaw_rad = math.radians(yaw_deg)
    next_north = current_north + distance_m * math.cos(yaw_rad)
    next_east = current_east + distance_m * math.sin(yaw_rad)
    return next_north, next_east


def normalize_ranges(msg):
    ranges = []
    for value in msg.ranges:
        if math.isinf(value):
            ranges.append(float(msg.range_max))
        elif math.isnan(value):
            ranges.append(0.0)
        else:
            ranges.append(float(value))
    return ranges


def scan_min(msg):
    if msg is None:
        return 0.0

    values = []
    for value in msg.ranges:
        if math.isinf(value):
            values.append(float(msg.range_max))
        elif math.isnan(value):
            continue
        else:
            values.append(float(value))

    return min(values) if values else 0.0


def calculate_ray_score(angle_deg, raw_range, window_min, window_avg):
    if raw_range < EMERGENCY_DIST:
        return -10000.0
    score = (
        4.0 * min(window_min, SATURATION_RANGE) +
        1.8 * min(window_avg, SATURATION_RANGE) +
        3.0 * math.cos(math.radians(angle_deg)) -
        1.5 * (abs(angle_deg) / MAX_SCAN_ANGLE_DEG) -
        4.0 * max(0.0, raw_range - OPEN_VOID_LIMIT) -
        7.0 * max(0.0, MIN_CLEARANCE - window_min)
    )
    return score


def center_min_from_scan(ranges):
    if len(ranges) < 7:
        return min(ranges) if ranges else 0.0

    center_start = max(0, (len(ranges) - 7) // 2)
    center = ranges[center_start:center_start + 7]
    return min(center) if center else 0.0


def near_obstacle_side(msg):
    ranges = normalize_ranges(msg)
    if len(ranges) < 3:
        return None

    midpoint = len(ranges) // 2
    left_ranges = ranges[:midpoint]
    right_ranges = ranges[midpoint:]

    left_min = min(left_ranges) if left_ranges else float(msg.range_max)
    right_min = min(right_ranges) if right_ranges else float(msg.range_max)
    center_min = center_min_from_scan(ranges)

    if (
        center_min < NEAR_OBSTACLE_DIST
        and left_min < NEAR_OBSTACLE_DIST
        and right_min < NEAR_OBSTACLE_DIST
    ):
        return "back"

    if left_min < NEAR_OBSTACLE_DIST and right_min >= left_min:
        return "right"

    if right_min < NEAR_OBSTACLE_DIST and left_min > right_min:
        return "left"

    return None


def find_best_ray(msg):
    ranges = normalize_ranges(msg)
    if len(ranges) <= WINDOW_RADIUS * 2:
        return None

    center_min = center_min_from_scan(ranges)
    best_score = None
    best_angle_deg = 0.0
    best_window_min = 0.0
    best_window_avg = 0.0

    for i in range(WINDOW_RADIUS, len(ranges) - WINDOW_RADIUS):
        angle_rad = msg.angle_min + i * msg.angle_increment
        angle_deg = math.degrees(angle_rad)
        if abs(angle_deg) > MAX_SCAN_ANGLE_DEG:
            continue

        window = ranges[i - WINDOW_RADIUS:i + WINDOW_RADIUS + 1]
        raw_range = ranges[i]
        window_min = min(window)
        window_avg = sum(window) / len(window)
        score = calculate_ray_score(angle_deg, raw_range, window_min, window_avg)

        if best_score is None or score > best_score:
            best_score = score
            best_angle_deg = angle_deg
            best_window_min = window_min
            best_window_avg = window_avg

    if best_score is None:
        return None

    return center_min, best_angle_deg, best_score, best_window_min, best_window_avg


async def return_home(drone, path):
    print("[VFH] Geri dönüş başlıyor")
    for north, east in reversed(path[1:-1]):
        await goto(drone, north, east, ALTITUDE, BASE_YAW, RETURN_WAIT_SEC)

    print("[VFH] Başlangıca yakın iniş noktasına geçiliyor")
    await goto(drone, 0.0, 0.5, ALTITUDE, BASE_YAW, 5)


async def soft_land(drone):
    print("Yumuşak iniş hazırlığı...")

    await goto(drone, 0.0, 0.5, -1.2, BASE_YAW, 4)
    await goto(drone, 0.0, 0.5, -0.9, BASE_YAW, 4)
    await goto(drone, 0.0, 0.5, -0.6, BASE_YAW, 4)
    await goto(drone, 0.0, 0.5, -0.4, BASE_YAW, 4)

    print("Offboard stop...")
    try:
        await drone.offboard.stop()
    except Exception as e:
        print(f"Offboard stop uyarısı: {e}")

    print("Landing...")
    await drone.action.land()
    await asyncio.sleep(20)


async def main():
    scan_monitor = start_scan_monitor()
    drone = System()
    await drone.connect(system_address="udp://:14540")

    await wait_until_connected(drone)
    await wait_until_ready(drone)

    print("İlk setpoint gönderiliyor...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, ALTITUDE, BASE_YAW)
    )

    print("Arm...")
    await drone.action.arm()

    print("Offboard start...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard başlatılamadı: {e._result.result}")
        await drone.action.disarm()
        scan_monitor.destroy_node()
        rclpy.shutdown()
        return

    path = [(0.0, 0.0)]
    current_north = 0.0
    current_east = 0.0
    current_yaw = BASE_YAW
    rotate_only_count = 0
    recovery_followup = False
    last_escape_side = None

    try:
        print("[VFH] Front-Laser Corridor Centerline Explorer başlıyor")
        await goto(drone, current_north, current_east, ALTITUDE, current_yaw, 6)

        if not await wait_for_scan(scan_monitor):
            print("[VFH] Güvenli yön bulunamadı, geri dönüş başlıyor")
        else:
            for step_index in range(1, MAX_STEPS + 1):
                latest_scan = scan_monitor.front_scan
                ranges = normalize_ranges(latest_scan)
                center_min = center_min_from_scan(ranges)

                if center_min < EMERGENCY_DIST:
                    print("[VFH] Acil yakınlık, geri dönüş başlıyor")
                    break

                recovery_direction = near_obstacle_side(latest_scan)
                if recovery_direction is not None:
                    if recovery_direction == "back":
                        print("[RECOVERY] Çok yakın engel algılandı, güvenli geri dönüş başlıyor")
                        break

                    yaw_delta = 15.0 if recovery_direction == "right" else -15.0
                    target_yaw = normalize_yaw(current_yaw + yaw_delta)
                    print(
                        f"[VFH] Adım {step_index}/{MAX_STEPS} | "
                        f"mode=RECOVERY | center_min={center_min:.2f} | "
                        f"target_yaw={target_yaw:.1f} | step=0.00"
                    )
                    print("[RECOVERY] Çok yakın engel algılandı, VFH atlandı, küçük kaçış yapılıyor")
                    await goto(
                        drone,
                        current_north,
                        current_east,
                        ALTITUDE,
                        target_yaw,
                        RECOVERY_WAIT_SEC
                    )
                    current_yaw = target_yaw
                    continue

                if center_min >= STRAIGHT_CLEAR_FRONT_MIN:
                    target_yaw = base_yaw_corrected(current_yaw)
                    step = FORWARD_STEP
                    next_north, next_east = projected_step(
                        current_north, current_east, target_yaw, step
                    )

                    print(
                        f"[VFH] Adım {step_index}/{MAX_STEPS} | "
                        f"mode=STRAIGHT | center_min={center_min:.2f} | "
                        f"target_yaw={target_yaw:.1f} | step={step:.2f}"
                    )

                    await goto(drone, next_north, next_east, ALTITUDE, target_yaw, MOVE_WAIT_SEC)
                    current_north = next_north
                    current_east = next_east
                    current_yaw = target_yaw
                    rotate_only_count = 0
                    recovery_followup = False
                    last_escape_side = None
                    path.append((current_north, current_east))
                    continue

                result = find_best_ray(latest_scan)
                if result is None:
                    print("[VFH] Güvenli yön bulunamadı, geri dönüş başlıyor")
                    break

                center_min, best_angle_deg, best_score, best_window_min, best_window_avg = result
                if best_score < MIN_ACCEPTABLE_SCORE:
                    print("[VFH] Güvenli yön bulunamadı, geri dönüş başlıyor")
                    break

                clamped_angle = clamp(
                    best_angle_deg,
                    -MAX_TURN_PER_STEP_DEG,
                    MAX_TURN_PER_STEP_DEG
                )
                target_yaw = normalize_yaw(current_yaw + clamped_angle)
                if (
                    center_min < ROTATE_ONLY_FRONT_MIN
                    or (recovery_followup and center_min < RECOVERY_TARGET_FRONT_MIN)
                ):
                    rotate_only_count += 1
                    print(
                        f"[VFH] Adım {step_index}/{MAX_STEPS} | "
                        "mode=ROTATE_ONLY | "
                        f"center_min={center_min:.2f} | "
                        f"best_angle={best_angle_deg:.1f} | "
                        f"score={best_score:.1f} | "
                        f"win_min={best_window_min:.2f} | "
                        f"win_avg={best_window_avg:.2f} | "
                        f"target_yaw={target_yaw:.1f} | "
                        f"step=0.00 | rotate_only={rotate_only_count}"
                    )

                    should_try_side_escape = (
                        rotate_only_count > MAX_ROTATE_ONLY_ATTEMPTS
                        or (recovery_followup and center_min < RECOVERY_TARGET_FRONT_MIN)
                    )

                    if not should_try_side_escape:
                        print("[VFH] Ön dar: ilerleme yok, sadece yaw hizalama yapılıyor")
                        await goto(
                            drone,
                            current_north,
                            current_east,
                            ALTITUDE,
                            target_yaw,
                            ROTATE_ONLY_WAIT_SEC
                        )
                        current_yaw = target_yaw
                        continue

                    right_min = scan_min(scan_monitor.right_scan)
                    left_min = scan_min(scan_monitor.left_scan)
                    right_clear = (
                        scan_monitor.right_scan is not None
                        and right_min > SIDE_ESCAPE_CLEARANCE
                    )
                    left_clear = (
                        scan_monitor.left_scan is not None
                        and left_min > SIDE_ESCAPE_CLEARANCE
                    )
                    print(
                        f"[ESCAPE] Rotate-only limiti aşıldı | "
                        f"right_min={right_min:.2f} | left_min={left_min:.2f}"
                    )

                    chosen_side = None
                    chosen_side_min = 0.0
                    if recovery_followup and last_escape_side == "right" and right_clear:
                        chosen_side = "right"
                        chosen_side_min = right_min
                    elif recovery_followup and last_escape_side == "left" and left_clear:
                        chosen_side = "left"
                        chosen_side_min = left_min
                    elif right_clear and left_clear:
                        if right_min >= left_min:
                            chosen_side = "right"
                            chosen_side_min = right_min
                        else:
                            chosen_side = "left"
                            chosen_side_min = left_min
                    elif right_clear:
                        chosen_side = "right"
                        chosen_side_min = right_min
                    elif left_clear:
                        chosen_side = "left"
                        chosen_side_min = left_min

                    if chosen_side == "right":
                        side_step = side_escape_step(chosen_side_min)
                        print(f"[ESCAPE] Sağ taraf güvenli, dinamik yanal kaçış: step={side_step:.2f}")
                        side_yaw = normalize_yaw(current_yaw + 90.0)
                    elif chosen_side == "left":
                        side_step = side_escape_step(chosen_side_min)
                        print(f"[ESCAPE] Sol taraf güvenli, dinamik yanal kaçış: step={side_step:.2f}")
                        side_yaw = normalize_yaw(current_yaw - 90.0)
                    else:
                        print("[ESCAPE] Sağ/sol kaçış güvenli değil, geri dönüş")
                        break

                    next_north, next_east = projected_step(
                        current_north, current_east, side_yaw, side_step
                    )
                    await goto(
                        drone,
                        next_north,
                        next_east,
                        ALTITUDE,
                        current_yaw,
                        ESCAPE_WAIT_SEC
                    )
                    current_north = next_north
                    current_east = next_east
                    rotate_only_count = 0
                    recovery_followup = True
                    last_escape_side = chosen_side
                    path.append((current_north, current_east))
                    continue

                step = FORWARD_STEP
                next_north, next_east = projected_step(
                    current_north, current_east, target_yaw, step
                )

                print(
                    f"[VFH] Adım {step_index}/{MAX_STEPS} | "
                    "mode=VFH | "
                    f"center_min={center_min:.2f} | "
                    f"best_angle={best_angle_deg:.1f} | "
                    f"score={best_score:.1f} | "
                    f"win_min={best_window_min:.2f} | "
                    f"win_avg={best_window_avg:.2f} | "
                    f"target_yaw={target_yaw:.1f} | "
                    f"step={step:.2f}"
                )

                await goto(drone, next_north, next_east, ALTITUDE, target_yaw, MOVE_WAIT_SEC)
                current_north = next_north
                current_east = next_east
                current_yaw = target_yaw
                rotate_only_count = 0
                recovery_followup = False
                last_escape_side = None
                path.append((current_north, current_east))
            else:
                print("[VFH] Maksimum adım tamamlandı, geri dönüş başlıyor")

        await return_home(drone, path)

    finally:
        await soft_land(drone)
        scan_monitor.destroy_node()
        rclpy.shutdown()
        print("[VFH] Uçuş tamamlandı.")


if __name__ == "__main__":
    asyncio.run(main())
