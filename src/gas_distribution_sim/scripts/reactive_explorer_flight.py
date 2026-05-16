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
FRONT_THRESHOLD = 2.0
SIDE_THRESHOLD = 1.5
SIDE_OPEN_MAX_AVG = 6.5
SIDE_OPEN_MAX_MIN = 6.5
EMERGENCY_DIST = 0.7
FORWARD_STEP = 0.8
SMALL_TURN_DEG = 45.0
LARGE_TURN_DEG = 90.0
MIN_FRONT_AVG_FOR_NARROW_FORWARD = 2.0
NARROW_FORWARD_STEP = 0.4
MIN_ALLOWED_FORWARD_PROGRESS = -0.05
EXIT_OPEN_FRONT_MIN = 6.0
EXIT_OPEN_FRONT_AVG = 6.0
COMMIT_STEPS = 3
MAX_EXPLORE_STEPS = 24
ROTATE_WAIT_SEC = 2
MOVE_WAIT_SEC = 3
RETURN_WAIT_SEC = 3
YAW_TOWARDS_CAVE = BASE_YAW

SECTOR_SIZE = 5
FRONT_SCAN_TOPIC = '/drone/front_scan'
LEFT_SCAN_TOPIC = '/drone/left_scan'
RIGHT_SCAN_TOPIC = '/drone/right_scan'

FORWARD_EXPLORE = 'FORWARD_EXPLORE'
CHOOSE_DIRECTION = 'CHOOSE_DIRECTION'
ROTATE_TO_HEADING = 'ROTATE_TO_HEADING'
VERIFY_FRONT_CLEAR = 'VERIFY_FRONT_CLEAR'
COMMIT_STEP = 'COMMIT_STEP'
RETURN_HOME = 'RETURN_HOME'


class MultiScanMonitor(Node):
    def __init__(self):
        super().__init__('fsm_reactive_explorer_monitor')
        self.front_stats = None
        self.left_stats = None
        self.right_stats = None

        self.create_subscription(LaserScan, FRONT_SCAN_TOPIC, self.front_callback, 10)
        self.create_subscription(LaserScan, LEFT_SCAN_TOPIC, self.left_callback, 10)
        self.create_subscription(LaserScan, RIGHT_SCAN_TOPIC, self.right_callback, 10)

        self.get_logger().info(f'Ön scan dinleniyor: {FRONT_SCAN_TOPIC}')
        self.get_logger().info(f'Sol scan dinleniyor: {LEFT_SCAN_TOPIC}')
        self.get_logger().info(f'Sağ scan dinleniyor: {RIGHT_SCAN_TOPIC}')

    def front_callback(self, msg):
        ranges = self._normalize_ranges(msg.ranges, msg.range_max)
        if len(ranges) < SECTOR_SIZE * 3:
            self.get_logger().warning(
                f'Yetersiz front_scan verisi: {len(ranges)} ray geldi.'
            )
            return

        center_start = (len(ranges) - SECTOR_SIZE) // 2
        center = ranges[center_start:center_start + SECTOR_SIZE]
        center_min, center_avg = self._stats(center)
        self.front_stats = {
            'front_center_min': center_min,
            'front_center_avg': center_avg,
        }

    def left_callback(self, msg):
        ranges = self._normalize_ranges(msg.ranges, msg.range_max)
        left_min, left_avg = self._stats(ranges)
        self.left_stats = {'left_min': left_min, 'left_avg': left_avg}

    def right_callback(self, msg):
        ranges = self._normalize_ranges(msg.ranges, msg.range_max)
        right_min, right_avg = self._stats(ranges)
        self.right_stats = {'right_min': right_min, 'right_avg': right_avg}

    def has_all_data(self):
        return (
            self.front_stats is not None and
            self.left_stats is not None and
            self.right_stats is not None
        )

    def front_clear(self):
        return self.front_stats['front_center_min'] > FRONT_THRESHOLD

    def emergency_close(self):
        return self.front_stats['front_center_min'] < EMERGENCY_DIST

    def right_clear(self):
        return (
            self.right_stats['right_min'] > SIDE_THRESHOLD and
            self.right_stats['right_avg'] < SIDE_OPEN_MAX_AVG and
            self.right_stats['right_min'] < SIDE_OPEN_MAX_MIN
        )

    def left_clear(self):
        return (
            self.left_stats['left_min'] > SIDE_THRESHOLD and
            self.left_stats['left_avg'] < SIDE_OPEN_MAX_AVG and
            self.left_stats['left_min'] < SIDE_OPEN_MAX_MIN
        )

    def log_status(self):
        if not self.has_all_data():
            print("Scan verisi eksik.")
            return

        print(
            f"Front center min/avg: "
            f"{self.front_stats['front_center_min']:.2f} / "
            f"{self.front_stats['front_center_avg']:.2f} m | "
            f"Left min/avg: {self.left_stats['left_min']:.2f} / "
            f"{self.left_stats['left_avg']:.2f} m | "
            f"Right min/avg: {self.right_stats['right_min']:.2f} / "
            f"{self.right_stats['right_avg']:.2f} m | "
            f"left_clear={self.left_clear()} | "
            f"right_clear={self.right_clear()} | "
            f"Aşırı boşluk filtre eşiği: {SIDE_OPEN_MAX_AVG:.1f}"
        )

    @staticmethod
    def _normalize_ranges(raw_ranges, range_max):
        normalized = []
        for value in raw_ranges:
            if math.isinf(value):
                normalized.append(float(range_max))
            elif math.isnan(value):
                normalized.append(0.0)
            else:
                normalized.append(float(value))
        return normalized

    @staticmethod
    def _stats(values):
        if not values:
            return 0.0, 0.0
        return min(values), sum(values) / len(values)


def start_multi_scan_monitor():
    rclpy.init(args=None)
    monitor = MultiScanMonitor()
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


async def wait_for_scan_data(scan_monitor):
    print("LaserScan verileri bekleniyor...")
    for _ in range(30):
        if scan_monitor.has_all_data():
            print("LaserScan verileri hazır.")
            return True
        await asyncio.sleep(0.5)

    print("Scan verisi eksik. Güvenli geri dönüşe geçilecek.")
    return False


async def goto(drone, north, east, down=ALTITUDE, yaw_deg=YAW_TOWARDS_CAVE, wait_sec=5):
    print(f"Setpoint -> N:{north:.2f} E:{east:.2f} D:{down:.2f} Yaw:{yaw_deg:.1f}")
    await drone.offboard.set_position_ned(
        PositionNedYaw(north, east, down, yaw_deg)
    )
    await asyncio.sleep(wait_sec)


def normalize_yaw(yaw_deg):
    return yaw_deg % 360.0


def angle_diff_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def projected_step(current_x, current_y, yaw_deg, distance_m):
    yaw_rad = math.radians(yaw_deg)
    north_delta = distance_m * math.cos(yaw_rad)
    east_delta = distance_m * math.sin(yaw_rad)
    return current_x + north_delta, current_y + east_delta


def cave_axis_progress(current_x, current_y, yaw_deg):
    _, next_y = projected_step(current_x, current_y, yaw_deg, FORWARD_STEP)
    return next_y - current_y


def is_exit_direction_risk(current_x, current_y, yaw_deg, scan_monitor):
    progress = cave_axis_progress(current_x, current_y, yaw_deg)
    if progress >= MIN_ALLOWED_FORWARD_PROGRESS:
        return False

    return (
        scan_monitor.front_stats['front_center_min'] >= EXIT_OPEN_FRONT_MIN or
        scan_monitor.front_stats['front_center_avg'] >= EXIT_OPEN_FRONT_AVG
    )


async def move_forward(drone, current_x, current_y, current_yaw):
    next_x, next_y = projected_step(current_x, current_y, current_yaw, FORWARD_STEP)
    await goto(drone, next_x, next_y, ALTITUDE, current_yaw, MOVE_WAIT_SEC)
    return next_x, next_y


async def return_home(drone, path):
    print("[STATE] RETURN_HOME / GERİ DÖNÜŞ BAŞLIYOR")
    for north, east in reversed(path[1:-1]):
        await goto(drone, north, east, ALTITUDE, BASE_YAW, RETURN_WAIT_SEC)

    print("Başlangıca yakın iniş noktasına geçiliyor...")
    await goto(drone, 0.0, 0.5, ALTITUDE, BASE_YAW, 5)


async def soft_land(drone):
    print("Yumuşak iniş hazırlığı...")

    await goto(drone, 0.0, 0.5, -1.2, YAW_TOWARDS_CAVE, 4)
    await goto(drone, 0.0, 0.5, -0.9, YAW_TOWARDS_CAVE, 4)
    await goto(drone, 0.0, 0.5, -0.6, YAW_TOWARDS_CAVE, 4)
    await goto(drone, 0.0, 0.5, -0.4, YAW_TOWARDS_CAVE, 4)

    print("Offboard stop...")
    try:
        await drone.offboard.stop()
    except Exception as e:
        print(f"Offboard stop uyarısı: {e}")

    print("Landing...")
    await drone.action.land()
    await asyncio.sleep(20)


async def main():
    scan_monitor = start_multi_scan_monitor()
    drone = System()
    await drone.connect(system_address="udp://:14540")

    await wait_until_connected(drone)
    await wait_until_ready(drone)

    print("İlk setpoint gönderiliyor...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, ALTITUDE, YAW_TOWARDS_CAVE)
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
    current_x = 0.0
    current_y = 0.0
    current_yaw = BASE_YAW
    target_yaw = BASE_YAW
    chosen_direction = None
    tried_directions = []
    commit_remaining = 0
    explore_steps = 0
    state = FORWARD_EXPLORE

    try:
        print("FSM tabanlı reactive explorer başlıyor...")
        await goto(drone, current_x, current_y, ALTITUDE, current_yaw, 6)

        if not await wait_for_scan_data(scan_monitor):
            state = RETURN_HOME

        while state != RETURN_HOME:
            if explore_steps >= MAX_EXPLORE_STEPS:
                print("Maksimum keşif adımına ulaşıldı.")
                state = RETURN_HOME
                break

            if not scan_monitor.has_all_data():
                print("Scan verisi yok: güvenli geri dönüş")
                state = RETURN_HOME
                break

            scan_monitor.log_status()

            if state == FORWARD_EXPLORE:
                print("[STATE] FORWARD_EXPLORE")
                if scan_monitor.emergency_close():
                    print("ACİL YAKINLIK: geri dönüş")
                    state = RETURN_HOME
                elif is_exit_direction_risk(current_x, current_y, current_yaw, scan_monitor):
                    print("[GÜVENLİK] Mevcut yön mağara çıkışına açık, yeni yön seçilecek")
                    state = CHOOSE_DIRECTION
                elif scan_monitor.front_clear():
                    current_x, current_y = await move_forward(
                        drone, current_x, current_y, current_yaw
                    )
                    path.append((current_x, current_y))
                    explore_steps += 1
                    print(f"KEŞİF ADIMI {explore_steps}/{MAX_EXPLORE_STEPS}")
                else:
                    state = CHOOSE_DIRECTION

            elif state == CHOOSE_DIRECTION:
                print("[STATE] CHOOSE_DIRECTION")
                if (
                    not scan_monitor.emergency_close() and
                    scan_monitor.front_stats['front_center_min'] <= FRONT_THRESHOLD and
                    scan_monitor.front_stats['front_center_avg'] >= MIN_FRONT_AVG_FOR_NARROW_FORWARD and
                    not is_exit_direction_risk(current_x, current_y, current_yaw, scan_monitor)
                ):
                    print("[DAR KORİDOR] Ön dar ama ortalama uygun, küçük ileri adım deneniyor")
                    next_x, next_y = projected_step(
                        current_x, current_y, current_yaw, NARROW_FORWARD_STEP
                    )
                    await goto(drone, next_x, next_y, ALTITUDE, current_yaw, MOVE_WAIT_SEC)
                    current_x, current_y = next_x, next_y
                    path.append((current_x, current_y))
                    explore_steps += 1
                    tried_directions = []
                    print(f"KEŞİF ADIMI {explore_steps}/{MAX_EXPLORE_STEPS}")
                    state = FORWARD_EXPLORE
                else:
                    candidates = [
                        ("hafif_sağ", normalize_yaw(current_yaw + SMALL_TURN_DEG)),
                        ("sağ", normalize_yaw(current_yaw + LARGE_TURN_DEG)),
                        ("hafif_sol", normalize_yaw(current_yaw - SMALL_TURN_DEG)),
                        ("sol", normalize_yaw(current_yaw - LARGE_TURN_DEG)),
                        ("base", BASE_YAW),
                    ]
                    best_candidate = None
                    best_score = None

                    for candidate_name, candidate_yaw in candidates:
                        if candidate_name in tried_directions:
                            continue
                        if is_exit_direction_risk(current_x, current_y, candidate_yaw, scan_monitor):
                            print(
                                f"[GÜVENLİK] {candidate_name} adayı mağara çıkışı riski taşıyor, elendi"
                            )
                            continue

                        progress = cave_axis_progress(current_x, current_y, candidate_yaw)
                        turn_cost = abs(angle_diff_deg(candidate_yaw, current_yaw)) / 90.0
                        score = progress * 2.0 - turn_cost
                        print(
                            f"[ADAY] {candidate_name}: yaw={candidate_yaw:.1f}, "
                            f"progress={progress:.2f}, score={score:.2f}"
                        )

                        if best_score is None or score > best_score:
                            best_score = score
                            best_candidate = (candidate_name, candidate_yaw)

                    if best_candidate is None:
                        print("[KARAR] Uygun aday yön yok: geri dönüş")
                        state = RETURN_HOME
                    else:
                        chosen_direction, target_yaw = best_candidate
                        print(
                            f"[KARAR] Aday yön seçildi: {chosen_direction}, target_yaw={target_yaw:.1f}"
                        )
                        state = ROTATE_TO_HEADING

            elif state == ROTATE_TO_HEADING:
                print(
                    f"[STATE] ROTATE_TO_HEADING target_yaw={target_yaw:.1f} "
                    f"direction={chosen_direction}"
                )
                await goto(
                    drone,
                    current_x,
                    current_y,
                    ALTITUDE,
                    target_yaw,
                    ROTATE_WAIT_SEC
                )
                current_yaw = target_yaw
                state = VERIFY_FRONT_CLEAR

            elif state == VERIFY_FRONT_CLEAR:
                print("[STATE] VERIFY_FRONT_CLEAR")
                await asyncio.sleep(1.0)
                scan_monitor.log_status()
                if scan_monitor.emergency_close():
                    print("ACİL YAKINLIK: geri dönüş")
                    state = RETURN_HOME
                else:
                    progress = cave_axis_progress(current_x, current_y, current_yaw)
                    print(f"[BİLGİ] mağara ekseni progress={progress:.2f}")
                    if is_exit_direction_risk(current_x, current_y, current_yaw, scan_monitor):
                        print("[GÜVENLİK] -Y yönü uzun açık alan gösteriyor, mağara çıkışı riski: diğer yön deneniyor")
                        tried_directions.append(chosen_direction)
                        state = CHOOSE_DIRECTION
                    elif scan_monitor.front_clear():
                        print("[DOĞRULAMA] Yeni ön açık, commit başlıyor")
                        commit_remaining = COMMIT_STEPS
                        tried_directions = []
                        state = COMMIT_STEP
                    else:
                        print("[DOĞRULAMA] Yeni ön kapalı, diğer yön deneniyor")
                        if chosen_direction in ("hafif_sağ", "hafif_sol"):
                            print("[DOĞRULAMA] 45 derece yeterli olmadı, aynı yönde 45 derece daha denenebilir")
                        tried_directions.append(chosen_direction)
                        state = CHOOSE_DIRECTION

            elif state == COMMIT_STEP:
                print(f"[STATE] COMMIT_STEP kalan={commit_remaining}")
                if scan_monitor.emergency_close():
                    print("ACİL YAKINLIK: geri dönüş")
                    state = RETURN_HOME
                elif is_exit_direction_risk(current_x, current_y, current_yaw, scan_monitor):
                    print("[GÜVENLİK] Commit sırasında mağara çıkışı riski algılandı, yeni yön seçilecek")
                    commit_remaining = 0
                    state = CHOOSE_DIRECTION
                elif scan_monitor.front_stats['front_center_min'] <= FRONT_THRESHOLD:
                    print("[COMMIT İPTAL] Ön tekrar daraldı, yeni yön seçilecek")
                    commit_remaining = 0
                    state = CHOOSE_DIRECTION
                else:
                    current_x, current_y = await move_forward(
                        drone, current_x, current_y, current_yaw
                    )
                    path.append((current_x, current_y))
                    explore_steps += 1
                    commit_remaining -= 1
                    print(f"KEŞİF ADIMI {explore_steps}/{MAX_EXPLORE_STEPS}")
                    if commit_remaining <= 0:
                        state = FORWARD_EXPLORE

        await return_home(drone, path)

    finally:
        await soft_land(drone)
        scan_monitor.destroy_node()
        rclpy.shutdown()
        print("Reactive explorer uçuşu tamamlandı.")


if __name__ == "__main__":
    asyncio.run(main())
