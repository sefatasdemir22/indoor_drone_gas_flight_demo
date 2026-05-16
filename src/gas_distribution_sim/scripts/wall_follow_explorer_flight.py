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
LEFT_ESCAPE_YAW = 45.0
YAW_TOWARDS_CAVE = BASE_YAW

FRONT_SCAN_TOPIC = '/drone/front_scan'
LEFT_SCAN_TOPIC = '/drone/left_scan'
RIGHT_SCAN_TOPIC = '/drone/right_scan'

SECTOR_SIZE = 5
MAX_EXPLORE_STEPS = 24
MAX_CONSECUTIVE_FRONT_BLOCKED = 3
INITIAL_FORWARD_STEPS = 5
MAX_RIGHT_SEARCH_OFFSET_M = 1.2

DESIRED_RIGHT_DISTANCE_M = 2.0
RIGHT_TOO_CLOSE_M = 1.2
RIGHT_TOO_FAR_M = 3.2
FRONT_BLOCKED_M = 1.8
FORWARD_STEP_M = 0.8
SIDE_STEP_M = 0.35
LEFT_ESCAPE_STEP_M = 0.5
MOVE_WAIT_SEC = 2


class MultiScanMonitor(Node):
    def __init__(self):
        super().__init__('wall_follow_multi_scan_monitor')
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
        min_value, avg_value = self._stats(center)
        self.front_stats = {'min': min_value, 'avg': avg_value}

    def left_callback(self, msg):
        ranges = self._normalize_ranges(msg.ranges, msg.range_max)
        min_value, avg_value = self._stats(ranges)
        self.left_stats = {'min': min_value, 'avg': avg_value}

    def right_callback(self, msg):
        ranges = self._normalize_ranges(msg.ranges, msg.range_max)
        min_value, avg_value = self._stats(ranges)
        self.right_stats = {'min': min_value, 'avg': avg_value}

    def has_data(self):
        return (
            self.front_stats is not None and
            self.left_stats is not None and
            self.right_stats is not None
        )

    def log_status(self):
        front = self._format_stats(self.front_stats)
        left = self._format_stats(self.left_stats)
        right = self._format_stats(self.right_stats)
        print(f"Front min/avg: {front} | Left min/avg: {left} | Right min/avg: {right}")

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

    @staticmethod
    def _format_stats(stats):
        if stats is None:
            return 'veri yok'
        return f"{stats['min']:.2f} / {stats['avg']:.2f} m"


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
        if scan_monitor.has_data():
            print("LaserScan verileri hazır.")
            return True
        await asyncio.sleep(0.5)

    print("UYARI: Tüm scan verileri gelmedi, görev kontrollü devam edecek.")
    return False


async def goto(drone, north, east, down=ALTITUDE, yaw_deg=YAW_TOWARDS_CAVE, wait_sec=5):
    print(f"Setpoint -> N:{north:.2f} E:{east:.2f} D:{down:.2f} Yaw:{yaw_deg:.1f}")
    await drone.offboard.set_position_ned(
        PositionNedYaw(north, east, down, yaw_deg)
    )
    await asyncio.sleep(wait_sec)


def forward_step(current_x, current_y, yaw_deg, distance_m):
    if yaw_deg == LEFT_ESCAPE_YAW:
        return current_x + distance_m, current_y + distance_m
    return current_x, current_y + distance_m


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
    consecutive_front_blocked = 0

    try:
        print("Wall-follow explorer deneme uçuşu başlıyor...")
        await goto(drone, current_x, current_y, ALTITUDE, current_yaw, 6)
        await wait_for_scan_data(scan_monitor)

        for step_index in range(1, MAX_EXPLORE_STEPS + 1):
            print(f"WALL FOLLOW ADIMI {step_index}/{MAX_EXPLORE_STEPS}")
            scan_monitor.log_status()

            if scan_monitor.front_stats is None or scan_monitor.right_stats is None:
                print("Karar sebebi: sensör verisi eksik, ileri kontrollü devam")
                next_x, next_y = forward_step(current_x, current_y, current_yaw, FORWARD_STEP_M)
            elif scan_monitor.front_stats['min'] < FRONT_BLOCKED_M:
                consecutive_front_blocked += 1
                print("Karar sebebi: ÖN KAPALI: sola kaçış")
                current_yaw = LEFT_ESCAPE_YAW
                next_x = current_x + LEFT_ESCAPE_STEP_M
                next_y = current_y + LEFT_ESCAPE_STEP_M
            elif step_index <= INITIAL_FORWARD_STEPS:
                consecutive_front_blocked = 0
                print("Karar sebebi: BAŞLANGIÇ MODU: önce mağaraya giriş için ileri")
                next_x, next_y = forward_step(current_x, current_y, current_yaw, FORWARD_STEP_M)
            elif scan_monitor.right_stats['min'] < RIGHT_TOO_CLOSE_M:
                consecutive_front_blocked = 0
                print("Karar sebebi: SAĞ DUVAR YAKIN: sola merkezleme")
                next_x = current_x + SIDE_STEP_M
                next_y = current_y
            elif scan_monitor.right_stats['avg'] > RIGHT_TOO_FAR_M:
                consecutive_front_blocked = 0
                if current_x <= -MAX_RIGHT_SEARCH_OFFSET_M:
                    print("Karar sebebi: Sağ arama limiti doldu, ileri devam")
                    next_x, next_y = forward_step(current_x, current_y, current_yaw, FORWARD_STEP_M)
                else:
                    print("Karar sebebi: SAĞ DUVAR UZAK: sağa yaklaşma")
                    next_x = max(current_x - SIDE_STEP_M, -MAX_RIGHT_SEARCH_OFFSET_M)
                    next_y = current_y
            else:
                consecutive_front_blocked = 0
                print("Karar sebebi: SAĞ DUVAR TAKİP: ileri")
                next_x, next_y = forward_step(current_x, current_y, current_yaw, FORWARD_STEP_M)

            await goto(drone, next_x, next_y, ALTITUDE, current_yaw, MOVE_WAIT_SEC)
            current_x, current_y = next_x, next_y
            path.append((current_x, current_y))

            if consecutive_front_blocked >= MAX_CONSECUTIVE_FRONT_BLOCKED:
                print("Üst üste 3 kez ön kapalı kaldı. Güvenli geri dönüş moduna geçiliyor.")
                break

        print("Kaydedilen path üzerinden geri dönülüyor...")
        for north, east in reversed(path[1:-1]):
            await goto(drone, north, east, ALTITUDE, BASE_YAW, 3)

        print("Başlangıca yakın iniş noktasına geçiliyor...")
        await goto(drone, 0.0, 0.5, ALTITUDE, BASE_YAW, 5)

    finally:
        await soft_land(drone)
        scan_monitor.destroy_node()
        rclpy.shutdown()
        print("Wall-follow explorer uçuşu tamamlandı.")


if __name__ == "__main__":
    asyncio.run(main())
