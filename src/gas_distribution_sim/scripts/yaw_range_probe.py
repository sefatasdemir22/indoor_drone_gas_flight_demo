#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import threading
from collections import deque

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


ALTITUDE = -1.5
HOVER_NORTH = 0.0
HOVER_EAST = 0.0
YAW_TESTS = [90.0, 45.0, 135.0, 90.0]
FRONT_RANGE_TOPIC = '/drone/front_range'


class FrontRangeBuffer(Node):
    def __init__(self):
        super().__init__('yaw_range_probe_buffer')
        self.samples = deque(maxlen=50)
        self.create_subscription(Range, FRONT_RANGE_TOPIC, self.range_callback, 10)
        self.get_logger().info(f'Ön mesafe topic dinleniyor: {FRONT_RANGE_TOPIC}')

    def range_callback(self, msg):
        self.samples.append(float(msg.range))

    def clear(self):
        self.samples.clear()

    def average_last(self, count=10):
        if not self.samples:
            return None

        values = list(self.samples)[-count:]
        return sum(values) / len(values)


def start_range_buffer():
    rclpy.init(args=None)
    node = FrontRangeBuffer()
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()
    return node, executor_thread


async def wait_until_connected(drone):
    print("PX4 bağlantısı bekleniyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("PX4 bağlantısı kuruldu.")
            return


async def wait_until_ready(drone):
    print("Local position hazır olması bekleniyor...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("Local position hazır.")
            return


async def set_hover_yaw(drone, yaw_deg, wait_sec):
    print(f"Yaw testi başlıyor: {yaw_deg:.1f} derece")
    await drone.offboard.set_position_ned(
        PositionNedYaw(HOVER_NORTH, HOVER_EAST, ALTITUDE, yaw_deg)
    )
    await asyncio.sleep(wait_sec)


async def soft_land(drone):
    print("Kademeli alçalma başlıyor...")

    await drone.offboard.set_position_ned(PositionNedYaw(HOVER_NORTH, HOVER_EAST, -1.2, 90.0))
    await asyncio.sleep(4)
    await drone.offboard.set_position_ned(PositionNedYaw(HOVER_NORTH, HOVER_EAST, -0.9, 90.0))
    await asyncio.sleep(4)
    await drone.offboard.set_position_ned(PositionNedYaw(HOVER_NORTH, HOVER_EAST, -0.6, 90.0))
    await asyncio.sleep(4)
    await drone.offboard.set_position_ned(PositionNedYaw(HOVER_NORTH, HOVER_EAST, -0.4, 90.0))
    await asyncio.sleep(4)

    print("Offboard stop...")
    try:
        await drone.offboard.stop()
    except Exception as e:
        print(f"Offboard stop uyarısı: {e}")

    print("Landing...")
    await drone.action.land()
    await asyncio.sleep(10)


async def main():
    range_node, spin_thread = start_range_buffer()

    drone = System()
    await drone.connect(system_address="udp://:14540")

    try:
        await wait_until_connected(drone)
        await wait_until_ready(drone)

        print("İlk hover setpoint gönderiliyor...")
        await drone.offboard.set_position_ned(
            PositionNedYaw(HOVER_NORTH, HOVER_EAST, ALTITUDE, 90.0)
        )

        print("Arm...")
        await drone.action.arm()

        print("Offboard start...")
        try:
            await drone.offboard.start()
        except OffboardError as e:
            print(f"Offboard başlatılamadı: {e._result.result}")
            await drone.action.disarm()
            return

        print("Yaw-range probe başlıyor. Drone pozisyonu sabit tutulacak.")
        for yaw in YAW_TESTS:
            range_node.clear()
            await set_hover_yaw(drone, yaw, 5)
            avg_range = range_node.average_last(10)

            if avg_range is None:
                print(f"Yaw {yaw:.1f} derece -> ortalama range: veri yok")
            else:
                print(f"Yaw {yaw:.1f} derece -> ortalama range: {avg_range:.2f} m")

    finally:
        await soft_land(drone)
        range_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        print("Yaw range probe tamamlandı.")


if __name__ == "__main__":
    asyncio.run(main())
