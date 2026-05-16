#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


async def wait_until_connected(drone: System):
    print("PX4 bağlantısı bekleniyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("PX4 bağlandı.")
            return


async def wait_until_local_position_ok(drone: System):
    print("Local position uygunluğu bekleniyor...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("Local position hazır.")
            return


async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    await wait_until_connected(drone)
    await wait_until_local_position_ok(drone)

    print("İlk offboard setpoint gönderiliyor...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -1.5, 0.0)
    )

    print("Arm ediliyor...")
    await drone.action.arm()

    print("Offboard başlatılıyor...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard başlatılamadı: {e._result.result}")
        print("Disarm ediliyor...")
        await drone.action.disarm()
        return

    try:
        print("Waypoint 1: hover / yükselme")
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -1.5, 0.0)
        )
        await asyncio.sleep(6)

        print("Waypoint 2: ileri")
        await drone.offboard.set_position_ned(
            PositionNedYaw(2.0, 0.0, -1.5, 0.0)
        )
        await asyncio.sleep(8)

        print("Waypoint 3: ileri + sağ")
        await drone.offboard.set_position_ned(
            PositionNedYaw(4.0, 1.5, -1.5, 0.0)
        )
        await asyncio.sleep(8)

        print("Waypoint 4: biraz daha ileri")
        await drone.offboard.set_position_ned(
            PositionNedYaw(6.0, 1.5, -1.5, 0.0)
        )
        await asyncio.sleep(8)

        print("Waypoint 5: hover")
        await drone.offboard.set_position_ned(
            PositionNedYaw(6.0, 1.5, -1.5, 0.0)
        )
        await asyncio.sleep(5)

        print("Başlangıca yakın dönüş")
        await drone.offboard.set_position_ned(
            PositionNedYaw(2.0, 0.5, -1.5, 0.0)
        )
        await asyncio.sleep(8)

    finally:
        print("Offboard durduruluyor...")
        try:
            await drone.offboard.stop()
        except Exception as e:
            print(f"Offboard stop uyarısı: {e}")

        print("Landing...")
        await drone.action.land()
        await asyncio.sleep(10)

        print("Flight script tamamlandı.")


if __name__ == "__main__":
    asyncio.run(main())
