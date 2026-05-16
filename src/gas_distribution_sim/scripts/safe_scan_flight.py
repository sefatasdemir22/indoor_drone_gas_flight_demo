#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError


ALTITUDE = -1.5  # NED: negative means up
YAW_TOWARDS_CAVE = 90.0  # +Y yönüne bak


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


async def goto(drone, north, east, down=ALTITUDE, yaw_deg=0.0, wait_sec=6):
    print(f"Setpoint → N:{north:.2f} E:{east:.2f} D:{down:.2f} Yaw:{yaw_deg:.1f}")
    await drone.offboard.set_position_ned(
        PositionNedYaw(north, east, down, yaw_deg)
    )
    await asyncio.sleep(wait_sec)


async def main():
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
        return

    try:
        print("Görev odaklı güvenli +Y mağara uçuşu başlıyor...")

        # 1) Kalkış / stabil hover
        await goto(drone, 0.0, 0.0, ALTITUDE, YAW_TOWARDS_CAVE, 6)

        # FAZ 1: Yeni spawn noktasından kısa içeri ilerleme
        print("FAZ 1: Kısa içeri ilerleme...")
        approach_points = [
            (0.0, 1.5, 6),
            (0.0, 3.0, 6),
        ]

        for north, east, wait_sec in approach_points:
            await goto(drone, north, east, ALTITUDE, YAW_TOWARDS_CAVE, wait_sec)

        # FAZ 2: Mağara içinde kısa X sağ-sol taraması
        print("FAZ 2: Mağara içi kısa koridor taraması...")
        corridor_scan_points = []
        for y in [4.0, 5.0]:
            corridor_scan_points.extend([
                (0.0, y, 6),
                (0.4, y, 4),
                (-0.4, y, 4),
                (0.0, y, 4),
            ])

        for north, east, wait_sec in corridor_scan_points:
            await goto(drone, north, east, ALTITUDE, YAW_TOWARDS_CAVE, wait_sec)

        # Mapper'ın son verileri yazması için kısa hover
        print("Tarama sonu hover...")
        await asyncio.sleep(5)

        # FAZ 3: Merkez hat üzerinden güvenli geri dönüş
        print("FAZ 3: Merkez hattan güvenli dönüş...")
        return_points = [
            (0.0, 3.0, 6),
            (0.0, 1.5, 6),
            (0.0, 0.5, 6),
        ]

        for north, east, wait_sec in return_points:
            await goto(drone, north, east, ALTITUDE, YAW_TOWARDS_CAVE, wait_sec)

    finally:
        print("Yumuşak iniş hazırlığı...")

        # Aynı noktada, daha uzun ve yumuşak kademeli alçalma
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

        print("Safe scan tamamlandı.")


if __name__ == "__main__":
    asyncio.run(main())
