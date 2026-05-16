#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


ALTITUDE = -1.5  # NED: negatif değer yukarı anlamına gelir
CENTER = (0.0, 0.0, ALTITUDE)


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


async def goto(drone, label, north, east, down=ALTITUDE, yaw_deg=0.0, wait_sec=5):
    print(f"{label}: N={north:.2f}, E={east:.2f}, D={down:.2f}")
    await drone.offboard.set_position_ned(
        PositionNedYaw(north, east, down, yaw_deg)
    )
    await asyncio.sleep(wait_sec)


async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    await wait_until_connected(drone)
    await wait_until_ready(drone)

    print("İlk hover setpoint'i gönderiliyor...")
    await drone.offboard.set_position_ned(PositionNedYaw(*CENTER, 0.0))

    print("Drone arm ediliyor...")
    await drone.action.arm()

    print("Offboard modu başlatılıyor...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard başlatılamadı: {e._result.result}")
        await drone.action.disarm()
        return

    try:
        await goto(drone, "Başlangıç hover", 0.0, 0.0, ALTITUDE, wait_sec=6)

        probe_points = [
            ("+X yön testi", 1.5, 0.0),
            ("Merkeze dönüş", 0.0, 0.0),
            ("-X yön testi", -1.5, 0.0),
            ("Merkeze dönüş", 0.0, 0.0),
            ("+Y yön testi", 0.0, 1.5),
            ("Merkeze dönüş", 0.0, 0.0),
            ("-Y yön testi", 0.0, -1.5),
            ("Merkeze dönüş", 0.0, 0.0),
        ]

        print("Kısa eksen yön testleri başlıyor.")
        for label, north, east in probe_points:
            await goto(drone, label, north, east, ALTITUDE, wait_sec=5)

        print("Yön testi tamamlandı. Kısa stabil hover.")
        await asyncio.sleep(3)

    finally:
        print("Yumuşak iniş hazırlığı başlıyor...")

        await goto(drone, "Alçalma 1", 0.0, 0.0, -1.0, wait_sec=4)
        await goto(drone, "Alçalma 2", 0.0, 0.0, -0.6, wait_sec=4)
        await goto(drone, "Alçalma 3", 0.0, 0.0, -0.35, wait_sec=4)

        print("Offboard modu durduruluyor...")
        try:
            await drone.offboard.stop()
        except Exception as e:
            print(f"Offboard stop uyarısı: {e}")

        print("Landing komutu gönderiliyor...")
        await drone.action.land()
        await asyncio.sleep(10)

        print("Direction probe uçuşu tamamlandı.")


if __name__ == "__main__":
    asyncio.run(main())
