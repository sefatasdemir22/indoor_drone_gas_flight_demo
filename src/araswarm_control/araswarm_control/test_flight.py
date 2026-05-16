#!/usr/bin/env python3
import asyncio
from mavsdk import System

async def run():
    # 1. Drone'a Bağlan
    drone = System()
    print("Drone aranıyor (Port: 14540)...")
    
    # 14550 portunu dinliyoruz (PX4 buraya kesin veri gönderir)
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Bağlantı bekleniyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Drone BAĞLANDI!")
            break

    # 2. Kalkış Öncesi Kontroller
    print("-- Arming (Motorlar çalıştırılıyor)...")
    try:
        await drone.action.arm()
    except Exception as e:
        print(f"Arming Hatası: {e}")

    # 3. Kalkış
    print("-- Kalkış yapılıyor...")
    try:
        await drone.action.takeoff()
    except Exception as e:
        print(f"Kalkış Hatası: {e}")

    # 4. Havada Bekle
    await asyncio.sleep(10)
    print("-- 10 saniye uçuş tamamlandı.")

    # 5. İniş
    print("-- İniş yapılıyor...")
    await drone.action.land()
    
    await asyncio.sleep(5)
    print("-- Görev Bitti.")

if __name__ == "__main__":
    asyncio.run(run())
