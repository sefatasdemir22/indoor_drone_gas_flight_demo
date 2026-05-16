# Indoor Drone Gas Flight Demo Notes

Bu klasor eski `araswarm_simulation` altyapisindan kopyalanmistir.

- Baslatma sirasi eski calisan `baslat.sh` mantigini korur.
- `MicroXRCEAgent`, Gazebo world, PX4 brain ve `spawn_entity.py` sirasi korunmustur.
- World dosyasi `simple_corridor_room.world` olarak degistirilmistir.
- PX4 binary baslatma mantigi korunmustur.
- Ilk asamada `iris_cave_range/model.sdf` degistirilmemistir.

Bu klasor henuz ayri bir ROS2 workspace install klasoru degildir. ROS package cozumlemesi icin mevcut `~/araswarm_ws/install/setup.bash` source edilmeye devam eder.

## MAVSDK Mission Manager

MAVSDK gorev yoneticisi artik bu klasorden calistirilabilir:

```bash
cd /home/sefa/Desktop/indoor_drone_gas_flight_demo
python3 src/mission_manager.py --mode sim --sleep 0
```

Gercek PX4/Gazebo testi icin once eski calisan baslatma mantigi baslatilir:

```bash
./baslat.sh
```

Ardindan ayri terminalde MAVSDK modu denenebilir:

```bash
python3 src/mission_manager.py --mode mavsdk --takeoff-altitude 1.5 --takeoff-timeout 25 --hover-seconds 5 --connection-timeout 30
```

## Gas Mapping / Heatmap Tools

Eski `indoor_drone_gas_demo` prototipinden gaz senaryosu, CSV uretimi ve heatmap araclari bu repoya tasindi.

- Python araclari: `src/demo_tools/`
- Calistirma scriptleri: `scripts/`
- Ornek heatmap gorselleri: `docs/media/`
- Runtime ciktilari: `results/` altinda uretilir ve Git'e alinmaz.

Bu kisim PX4/Gazebo ucusunu dogrudan kontrol etmez. Simule rota uzerinden gaz olcumu ve haritalama ciktisi uretir.
