# Indoor Drone Gas Flight Demo

## Amaç

Bu proje, GPS olmayan kapalı ortamda PX4/Gazebo tabanlı drone uçuş demosu hazırlamak ve sonraki aşamada gaz haritalama altyapısını entegre etmek için oluşturulmuştur.

Demo, eski çalışan Araswarm PX4/Gazebo başlatma altyapısı temel alınarak daha sade bir indoor corridor-room world ile uyarlanmıştır. Bu aşamada odak, Gazebo Classic içinde PX4 iris modelini başlatmak, spawn etmek ve MAVSDK ile temel uçuş doğrulaması yapmaktır.

## Kullanılan Teknolojiler

- ROS2 Humble
- Gazebo Classic
- PX4 SITL
- MicroXRCEAgent
- MAVSDK
- Python

## Başlatma

Terminal 1:

```bash
cd /home/sefa/Desktop/indoor_drone_gas_flight_demo
./baslat.sh
```

Bu komut Gazebo GUI, `simple_corridor_room.world`, PX4 SITL ve iris spawn akışını eski çalışan Araswarm başlatma mantığıyla çalıştırır.

## MAVSDK Takeoff/Land Test

Terminal 2:

```bash
cd /home/sefa/Desktop/indoor_drone_gas_flight_demo
python3 src/mission_manager.py --mode mavsdk --takeoff-altitude 1.5 --takeoff-timeout 25 --hover-seconds 5 --connection-timeout 30
```

Bu test oda gezme veya gaz ölçümü yapmaz. Sadece çalışan PX4/Gazebo oturumuna MAVSDK ile bağlanır, güvenli kalkış, kısa hover ve iniş akışını doğrular.

## Short Move Test

Terminal 2:

```bash
cd /home/sefa/Desktop/indoor_drone_gas_flight_demo
python3 src/mission_manager.py --mode mavsdk --takeoff-altitude 1.5 --takeoff-timeout 25 --hover-seconds 5 --enable-short-move --move-forward-seconds 4 --move-forward-speed 0.5 --move-rate-hz 10 --connection-timeout 30
```

Bu test sadece kısa ve kontrollü ileri hareket içindir. Oda gezme, engelden kaçınma ve gaz ölçüm entegrasyonu sonraki aşamadadır.

## Simulated Mission FSM

PX4/Gazebo başlatmadan görev akışını terminalde simüle etmek için:

```bash
python3 src/mission_manager.py --mode sim --sleep 0
```

## Notlar

- Bu demo eski çalışan `araswarm_simulation` altyapısından sade indoor world için uyarlanmıştır.
- `baslat.sh`, MicroXRCEAgent, Gazebo launch, PX4 ve `spawn_entity.py` sırasını korur.
- Gaz haritalama/heatmap modülü bu repoya henüz taşınmamıştır; sonraki entegrasyon aşamasında eklenecektir.
