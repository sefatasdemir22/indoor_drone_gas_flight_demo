# Indoor Drone Gas Flight Demo

PX4/Gazebo tabanlı bu demo, GPS olmayan kapalı ortamda drone'un koridorda ilerlemesini, oda/açıklık adaylarını algılamasını, oda içine yönelip gaz örneği almasını ve koridora geri dönmesini gösterir.

Mevcut ana checkpoint:

- `133f89f Add room-facing yaw interpolation`
- Ana görev scripti: `src/opening_based_gas_survey_mission.py`
- Ana çıktı: `results/opening_inspection_events.json`

Bu proje tam SLAM, global planner veya tam bilinmeyen ortam keşfi iddiası taşımaz. Mevcut hedef, sınırlı corridor-room dünyasında güvenli ve açıklık tabanlı oda inspection akışını doğrulamaktır.

## Kullanılan Teknolojiler

- ROS2 Humble
- Gazebo Classic
- PX4 SITL
- MicroXRCEAgent
- MAVSDK
- Python

## Mevcut Çalışan Akış

Ana room-inspection akışı şu davranışları içerir:

- opening candidate detection
- no-backtrack door capture
- door forward offset
- room-facing yaw entry
- yaw interpolation
- post-yaw realignment
- front-distance stop
- gas baseline ve inspection ölçümü
- incremental room exit
- corridor continuation
- `max_inspections` policy

Drone oda koordinatlarını önceden bilmez. Açıklık adayları LaserScan tabanlı karar mantığıyla algılanır. Oda içine girişte drone yaw değiştirerek odaya kendi front yönüyle girer; oda içinde durma kararı `front_min` mesafesine dayanır.

## Başlatma

Terminal 1:

```bash
cd /home/sefa/Desktop/indoor_drone_gas_flight_demo
./baslat.sh
```

Bu komut Gazebo GUI, `simple_corridor_room.world`, PX4 SITL ve iris spawn akışını başlatır.

Topic kontrolü için:

```bash
ros2 topic list | grep -Ei "front_scan|left_scan|right_scan|decision_scan|vehicle_local_position"
```

## Ana Room-Facing Mission Komutu

Terminal 2:

```bash
cd /home/sefa/Desktop/indoor_drone_gas_flight_demo
python3 src/opening_based_gas_survey_mission.py \
  --position-room-inspection-check \
  --enable-no-backtrack-door-capture \
  --enable-room-facing-yaw-entry \
  --enable-room-facing-post-yaw-realign \
  --position-step-count 30 \
  --position-forward-step 0.4 \
  --position-hold-seconds 2.5 \
  --position-altitude 1.2 \
  --position-yaw 90.0 \
  --room-facing-step-distance 0.25 \
  --room-facing-max-distance 8.0 \
  --room-facing-front-stop-distance 1.5 \
  --room-facing-exit-step-distance 0.5 \
  --room-facing-step-hold-seconds 0.8 \
  --room-facing-door-forward-offset 0.25 \
  --room-facing-yaw-hold-before-seconds 0.5 \
  --room-facing-yaw-hold-after-seconds 1.0 \
  --room-facing-yaw-settle-repeat-count 5 \
  --room-facing-yaw-settle-repeat-interval 0.2 \
  --room-facing-yaw-interpolation-step-deg 15 \
  --room-facing-yaw-interpolation-hold-seconds 0.2 \
  --room-facing-post-yaw-min-front-clearance 2.0 \
  --room-facing-post-yaw-forward-offset-step 0.10 \
  --room-facing-post-yaw-max-forward-offset 0.30 \
  --door-capture-confirm-frames 3 \
  --door-capture-crawl-step 0.15 \
  --door-capture-max-crawl-steps 2 \
  --door-capture-hold-seconds 0.8 \
  --max-inspections 1 \
  --gas-scenario possible_gas_zone_4 \
  --gas-seed 1
```

Bu komut tek inspection sonrası corridor continuation davranışını da gözlemlemek için uzun tutulmuştur. `--max-inspections 1` ikinci opening adayını intentionally inspect etmez.

## Event JSON

Mission sonucu şu dosyaya yazılır:

```text
results/opening_inspection_events.json
```

Önemli alanlar:

- `side`
- `opening_id`
- `entry_anchor_source`
- `room_traversal_mode`
- `room_traverse_stop_reason`
- `room_traverse_actual_distance_m`
- `room_facing_final_front_min_m`
- `baseline_avg_ppm`
- `inspection_avg_ppm`
- `delta_ppm`
- `gas_candidate`
- `room_facing_yaw_interpolation_step_deg`
- `room_facing_exit_steps`

Son başarılı checkpoint testlerinde `room_traversal_mode="room_facing_yaw"` ve `room_traverse_stop_reason="front_stop_distance"` üretilmiştir.

## Dry-Run Kontrolleri

PX4/Gazebo başlatmadan hızlı kontrol:

```bash
python3 -m py_compile src/opening_based_gas_survey_mission.py
python3 -m py_compile src/opening_mission_types.py
python3 -m py_compile src/opening_scan_decision.py
python3 src/opening_based_gas_survey_mission.py --dry-run --dry-run-scenario normal_corridor_side_distance
python3 src/opening_based_gas_survey_mission.py --dry-run --dry-run-scenario front_blocked_left_open
```

Beklenen:

- `normal_corridor_side_distance` -> `FOLLOW_FORWARD`
- `front_blocked_left_open` -> `BYPASS_LEFT`

## Yardımcı Gaz Görselleştirme

Gaz heatmap araçları hâlâ ayrı kullanılabilir. Bunlar drone'u kontrol etmez; CSV/JSON/PNG üretir.

```bash
./scripts/run_mapper.sh random
./scripts/run_mapper.sh no_gas
./scripts/run_mapper.sh multi_1_2
./scripts/run_mapper.sh multi_all
```

Live ROS2 gas mapping de ayrı bir yardımcı akıştır:

```bash
python3 src/live_gas_mapping_ros2.py \
  --scenario possible_gas_zone_4 \
  --duration-seconds 30 \
  --sample-rate-hz 5 \
  --seed 1 \
  --hide-source-marker \
  --route-min-altitude 0.5
```

## Legacy / Diagnostic Akışlar

Repo içinde önceki fazlardan kalan diagnostic scriptler korunur:

- `src/mission_manager.py`
- `src/safe_corridor_mission.py`
- velocity/crab-mode inspection seçenekleri

Bunlar ana demo akışı değildir. Ana güncel demo `src/opening_based_gas_survey_mission.py --position-room-inspection-check` ve room-facing yaw modudur.

## Bilinen Sınırlar

- Full SLAM yok.
- Global planner yok.
- Return-home henüz ana mission akışında yok.
- Multi-room inspection henüz checkpoint olarak alınmadı.
- Landing bounce bazı testlerde gözlendi; ayrı landing stability konusu.
- Gas candidate boolean threshold konservatif; düşük delta değerleri event JSON'da görünse de `gas_candidate=false` kalabilir.
- Red/low obstacles ve lidar geometri sınırları simülasyon dünyasına bağlıdır.

## Ek Dokümantasyon

- `docs/phase_checkpoint.md`: güncel checkpoint özeti
- `docs/demo_cookbook.md`: test ve demo komutları
- `docs/flight_demo_notes.md`: eski altyapı notları

