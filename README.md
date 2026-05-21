# Indoor Drone Gas Flight Demo

PX4/Gazebo tabanli bu demo, GPS olmayan kapali ortamda drone'un koridorda ilerlemesini, gordugu farkli oda/acikliklari inspection etmesini, gaz olcumu almasini, koridora geri donmesini ve gorev sonunda local home noktasina return-home yapip yumusak inmesini gosterir.

Mevcut ana checkpoint:

- Git checkpoint: `ce86e82 Add inspect-all openings mission mode`
- Tag: `milestone_inspect_all_v1`
- Ana gorev scripti: `src/opening_based_gas_survey_mission.py`
- Ana cikti: `results/opening_inspection_events.json`

Bu proje tam SLAM, global planner veya bilinmeyen ortam topoloji haritasi iddiasi tasimaz. Mevcut hedef, sinirli corridor-room dunyasinda LaserScan tabanli aciklik algilama, coklu oda inspection, return-home ve stabil landing akisini dogrulamaktir.

## Kullanilan Teknolojiler

- ROS2 Humble
- Gazebo Classic
- PX4 SITL
- MicroXRCEAgent
- MAVSDK
- Python

## Guncel Calisan Akis

Ana position-mode room-inspection akisi su davranislari icerir:

- inspect-all openings modu
- opening candidate detection
- same-side suppression ile ayni odaya tekrar girmeme
- no-backtrack door capture
- door forward offset
- room-facing yaw entry
- yaw interpolation
- post-yaw realignment
- front-distance stop traversal
- gas baseline ve inspection olcumu
- incremental room exit
- corridor continuation
- return-home
- staged landing stabilization

Drone oda koordinatlarini onceden bilmez. Aciklik adaylari dar decision LaserScan verileriyle algilanir. Oda icine giriste drone yaw degistirerek odaya kendi front yonuyle girer; oda icinde durma karari `front_min` mesafesine dayanir.

## Baslatma

Terminal 1:

```bash
cd ~/Desktop/indoor_drone_gas_flight_demo
./baslat.sh
```

Bu komut Gazebo GUI, `simple_corridor_room.world`, PX4 SITL ve iris spawn akisini baslatir.

Topic kontrolu icin:

```bash
ros2 topic list | grep -Ei "front_scan|left_scan|right_scan|decision_scan|vehicle_local_position"
```

## Ana Inspect-All Mission Komutu

Terminal 2:

```bash
cd ~/Desktop/indoor_drone_gas_flight_demo
python3 src/opening_based_gas_survey_mission.py \
  --position-room-inspection-check \
  --inspect-all-openings \
  --enable-no-backtrack-door-capture \
  --enable-room-facing-yaw-entry \
  --enable-room-facing-post-yaw-realign \
  --enable-position-return-home \
  --enable-position-landing-stabilization \
  --room-facing-front-stop-distance 1.5 \
  --max-inspections 5 \
  --gas-scenario possible_gas_zone_4 \
  --gas-seed 1
```

Bu README komutu ana modlari gosterir. Tam validasyon parametreleri icin `docs/demo_cookbook.md` dosyasindaki "Current Inspect-All Mission" komutu kullanilmalidir.

`--inspect-all-openings` aktifken `--max-inspections` hedef oda sayisi degil, safety cap olarak kullanilir. Mevcut demo dunyasinda birden fazla farkli opening inspection edilmesi, ayni opening devamlarinin suppression ile atlanmasi, return-home'un tamamlanmasi ve staged landing'in calismasi beklenir.

## Event JSON

Mission sonucu su dosyaya yazilir:

```text
results/opening_inspection_events.json
```

Ust seviye alanlar:

- `events`
- `return_home`

Her event icin onemli alanlar:

- `inspection_index`
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

Return-home icin onemli alanlar:

- `enabled`
- `attempted`
- `status`
- `steps`
- `actual_distance_m`
- `final_distance_to_home_m`

Basarili inspect-all checkpoint testinde birden fazla valid event uretilir ve `return_home.status="completed"` beklenir.

## Dry-Run Kontrolleri

PX4/Gazebo baslatmadan hizli kontrol:

```bash
python3 -m py_compile src/opening_based_gas_survey_mission.py
python3 -m py_compile src/opening_event_builder.py
python3 -m py_compile src/opening_mission_types.py
python3 -m py_compile src/opening_scan_decision.py
python3 src/opening_based_gas_survey_mission.py --dry-run --dry-run-scenario normal_corridor_side_distance
python3 src/opening_based_gas_survey_mission.py --dry-run --dry-run-scenario front_blocked_left_open
```

Beklenen:

- `normal_corridor_side_distance` -> `FOLLOW_FORWARD`
- `front_blocked_left_open` -> `BYPASS_LEFT`

## Yardimci Gaz Gorsellestirme

Gaz heatmap araclari ayri kullanilabilir. Bunlar drone'u kontrol etmez; CSV/JSON/PNG uretir.

```bash
./scripts/run_mapper.sh random
./scripts/run_mapper.sh no_gas
./scripts/run_mapper.sh multi_1_2
./scripts/run_mapper.sh multi_all
```

Live ROS2 gas mapping de ayri bir yardimci akistir:

```bash
python3 src/live_gas_mapping_ros2.py \
  --scenario possible_gas_zone_4 \
  --duration-seconds 30 \
  --sample-rate-hz 5 \
  --seed 1 \
  --hide-source-marker \
  --route-min-altitude 0.5
```

## Legacy / Diagnostic Akislar

Repo icinde onceki fazlardan kalan diagnostic scriptler korunur:

- `src/mission_manager.py`
- `src/safe_corridor_mission.py`
- velocity/crab-mode inspection secenekleri
- `--position-side-sign-check`

Bunlar ana demo akisi degildir. Ana guncel demo `src/opening_based_gas_survey_mission.py --position-room-inspection-check --inspect-all-openings` ve room-facing yaw modudur.

## Bilinen Sinirlar

- Full SLAM yok.
- Global planner yok.
- `position_step_count` halen corridor mission siniridir.
- `max_inspections`, inspect-all modunda safety cap olarak kalir.
- Gas candidate boolean threshold konservatiftir; delta degerleri event JSON'da gorunse de `gas_candidate=false` kalabilir.
- Red/low obstacle ve lidar geometri sinirlari simulasyon dunyasina baglidir.
- Yuksek gaz bulununca gorevi erken bitirme henuz eklenmedi.

## Ek Dokumantasyon

- `docs/phase_checkpoint.md`: guncel checkpoint ozeti
- `docs/demo_cookbook.md`: test ve demo komutlari
- `docs/flight_demo_notes.md`: eski altyapi notlari
