# Proposal vs Implementation Traceability

## Purpose

This document maps the original proposal scope to the implemented thesis/demo
system. It is intended to keep the thesis wording accurate and to avoid
overclaiming beyond the final validated pipeline.

## Summary

The final system validates an autonomous indoor drone gas inspection workflow in
simulation. It uses PX4/Gazebo, MAVSDK, ROS2 LaserScan inputs, room-level gas
measurements, event JSON, CSV export, sparse 2D heatmaps, sparse 3D
visualization, experiment comparison figures, return-home, and landing
stabilization.

The system does not claim full SLAM, topology mapping, continuous gas mapping,
environment reconstruction, active camera-based perception, or general obstacle
avoidance.

## Traceability Table

| Planned | Status | Implemented | Evidence | Thesis wording |
|---|---|---|---|---|
| ROS2 Humble | implemented | ROS2 is used for simulation sensor topics and helper nodes. | `front_scan`, `left_scan`, `right_scan`, decision scan topics; `rclpy` nodes. | ROS2 Humble, simulasyon sensör verilerinin aktarımı ve yardımcı düğümler için kullanılmıştır. |
| Gazebo Classic | implemented | Indoor world and drone model are simulated in Gazebo Classic. | `simple_corridor_room.world`, `iris_cave_range/model.sdf`, `baslat.sh`. | Kapalı ortam ve drone modeli Gazebo Classic üzerinde simüle edilmiştir. |
| MAVSDK | implemented | Autonomous position-mode mission commands are sent through MAVSDK. | `src/opening_based_gas_survey_mission.py`, room inspection, return-home, landing. | Otonom uçuş komutları MAVSDK üzerinden PX4'e gönderilmiştir. |
| PX4 | implemented | PX4 SITL is used as the flight control stack. | PX4 startup in `baslat.sh`; local position/yaw/offboard behavior in mission logs. | PX4 SITL uçuş kontrol altyapısı olarak kullanılmıştır. |
| Python | implemented | Main mission logic and post-processing tools are written in Python. | Mission, event builder, summary, CSV exporter, heatmap, 3D plot, comparison plot. | Ana görev mantığı ve veri işleme Python ile geliştirilmiştir. |
| C++ | partial | C++ exists in Gazebo plugin infrastructure, but the final evaluation pipeline is Python-heavy. | `src/gas_distribution_sim/src/gas_field_plugin.cpp`. | Gazebo eklenti altyapısında C++ bileşenler bulunmakla birlikte final deney akışı Python ağırlıklıdır. |
| Kamera | partial | Camera/visual observation infrastructure exists in the world, but it was not used as an active sensing input in the final evaluation pipeline. | `simple_corridor_room.world` user camera; no active camera processing in final mission. | Kamera, simülasyon/görsel gözlem altyapısı olarak mevcuttur; final değerlendirme pipeline'ında aktif algılama girdisi olarak kullanılmamıştır. |
| LIDAR | implemented | LaserScan data drives opening detection, room traversal, and safety stopping. | `front/left/right_scan`, `front/left/right_decision_scan`, opening detection, `front_min` stop. | LIDAR verisi opening detection, room traversal ve güvenli durma kararlarında aktif kullanılmıştır. |
| IMU | partial | IMU is part of PX4/Gazebo flight estimation, but the Python mission does not directly process IMU samples. | PX4/Gazebo model and flight stack; no direct mission-level IMU algorithm. | IMU, PX4 state estimation/uçuş stabilizasyon altyapısında dolaylı kullanılmıştır. |
| Gaz sensörü | implemented | Simulated gas measurements affect event metrics and result figures. | baseline/inspection ppm, `delta_ppm`, event JSON, summary, CSV, heatmap. | Simüle gaz ölçümleri oda seviyesinde kaydedilmiş ve sonuç görselleştirmelerine aktarılmıştır. |
| 2B veri işleme | implemented | Room-level CSV samples are visualized as sparse 2D inspection heatmaps. | `opening_room_samples.csv`, `opening_event_summary_heatmap.png`. | Oda seviyesindeki event-average gaz ölçümleri 2B sparse görsellerle sunulmuştur. |
| 3B veri işleme/görselleştirme | partial | Sparse 3D visualization is generated for presentation support; it is not a continuous 3D map. | `src/opening_summary_3d_plot.py`, appendix 3D sparse figures. | 3B çıktı, seyrek oda ölçümlerinin görsel destek amaçlı sunumudur; sürekli 3B gaz haritası değildir. |
| OpenCV | future_work | OpenCV is not used in the final visualization pipeline. | Final figures are generated with Matplotlib scripts. | Final sistemde görselleştirme Matplotlib ile yapılmıştır; OpenCV bu sürümde kullanılmamıştır. |
| Matplotlib | implemented | Matplotlib is used for result visualization. | `opening_summary_heatmap.py`, `opening_summary_3d_plot.py`, `experiment_comparison_plot.py`. | Deney sonuçlarının görselleştirilmesinde Matplotlib kullanılmıştır. |
| ROS2 topic tabanlı veri aktarımı | implemented | LIDAR data is consumed through ROS2 topic infrastructure. | scan topics listed in README/cookbook and used by the mission. | LIDAR verileri ROS2 topic altyapısı üzerinden alınmıştır. |
| Otonom uçuş testi | implemented | The autonomous workflow is validated in S1/S2/S3 experiments. | inspect-all, no revisit, room-facing entry, return-home, landing; `experiment_summary.csv`. | Otonom görev akışı simülasyonda S1/S2/S3 deneyleriyle doğrulanmıştır. |
| Gaz yoğunluk haritalama | partial | The result is room-level sparse event-average visualization. | S1/S2/S3 sparse heatmaps and summary CSV. | Bu çalışma oda seviyesinde room-level sparse event-average visualization üretmektedir. |
| Obstacle avoidance | future_work | General detour-style obstacle avoidance is not implemented; front-stop and room safety checks are implemented. | `front_min` stop, traversal safety; no global/local obstacle avoidance planner. | Genel engelden kaçınma bu çalışmada kapsam dışı bırakılmış, güvenli durma ve oda geçiş kontrolleri uygulanmıştır. |

## Sensor-Specific Notes

- **Camera:** Available for simulation visualization/observation, but not used as
  an active sensing input in the final evaluation pipeline.
- **LIDAR:** Central active sensing input for opening detection, room traversal,
  and front-distance stopping.
- **IMU:** Used indirectly through PX4/Gazebo state estimation and stabilization;
  not directly consumed by mission-layer Python logic.
- **Gas sensor:** Directly contributes to baseline ppm, inspection ppm,
  `delta_ppm`, summaries, CSV samples, heatmaps, and experiment comparison.

## Mapping and Visualization Notes

The final mapping output should be described as **room-level sparse
event-average visualization**. It uses one event-average sample per completed
room inspection. It is appropriate for thesis/demo result visualization, but it
is not continuous gas mapping and not environment reconstruction.

## Avoided Overclaims

Do not claim:

- camera was used as an active sensing input in the final evaluation pipeline
- OpenCV was used in the final result pipeline
- continuous gas mapping
- environment reconstruction
- continuous 3D gas concentration mapping
- full SLAM or topology mapping
- general obstacle avoidance

Use:

- camera infrastructure was available for simulation/visual observation
- final visualization was generated with Matplotlib
- room-level sparse event-average visualization
- LIDAR-based opening detection and safe stopping
- general obstacle avoidance is future work

## Future Work

- active camera perception or camera-based validation
- OpenCV-based image processing, if needed by a later scope
- continuous gas sampling and denser mapping
- general obstacle avoidance and detour recovery
- SLAM/topology mapping or global path planning
