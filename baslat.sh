#!/bin/bash

echo "🧹 Temizlik yapılıyor..."
killall -9 px4 gzserver gzclient python3 MicroXRCEAgent 2>/dev/null
sleep 2

echo "⚙️ ROS ortamı hazırlanıyor..."
ROS_SETUP="source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; source ~/araswarm_ws/install/setup.bash"

DEMO_ROOT="$HOME/Desktop/indoor_drone_gas_flight_demo"
DEMO_WORLD="$DEMO_ROOT/src/gas_distribution_sim/worlds/simple_corridor_room.world"
DEMO_IRIS_MODEL="$DEMO_ROOT/src/gas_distribution_sim/models/iris_cave_range/model.sdf"

echo "⚙️ Indoor world fizik ayarları (250 Hz) kontrol ediliyor..."
sed -i -E 's#<real_time_update_rate>[^<]+</real_time_update_rate>#<real_time_update_rate>250</real_time_update_rate>#g' "$DEMO_WORLD"
sed -i -E 's#<max_step_size>[^<]+</max_step_size>#<max_step_size>0.004</max_step_size>#g' "$DEMO_WORLD"

echo "🚀 Simülasyon başlatılıyor..."

# 0. MicroXRCEAgent
gnome-terminal --tab --title="MicroXRCEAgent" -- bash -c "$ROS_SETUP; MicroXRCEAgent udp4 -p 8888; exec bash"

sleep 3

# 1. INDOOR WORLD
gnome-terminal --tab --title="Indoor World" -- bash -c "cd '$DEMO_ROOT'; $ROS_SETUP; ros2 launch '$DEMO_ROOT/src/gas_distribution_sim/launch/cave_simulation.launch.py'; exec bash"

sleep 5

# 2. PX4 BEYNİ
gnome-terminal --tab --title="PX4 Brain" -- bash -c "cd ~/src/PX4-Autopilot/build/px4_sitl_default; export PX4_SYS_AUTOSTART=10015; ./bin/px4 -i 0 -d \"\$PWD/etc\" -w sitl_iris_0; exec bash"

sleep 5

# 3. DRONE SPAWN
gnome-terminal --tab --title="Spawn Drone" -- bash -c "source ~/src/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/src/PX4-Autopilot ~/src/PX4-Autopilot/build/px4_sitl_default; $ROS_SETUP; ros2 run gazebo_ros spawn_entity.py -entity iris_cave_range -file '$DEMO_IRIS_MODEL' -x 0.0 -y 0.0 -z 0.2; exec bash"

echo "✅ SİSTEM HAZIR! Drone mağara girişine yerleşti."
echo "📌 Manuel terminal açarsan şunu source et:"
echo "source /opt/ros/humble/setup.bash"
echo "source ~/ros2_ws/install/setup.bash"
echo "source ~/araswarm_ws/install/setup.bash"
