#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import ast
import yaml
import csv

import rclpy
from datetime import datetime
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float32
from px4_msgs.msg import VehicleLocalPosition


class GasSensorNode(Node):
    def __init__(self):
        super().__init__('gas_sensor_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        default_yaml = os.path.expanduser(
            '~/araswarm_ws/src/araswarm_simulation/src/gas_distribution_sim/config/gas_scenarios.yaml'
        )

        self.declare_parameter('scenario_yaml', default_yaml)
        self.declare_parameter('position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('scenario_topic', '/gas_distribution/active_zones')
        self.declare_parameter('ppm_topic', '/gas_sensor/ppm')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('background_ppm', 0.0)
        self.declare_parameter('default_max_ppm', 100.0)
        self.declare_parameter('sigma_scale', 2.0)   # sigma = radius / sigma_scale
        self.declare_parameter('use_3d_distance', True)
        self.declare_parameter('max_total_ppm', 5000.0)
        self.declare_parameter('enable_csv_logging', True)
        self.declare_parameter(
            'csv_path',
            os.path.expanduser('~/araswarm_ws/gas_sensor_logs/gas_measurements.csv')
        )

        self.scenario_yaml = self.get_parameter('scenario_yaml').value
        self.position_topic = self.get_parameter('position_topic').value
        self.scenario_topic = self.get_parameter('scenario_topic').value
        self.ppm_topic = self.get_parameter('ppm_topic').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.background_ppm = float(self.get_parameter('background_ppm').value)
        self.default_max_ppm = float(self.get_parameter('default_max_ppm').value)
        self.sigma_scale = float(self.get_parameter('sigma_scale').value)
        self.use_3d_distance = bool(self.get_parameter('use_3d_distance').value)
        self.max_total_ppm = float(self.get_parameter('max_total_ppm').value)
        self.enable_csv_logging = bool(self.get_parameter('enable_csv_logging').value)
        self.csv_path = self.get_parameter('csv_path').value

        # -----------------------------
        # State
        # -----------------------------
        self.current_position = None
        self.active_scenario_name = None
        self.scenarios = {}
        self.available_zones = {}
        self.csv_initialized = False

        self.load_yaml()
        if self.enable_csv_logging:
            self.init_csv()

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.ppm_pub = self.create_publisher(Float32, self.ppm_topic, 10)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            self.position_topic,
            self.position_callback,
            px4_qos
        )

        self.scenario_sub = self.create_subscription(
            String,
            self.scenario_topic,
            self.scenario_callback,
            10
        )

        timer_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.2
        self.timer = self.create_timer(timer_period, self.publish_ppm_timer)

        self.get_logger().info('GasSensorNode başlatıldı.')
        self.get_logger().info(f'Position topic : {self.position_topic}')
        self.get_logger().info(f'Scenario topic : {self.scenario_topic}')
        self.get_logger().info(f'PPM topic      : {self.ppm_topic}')
        self.get_logger().info(f'YAML file      : {self.scenario_yaml}')

    # -------------------------------------------------
    # YAML loading
    # -------------------------------------------------
    def load_yaml(self):
        if not os.path.exists(self.scenario_yaml):
            self.get_logger().error(f'YAML bulunamadı: {self.scenario_yaml}')
            return

        try:
            with open(self.scenario_yaml, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}

            self.scenarios = data.get('scenarios', {}) or {}
            self.available_zones = data.get('available_zones', {}) or {}

            self.get_logger().info(
                f'YAML yüklendi | scenarios={len(self.scenarios)} available_zones={len(self.available_zones)}'
            )

        except Exception as e:
            self.get_logger().error(f'YAML okunamadı: {e}')

    def init_csv(self):
        try:
            csv_dir = os.path.dirname(self.csv_path)
            os.makedirs(csv_dir, exist_ok=True)

            file_exists = os.path.exists(self.csv_path)

            with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                if not file_exists:
                    writer.writerow([
                        'wall_time_iso',
                        'ros_time_sec',
                        'x',
                        'y',
                        'z',
                        'ppm',
                        'scenario'
                    ])

            self.csv_initialized = True
            self.get_logger().info(f'CSV log hazır: {self.csv_path}')

        except Exception as e:
            self.csv_initialized = False
            self.get_logger().error(f'CSV başlatılamadı: {e}')

    def log_to_csv(self, ppm_value):
        if not self.enable_csv_logging or not self.csv_initialized:
            return

        if self.current_position is None:
            return

        try:
            now_msg = self.get_clock().now().to_msg()
            ros_time_sec = float(now_msg.sec) + float(now_msg.nanosec) / 1e9

            with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    datetime.now().isoformat(timespec='milliseconds'),
                    ros_time_sec,
                    self.current_position['x'],
                    self.current_position['y'],
                    self.current_position['z'],
                    float(ppm_value),
                    self.active_scenario_name if self.active_scenario_name else ''
                ])

        except Exception as e:
            self.get_logger().error(f'CSV log yazma hatası: {e}')



    # -------------------------------------------------
    # Scenario callback
    # -------------------------------------------------
    def scenario_callback(self, msg: String):
        raw = msg.data.strip()
        scenario_name = None

        # Beklenen örnek:
        # "{'scenario': 'random_hazard'}"
        try:
            parsed = ast.literal_eval(raw)
            if isinstance(parsed, dict):
                scenario_name = parsed.get('scenario')
        except Exception:
            pass

        if scenario_name is None:
            try:
                parsed_yaml = yaml.safe_load(raw)
                if isinstance(parsed_yaml, dict):
                    scenario_name = parsed_yaml.get('scenario')
                elif isinstance(parsed_yaml, str):
                    scenario_name = parsed_yaml
            except Exception:
                pass

        if scenario_name is None and raw:
            scenario_name = raw

        if scenario_name != self.active_scenario_name:
            self.active_scenario_name = scenario_name
            self.get_logger().info(f'Aktif senaryo: {self.active_scenario_name}')

    # -------------------------------------------------
    # Position callback
    # -------------------------------------------------
    def position_callback(self, msg: VehicleLocalPosition):
        # PX4 local position:
        # x = north, y = east, z = down
        # Burada dünya-benzeri kullanım için z'yi ters çeviriyoruz.
        if math.isnan(msg.x) or math.isnan(msg.y) or math.isnan(msg.z):
            return

        self.current_position = {
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(-msg.z)
        }

    # -------------------------------------------------
    # Main timer
    # -------------------------------------------------
    def publish_ppm_timer(self):
        ppm_value = self.compute_current_ppm()
        
        msg = Float32()
        msg.data = float(ppm_value)
        self.ppm_pub.publish(msg)
        
        self.log_to_csv(ppm_value)

    # -------------------------------------------------
    # PPM computation
    # -------------------------------------------------
    def compute_current_ppm(self):
        if self.current_position is None:
            return self.background_ppm

        if not self.active_scenario_name:
            return self.background_ppm

        scenario = self.scenarios.get(self.active_scenario_name)
        if scenario is None:
            self.get_logger().warn(
                f'Aktif senaryo YAML içinde bulunamadı: {self.active_scenario_name}',
                throttle_duration_sec=5.0
            )
            return self.background_ppm

        active_zone_names = scenario.get('active_zones', [])
        if not active_zone_names:
            return self.background_ppm

        total_ppm = self.background_ppm

        for zone_name in active_zone_names:
            zone = self.available_zones.get(zone_name)
            if zone is None:
                self.get_logger().warn(
                    f'Zone bulunamadı: {zone_name}',
                    throttle_duration_sec=5.0
                )
                continue

            zone_ppm = self.compute_zone_contribution(self.current_position, zone_name, zone)
            total_ppm += zone_ppm

        total_ppm = min(total_ppm, self.max_total_ppm)
        return total_ppm

    def compute_zone_contribution(self, drone_pos, zone_name, zone):
        position = zone.get('position', [0.0, 0.0, 0.0])

        if not isinstance(position, list) or len(position) < 3:
            self.get_logger().warn(
                f'Zone position formatı hatalı: {zone_name}',
                throttle_duration_sec=5.0
            )
            return 0.0

        zx = float(position[0])
        zy = float(position[1])
        zz = float(position[2])

        radius = float(zone.get('radius', 1.0))
        max_ppm = float(zone.get('max_ppm', self.default_max_ppm))

        # Gaussian yayılım parametresi
        sigma = radius / self.sigma_scale if self.sigma_scale > 0.0 else radius / 2.0
        sigma = max(sigma, 0.1)

        dx = drone_pos['x'] - zx
        dy = drone_pos['y'] - zy
        dz = drone_pos['z'] - zz

        if self.use_3d_distance:
            d = math.sqrt(dx * dx + dy * dy + dz * dz)
        else:
            d = math.sqrt(dx * dx + dy * dy)

        # Gaussian decay
        contribution = max_ppm * math.exp(-(d * d) / (2.0 * sigma * sigma))

        return contribution


def main(args=None):
    rclpy.init(args=args)
    node = GasSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
