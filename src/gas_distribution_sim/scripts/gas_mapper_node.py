#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32
from px4_msgs.msg import VehicleLocalPosition


class GasMapperNode(Node):
    def __init__(self):
        super().__init__('gas_mapper_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('ppm_topic', '/gas_sensor/ppm')

        # Map bounds and resolution
        self.declare_parameter('x_min', -5.0)
        self.declare_parameter('x_max', 25.0)
        self.declare_parameter('y_min', -10.0)
        self.declare_parameter('y_max', 10.0)
        self.declare_parameter('resolution', 0.5)

        # Export
        self.declare_parameter(
            'map_csv_path',
            os.path.expanduser('~/araswarm_ws/gas_map_logs/gas_map_grid.csv')
        )
        self.declare_parameter(
            'samples_csv_path',
            os.path.expanduser('~/araswarm_ws/gas_map_logs/gas_map_samples.csv')
        )
        self.declare_parameter('export_period_sec', 5.0)
        self.declare_parameter('enable_sample_logging', True)

        # Optional filter
        self.declare_parameter('min_movement_distance', 0.10)

        self.position_topic = self.get_parameter('position_topic').value
        self.ppm_topic = self.get_parameter('ppm_topic').value

        self.x_min = float(self.get_parameter('x_min').value)
        self.x_max = float(self.get_parameter('x_max').value)
        self.y_min = float(self.get_parameter('y_min').value)
        self.y_max = float(self.get_parameter('y_max').value)
        self.resolution = float(self.get_parameter('resolution').value)

        self.map_csv_path = self.get_parameter('map_csv_path').value
        self.samples_csv_path = self.get_parameter('samples_csv_path').value
        self.export_period_sec = float(self.get_parameter('export_period_sec').value)
        self.enable_sample_logging = bool(self.get_parameter('enable_sample_logging').value)

        self.min_movement_distance = float(self.get_parameter('min_movement_distance').value)

        # -----------------------------
        # Grid setup
        # -----------------------------
        self.grid_width = int(math.ceil((self.x_max - self.x_min) / self.resolution))
        self.grid_height = int(math.ceil((self.y_max - self.y_min) / self.resolution))

        # Each cell stores:
        # {'sum_ppm': float, 'count': int, 'avg_ppm': float}
        self.grid = [
            [
                {'sum_ppm': 0.0, 'count': 0, 'avg_ppm': 0.0}
                for _ in range(self.grid_width)
            ]
            for _ in range(self.grid_height)
        ]

        # -----------------------------
        # State
        # -----------------------------
        self.current_position = None
        self.current_ppm = None
        self.last_logged_position = None

        # -----------------------------
        # QoS for PX4 topic
        # -----------------------------
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # -----------------------------
        # Subscriptions
        # -----------------------------
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            self.position_topic,
            self.position_callback,
            px4_qos
        )

        self.ppm_sub = self.create_subscription(
            Float32,
            self.ppm_topic,
            self.ppm_callback,
            10
        )

        # -----------------------------
        # Timers
        # -----------------------------
        self.export_timer = self.create_timer(self.export_period_sec, self.export_map_csv)

        # -----------------------------
        # Files
        # -----------------------------
        self.init_output_files()

        self.get_logger().info('GasMapperNode başlatıldı.')
        self.get_logger().info(f'Position topic   : {self.position_topic}')
        self.get_logger().info(f'PPM topic        : {self.ppm_topic}')
        self.get_logger().info(
            f'Map bounds        : x[{self.x_min}, {self.x_max}] y[{self.y_min}, {self.y_max}] res={self.resolution}'
        )
        self.get_logger().info(f'Grid size         : {self.grid_width} x {self.grid_height}')
        self.get_logger().info(f'Map CSV           : {self.map_csv_path}')
        self.get_logger().info(f'Samples CSV       : {self.samples_csv_path}')

    # -------------------------------------------------
    # Init files
    # -------------------------------------------------
    def init_output_files(self):
        map_dir = os.path.dirname(self.map_csv_path)
        os.makedirs(map_dir, exist_ok=True)

        samples_dir = os.path.dirname(self.samples_csv_path)
        os.makedirs(samples_dir, exist_ok=True)

        if self.enable_sample_logging and not os.path.exists(self.samples_csv_path):
            with open(self.samples_csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'wall_time_iso',
                    'ros_time_sec',
                    'x',
                    'y',
                    'z',
                    'ppm',
                    'grid_ix',
                    'grid_iy'
                ])

    # -------------------------------------------------
    # Callbacks
    # -------------------------------------------------
    def position_callback(self, msg: VehicleLocalPosition):
        if math.isnan(msg.x) or math.isnan(msg.y) or math.isnan(msg.z):
            return

        self.current_position = {
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(-msg.z)  # PX4 z is down -> convert to up
        }

    def ppm_callback(self, msg: Float32):
        self.current_ppm = float(msg.data)
        self.try_update_map()

    # -------------------------------------------------
    # Mapping logic
    # -------------------------------------------------
    def try_update_map(self):
        if self.current_position is None or self.current_ppm is None:
            return

        x = self.current_position['x']
        y = self.current_position['y']
        z = self.current_position['z']
        ppm = self.current_ppm

        # Skip if outside map bounds
        grid_index = self.world_to_grid(x, y)
        if grid_index is None:
            return

        # Optional movement filter
        if self.last_logged_position is not None:
            dx = x - self.last_logged_position['x']
            dy = y - self.last_logged_position['y']
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < self.min_movement_distance:
                return

        ix, iy = grid_index

        cell = self.grid[iy][ix]
        cell['sum_ppm'] += ppm
        cell['count'] += 1
        cell['avg_ppm'] = cell['sum_ppm'] / cell['count']

        self.last_logged_position = {'x': x, 'y': y, 'z': z}

        if self.enable_sample_logging:
            self.append_sample_csv(x, y, z, ppm, ix, iy)

    def world_to_grid(self, x, y):
        if x < self.x_min or x >= self.x_max or y < self.y_min or y >= self.y_max:
            return None

        ix = int((x - self.x_min) / self.resolution)
        iy = int((y - self.y_min) / self.resolution)

        if ix < 0 or ix >= self.grid_width or iy < 0 or iy >= self.grid_height:
            return None

        return ix, iy

    def grid_to_world_center(self, ix, iy):
        cx = self.x_min + (ix + 0.5) * self.resolution
        cy = self.y_min + (iy + 0.5) * self.resolution
        return cx, cy

    # -------------------------------------------------
    # CSV output
    # -------------------------------------------------
    def append_sample_csv(self, x, y, z, ppm, ix, iy):
        try:
            now_msg = self.get_clock().now().to_msg()
            ros_time_sec = float(now_msg.sec) + float(now_msg.nanosec) / 1e9

            with open(self.samples_csv_path, 'a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    datetime.now().isoformat(timespec='milliseconds'),
                    ros_time_sec,
                    x,
                    y,
                    z,
                    ppm,
                    ix,
                    iy
                ])
        except Exception as e:
            self.get_logger().error(f'Sample CSV yazma hatası: {e}')

    def export_map_csv(self):
        try:
            with open(self.map_csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'grid_ix',
                    'grid_iy',
                    'center_x',
                    'center_y',
                    'sample_count',
                    'sum_ppm',
                    'avg_ppm'
                ])

                non_empty = 0

                for iy in range(self.grid_height):
                    for ix in range(self.grid_width):
                        cell = self.grid[iy][ix]
                        if cell['count'] == 0:
                            continue

                        cx, cy = self.grid_to_world_center(ix, iy)
                        writer.writerow([
                            ix,
                            iy,
                            cx,
                            cy,
                            cell['count'],
                            cell['sum_ppm'],
                            cell['avg_ppm']
                        ])
                        non_empty += 1

            self.get_logger().info(
                f'Map CSV export edildi | dolu_hucre={non_empty}',
                throttle_duration_sec=10.0
            )

        except Exception as e:
            self.get_logger().error(f'Map CSV export hatası: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GasMapperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.export_map_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
