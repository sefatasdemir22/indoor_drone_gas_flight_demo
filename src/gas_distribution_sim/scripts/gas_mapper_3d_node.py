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


class GasMapper3DNode(Node):
    def __init__(self):
        super().__init__('gas_mapper_3d_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('ppm_topic', '/gas_sensor/ppm')

        # 3D map bounds
        self.declare_parameter('x_min', -5.0)
        self.declare_parameter('x_max', 25.0)
        self.declare_parameter('y_min', -10.0)
        self.declare_parameter('y_max', 10.0)
        self.declare_parameter('z_min', 0.0)
        self.declare_parameter('z_max', 5.0)
        self.declare_parameter('resolution', 0.5)

        # Export
        self.declare_parameter(
            'voxel_csv_path',
            os.path.expanduser('~/araswarm_ws/gas_map_logs/gas_map_voxel.csv')
        )
        self.declare_parameter(
            'samples_csv_path',
            os.path.expanduser('~/araswarm_ws/gas_map_logs/gas_map_samples_3d.csv')
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
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.resolution = float(self.get_parameter('resolution').value)

        self.voxel_csv_path = self.get_parameter('voxel_csv_path').value
        self.samples_csv_path = self.get_parameter('samples_csv_path').value
        self.export_period_sec = float(self.get_parameter('export_period_sec').value)
        self.enable_sample_logging = bool(self.get_parameter('enable_sample_logging').value)

        self.min_movement_distance = float(self.get_parameter('min_movement_distance').value)

        # -----------------------------
        # Sparse voxel grid
        # key = (ix, iy, iz)
        # value = {'sum_ppm': float, 'count': int, 'avg_ppm': float}
        # -----------------------------
        self.voxel_grid = {}

        self.grid_width = int(math.ceil((self.x_max - self.x_min) / self.resolution))
        self.grid_height = int(math.ceil((self.y_max - self.y_min) / self.resolution))
        self.grid_depth = int(math.ceil((self.z_max - self.z_min) / self.resolution))

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
        self.export_timer = self.create_timer(self.export_period_sec, self.export_voxel_csv)

        # -----------------------------
        # Files
        # -----------------------------
        self.init_output_files()

        self.get_logger().info('GasMapper3DNode başlatıldı.')
        self.get_logger().info(f'Position topic : {self.position_topic}')
        self.get_logger().info(f'PPM topic      : {self.ppm_topic}')
        self.get_logger().info(
            f'3D bounds       : x[{self.x_min}, {self.x_max}] '
            f'y[{self.y_min}, {self.y_max}] '
            f'z[{self.z_min}, {self.z_max}] '
            f'res={self.resolution}'
        )
        self.get_logger().info(
            f'Voxel size       : {self.grid_width} x {self.grid_height} x {self.grid_depth}'
        )
        self.get_logger().info(f'Voxel CSV        : {self.voxel_csv_path}')
        self.get_logger().info(f'Samples CSV      : {self.samples_csv_path}')

    # -------------------------------------------------
    # Init files
    # -------------------------------------------------
    def init_output_files(self):
        voxel_dir = os.path.dirname(self.voxel_csv_path)
        os.makedirs(voxel_dir, exist_ok=True)

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
                    'voxel_ix',
                    'voxel_iy',
                    'voxel_iz'
                ])

    # -------------------------------------------------
    # Callbacks
    # -------------------------------------------------
    def position_callback(self, msg: VehicleLocalPosition):
        if math.isnan(msg.x) or math.isnan(msg.y) or math.isnan(msg.z):
            return

        # PX4 local position:
        # x = north, y = east, z = down
        # convert z to up-positive
        self.current_position = {
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(-msg.z)
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

        voxel_index = self.world_to_voxel(x, y, z)
        if voxel_index is None:
            return

        # Optional movement filter (3D)
        if self.last_logged_position is not None:
            dx = x - self.last_logged_position['x']
            dy = y - self.last_logged_position['y']
            dz = z - self.last_logged_position['z']
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < self.min_movement_distance:
                return

        key = voxel_index

        if key not in self.voxel_grid:
            self.voxel_grid[key] = {
                'sum_ppm': 0.0,
                'count': 0,
                'avg_ppm': 0.0
            }

        cell = self.voxel_grid[key]
        cell['sum_ppm'] += ppm
        cell['count'] += 1
        cell['avg_ppm'] = cell['sum_ppm'] / cell['count']

        self.last_logged_position = {'x': x, 'y': y, 'z': z}

        if self.enable_sample_logging:
            ix, iy, iz = key
            self.append_sample_csv(x, y, z, ppm, ix, iy, iz)

    def world_to_voxel(self, x, y, z):
        if x < self.x_min or x >= self.x_max:
            return None
        if y < self.y_min or y >= self.y_max:
            return None
        if z < self.z_min or z >= self.z_max:
            return None

        ix = int((x - self.x_min) / self.resolution)
        iy = int((y - self.y_min) / self.resolution)
        iz = int((z - self.z_min) / self.resolution)

        if ix < 0 or ix >= self.grid_width:
            return None
        if iy < 0 or iy >= self.grid_height:
            return None
        if iz < 0 or iz >= self.grid_depth:
            return None

        return ix, iy, iz

    def voxel_to_world_center(self, ix, iy, iz):
        cx = self.x_min + (ix + 0.5) * self.resolution
        cy = self.y_min + (iy + 0.5) * self.resolution
        cz = self.z_min + (iz + 0.5) * self.resolution
        return cx, cy, cz

    # -------------------------------------------------
    # CSV output
    # -------------------------------------------------
    def append_sample_csv(self, x, y, z, ppm, ix, iy, iz):
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
                    iy,
                    iz
                ])
        except Exception as e:
            self.get_logger().error(f'Sample CSV yazma hatası: {e}')

    def export_voxel_csv(self):
        try:
            with open(self.voxel_csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'voxel_ix',
                    'voxel_iy',
                    'voxel_iz',
                    'center_x',
                    'center_y',
                    'center_z',
                    'sample_count',
                    'sum_ppm',
                    'avg_ppm'
                ])

                non_empty = 0

                for (ix, iy, iz), cell in self.voxel_grid.items():
                    cx, cy, cz = self.voxel_to_world_center(ix, iy, iz)
                    writer.writerow([
                        ix,
                        iy,
                        iz,
                        cx,
                        cy,
                        cz,
                        cell['count'],
                        cell['sum_ppm'],
                        cell['avg_ppm']
                    ])
                    non_empty += 1

            self.get_logger().info(
                f'Voxel CSV export edildi | dolu_voxel={non_empty}',
                throttle_duration_sec=10.0
            )

        except Exception as e:
            self.get_logger().error(f'Voxel CSV export hatası: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GasMapper3DNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.export_voxel_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
