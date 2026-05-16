#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


FRONT_SCAN_TOPIC = '/drone/front_scan'
LEFT_SCAN_TOPIC = '/drone/left_scan'
RIGHT_SCAN_TOPIC = '/drone/right_scan'

SECTOR_SIZE = 5
FORWARD_AVG_CLEARANCE_M = 3.5
FORWARD_MIN_CLEARANCE_M = 2.0
FRONT_ESCAPE_MIN_CLEARANCE_M = 1.3
SIDE_ESCAPE_MIN_CLEARANCE_M = 1.0


class MultiScanMonitorNode(Node):
    def __init__(self):
        super().__init__('multi_scan_monitor_node')
        self.front_stats = None
        self.left_stats = None
        self.right_stats = None

        self.create_subscription(LaserScan, FRONT_SCAN_TOPIC, self.front_callback, 10)
        self.create_subscription(LaserScan, LEFT_SCAN_TOPIC, self.left_callback, 10)
        self.create_subscription(LaserScan, RIGHT_SCAN_TOPIC, self.right_callback, 10)

        self.get_logger().info('Çok yönlü LaserScan monitor başlatıldı.')
        self.get_logger().info(f'Ön topic: {FRONT_SCAN_TOPIC}')
        self.get_logger().info(f'Sol topic: {LEFT_SCAN_TOPIC}')
        self.get_logger().info(f'Sağ topic: {RIGHT_SCAN_TOPIC}')

    def front_callback(self, msg):
        ranges = self._normalize_ranges(msg.ranges, msg.range_max)
        if len(ranges) < SECTOR_SIZE * 3:
            self.get_logger().warning(
                f'Yetersiz front_scan verisi: {len(ranges)} ray geldi, en az {SECTOR_SIZE * 3} gerekli.'
            )
            return

        left = ranges[:SECTOR_SIZE]
        center_start = (len(ranges) - SECTOR_SIZE) // 2
        center = ranges[center_start:center_start + SECTOR_SIZE]
        right = ranges[-SECTOR_SIZE:]

        left_min, left_avg = self._sector_stats(left)
        center_min, center_avg = self._sector_stats(center)
        right_min, right_avg = self._sector_stats(right)

        self.front_stats = {
            'left_min': left_min,
            'left_avg': left_avg,
            'center_min': center_min,
            'center_avg': center_avg,
            'right_min': right_min,
            'right_avg': right_avg,
        }
        self._print_summary()

    def left_callback(self, msg):
        self.left_stats = self._scan_stats(msg)

    def right_callback(self, msg):
        self.right_stats = self._scan_stats(msg)

    def _print_summary(self):
        if self.front_stats is None:
            return

        decision = self._make_decision()
        left_text = self._format_side_stats(self.left_stats)
        right_text = self._format_side_stats(self.right_stats)

        print(
            f"Front sol min/avg: {self.front_stats['left_min']:.2f} / {self.front_stats['left_avg']:.2f} m | "
            f"Front orta min/avg: {self.front_stats['center_min']:.2f} / {self.front_stats['center_avg']:.2f} m | "
            f"Front sağ min/avg: {self.front_stats['right_min']:.2f} / {self.front_stats['right_avg']:.2f} m"
        )
        print(f"Left scan min/avg: {left_text} | Right scan min/avg: {right_text}")
        print(f"Karar: {decision}")

    def _make_decision(self):
        center_clear = (
            self.front_stats['center_avg'] > FORWARD_AVG_CLEARANCE_M and
            self.front_stats['center_min'] > FORWARD_MIN_CLEARANCE_M
        )
        if center_clear:
            return 'İLERİ AÇIK'

        left_clear = (
            self.front_stats['left_min'] > FRONT_ESCAPE_MIN_CLEARANCE_M and
            self.left_stats is not None and
            self.left_stats['min'] > SIDE_ESCAPE_MIN_CLEARANCE_M
        )
        right_clear = (
            self.front_stats['right_min'] > FRONT_ESCAPE_MIN_CLEARANCE_M and
            self.right_stats is not None and
            self.right_stats['min'] > SIDE_ESCAPE_MIN_CLEARANCE_M
        )

        if left_clear and right_clear:
            if self.left_stats['avg'] > self.right_stats['avg']:
                return 'SOLA KAÇ'
            if self.right_stats['avg'] > self.left_stats['avg']:
                return 'SAĞA KAÇ'
            return 'DUR / GERİ DÖN'

        if left_clear:
            return 'SOLA KAÇ'

        if right_clear:
            return 'SAĞA KAÇ'

        return 'DUR / GERİ DÖN'

    def _scan_stats(self, msg):
        ranges = self._normalize_ranges(msg.ranges, msg.range_max)
        min_value, avg_value = self._sector_stats(ranges)
        return {'min': min_value, 'avg': avg_value}

    @staticmethod
    def _normalize_ranges(raw_ranges, range_max):
        normalized = []
        for value in raw_ranges:
            if math.isinf(value):
                normalized.append(float(range_max))
            elif math.isnan(value):
                normalized.append(0.0)
            else:
                normalized.append(float(value))
        return normalized

    @staticmethod
    def _sector_stats(values):
        if not values:
            return 0.0, 0.0
        return min(values), sum(values) / len(values)

    @staticmethod
    def _format_side_stats(stats):
        if stats is None:
            return 'veri yok'
        return f"{stats['min']:.2f} / {stats['avg']:.2f} m"


def main(args=None):
    rclpy.init(args=args)
    node = MultiScanMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Multi scan monitor kapatılıyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
