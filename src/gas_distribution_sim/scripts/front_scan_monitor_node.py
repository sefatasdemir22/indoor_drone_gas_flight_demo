#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


FRONT_SCAN_TOPIC = '/drone/front_scan'
SECTOR_SIZE = 5
FORWARD_AVG_CLEARANCE_M = 2.0
FORWARD_MIN_CLEARANCE_M = 1.0


class FrontScanMonitorNode(Node):
    def __init__(self):
        super().__init__('front_scan_monitor_node')
        self.create_subscription(LaserScan, FRONT_SCAN_TOPIC, self.scan_callback, 10)
        self.get_logger().info(f'Ön laser scan topic dinleniyor: {FRONT_SCAN_TOPIC}')

    def scan_callback(self, msg):
        ranges = self._normalize_ranges(msg.ranges, msg.range_max)
        if len(ranges) < SECTOR_SIZE * 3:
            self.get_logger().warning(
                f'Yetersiz scan verisi: {len(ranges)} ray geldi, en az {SECTOR_SIZE * 3} gerekli.'
            )
            return

        left = ranges[:SECTOR_SIZE]
        center_start = (len(ranges) - SECTOR_SIZE) // 2
        center = ranges[center_start:center_start + SECTOR_SIZE]
        right = ranges[-SECTOR_SIZE:]

        left_min, left_avg = self._sector_stats(left)
        center_min, center_avg = self._sector_stats(center)
        right_min, right_avg = self._sector_stats(right)
        decision = self._make_decision(left_avg, center_min, center_avg, right_avg)

        print(
            f"Sol min/avg: {left_min:.2f} / {left_avg:.2f} m | "
            f"Orta min/avg: {center_min:.2f} / {center_avg:.2f} m | "
            f"Sağ min/avg: {right_min:.2f} / {right_avg:.2f} m | "
            f"Karar: {decision}"
        )

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
    def _make_decision(left_avg, center_min, center_avg, right_avg):
        if center_avg > FORWARD_AVG_CLEARANCE_M and center_min > FORWARD_MIN_CLEARANCE_M:
            return "İLERİ AÇIK"

        if left_avg <= FORWARD_MIN_CLEARANCE_M and right_avg <= FORWARD_MIN_CLEARANCE_M:
            return "DUR / GERİ DÖN"

        if left_avg > right_avg:
            return "SOLA KAÇ"

        if right_avg > left_avg:
            return "SAĞA KAÇ"

        return "DUR / GERİ DÖN"


def main(args=None):
    rclpy.init(args=args)
    node = FrontScanMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Front scan monitor kapatılıyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
