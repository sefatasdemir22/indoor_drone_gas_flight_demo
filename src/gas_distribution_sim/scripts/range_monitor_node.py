#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class RangeMonitorNode(Node):
    def __init__(self):
        super().__init__('range_monitor_node')
        self.topic_name = '/drone/front_range'
        self.subscription = self.create_subscription(
            Range,
            self.topic_name,
            self.range_callback,
            10
        )
        self.get_logger().info(f'Ön mesafe topic dinleniyor: {self.topic_name}')

    def range_callback(self, msg):
        comment = self.classify_range(msg.range)
        self.get_logger().info(
            f'range={msg.range:.3f} m | '
            f'min_range={msg.min_range:.3f} m | '
            f'max_range={msg.max_range:.3f} m | '
            f'yorum={comment}'
        )

    def classify_range(self, range_value):
        if range_value > 3.0:
            return 'AÇIK'
        if range_value > 1.5:
            return 'YAKIN'
        return 'ENGEL'


def main(args=None):
    rclpy.init(args=args)
    node = RangeMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
