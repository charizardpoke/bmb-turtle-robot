#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LidarBuzzerWarning(Node):
    def __init__(self):
        super().__init__('lidar_buzzer_warning')

        # ===== Settings =====
        self.buzz_distance = 0.45      # ON if obstacle <= 45 cm
        self.clear_distance = 0.55     # OFF if obstacle >= 55 cm
        self.front_angle_deg = 120     # front lidar area

        self.buzzer_state = 'OFF'
        self.last_min_distance = None

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.buzzer_pub = self.create_publisher(
            String,
            '/buzzer_cmd',
            10
        )

        # Publish buzzer state repeatedly
        self.timer = self.create_timer(0.2, self.publish_buzzer_state)

        self.get_logger().info('Lidar buzzer warning started.')
        self.get_logger().info('Publishing to /buzzer_cmd')
        self.get_logger().info('Buzzer ON <= 0.45 m, OFF >= 0.55 m')

    def scan_callback(self, msg):
        front_ranges = []
        angle = msg.angle_min

        for distance in msg.ranges:
            angle_deg = math.degrees(angle)

            if abs(angle_deg) <= self.front_angle_deg / 2:
                if math.isfinite(distance):
                    if msg.range_min <= distance <= msg.range_max:
                        front_ranges.append(distance)

            angle += msg.angle_increment

        if not front_ranges:
            self.buzzer_state = 'OFF'
            self.last_min_distance = None
            return

        min_distance = min(front_ranges)
        self.last_min_distance = min_distance

        if min_distance <= self.buzz_distance:
            if self.buzzer_state != 'ON':
                self.get_logger().warn(f'Obstacle close: {min_distance:.2f} m - BUZZER ON')
            self.buzzer_state = 'ON'

        elif min_distance >= self.clear_distance:
            if self.buzzer_state != 'OFF':
                self.get_logger().info(f'Obstacle cleared: {min_distance:.2f} m - BUZZER OFF')
            self.buzzer_state = 'OFF'

    def publish_buzzer_state(self):
        msg = String()
        msg.data = self.buzzer_state
        self.buzzer_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarBuzzerWarning()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Turn buzzer off when closing
    msg = String()
    msg.data = 'OFF'
    node.buzzer_pub.publish(msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()