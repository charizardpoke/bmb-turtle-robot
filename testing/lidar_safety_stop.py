#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


class LidarSafetyStop(Node):
    def __init__(self):
        super().__init__('lidar_safety_stop')

        self.stop_distance = 0.45
        self.clear_distance = 0.55
        self.front_angle_deg = 120

        self.obstacle_detected = False

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel_raw',
            self.cmd_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )

        self.buzzer_pub = self.create_publisher(
            String,
            '/buzzer_cmd',
            10
        )

        self.get_logger().info('Lidar safety stop is running.')
        self.get_logger().info('Controller input: /cmd_vel_raw')
        self.get_logger().info('Robot output: /diff_drive_controller/cmd_vel')

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
            return

        min_distance = min(front_ranges)

        if min_distance <= self.stop_distance:
            if not self.obstacle_detected:
                self.get_logger().warn(f'Obstacle too close: {min_distance:.2f} m')

            self.obstacle_detected = True
            self.stop_robot()
            self.send_buzzer('ON')

        elif min_distance >= self.clear_distance:
            if self.obstacle_detected:
                self.get_logger().info(f'Obstacle cleared: {min_distance:.2f} m')

            self.obstacle_detected = False
            self.send_buzzer('OFF')

    def cmd_callback(self, msg):
        if self.obstacle_detected:
            self.stop_robot()
            self.send_buzzer('ON')
            return

        safe_msg = TwistStamped()
        safe_msg.header.stamp = self.get_clock().now().to_msg()
        safe_msg.header.frame_id = 'base_footprint'
        safe_msg.twist = msg.twist

        self.cmd_pub.publish(safe_msg)

    def stop_robot(self):
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_footprint'

        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.linear.y = 0.0
        stop_msg.twist.linear.z = 0.0
        stop_msg.twist.angular.x = 0.0
        stop_msg.twist.angular.y = 0.0
        stop_msg.twist.angular.z = 0.0

        self.cmd_pub.publish(stop_msg)

    def send_buzzer(self, state):
        msg = String()
        msg.data = state
        self.buzzer_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSafetyStop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.send_buzzer('OFF')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()