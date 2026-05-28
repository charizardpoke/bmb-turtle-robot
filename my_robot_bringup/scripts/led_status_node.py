#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from gpiozero import LED, Buzzer


class LedStatusNode(Node):
    def __init__(self):
        super().__init__("led_status_node")

        # Your GPIO pins
        self.green_led = LED(13)
        self.red_led = LED(19)
        self.buzzer = Buzzer(26)

        # Startup status
        self.red_led.off()
        self.green_led.on()

        # Buzzer makes one short sound
        self.buzzer.on()
        time.sleep(0.3)
        self.buzzer.off()

        self.get_logger().info("Green LED ON: robot system is operating correctly")

    def ctrl_c_shutdown(self):
        self.get_logger().warn("Ctrl+C detected: Red LED ON")

        self.green_led.off()
        self.buzzer.off()
        self.red_led.on()

        # Keep red LED on for 3 seconds so you can see it
        time.sleep(3.0)

    def normal_shutdown(self):
        self.green_led.off()
        self.red_led.off()
        self.buzzer.off()


def main(args=None):
    rclpy.init(args=args)
    node = LedStatusNode()

    try:
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        node.ctrl_c_shutdown()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()