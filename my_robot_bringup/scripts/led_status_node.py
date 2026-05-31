#!/usr/bin/env python3

import time
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from gpiozero import LED, Buzzer


# =========================
# GPIO PINS
# =========================
GREEN_LED_PIN = 13
RED_LED_PIN = 19
BUZZER_PIN = 26


class LedStatusNode(Node):
    def __init__(self):
        super().__init__("led_status_node")

        self.green_led = LED(GREEN_LED_PIN)
        self.red_led = LED(RED_LED_PIN)
        self.buzzer = Buzzer(BUZZER_PIN)

        self.already_shutdown = False

        # =========================
        # ROS2 RUNNING STATUS
        # =========================
        self.red_led.off()
        self.buzzer.off()
        self.green_led.on()

        # Startup beep
        self.buzzer.on()
        time.sleep(0.2)
        self.buzzer.off()

        self.get_logger().info("ROS2 is ON: GREEN LED ON")

    def warning_shutdown(self):
        if self.already_shutdown:
            return

        self.already_shutdown = True

        print("\nROS2 is OFF: GREEN OFF, RED ON, BUZZER BEEP")

        # ROS2 stopped status
        self.green_led.off()
        self.red_led.on()

        # Buzzer alarm
        for i in range(5):
            self.buzzer.on()
            time.sleep(0.25)
            self.buzzer.off()
            time.sleep(0.25)

        # Keep red LED ON
        self.red_led.on()
        self.green_led.off()
        self.buzzer.off()

        # Keep red LED visible before program exits
        time.sleep(10)


def main(args=None):
    rclpy.init(args=args)
    node = LedStatusNode()

    def stop_signal(sig, frame):
        node.warning_shutdown()

        if rclpy.ok():
            rclpy.shutdown()

        sys.exit(0)

    # Handles Ctrl+C and ros2 launch shutdown
    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.warning_shutdown()

    except ExternalShutdownException:
        node.warning_shutdown()

    finally:
        node.warning_shutdown()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()