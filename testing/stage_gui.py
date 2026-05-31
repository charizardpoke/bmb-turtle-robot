# RUNNING COMMANDS - LAPTOP:
# cd ~/ros2_ws
# source /opt/ros/jazzy/setup.bash
# source install/setup.bash
# python3 stage_gui.py
#
# Install first on laptop:
# sudo apt install sshpass terminator -y

import tkinter as tk
import subprocess
import shutil
import os
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


PI_USER = "pi"
PI_IP = "192.168.8.155"
PI_PASSWORD = "1"


class StageGUI(Node):
    def __init__(self):
        super().__init__("stage_gui_node")

        self.process = None

        # ROS settings
        self.cmd_topic = "/diff_drive_controller/cmd_vel"
        self.rate_hz = 10

        # Same speed settings as auto_script.sh
        self.forward_speed = 0.5
        self.turn_speed = 0.5

        # Stop between each motion
        self.stop_time = 0.5

        # Create ROS publisher one time
        self.publisher_ = self.create_publisher(
            TwistStamped,
            self.cmd_topic,
            10
        )

        self.stop_requested = False
        self.motion_thread = None

        self.root = tk.Tk()
        self.root.title("WALL-E Direct ROS2 GUI")
        self.root.geometry("380x600+1000+100")

        tk.Label(
            self.root,
            text="WALL-E Auto Control",
            font=("Arial", 16)
        ).pack(pady=15)

        self.status_label = tk.Label(
            self.root,
            text="Status: Ready",
            font=("Arial", 11)
        )
        self.status_label.pack(pady=10)

        tk.Button(
            self.root,
            text="Connect Pi + Launch Robot",
            width=28,
            height=2,
            bg="purple",
            fg="white",
            command=self.connect_pi_and_launch_robot
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="Clockwise",
            width=28,
            height=2,
            bg="green",
            fg="white",
            command=self.start_clockwise
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="Anticlockwise",
            width=28,
            height=2,
            bg="blue",
            fg="white",
            command=self.start_anticlockwise
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="Turn 180",
            width=28,
            height=2,
            bg="orange",
            fg="black",
            command=self.start_turn180
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="STOP",
            width=28,
            height=2,
            bg="red",
            fg="white",
            command=self.stop_robot
        ).pack(pady=15)

        self.root.protocol("WM_DELETE_WINDOW", self.close)

    # -------------------------
    # Pi SSH launch functions
    # -------------------------

    def check_program(self, program_name):
        if shutil.which(program_name) is None:
            self.status_label.config(text=f"Status: Missing {program_name}")
            print(f"Missing program: {program_name}")
            print(f"Install it with: sudo apt install {program_name} -y")
            return False
        return True

    def connect_pi_and_launch_robot(self):
        if not self.check_program("sshpass"):
            return

        if not self.check_program("terminator"):
            return

        self.status_label.config(text="Status: Opening Pi launch terminal")

        script_path = "/tmp/walle_launch_robot.sh"

        script_content = f"""#!/bin/bash

echo "Connecting to Raspberry Pi..."
echo "IP: {PI_IP}"
echo ""

sshpass -p '{PI_PASSWORD}' ssh -tt -o StrictHostKeyChecking=no {PI_USER}@{PI_IP} << 'EOF'
echo "Connected to Pi."
echo "Waiting 5 seconds before launching robot..."
sleep 5

cd ~/ros2_ws
colcon build --symlink-install
source ~/.bashrc
ros2 launch my_robot_bringup my_robot.launch.xml
EOF

echo ""
echo "SSH session ended."
exec bash
"""

        with open(script_path, "w") as file:
            file.write(script_content)

        os.chmod(script_path, 0o755)

        subprocess.Popen([
            "terminator",
            "-e",
            f"bash {script_path}"
        ])

    # -------------------------
    # ROS movement functions
    # -------------------------

    def publish_cmd(self, linear_x, angular_z):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"

        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)

        self.publisher_.publish(msg)

    def move_for_time(self, linear_x, angular_z, seconds):
        start_time = time.time()

        while time.time() - start_time < seconds:
            if self.stop_requested:
                break

            self.publish_cmd(linear_x, angular_z)
            time.sleep(1.0 / self.rate_hz)

    def stop_for_time(self, seconds):
        start_time = time.time()

        while time.time() - start_time < seconds:
            self.publish_cmd(0.0, 0.0)
            time.sleep(1.0 / self.rate_hz)

    def stop_now(self):
        self.set_status("STOPPING robot")
        self.stop_for_time(self.stop_time)

    # -------------------------
    # Same motions as auto_script.sh
    # -------------------------

    def short_forward(self):
        self.set_status("Go straight for 1 second")
        self.move_for_time(self.forward_speed, 0.0, 1.0)
        self.stop_now()

    def long_forward(self):
        self.set_status("Go straight for 1.7 seconds")
        self.move_for_time(self.forward_speed, 0.0, 1.7)
        self.stop_now()

    def turn_right(self):
        self.set_status("Turn right 90 degrees")
        self.move_for_time(0.0, -self.turn_speed, 0.6)
        self.stop_now()

    def turn_left(self):
        self.set_status("Turn left 90 degrees")
        self.move_for_time(0.0, self.turn_speed, 0.6)
        self.stop_now()

    def turn_right_180(self):
        self.set_status("Turn right 180 degrees")
        self.move_for_time(0.0, -self.turn_speed, 1.4)
        self.stop_now()

    # -------------------------
    # Full paths
    # -------------------------

    def clockwise_path(self):
        self.stop_requested = False
        self.set_status("Running clockwise")

        self.short_forward()
        self.turn_right()
        self.long_forward()
        self.turn_right()
        self.short_forward()

        self.stop_now()
        self.set_status("Path complete")

    def anticlockwise_path(self):
        self.stop_requested = False
        self.set_status("Running anticlockwise")

        self.short_forward()
        self.turn_left()
        self.long_forward()
        self.turn_left()
        self.short_forward()

        self.stop_now()
        self.set_status("Path complete")

    def turn180_path(self):
        self.stop_requested = False
        self.set_status("Running 180 turn")

        self.turn_right_180()

        self.set_status("Path complete")

    # -------------------------
    # GUI button functions
    # -------------------------

    def start_motion(self, target_function):
        if self.motion_thread is not None and self.motion_thread.is_alive():
            self.set_status("Robot is already moving")
            return

        self.stop_requested = False

        self.motion_thread = threading.Thread(
            target=target_function,
            daemon=True
        )
        self.motion_thread.start()

    def start_clockwise(self):
        self.start_motion(self.clockwise_path)

    def start_anticlockwise(self):
        self.start_motion(self.anticlockwise_path)

    def start_turn180(self):
        self.start_motion(self.turn180_path)

    def stop_robot(self):
        self.stop_requested = True
        self.set_status("STOP sent")
        self.stop_now()

    def set_status(self, text):
        self.root.after(
            0,
            lambda: self.status_label.config(text=f"Status: {text}")
        )

    def close(self):
        self.stop_requested = True
        self.stop_now()
        self.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    gui = StageGUI()
    gui.run()


if __name__ == "__main__":
    main()
