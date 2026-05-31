# RUNNING COMMANDS - LAPTOP:
# python3 robot_gui.py
#
# Install first on laptop:
# sudo apt install sshpass terminator -y
#
# Make sure ROS2 is sourced before running:
# source /opt/ros/jazzy/setup.bash
# source ~/ros2_ws/install/setup.bash

import tkinter as tk
import subprocess
import shutil
import os
import signal

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


SCRIPT_PATH = "./auto_script.sh"

PI_USER = "pi"
PI_IP = "192.168.8.155"
PI_PASSWORD = "1"


class RobotGUI(Node):
    def __init__(self):
        super().__init__('robot_gui_teleop')

        # =========================
        # ROS2 Publisher
        # =========================
        self.publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )

        self.linear_speed = 0.10
        self.angular_speed = 0.60

        self.current_linear = 0.0
        self.current_angular = 0.0

        self.process = None

        # =========================
        # Tkinter GUI
        # =========================
        self.root = tk.Tk()
        self.root.title("BMB Turtle Robot Control")
        self.root.geometry("950x700+400+80")

        tk.Label(
            self.root,
            text="BMB Turtle Robot Control",
            font=("Arial", 18)
        ).pack(pady=10)

        self.status_label = tk.Label(
            self.root,
            text="Status: Ready",
            font=("Arial", 11)
        )
        self.status_label.pack(pady=5)

        # Main frame
        main_frame = tk.Frame(self.root)
        main_frame.pack(pady=10)

        # ==================================================
        # ROW 1: Pi Setup / Auto Commands / Emergency Stop
        # ==================================================

        # =========================
        # Row 1, Column 1: Pi Launch
        # =========================
        col1 = tk.Frame(main_frame)
        col1.grid(row=0, column=0, padx=15, pady=10)

        tk.Label(
            col1,
            text="Pi Setup",
            font=("Arial", 13, "bold")
        ).pack(pady=5)

        tk.Button(
            col1,
            text="Connect Pi + Launch Robot",
            width=28,
            height=3,
            bg="purple",
            fg="white",
            command=self.connect_pi_and_launch_robot
        ).pack(pady=8)

        # =========================
        # Row 1, Column 2: Auto Movement
        # =========================
        col2 = tk.Frame(main_frame)
        col2.grid(row=0, column=1, padx=15, pady=10)

        tk.Label(
            col2,
            text="Auto Commands",
            font=("Arial", 13, "bold")
        ).pack(pady=5)

        tk.Button(
            col2,
            text="Load auto_script.sh",
            width=28,
            height=2,
            command=self.load_script
        ).pack(pady=5)

        tk.Button(
            col2,
            text="Clockwise",
            width=28,
            height=2,
            bg="green",
            fg="white",
            command=self.clockwise
        ).pack(pady=5)

        tk.Button(
            col2,
            text="Anticlockwise",
            width=28,
            height=2,
            bg="blue",
            fg="white",
            command=self.anticlockwise
        ).pack(pady=5)

        tk.Button(
            col2,
            text="Turn 180",
            width=28,
            height=2,
            bg="orange",
            fg="black",
            command=self.turn180
        ).pack(pady=5)

        # =========================
        # Row 1, Column 3: Stop / Cancel
        # =========================
        col3 = tk.Frame(main_frame)
        col3.grid(row=0, column=2, padx=15, pady=10)

        tk.Label(
            col3,
            text="Emergency",
            font=("Arial", 13, "bold")
        ).pack(pady=5)

        tk.Button(
            col3,
            text="STOP / Cancel Everything",
            width=28,
            height=5,
            bg="red",
            fg="white",
            command=self.stop_everything
        ).pack(pady=8)

        # ==================================================
        # ROW 2: Manual Robot Control
        # ==================================================
        manual_frame = tk.Frame(main_frame)
        manual_frame.grid(row=1, column=0, columnspan=3, pady=25)

        tk.Label(
            manual_frame,
            text="Manual Drive Control",
            font=("Arial", 14, "bold")
        ).pack(pady=5)

        tk.Label(
            manual_frame,
            text="Hold button to move. Release to stop.",
            font=("Arial", 10)
        ).pack(pady=5)

        button_frame = tk.Frame(manual_frame)
        button_frame.pack(pady=5)

        self.make_drive_button(button_frame, "Forward", 0, 1, self.forward)
        self.make_drive_button(button_frame, "Left", 1, 0, self.left)
        self.make_drive_button(button_frame, "Stop", 1, 1, self.manual_stop)
        self.make_drive_button(button_frame, "Right", 1, 2, self.right)
        self.make_drive_button(button_frame, "Backward", 2, 1, self.backward)

        # Keep ROS2 running inside Tkinter
        self.root.after(100, self.update_ros)

    # ==================================================
    # Manual Drive Buttons
    # ==================================================
    def make_drive_button(self, frame, text, row, column, command):
        button = tk.Button(
            frame,
            text=text,
            width=12,
            height=3
        )
        button.grid(row=row, column=column, padx=6, pady=6)

        button.bind("<ButtonPress-1>", lambda event: command())
        button.bind("<ButtonRelease-1>", lambda event: self.manual_stop())

    def publish_cmd(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"

        msg.twist.linear.x = self.current_linear
        msg.twist.angular.z = self.current_angular

        self.publisher.publish(msg)

    def forward(self):
        self.status_label.config(text="Status: Manual forward")
        self.current_linear = self.linear_speed
        self.current_angular = 0.0

    def backward(self):
        self.status_label.config(text="Status: Manual backward")
        self.current_linear = -self.linear_speed
        self.current_angular = 0.0

    def left(self):
        self.status_label.config(text="Status: Manual left")
        self.current_linear = 0.0
        self.current_angular = self.angular_speed

    def right(self):
        self.status_label.config(text="Status: Manual right")
        self.current_linear = 0.0
        self.current_angular = -self.angular_speed

    def manual_stop(self):
        self.status_label.config(text="Status: Manual stop")
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.publish_cmd()

    def update_ros(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.publish_cmd()
        self.root.after(100, self.update_ros)

    # ==================================================
    # Pi Launch / Auto Script Functions
    # ==================================================
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

    def load_script(self):
        command = f"source {SCRIPT_PATH} && echo loaded"

        result = subprocess.run(
            ["bash", "-lc", command],
            capture_output=True,
            text=True
        )

        if result.returncode == 0:
            self.status_label.config(text="Status: auto_script.sh loaded")
        else:
            self.status_label.config(text="Status: Load failed")
            print(result.stderr)

    def run_local_function(self, function_name, status_text):
        self.status_label.config(text=status_text)

        command = f"source {SCRIPT_PATH} && {function_name}"

        self.process = subprocess.Popen(
            ["bash", "-lc", command],
            start_new_session=True
        )

    def clockwise(self):
        self.run_local_function("clockwise", "Status: Running clockwise")

    def anticlockwise(self):
        self.run_local_function("anticlockwise", "Status: Running anticlockwise")

    def turn180(self):
        self.run_local_function("turn180", "Status: Running 180 turn")

    # ==================================================
    # Emergency Stop
    # ==================================================
    def stop_everything(self):
        self.status_label.config(text="Status: STOP / Cancel sent")

        # Stop manual movement
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.publish_cmd()

        # Cancel currently running local auto command
        if self.process is not None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except Exception as error:
                print("Could not cancel running process:", error)

            self.process = None

        # Send stop command from auto_script.sh
        command = f"source {SCRIPT_PATH} && stop_now"

        subprocess.Popen(
            ["bash", "-lc", command]
        )

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    gui = RobotGUI()

    try:
        gui.run()
    except KeyboardInterrupt:
        pass

    gui.stop_everything()
    gui.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()