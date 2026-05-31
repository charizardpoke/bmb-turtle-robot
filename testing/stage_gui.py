# RUNNING COMMANDS - LAPTOP:
# python3 stage_gui.py
#
# Install first on laptop:
# sudo apt install sshpass terminator -y

import tkinter as tk
import subprocess
import shutil
import os


SCRIPT_PATH = "./auto_script.sh"

PI_USER = "pi"
PI_IP = "192.168.8.155"
PI_PASSWORD = "1"


class StageGUI:
    def __init__(self):
        self.process = None

        self.root = tk.Tk()
        self.root.title("WALL-E Auto Script GUI")
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
            text="Load auto_script.sh",
            width=28,
            height=2,
            command=self.load_script
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="Clockwise",
            width=28,
            height=2,
            bg="green",
            fg="white",
            command=self.clockwise
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="Anticlockwise",
            width=28,
            height=2,
            bg="blue",
            fg="white",
            command=self.anticlockwise
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="Turn 180",
            width=28,
            height=2,
            bg="orange",
            fg="black",
            command=self.turn180
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
            ["bash", "-lc", command]
        )

    def clockwise(self):
        self.run_local_function("clockwise", "Status: Running clockwise")

    def anticlockwise(self):
        self.run_local_function("anticlockwise", "Status: Running anticlockwise")

    def turn180(self):
        self.run_local_function("turn180", "Status: Running 180 turn")

    def stop_robot(self):
        self.run_local_function("stop_now", "Status: STOP sent")

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    gui = StageGUI()
    gui.run()