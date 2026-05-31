# RUNNING COMMANDS - LAPTOP:
# python3 stage_gui.py
#
# Install first on laptop:
# sudo apt install sshpass terminator -y

import tkinter as tk
import subprocess
import shutil
import os
import signal


SCRIPT_PATH = "./auto_script.sh"

PI_USER = "pi"
PI_IP = "192.168.8.155"
PI_PASSWORD = "1"


class StageGUI:
    def __init__(self):
        self.process = None

        self.root = tk.Tk()
        self.root.title("WALL-E Auto Script GUI")
        self.root.geometry("900x420+500+100")

        tk.Label(
            self.root,
            text="WALL-E Auto Control",
            font=("Arial", 18)
        ).pack(pady=10)

        self.status_label = tk.Label(
            self.root,
            text="Status: Ready",
            font=("Arial", 11)
        )
        self.status_label.pack(pady=5)

        main_frame = tk.Frame(self.root)
        main_frame.pack(pady=15)

        # =========================
        # Column 1: Pi Launch
        # =========================
        col1 = tk.Frame(main_frame)
        col1.grid(row=0, column=0, padx=15)

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
        # Column 2: Auto Movement
        # =========================
        col2 = tk.Frame(main_frame)
        col2.grid(row=0, column=1, padx=15)

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
        # Column 3: Stop / Cancel
        # =========================
        col3 = tk.Frame(main_frame)
        col3.grid(row=0, column=2, padx=15)

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
            command=self.stop_robot
        ).pack(pady=8)

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

    def stop_robot(self):
        self.status_label.config(text="Status: STOP / Cancel sent")

        # Cancel currently running local auto command
        if self.process is not None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except Exception as error:
                print("Could not cancel running process:", error)

            self.process = None

        # Send stop command to robot
        command = f"source {SCRIPT_PATH} && stop_now"

        subprocess.Popen(
            ["bash", "-lc", command]
        )

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    gui = StageGUI()
    gui.run()