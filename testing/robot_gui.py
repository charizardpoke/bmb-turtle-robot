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
LIDAR_SAFETY_PATH = "./lidar_safety_stop.py"

PI_USER = "pi"
PI_IP = "192.168.8.155"
PI_PASSWORD = "1"


class RobotGUI(Node):
    def __init__(self):
        super().__init__("robot_gui_teleop")

        # =========================
        # ROS2 Publisher
        # =========================
        self.publisher = self.create_publisher(
            TwistStamped,
            "/diff_drive_controller/cmd_vel",
            10
        )

        self.linear_speed = 0.10
        self.angular_speed = 0.60

        self.current_linear = 0.0
        self.current_angular = 0.0

        self.process = None
        self.lidar_process = None
        self.pi_terminal_process = None

        self.robot_confirmed_on = False
        self.robot_check_count = 0
        self.stop_requested = False

        # =========================
        # Tkinter GUI
        # =========================
        self.root = tk.Tk()
        self.root.title("BMB Turtle Robot Control")
        self.root.geometry("950x850+400+80")

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

        main_frame = tk.Frame(self.root)
        main_frame.pack(pady=10)

        # =========================
        # Column 1: Pi Setup
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
        # Column 2: Auto Commands
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
            text="Load lidar_safety_stop.py",
            width=28,
            height=2,
            bg="gray",
            fg="white",
            command=self.load_lidar_safety
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
        # Column 3: LED Status + Emergency
        # =========================
        col3 = tk.Frame(main_frame)
        col3.grid(row=0, column=2, padx=15, pady=10)

        tk.Label(
            col3,
            text="LED Status",
            font=("Arial", 13, "bold")
        ).pack(pady=5)

        led_frame = tk.Frame(col3)
        led_frame.pack(pady=5)

        self.green_led = tk.Label(
            led_frame,
            text="● Green",
            font=("Arial", 12),
            fg="gray"
        )
        self.green_led.pack(pady=2)

        self.red_led = tk.Label(
            led_frame,
            text="● Red",
            font=("Arial", 12),
            fg="gray"
        )
        self.red_led.pack(pady=2)

        tk.Label(
            col3,
            text="Emergency",
            font=("Arial", 13, "bold")
        ).pack(pady=10)

        tk.Button(
            col3,
            text="STOP / Cancel Everything",
            width=28,
            height=5,
            bg="red",
            fg="white",
            command=self.stop_everything
        ).pack(pady=8)

        self.set_led_status("off")

        # =========================
        # Manual Robot Control
        # =========================
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

        self.root.after(100, self.update_ros)

    # ==================================================
    # LED Status in GUI only
    # ==================================================
    def set_led_status(self, status):
        if status == "off":
            self.green_led.config(fg="gray")
            self.red_led.config(fg="gray")

        elif status == "running":
            self.green_led.config(fg="green")
            self.red_led.config(fg="gray")

        elif status == "stop":
            self.green_led.config(fg="gray")
            self.red_led.config(fg="red")

    # ==================================================
    # Manual Drive
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
    # Program Check
    # ==================================================
    def check_program(self, program_name):
        if shutil.which(program_name) is None:
            self.status_label.config(text=f"Status: Missing {program_name}")
            print(f"Missing program: {program_name}")
            print(f"Install it with: sudo apt install {program_name} -y")
            return False
        return True

    # ==================================================
    # Connect Pi + Launch Robot
    # ==================================================
    def connect_pi_and_launch_robot(self):
        if not self.check_program("sshpass"):
            return

        if not self.check_program("terminator"):
            return

        self.status_label.config(text="Status: Opening Pi launch terminal")

        self.set_led_status("off")
        self.robot_confirmed_on = False
        self.robot_check_count = 0
        self.stop_requested = False

        script_path = "/tmp/walle_launch_robot.sh"

        script_content = f"""#!/bin/bash

echo "Connecting to Raspberry Pi..."
echo "IP: {PI_IP}"
echo ""

sshpass -p '{PI_PASSWORD}' ssh -tt -o StrictHostKeyChecking=no {PI_USER}@{PI_IP} << 'EOF'
echo "Connected to Pi."
echo "Launching robot now..."

cd ~/ros2_ws
colcon build --symlink-install
source ~/.bashrc

echo "Starting robot launch..."
echo "PID file: /tmp/walle_robot_launch.pid"

setsid ros2 launch my_robot_bringup my_robot.launch.xml &
LAUNCH_PID=$!

echo $LAUNCH_PID > /tmp/walle_robot_launch.pid
echo "Robot launch PID: $LAUNCH_PID"

wait $LAUNCH_PID
EOF

echo ""
echo "SSH session ended."
exec bash
"""

        with open(script_path, "w") as file:
            file.write(script_content)

        os.chmod(script_path, 0o755)

        self.pi_terminal_process = subprocess.Popen([
            "terminator",
            "-e",
            f"bash {script_path}"
        ])

        self.root.after(8000, self.check_robot_nodes_on_pi)

    # ==================================================
    # Check Robot Is Really ON
    # ==================================================
    def check_robot_nodes_on_pi(self):
        if self.stop_requested:
            return

        self.robot_check_count += 1

        command = f"""
timeout 5 sshpass -p '{PI_PASSWORD}' ssh -o ConnectTimeout=3 -o StrictHostKeyChecking=no {PI_USER}@{PI_IP} '
source ~/.bashrc
ros2 node list 2>/dev/null | grep -E "controller_manager|robot_state_publisher|diff_drive_controller"
'
"""

        result = subprocess.run(
            ["bash", "-lc", command],
            capture_output=True,
            text=True
        )

        if self.stop_requested:
            return

        if result.returncode == 0 and result.stdout.strip():
            self.robot_confirmed_on = True
            self.status_label.config(text="Status: Robot is ON")
            self.set_led_status("running")
            return

        if self.robot_check_count < 10:
            self.status_label.config(text="Status: Waiting for robot ROS2 nodes...")
            self.set_led_status("off")
            self.root.after(3000, self.check_robot_nodes_on_pi)
        else:
            self.status_label.config(text="Status: Robot not confirmed")
            self.set_led_status("off")

    # ==================================================
    # Auto Script
    # ==================================================
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
            self.status_label.config(text="Status: auto_script.sh load failed")
            self.set_led_status("stop")
            print(result.stderr)

    def load_lidar_safety(self):
        if self.lidar_process is not None:
            self.status_label.config(text="Status: lidar_safety_stop.py already running")
            return

        if not os.path.exists(LIDAR_SAFETY_PATH):
            self.status_label.config(text="Status: lidar_safety_stop.py not found")
            self.set_led_status("stop")
            print(f"Could not find: {LIDAR_SAFETY_PATH}")
            return

        self.status_label.config(text="Status: Running lidar_safety_stop.py")

        command = f"""
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 {LIDAR_SAFETY_PATH}
"""

        self.lidar_process = subprocess.Popen(
            ["bash", "-lc", command],
            start_new_session=True
        )

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
    # Stop Laptop ROS Nodes
    # ==================================================
    def stop_laptop_ros_nodes(self):
        self.status_label.config(text="Status: Stopping laptop ROS nodes")

        laptop_kill_command = """
echo "===== STOP LAPTOP ROS NODES ====="

echo "1) Terminate laptop Nav2 / RViz / ROS tools..."
pkill -TERM -u "$USER" -f "[n]avigation2.launch.py" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[r]viz2" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[b]t_navigator" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[p]lanner_server" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[c]ontroller_server" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[b]ehavior_server" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[s]moother_server" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[w]aypoint_follower" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[v]elocity_smoother" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[c]ollision_monitor" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[l]ifecycle_manager" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[g]lobal_costmap" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[l]ocal_costmap" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[m]ap_server" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[a]mcl" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[s]lam_toolbox" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[f]oxglove_bridge" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[l]idar_buzzer_warning" 2>/dev/null || true

sleep 1

echo "2) Force kill laptop leftovers..."
pkill -KILL -u "$USER" -f "[n]avigation2.launch.py" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[r]viz2" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[b]t_navigator" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[p]lanner_server" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[c]ontroller_server" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[b]ehavior_server" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[s]moother_server" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[w]aypoint_follower" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[v]elocity_smoother" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[c]ollision_monitor" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[l]ifecycle_manager" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[g]lobal_costmap" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[l]ocal_costmap" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[m]ap_server" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[a]mcl" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[s]lam_toolbox" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[f]oxglove_bridge" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[l]idar_buzzer_warning" 2>/dev/null || true

echo "3) Stop laptop ROS daemon..."
ros2 daemon stop 2>/dev/null || true

echo "===== LAPTOP STOP FINISHED ====="
"""

        subprocess.Popen(
            ["bash", "-lc", laptop_kill_command],
            start_new_session=True
        )

    # ==================================================
    # Stop Robot Launch on Pi
    # GPIO alert is done directly on Pi
    # ==================================================
    def stop_pi_robot_launch(self):
        if not self.check_program("sshpass"):
            return

        if not self.check_program("terminator"):
            return

        self.status_label.config(text="Status: Stopping Pi robot launch")

        stop_script_path = "/tmp/walle_stop_robot_from_gui.sh"

        stop_script_content = f"""#!/bin/bash

echo "Connecting to Pi for STOP..."
echo "IP: {PI_IP}"
echo ""

sshpass -p '{PI_PASSWORD}' ssh -tt -o ConnectTimeout=3 -o StrictHostKeyChecking=no {PI_USER}@{PI_IP} 'bash -s' << 'EOF'
echo "===== BMB Turtle Robot STOP ON PI ====="

echo ""
echo "0) GPIO STOP ALERT FIRST"
python3 - << 'PYGPIO'
import time
from gpiozero import LED, Buzzer

green = LED(13)
red = LED(19)
buzzer = Buzzer(26)

green.off()
red.on()

for i in range(5):
    buzzer.on()
    time.sleep(0.25)
    buzzer.off()
    time.sleep(0.25)

green.off()
red.on()
buzzer.off()

print("GPIO STOP ALERT DONE: green OFF, red ON, buzzer beeped")
PYGPIO

echo ""
echo "1) Pi ROS processes before stopping:"
ps -u "$USER" -o pid,ppid,pgid,cmd | grep -E "ros2|my_robot|controller|robot_state|joint_state|diff_drive|foxglove|to_foxglove|republish|lidar|rplidar|sllidar|teleop|twist_stamper|joy_node|image_transport" | grep -v grep || true

echo ""
echo "2) Kill GUI saved launch PID if it exists..."
if [ -f /tmp/walle_robot_launch.pid ]; then
    SAVED_PID=$(cat /tmp/walle_robot_launch.pid)
    echo "Saved PID: $SAVED_PID"

    SAVED_PGID=$(ps -o pgid= -p "$SAVED_PID" 2>/dev/null | tr -d " " || true)

    if [ -n "$SAVED_PGID" ]; then
        echo "Saved PGID: $SAVED_PGID"

        kill -INT -- -$SAVED_PGID 2>/dev/null || true
        sleep 2

        kill -TERM -- -$SAVED_PGID 2>/dev/null || true
        sleep 1

        kill -KILL -- -$SAVED_PGID 2>/dev/null || true
    fi

    kill -KILL "$SAVED_PID" 2>/dev/null || true
    rm -f /tmp/walle_robot_launch.pid
else
    echo "No saved GUI PID file found. This is OK if robot was started manually."
fi

echo ""
echo "3) Kill any manual or GUI robot launch process groups..."
for PID in $(pgrep -u "$USER" -f "[r]os2 launch my_robot_bringup my_robot.launch.xml" || true); do
    PGID=$(ps -o pgid= -p "$PID" 2>/dev/null | tr -d " " || true)

    if [ -n "$PGID" ]; then
        echo "Killing launch PID $PID with PGID $PGID"

        kill -INT -- -$PGID 2>/dev/null || true
        sleep 2

        kill -TERM -- -$PGID 2>/dev/null || true
        sleep 1

        kill -KILL -- -$PGID 2>/dev/null || true
    fi

    kill -KILL "$PID" 2>/dev/null || true
done

echo ""
echo "4) Kill robot bringup and child node process names..."
pkill -INT  -u "$USER" -f "[m]y_robot.launch.xml" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[m]y_robot.launch.xml" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[m]y_robot.launch.xml" 2>/dev/null || true

pkill -TERM -u "$USER" -f "[r]os2_control_node" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[c]ontroller_manager" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[r]obot_state_publisher" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[j]oint_state_broadcaster" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[d]iff_drive_controller" 2>/dev/null || true

pkill -TERM -u "$USER" -f "[r]plidar" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[s]llidar" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[l]idar" 2>/dev/null || true

pkill -TERM -u "$USER" -f "[f]oxglove_bridge" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[t]o_foxglove" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[r]epublish raw" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[i]mage_transport/republish" 2>/dev/null || true

pkill -TERM -u "$USER" -f "[t]wist_stamper" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[t]eleop_node" 2>/dev/null || true
pkill -TERM -u "$USER" -f "[j]oy_node" 2>/dev/null || true

sleep 1

echo ""
echo "5) Force kill stubborn robot leftovers..."
pkill -KILL -u "$USER" -f "[r]os2_control_node" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[c]ontroller_manager" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[r]obot_state_publisher" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[j]oint_state_broadcaster" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[d]iff_drive_controller" 2>/dev/null || true

pkill -KILL -u "$USER" -f "[r]plidar" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[s]llidar" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[l]idar" 2>/dev/null || true

pkill -KILL -u "$USER" -f "[f]oxglove_bridge" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[t]o_foxglove" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[r]epublish raw" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[i]mage_transport/republish" 2>/dev/null || true

pkill -KILL -u "$USER" -f "[t]wist_stamper" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[t]eleop_node" 2>/dev/null || true
pkill -KILL -u "$USER" -f "[j]oy_node" 2>/dev/null || true

echo ""
echo "6) FINAL GPIO STOP ALERT AFTER KILL"
python3 - << 'PYGPIO2'
import time
from gpiozero import LED, Buzzer

green = LED(13)
red = LED(19)
buzzer = Buzzer(26)

green.off()
red.on()

for i in range(3):
    buzzer.on()
    time.sleep(0.25)
    buzzer.off()
    time.sleep(0.25)

green.off()
red.on()
buzzer.off()

print("FINAL GPIO STATUS: green OFF, red ON, buzzer OFF")
PYGPIO2

echo ""
echo "7) Stop Pi ROS daemon..."
source ~/.bashrc 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true

echo ""
echo "8) Pi ROS processes after stopping:"
ps -u "$USER" -o pid,ppid,pgid,cmd | grep -E "ros2|my_robot|controller|robot_state|joint_state|diff_drive|foxglove|to_foxglove|republish|lidar|rplidar|sllidar|teleop|twist_stamper|joy_node|image_transport" | grep -v grep || true

echo ""
echo "===== PI STOP FINISHED ====="
exit
EOF

echo ""
echo "STOP SSH session ended."
exec bash
"""

        with open(stop_script_path, "w") as file:
            file.write(stop_script_content)

        os.chmod(stop_script_path, 0o755)

        subprocess.Popen([
            "terminator",
            "-e",
            f"bash {stop_script_path}"
        ])

    # ==================================================
    # Emergency Stop
    # ==================================================
    def stop_everything(self):
        self.status_label.config(text="Status: STOP EVERYTHING")
        self.set_led_status("stop")

        self.stop_requested = True
        self.robot_confirmed_on = False

        # Stop manual movement from GUI
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.publish_cmd()

        # Cancel local auto command
        if self.process is not None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except Exception as error:
                print("Could not cancel auto_script process:", error)

            self.process = None

        # Cancel local lidar safety script
        if self.lidar_process is not None:
            try:
                os.killpg(os.getpgid(self.lidar_process.pid), signal.SIGTERM)
            except Exception as error:
                print("Could not cancel lidar_safety_stop.py:", error)

            self.lidar_process = None

        # Stop Pi first, including real GPIO alert
        self.stop_pi_robot_launch()

        # Stop laptop ROS nodes
        self.stop_laptop_ros_nodes()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    gui = RobotGUI()

    try:
        gui.run()
    except KeyboardInterrupt:
        pass

    gui.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()