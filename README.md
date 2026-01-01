# BMB Turtle Robot 🤖🐢
A differential-drive autonomous mobile robot built on **Raspberry Pi 5** + **ROS 2 Jazzy**, designed to run a complete real-world navigation stack (SLAM + Nav2) with a dedicated low-level controller on **STM32**.

## Video Demo
Watch the robot navigating here:  
https://photos.app.goo.gl/bRwovGnZpZGCW4xPA

---

## What This Robot Does
- **Maps indoor environments** using 2D LiDAR SLAM  
- **Localizes + navigates** using the **Nav2** stack (point-to-point autonomy)
- **Avoids obstacles dynamically**
- **Teleop mode** with a live USB camera stream for remote driving/monitoring
- **Repeatable bringup** (organized launch + params for one-command deployments)

---

## System Architecture (Simple)
- **Raspberry Pi 5** *(high-level brain)* runs:
  - SLAM / Localization
  - Nav2 planning + costmaps
  - ROS 2 nodes + launch system

- **STM32 NUCLEO-H563ZI** sits **between the Pi and the motors/encoders** and handles:
  - Real-time motor control (PWM)
  - Encoder interrupts + odometry calculations
  - Communication to Pi via `ros2_control` hardware interface

---

## Hardware
**Compute**
- Raspberry Pi 5 (Ubuntu 24.04)

**Low-Level Control**
- STM32 Nucleo-144: **NUCLEO-H563ZI**

**Sensors**
- **RPLIDAR C1** (2D LiDAR for SLAM + Nav2)
- **USB camera** (teleop video feed)

**Power & Electronics**
- High-voltage LiPo battery
- Custom **power distribution PCB** (KiCad)
  - High-current motor traces
  - Strain relief + cleaner internal wiring

**Chassis**
- Custom chassis + 3D-printed electronics enclosure  
- Standardized fasteners (faster swaps during maintenance)

---

## Software Stack
- **OS:** Ubuntu 24.04 (Noble)
- **ROS 2:** Jazzy Jalisco
- **Navigation:** Nav2 (tuned costmaps + custom recovery behaviors)
- **Control:** `ros2_control` (diff drive)
- **Simulation/Visualization:** Gazebo + RViz2
- **Languages:** C, C++, Python

---

## Performance (Current)
Tested in a complex floor plan:
- **50+ autonomous runs**
- ~**60% goal success rate** in dynamic scenarios
- **~10–15 cm** average localization error

> Note: Performance varies depending on lighting, floor traction, obstacle motion, and map quality.

---

## Getting Started

### 1) Clone the repo
```bash
mkdir -p ~/bmb_ws/src
cd ~/bmb_ws/src
git clone https://github.com/charizardpoke/bmb-turtle-robot.git
```

### 2) Install dependencies (rosdep)
```bash
cd ~/bmb_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3) Build
```bash
cd ~/bmb_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Running the Robot

### Bring up hardware (drivers + STM32 bridge + descriptions)
```bash
ros2 launch bmb_bringup robot_bringup.launch.py
```

### Start navigation (Nav2)
> Provide a valid map file (`.yaml`).
```bash
ros2 launch bmb_nav2 navigation.launch.py map:=/path/to/map.yaml
```

### Teleoperation (keyboard)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Safety Notes
- First time running a new controller/firmware: **lift the wheels off the ground**
- Keep an easy “kill” method ready (power switch / unplug motor power)
- Verify correct wheel directions before enabling Nav2

---

## Future Work
- Finalizing migration of the `ros2_control` hardware interface to the **NUCLEO-H563ZI**
  - Goal: higher-frequency PWM + cleaner encoder interrupt handling
  - Reduce jitter vs driving motors directly from the Pi GPIO
- Add **depth camera** support for stronger 3D obstacle avoidance

---

## Author
**Doan Phan**  
B.S. Mechanical Engineering — Seattle Pacific University (Expected 2027)

GitHub: https://github.com/charizardpoke/bmb-turtle-robot.git

LinkedIn: www.linkedin.com/in/doan-phan-717a62347

---

## Acknowledgements
ROS 2, Nav2, `ros2_control`, Gazebo, RViz2, and the open-source robotics community.
