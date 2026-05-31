#!/bin/bash

# =========================
# WALL-E Auto Movement Script
# =========================
# RUN:
# source ./auto_script.sh
# clockwise
# anticlockwise
# turn180
# stop_now

CMD_TOPIC=/diff_drive_controller/cmd_vel
RATE=10

# Speed settings
FORWARD_SPEED=0.5
TURN_SPEED=0.5

# =========================
# Publish velocity command
# =========================
publish_cmd() {
  local linear_x=$1
  local angular_z=$2
  local times=$3

  ros2 topic pub --times "$times" -r "$RATE" "$CMD_TOPIC" geometry_msgs/msg/TwistStamped \
  "{\"header\":{\"frame_id\":\"base_footprint\"},\"twist\":{\"linear\":{\"x\":$linear_x},\"angular\":{\"z\":$angular_z}}}"
}

# =========================
# Stop robot
# =========================
stop_now() {
  echo "STOPPING robot..."
  publish_cmd 0.0 0.0 5
  echo "Robot stop command sent."
}

# =========================
# Basic movement functions
# =========================
shortForward() {
  echo "Go straight for 1.5 second"
  publish_cmd "$FORWARD_SPEED" 0.0 15
  stop_now
}

longForward() {
  echo "Go straight for 1.7 seconds"
  publish_cmd "$FORWARD_SPEED" 0.0 17
  stop_now
}

turnRight() {
  echo "Turn right 90 degrees"
  publish_cmd 0.0 "-$TURN_SPEED" 6
  stop_now
}

turnLeft() {
  echo "Turn left 90 degrees"
  publish_cmd 0.0 "$TURN_SPEED" 6
  stop_now
}

turnRight180() {
  echo "Turn right 180 degrees"
  publish_cmd 0.0 "-$TURN_SPEED" 14
  stop_now
}

# =========================
# Auto paths
# =========================
clockwise() {
  stop_now

  echo "Running clockwise path"

  shortForward
  turnRight
  longForward
  turnRight
  shortForward

  stop_now
  echo "Clockwise path complete."
}

anticlockwise() {
  stop_now

  echo "Running anticlockwise path"

  shortForward
  turnLeft
  longForward
  turnLeft
  shortForward

  stop_now
  echo "Anticlockwise path complete."
}

turn180() {
  stop_now

  echo "Running 180 degree turn"

  turnRight180

  stop_now
  echo "180 turn complete."
}