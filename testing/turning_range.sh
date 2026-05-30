#!/bin/bash

CMD_TOPIC=/diff_drive_controller/cmd_vel
RATE=10

# Speed settings
TURN_SPEED=0.5

# Turning time settings
# At -r 10:
# 5 messages  = short turn test
# 32 messages = about 90 degrees
# 63 messages = about 180 degrees
# 126 messages = about 360 degrees
TURN_360_TIMES=126

publish_cmd() {
  local linear_x=$1
  local angular_z=$2
  local times=$3

  ros2 topic pub --times "$times" -r "$RATE" "$CMD_TOPIC" geometry_msgs/msg/TwistStamped \
  "{\"header\":{\"frame_id\":\"base_footprint\"},\"twist\":{\"linear\":{\"x\":$linear_x},\"angular\":{\"z\":$angular_z}}}"
}

stop_now() {
  echo "STOPPING robot..."
  publish_cmd 0.0 0.0 10
}

clockwise() {
  echo "Turning clockwise 360 degrees"
  publish_cmd 0.0 "-$TURN_SPEED" "$TURN_360_TIMES"
  stop_now
  echo "Clockwise turn complete."
}

anticlockwise() {
  echo "Turning anticlockwise 360 degrees"
  publish_cmd 0.0 "$TURN_SPEED" "$TURN_360_TIMES"
  stop_now
  echo "Anticlockwise turn complete."
}
