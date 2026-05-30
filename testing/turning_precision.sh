# RUNNING COMMANDS:
# anticlockwise_turn_90
# anticlockwise_turn_180
# anticlockwise_turn_360
# clockwise_90
# clockwise_180
# lockwise_360

#!/bin/bash

CMD_TOPIC=/diff_drive_controller/cmd_vel
RATE=10

TURN_SPEED=0.5

publish_cmd_seconds() {
  local linear_x=$1
  local angular_z=$2
  local seconds=$3

  local times
  times=$(python3 - <<EOF
print(round($seconds * $RATE))
EOF
)

  ros2 topic pub --times "$times" -r "$RATE" "$CMD_TOPIC" geometry_msgs/msg/TwistStamped \
  "{\"header\":{\"frame_id\":\"base_footprint\"},\"twist\":{\"linear\":{\"x\":$linear_x},\"angular\":{\"z\":$angular_z}}}"
}

stop_now() {
  echo "STOPPING robot..."
  publish_cmd_seconds 0.0 0.0 1.0
}

turn_90() {
  echo "Turning 90 degrees for 0.3 seconds"
  publish_cmd_seconds 0.0 "$TURN_SPEED" 0.3
  stop_now
}

turn_180() {
  echo "Turning 180 degrees for 1.3 seconds"
  publish_cmd_seconds 0.0 "$TURN_SPEED" 1.3
  stop_now
}

turn_360() {
  echo "Turning 360 degrees for 2.95 seconds"
  publish_cmd_seconds 0.0 "$TURN_SPEED" 2.95
  stop_now
}

clockwise_90() {
  echo "Turning clockwise 90 degrees for 0.3 seconds"
  publish_cmd_seconds 0.0 "-$TURN_SPEED" 0.3
  stop_now
}

clockwise_180() {
  echo "Turning clockwise 180 degrees for 1.3 seconds"
  publish_cmd_seconds 0.0 "-$TURN_SPEED" 1.3
  stop_now
}

clockwise_360() {
  echo "Turning clockwise 360 degrees for 3.1 seconds"
  publish_cmd_seconds 0.0 "-$TURN_SPEED" 3.1
  stop_now
}

echo "Loaded turning commands."
echo "Use:"
echo "  anticlockwise_turn_90"
echo "  anticlockwise_turn_180"
echo "  anticlockwise_turn_360"
echo "  clockwise_90"
echo "  clockwise_180"
echo "  clockwise_360"