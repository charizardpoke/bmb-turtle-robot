# RUNNING COMMANDS:
# clockwise
# anticlockwise
# clockwise
# anticlockwise

#!/bin/bash

CMD_TOPIC=/diff_drive_controller/cmd_vel
RATE=10

TURN_SPEED=0.5

# Change this to tune full 360 turn time
# Bigger number = turns longer
# Smaller number = turns less
TURN_SECONDS=3.2

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

clockwise() {
  echo "Turning clockwise 360 degrees for $TURN_SECONDS seconds"
  publish_cmd_seconds 0.0 "-$TURN_SPEED" "$TURN_SECONDS"
  stop_now
}

anticlockwise() {
  echo "Turning anticlockwise 360 degrees for $TURN_SECONDS seconds"
  publish_cmd_seconds 0.0 "$TURN_SPEED" "$TURN_SECONDS"
  stop_now
}

echo "Loaded turning commands."
echo "Type: clockwise"
echo "Type: anticlockwise"
