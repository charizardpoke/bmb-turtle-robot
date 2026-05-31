# RUNNING COMMANDS:
# source ./auto_script.sh
# stage_1
# stage_2
# stage_3
# stage_4
# stage_5
# stage_6
# stage_7
# stage_8
# stage_9
# stage_10
# stage_11

# run_all
# #!/bin/bash

CMD_TOPIC=/diff_drive_controller/cmd_vel
RATE=10

# Speed settings
FORWARD_SPEED=0.5
TURN_SPEED=0.5

# Time settings converted to message counts
# At -r 10:
# 9 messages  = 0.9 sec
# 17 messages = 1.7 sec
# 10 messages = 1.0 sec stop

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

shortForward() {
  echo "Go straight for 1 seconds"
  publish_cmd "$FORWARD_SPEED" 0.0 10
  stop_now
}

turnRight() {
  echo "Turn right 90 degrees"
  publish_cmd 0.0 "-$TURN_SPEED" 6
  stop_now
}

turnRight180() {
  echo "Turn right 180 degrees"
  publish_cmd 0.0 "-$TURN_SPEED" 14
  stop_now
}

longForward() {
  echo "Go straight for 1.7 seconds"
  publish_cmd "$FORWARD_SPEED" 0.0 17
  stop_now
}


clockwise() {
  echo "Running path: Stage 1 to Stage 5 only"

  shortForward
  turnRight
  longForward
  turnRight
  shortForward

  stop_now
  echo "Path complete."
}

turnLeft() {
  echo "Turn right 90 degrees"
  publish_cmd 0.0 "$TURN_SPEED" 6
  stop_now
}


anticlockwise() {
  echo "Running path: Stage 1 to Stage 5 only"

  shortForward
  turnLeft
  longForward
  turnLeft
  shortForward

  stop_now
  echo "Path complete."
}

turn180() {
  echo "Turn 180 degrees"

  turnRight180

  echo "Path complete."
}