#!/bin/bash

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

stage_1() {
  echo "Stage 1: Go straight for 0.9 seconds"
  publish_cmd "$FORWARD_SPEED" 0.0 9
  stop_now
}

stage_2() {
  echo "Stage 2: Turn right 90 degrees"
  publish_cmd 0.0 "-$TURN_SPEED" 5
  stop_now
}

stage_3() {
  echo "Stage 3: Go straight for 1.7 seconds"
  publish_cmd "$FORWARD_SPEED" 0.0 17
  stop_now
}

stage_4() {
  echo "Stage 4: Turn right 90 degrees"
  publish_cmd 0.0 "-$TURN_SPEED" 5
  stop_now
}

stage_5() {
  echo "Stage 5: Go straight for 0.9 seconds"
  publish_cmd "$FORWARD_SPEED" 0.0 9
  stop_now
}

run_all() {
  echo "Running path: Stage 1 to Stage 5 only"

  stage_1
  stage_2
  stage_3
  stage_4
  stage_5

  stop_now
  echo "Path complete."
}
