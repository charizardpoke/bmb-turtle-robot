#!/bin/bash

CMD_TOPIC=/diff_drive_controller/cmd_vel
RATE=10

# Speed settings
FORWARD_SPEED=0.3
TURN_SPEED=0.5

# Time settings converted to message counts
# At -r 10:
# 12 messages = 1.2 sec
# 20 messages = 2 sec
# 30 messages = 3 sec
# 32 messages = about 90 degree turn
# 63 messages = about 180 degree turn

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

stage_6() {
  echo "Stage 6: Turn 180 degrees"
  publish_cmd 0.0 "$TURN_SPEED" 13
  stop_now
}

stage_7() {
  echo "Stage 7: Go straight for 0.9 seconds"
  publish_cmd "$FORWARD_SPEED" 0.0 9
  stop_now
}

stage_8() {
  echo "Stage 8: Turn left 90 degrees"
  publish_cmd 0.0 "$TURN_SPEED" 5
  stop_now
}

stage_9() {
  echo "Stage 9: Go straight for 1.7 seconds"
  publish_cmd "$FORWARD_SPEED" 0.0 17
  stop_now
}

stage_10() {
  echo "Stage 10: Turn left 90 degrees"
  publish_cmd 0.0 "$TURN_SPEED" 5
  stop_now
}

stage_11() {
  echo "Stage 11: Go straight for 0.9 seconds"
  publish_cmd "$FORWARD_SPEED" 0.0 9
  stop_now
}

stage_12() {
  echo "Stage 12: Turn 180 degrees"
  publish_cmd 0.0 "$TURN_SPEED" 12
  stop_now
}

run_all() {
  echo "Running full path..."
  echo "Stage 1 starts immediately."

  stage_1
  stage_2
  stage_3
  stage_4
  stage_5
  stage_6
  stage_7
  stage_8
  stage_9
  stage_10
  stage_11
  stage_12

  stop_now
  echo "Full path complete."
}
