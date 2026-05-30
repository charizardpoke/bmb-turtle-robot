#!/bin/bash

CMD_TOPIC=/diff_drive_controller/cmd_vel
RATE=10

# Speed settings
FORWARD_SPEED=0.5
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
  echo "Stage 3: Turn right 90 degrees"
  publish_cmd 0.0 "-$TURN_SPEED" 5
  stop_now
}

stage_3() {
  echo "Stage 4: Go straight for 1.5 seconds"
  publish_cmd "$FORWARD_SPEED" 0.0 15
  stop_now
}

stage_4() {
  echo "Stage 5: Turn right 90 degrees"
  publish_cmd 0.0 "-$TURN_SPEED" 5
  stop_now
}

stage_5() {
  echo "Stage 6: Go straight for 0.9 seconds"
  publish_cmd "$FORWARD_SPEED" 0.0 9
  stop_now
}

stage_6() {
  echo "Stage 7: Turn 180 degrees"
  publish_cmd 0.0 "$TURN_SPEED" 13
  stop_now
}

stage_7() {
  echo "Stage 8: Go straight for 0.9 seconds"
  publish_cmd "$FORWARD_SPEED" 0.0 9
  stop_now
}

stage_8() {
  echo "Stage 9: Turn left 90 degrees"
  publish_cmd 0.0 "$TURN_SPEED" 5
  stop_now
}

stage_9() {
  echo "Stage 10: Go straight for 1.5 seconds"
  publish_cmd "$FORWARD_SPEED" 0.0 15
  stop_now
}

stage_10() {
  echo "Stage 11: Turn left 90 degrees"
  publish_cmd 0.0 "$TURN_SPEED" 5
  stop_now
}

stage_11() {
  echo "Stage 12: Go straight for 0.9 seconds"
  publish_cmd "$FORWARD_SPEED" 0.0 9
  stop_now
}

stage_12() {
  echo "Stage 13: Turn 180 degrees"
  publish_cmd 0.0 "$TURN_SPEED" 13
  stop_now
}

run_all() {
  echo "Running full path..."
  echo "Stage 1 starts immediately."

  stage_1
  echo "Waiting 5 seconds before next stage..."
  sleep 5

  stage_2
  echo "Waiting 5 seconds before next stage..."
  sleep 5

  stage_3
  echo "Waiting 5 seconds before next stage..."
  sleep 5

  stage_4
  echo "Waiting 5 seconds before next stage..."
  sleep 5

  stage_5
  echo "Waiting 5 seconds before next stage..."
  sleep 5

  stage_6
  echo "Waiting 5 seconds before next stage..."
  sleep 5

  stage_7
  echo "Waiting 5 seconds before next stage..."
  sleep 5

  stage_8
  echo "Waiting 5 seconds before next stage..."
  sleep 5

  stage_9
  echo "Waiting 5 seconds before next stage..."
  sleep 5

  stage_10
  echo "Waiting 5 seconds before next stage..."
  sleep 5

  stage_11
  echo "Waiting 5 seconds before next stage..."
  sleep 5

  stage_12

  stop_now
  echo "Full path complete."
}