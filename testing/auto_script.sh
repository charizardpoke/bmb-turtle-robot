<<<<<<< HEAD
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
=======
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
>>>>>>> 2b1ab17 (Initial commit)

CMD_TOPIC=/diff_drive_controller/cmd_vel
RATE=10

<<<<<<< HEAD
# Speed settings
FORWARD_SPEED=0.5
TURN_SPEED=0.5

# Time settings converted to message counts
# At -r 10:
# 9 messages  = 0.9 sec
# 17 messages = 1.7 sec
# 10 messages = 1.0 sec stop

=======
FORWARD_SPEED=0.5
TURN_SPEED=0.5

# =========================
# Publish velocity command
# =========================
>>>>>>> 2b1ab17 (Initial commit)
publish_cmd() {
  local linear_x=$1
  local angular_z=$2
  local times=$3

  ros2 topic pub --times "$times" -r "$RATE" "$CMD_TOPIC" geometry_msgs/msg/TwistStamped \
  "{\"header\":{\"frame_id\":\"base_footprint\"},\"twist\":{\"linear\":{\"x\":$linear_x},\"angular\":{\"z\":$angular_z}}}"
}

<<<<<<< HEAD
stop_now() {
  echo "STOPPING robot..."
  publish_cmd 0.0 0.0 10
}

shortForward() {
  echo "Go straight for 1 seconds"
=======
# =========================
# Hard stop
# =========================
stop_now() {
  echo "STOPPING robot..."

  publish_cmd 0.0 0.0 20

  echo "Robot stop command sent."
}

# =========================
# Movement functions
# =========================
shortForward() {
  echo "Go straight for 1 second"
>>>>>>> 2b1ab17 (Initial commit)
  publish_cmd "$FORWARD_SPEED" 0.0 10
  stop_now
}

<<<<<<< HEAD
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

=======
>>>>>>> 2b1ab17 (Initial commit)
longForward() {
  echo "Go straight for 1.7 seconds"
  publish_cmd "$FORWARD_SPEED" 0.0 17
  stop_now
}

<<<<<<< HEAD

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
=======
turnRight() {
  echo "Turn right 90 degrees"
  publish_cmd 0.0 "-$TURN_SPEED" 6
  stop_now
}

turnLeft() {
  echo "Turn left 90 degrees"
>>>>>>> 2b1ab17 (Initial commit)
  publish_cmd 0.0 "$TURN_SPEED" 6
  stop_now
}

<<<<<<< HEAD

anticlockwise() {
  echo "Running path: Stage 1 to Stage 5 only"
=======
turnRight180() {
  echo "Turn right 180 degrees"
  publish_cmd 0.0 "-$TURN_SPEED" 14
  stop_now
}

# =========================
# Paths
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
>>>>>>> 2b1ab17 (Initial commit)

  shortForward
  turnLeft
  longForward
  turnLeft
  shortForward

  stop_now
<<<<<<< HEAD
  echo "Path complete."
}

turn180() {
  echo "Turn 180 degrees"

  turnRight180

  echo "Path complete."
=======
  echo "Anticlockwise path complete."
}

turn180() {
  stop_now

  echo "Running 180 degree turn"

  turnRight180

  stop_now
  echo "180 turn complete."
>>>>>>> 2b1ab17 (Initial commit)
}