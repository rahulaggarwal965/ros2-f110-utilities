#!/bin/sh

PACKAGE_NAME="$1"
NODE_NAME="$2"
PARAMS_FILE="$3"

shift 3

SHELL_NAME="$(basename ${SHELL})"
CONFIG_DIR="src/${PACKAGE_NAME}/config"

tmux new-session -d -s ${PACKAGE_NAME}
tmux send-keys "source install/setup.${SHELL_NAME}" Enter
tmux send-keys "ros2 launch f1tenth_gym_ros gym_bridge_launch.py" "C-l"
tmux split-window -h
tmux send-keys "source install/setup.${SHELL_NAME}" Enter
tmux send-keys "ros2 run ${PACKAGE_NAME} ${NODE_NAME} --ros-args --params-file ${CONFIG_DIR}/${PARAMS_FILE} ${@}" "C-l"
tmux split-window -v -t 1
tmux send-keys "cd ${CONFIG_DIR}" Enter
tmux send-keys "vim ${PARAMS_FILE}" Enter
tmux new-window
tmux send-keys "colcon build --packages-up-to ${PACKAGE_NAME}" "C-l"
tmux select-window -t ${PACKAGE_NAME}:0
tmux select-pane -t 0
tmux attach-session -t ${PACKAGE_NAME}
