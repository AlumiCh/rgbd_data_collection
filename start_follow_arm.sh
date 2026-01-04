#!/usr/bin/env bash
# Copyright [2023] Boston Dynamics AI Institute, Inc.

# Ensure you have 'export VLFM_PYTHON=<PATH_TO_PYTHON>' in your .bashrc, where
# <PATH_TO_PYTHON> is the path to the python executable for your conda env
# (e.g., PATH_TO_PYTHON=`conda activate <env_name> && which python`)


session_name=airbot_666_follow

# Create a detached tmux session
tmux new-session -d -s ${session_name}

# Split both panes horizontally
tmux split-window -h -t ${session_name}:0.0

# Run commands in each pane
echo "launch follower 0"
tmux send-keys -t ${session_name}:0.0 "airbot_server -i can_left -p 50051" C-m
sleep 15
echo "launch follower 1"
tmux send-keys -t ${session_name}:0.1 "airbot_server -i can_right -p 50053" C-m

# Attach to the tmux session to view the windows
echo "Created tmux session '${session_name}'"
echo "Run the following to monitor all the server commands:"
echo "tmux attach-session -t ${session_name}"
tmux attach-session -t ${session_name}
