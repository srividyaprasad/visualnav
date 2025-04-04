#!/bin/bash

# Create a new tmux session
session_name="vint_locobot_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into four panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves

# Run the navigate.py script with command line args in the second pane
tmux select-pane -t 0
#tmux send-keys "source ../../.venv/bin/activate" Enter
tmux send-keys "python explore.py --model nomad" Enter

# Run the pd_controller.py script in the fourth pane
tmux select-pane -t 1
# tmux send-keys "conda activate vint_deployment" Enter
tmux send-keys "python pd_controller.py" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name
