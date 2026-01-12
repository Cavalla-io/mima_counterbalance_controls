#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash

tmux new-session -d -s robot

# Control
tmux rename-window -t robot:0 'control_node'
tmux send-keys -t robot:0 'ros2 launch forklift_control control.launch.py' Enter

tmux attach -t robot
```

**docker/.tmux.conf:**
```
set -g mouse on
set -g history-limit 10000
set -g default-terminal "screen-256color"
bind | split-window -h
bind - split-window -v
bind r source-file ~/.tmux.conf \; display "Reloaded!"
set -g status-bg black
set -g status-fg white
set -g status-left '#[fg=green]#S '
set -g status-right '#[fg=yellow]%H:%M'