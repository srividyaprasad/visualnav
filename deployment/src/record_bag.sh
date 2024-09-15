#!/bin/bash

# Wait for user to start recording
echo "Launching teleop in a separate window..."
echo "Press Enter when ready to start recording..."

# Launch teleop in a separate terminal
gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard"

read  # Wait for user input

# Start recording the bag
echo "Starting to record bag: $1"
cd ../topomaps/bags && ros2 bag record /rgb -o $1