#!/bin/bash
set -e
WORKSPACE="$HOME/catkin_ws"
cd "$WORKSPACE"
echo "Building catkin workspace..."
catkin_make
echo "Sourcing setup.bash"
source devel/setup.bash
echo "To run the core launch:"
echo "  roslaunch core core.launch"
