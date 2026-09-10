#!/bin/bash
echo "Setting up CivicLens AI Workspace..."
cd ~/Downloads/sa
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
echo "Setup complete! Run ./src/civiclens_gazebo/scripts/run.sh to start."
