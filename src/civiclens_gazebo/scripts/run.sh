#!/bin/bash
cd /home/aadhidhya/Downloads/sa
[ -f /opt/ros/lyrical/setup.bash ] && source /opt/ros/lyrical/setup.bash
source install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH}:/home/aadhidhya/Downloads/sa/install/civiclens_gazebo/lib/civiclens_gazebo
ros2 launch civiclens_gazebo civiclens_sim.launch.py

