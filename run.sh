#!/usr/bin/env bash
# ==============================================================================
# CivicLens AI — Autonomous Smart Waste Management System
# Complete ROS 2 + Gazebo Sim + AI Vision IoT + Live Telemetry Dashboard Launcher
# ==============================================================================

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_DIR="$SCRIPT_DIR"

echo "======================================================================"
echo "           CIVICLENS AI — AUTONOMOUS WASTE MANAGEMENT SYSTEM          "
echo "======================================================================"
echo " Distro: ROS 2 Lyrical | Sim: Gazebo Sim | Dashboard: http://localhost:8080"
echo " Streets: 5 Streets (DB-01 to DB-05) | Segregation: Bio / Non-Bio / Plastic"
echo "======================================================================"

# 1. Source ROS 2 environment
if [ -f "/opt/ros/lyrical/setup.bash" ]; then
    source /opt/ros/lyrical/setup.bash
    echo "[✓] Sourced ROS 2 Lyrical"
else
    echo "[✗] ROS 2 Lyrical not found at /opt/ros/lyrical"
    echo "    This repository currently uses modern gz-* Gazebo APIs."
    echo "    ROS 2 Humble normally uses Gazebo Fortress; a Humble port is still required."
    echo "    Read the Runtime compatibility section in README.md before changing ROS_DISTRO."
    exit 1
fi

# 2. Source Workspace environment
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
    echo "[✓] Sourced Workspace Overlay ($WORKSPACE_DIR/install)"
fi

# 3. Export Gazebo Environment Paths
export GZ_SIM_RESOURCE_PATH="$WORKSPACE_DIR/src/civiclens_gazebo/models:$GZ_SIM_RESOURCE_PATH"
export SDF_PATH="$WORKSPACE_DIR/src/civiclens_gazebo/models:$SDF_PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$WORKSPACE_DIR/install/civiclens_gazebo/lib/civiclens_gazebo:$WORKSPACE_DIR/install/civiclens_gazebo/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GAZEBO_PLUGIN_PATH="$WORKSPACE_DIR/install/civiclens_gazebo/lib/civiclens_gazebo:$WORKSPACE_DIR/install/civiclens_gazebo/lib:$GAZEBO_PLUGIN_PATH"
export LD_LIBRARY_PATH="$WORKSPACE_DIR/install/civiclens_gazebo/lib/civiclens_gazebo:$WORKSPACE_DIR/install/civiclens_gazebo/lib:$LD_LIBRARY_PATH"

echo "[✓] Configured Gazebo Sim Resource & Plugin Paths"
echo "----------------------------------------------------------------------"
echo "Starting CivicLens AI Full System Launch..."
echo "Open your browser at: http://localhost:8080 for Live Operations Dashboard"
echo "Press Ctrl+C to terminate the entire mission fleet."
echo "======================================================================"

ros2 launch civiclens_gazebo civiclens_sim.launch.py
