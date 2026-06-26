#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share
export PATH=$PATH:/opt/ros/jazzy/opt/gz_tools_vendor/bin
export TURTLEBOT4_MODEL=standard

if [ -z "${XDG_RUNTIME_DIR:-}" ]; then
  export XDG_RUNTIME_DIR=/tmp/runtime-root
fi
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

function cleanup_stale_processes() {
  pkill -f 'ros2 run prob_pkg' || true
  pkill -f 'ros2 run tf2_ros static_transform_publisher' || true
  pkill -f 'static_transform_publisher' || true
  pkill -f 'ros2 run rviz2 rviz2' || true
  pkill -f 'rviz2' || true
  pkill -f 'ros2 launch nav2_bringup tb4_simulation_launch.py' || true
  pkill -f 'gz sim' || true
  pkill -f 'gzserver' || true
  pkill -f 'gzclient' || true
  ros2 daemon stop || true
}

cleanup_stale_processes

headless=false
if [ -z "${DISPLAY:-}" ]; then
  echo "DISPLAY is not set; starting the simulation headless."
  headless=true
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_CONFIG="${SCRIPT_DIR}/rviz/config_covar.rviz"

if [ "$headless" = true ]; then
  echo "Launching simulation headless..."
  ros2 launch nav2_bringup tb4_simulation_launch.py headless:=True &
else
  echo "Launching simulation with GUI..."
  ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False &
fi
sim_pid=$!

trap 'echo "Stopping processes..."; cleanup_stale_processes; exit' EXIT

# wait until the simulation begins publishing /clock before launching the filters
echo "Waiting for simulation to publish /clock..."
clock_ready=false
for i in $(seq 1 30); do
  if ros2 topic list --spin-time 1 | grep -qx '/clock'; then
    clock_ready=true
    break
  fi
  sleep 1
done
if [ "$clock_ready" = false ]; then
  echo "Warning: /clock did not appear within 30s. Continuing anyway." >&2
fi

echo "Starting filter nodes, S-curve driver, TF publisher, and RViz..."
ros2 run prob_pkg kf_node &
ros2 run prob_pkg ekf_node &
ros2 run prob_pkg pf_node &
ros2 run prob_pkg s_curve_node &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &

if [ "$headless" = false ]; then
  if [ -f "$RVIZ_CONFIG" ]; then
    ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" &
  else
    echo "Warning: RViz config not found at $RVIZ_CONFIG"
    ros2 run rviz2 rviz2 &
  fi
fi

wait $sim_pid
