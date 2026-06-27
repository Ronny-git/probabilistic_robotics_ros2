# probabilistic_robotics_ros2


🚀 ROS2 Nav2 + TurtleBot4 Simulation (Docker Setup)

🧠 Ziel
Nav2 Simulation starten
TurtleBot4 Simulation (TB4)
Workspace + eigene ROS2 Nodes bleiben getrennt

1) 🟢 Host vorbereiten (einmalig pro Neustart)

xhost +local:docker

2) 🐳 Container starten (WICHTIG: NICHT neu bauen!)

docker start ros2_nav2
docker exec -it ros2_nav2 bash

3) 📦 ROS Umgebung laden (IM CONTAINER)

source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

4) 🤖 TurtleBot4 Setup + Simulation Variablen

export TURTLEBOT3_MODEL=waffle

export PATH=$PATH:/opt/ros/jazzy/opt/gz_tools_vendor/bin
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share

5) 🚀 Simulation starten (DEIN WORKING COMMAND)

ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False

6) ⚡ Alternative: über Script starten

xhost +local:docker

docker start ros2_nav2

docker exec -it ros2_nav2 bash

Wenn start_sim.sh existiert:

cd /root/ros2_ws
./start_sim.sh

Measurement dropout (sensor interruption) simulation
-------------------------------------------------
Each filter node supports simulated measurement dropouts via ROS parameters. Configure them before running the node.

Parameters (namespace `dropout`):
- `enabled` (bool): enable dropout simulation (default: false)
- `period` (double): period of a repeating dropout window in seconds (default: 0.0)
- `duration` (double): duration of each dropout window in seconds (default: 0.0)
- `random` (bool): enable per-measurement random dropout (default: false)
- `probability` (double): probability [0,1] of dropping an individual measurement when `random` is true (default: 0.0)

Examples:
- Enable periodic dropouts of 1 second every 10 seconds:
    `ros2 run prob_pkg kf_node --ros-args --param dropout.enabled:=true --param dropout.period:=10.0 --param dropout.duration:=1.0`
- Enable random dropouts with 10% probability per measurement:
    `ros2 run prob_pkg ekf_node --ros-args --param dropout.enabled:=true --param dropout.random:=true --param dropout.probability:=0.1`

The nodes will ignore incoming measurements during simulated dropouts; use this to evaluate estimator robustness and compute RMSE against ground truth while varying dropout parameters.
