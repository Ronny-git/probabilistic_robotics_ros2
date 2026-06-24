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

docker start ros2_nav2

docker exec -it ros2_nav2 bash

Wenn start_sim.sh existiert:

cd /root/ros2_ws
./start_sim.sh

