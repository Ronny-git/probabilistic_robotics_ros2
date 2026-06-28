Anleitung Docker container starten und über start script Simulation starten.

xhost +local:docker

docker start ros2_nav2

docker exec -it ros2_nav2 bash

Wenn start_sim.sh existiert:

cd /root/ros2_ws
./start_sim.sh






Der vollständige Workspace für die Funktionierende Simulation befindet sich im Branch Version4
