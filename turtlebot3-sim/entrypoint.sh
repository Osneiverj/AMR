#!/bin/bash
set -e

echo "--- [DEBUG] Entrypoint iniciado. ---"

# 1) Cargar entornos ROS 2
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# 2) Arrancar Gazebo con TurtleBot3
echo "--- [DEBUG] Iniciando Gazebo TurtleBot3 world ---"
# Model debe estar seteado por tu docker-compose: TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &  # :contentReference[oaicite:0]{index=0}
sleep 5  # dejar unos segundos para que Gazebo lance el mundo

# 3) Iniciar Map Server (sin activarlo)
ros2 run nav2_map_server map_server \
  --ros-args \
    -p yaml_filename:="/root/maps/default.yaml" \
    -p use_sim_time:=True &

# 4) Iniciar lifecycle manager del Map Server, sin autostart
echo "--- [DEBUG] Iniciando lifecycle_manager_map_server (autostart=False) ---"
ros2 run nav2_lifecycle_manager lifecycle_manager \
  --ros-args \
    -p use_sim_time:=True \
    -p autostart:=False \
    -p node_names:="['map_server']" \
    -r __node:=lifecycle_manager_map_server &

sleep 2

# 5) Arrancar rosbridge_websocket en background
echo "--- [DEBUG] Iniciando rosbridge_websocket en puerto 9090 ---"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &

# 6) Esperar a que tome el puerto
sleep 2

# 7) Ejecutar Orchestrator
echo "--- [DEBUG] Ejecutando Orchestrator ---"
exec /ros2_ws/install/bin/orchestrator "$@"
