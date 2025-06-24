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

# 3) Iniciar Map Server sin cargar un mapa inicial
ros2 run nav2_map_server map_server \
  --ros-args \
    -p use_sim_time:=True &

# 4) Iniciar SLAM Toolbox en modo async
echo "--- [DEBUG] Iniciando slam_toolbox (online_async_launch.py, nodo: slam_toolbox) ---"
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=True &


# 5) Iniciar lifecycle manager del Map Server, sin autostart
echo "--- [DEBUG] Iniciando lifecycle_manager_map_server (autostart=False) ---"
ros2 run nav2_lifecycle_manager lifecycle_manager \
  --ros-args \
    -p use_sim_time:=True \
    -p autostart:=False \
    -p node_names:="['map_server']" \
    -r __node:=lifecycle_manager_map_server &

# 6) Iniciar lifecycle manager para slam_toolbox
echo "--- [DEBUG] Iniciando lifecycle_manager_slam_toolbox (autostart=False) ---"
ros2 run nav2_lifecycle_manager lifecycle_manager \
  --ros-args \
    -p use_sim_time:=True \
    -p autostart:=False \
    -p node_names:="['slam_toolbox']" \
    -r __node:=lifecycle_manager_slam_toolbox &

# 7) Esperar a que los nodos arranquen
sleep 5

# Debug: listar nodos y servicios activos
echo "--- [DEBUG] Nodos activos tras arranque ---"
ros2 node list
echo "--- [DEBUG] Servicios activos tras arranque ---"
ros2 service list

# 8) Arrancar rosbridge_websocket en background
echo "--- [DEBUG] Iniciando rosbridge_websocket en puerto 9090 ---"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &

# 9) Esperar a que tome el puerto
sleep 5

# 10) Ejecutar Orchestrator
echo "--- [DEBUG] Ejecutando Orchestrator ---"
exec /ros2_ws/install/bin/orchestrator "$@"