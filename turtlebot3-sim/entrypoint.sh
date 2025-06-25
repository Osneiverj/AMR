#!/bin/bash
set -e

echo "--- [DEBUG] Entrypoint iniciado. ---"

# 1) Cargar entornos ROS 2
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# 2) Lanzar todo el sistema con un único launch file desde el paquete ROS 2
echo "--- [DEBUG] Lanzando orchestrator/system_launch.py (todo el stack) ---"
ros2 launch orchestrator system_launch.py

# (No hace falta lanzar nodos individuales ni sleeps)