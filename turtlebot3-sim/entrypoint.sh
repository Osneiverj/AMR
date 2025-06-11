#!/bin/bash

echo "--- [DEBUG] Entrypoint iniciado. Usando shell: $SHELL ---"

echo ""
echo "--- [DEBUG] Verificando archivos de setup..."
ls -l /opt/ros/humble/setup.bash
ls -l /ros2_ws/install/setup.bash
echo ""

# --- Paso A: Cargar el entorno global de ROS ---
echo "--- [DEBUG] Cargando /opt/ros/humble/setup.bash..."
source /opt/ros/humble/setup.bash
echo "--- [DEBUG] -> ¿Se encontró el comando 'ros2'?: $(which ros2 || echo 'No encontrado')"
echo "--- [DEBUG] -> ¿Variable ROS_DISTRO?: '$ROS_DISTRO'"
echo ""

# --- Paso B: Cargar el entorno del workspace local ---
echo "--- [DEBUG] Cargando /ros2_ws/install/setup.bash..."
source /ros2_ws/install/setup.bash
echo "--- [DEBUG] -> ¿ROS_PACKAGE_PATH final?: '$ROS_PACKAGE_PATH'"
echo ""

# --- Paso C: Intentar la ejecución ---
echo "--- [DEBUG] Intentando ejecutar el orquestador..."
exec ros2 run orchestrator orchestrator