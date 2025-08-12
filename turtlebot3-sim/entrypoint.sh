#!/bin/bash
# Avoid -u while sourcing ROS setup (COLCON_* may be unset)
set -eo pipefail

# Carga el entorno de ROS 2
source /ros2_ws/install/setup.bash
# Now enable -u for the rest of the script
set -u

# Preparar logging persistente hacia el volumen mapeado
ROS_LOG_DIR="${ROS_LOG_DIR:-/root/.ros/log}"
mkdir -p "$ROS_LOG_DIR" /root/maps
OUT_LOG="/root/maps/launch_stdout.log"

{
  echo "== ENV =="
  echo "ROS_LOG_DIR=$ROS_LOG_DIR"
  echo "TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-}" 
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"
  echo "CMD: $*"
  echo "============"
} | tee -a "$OUT_LOG"

# Ejecuta el comando y multiplica salida a archivo en /root/maps
"$@" 2>&1 | tee -a "$OUT_LOG"
status=${PIPESTATUS[0]}

{
  echo "== LAUNCH EXIT == code=$status"
  echo "Logs en: $ROS_LOG_DIR"
  ls -l "$ROS_LOG_DIR" || true
} | tee -a "$OUT_LOG"

exit $status
