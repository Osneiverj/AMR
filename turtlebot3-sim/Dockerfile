# Usa la imagen oficial de ROS 2 Humble Desktop como base
FROM osrf/ros:humble-desktop

# Ensure colcon env vars are defined to avoid unbound references
ENV COLCON_TRACE=0
ENV ROS_DOMAIN_ID=30 \
    TURTLEBOT3_MODEL=burger \
    TB3_SIM_GUI=false \
    ROS_LOG_DIR=/root/maps/ros_logs \
    PYTHONUNBUFFERED=1

# Instala todas las dependencias de ROS 2 y del orquestador
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    sed \
    python3-pip \
    graphviz \
    ros-humble-turtlebot3-* \
    ros-humble-turtlebot3-gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-rosbridge-server \
    ros-humble-slam-toolbox \
    ros-humble-py-trees-ros \
    ros-humble-rosidl-default-generators && \
    pip install py-trees pydot && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Crea, copia y compila el workspace de ROS 2
WORKDIR /ros2_ws

# Clone TB3 sources INSIDE workspace
RUN apt-get update && apt-get install -y git && git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git src/turtlebot3
    
COPY orchestrator_interfaces ./src/orchestrator_interfaces
COPY orchestrator ./src/orchestrator

# Compila el workspace
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --merge-install

# --- CONFIGURACIÓN DEL ENTRYPOINT ---
# Copia el script de entrada a la raíz de la imagen
COPY entrypoint.sh /

# SOLUCIÓN: Corrige las terminaciones de línea de Windows (CRLF) a Unix (LF)
RUN sed -i 's/\r$//' /entrypoint.sh

# Da permisos de ejecución al script
RUN chmod +x /entrypoint.sh

# Establece el script como el punto de entrada del contenedor
ENTRYPOINT ["/entrypoint.sh"]

CMD ["ros2", "launch", "orchestrator", "system_launch.py"]
