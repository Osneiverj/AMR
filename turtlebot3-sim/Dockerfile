# Usa la imagen oficial de ROS 2 Humble Desktop como base
FROM osrf/ros:humble-desktop

# Instala todas las dependencias de ROS 2
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    sed \
    ros-humble-turtlebot3-* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-nav2-bringup \
    ros-humble-rosbridge-suite \
    ros-humble-slam-toolbox && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Crea, copia y compila el workspace de ROS 2
WORKDIR /ros2_ws
COPY orchestrator ./src/orchestrator
 # Usa bash para cargar correctamente el entorno de ROS 2 y construir
# build en “merge-install” para que quede todo en /ros2_ws/install/{bin,share,…}
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    colcon build --merge-install --packages-select orchestrator \
  " && \
  rm -rf log src

# --- CONFIGURACIÓN DEL ENTRYPOINT ---
# Copia el script de entrada a la raíz de la imagen
COPY entrypoint.sh /

# SOLUCIÓN: Corrige las terminaciones de línea de Windows (CRLF) a Unix (LF)
RUN sed -i 's/\r$//' /entrypoint.sh

# Da permisos de ejecución al script
RUN chmod +x /entrypoint.sh

# Establece el script como el punto de entrada del contenedor
ENTRYPOINT ["/entrypoint.sh"]

# El CMD ahora está vacío, ya que el entrypoint se encarga de todo
CMD []