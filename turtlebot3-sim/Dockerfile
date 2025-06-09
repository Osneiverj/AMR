# turtlebot3-sim/Dockerfile
FROM osrf/ros:humble-desktop

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-humble-turtlebot3-* \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
        ros-humble-nav2-bringup && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-rosbridge-suite && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y ros-humble-slam-toolbox

# Copiar y compilar paquete orchestrator
COPY orchestrator /ros2_ws/src/orchestrator
RUN . /opt/ros/humble/setup.sh && \
    cd /ros2_ws && colcon build --packages-select orchestrator && \
    rm -rf build log


ENV TURTLEBOT3_MODEL=burger
CMD ["/bin/bash"]
