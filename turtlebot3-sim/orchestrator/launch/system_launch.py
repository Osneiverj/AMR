#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource, XmlLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Obtener rutas de los paquetes instalados
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    slam_toolbox_pkg     = get_package_share_directory('slam_toolbox')
    nav2_bringup_pkg     = get_package_share_directory('nav2_bringup')
    rosbridge_pkg        = get_package_share_directory('rosbridge_server')

    # Construir paths absolutos a cada launch file
    gazebo_launch_file    = os.path.join(turtlebot3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')
    slam_launch_file      = os.path.join(slam_toolbox_pkg,       'launch', 'online_async_launch.py')
    nav2_launch_file      = os.path.join(nav2_bringup_pkg,       'launch', 'navigation_launch.py')
    rosbridge_launch_file = os.path.join(rosbridge_pkg,          'launch', 'rosbridge_websocket_launch.xml')

    # DEBUG: Log existencia de archivos
    debug_logs = []
    for path in [gazebo_launch_file, slam_launch_file, nav2_launch_file, rosbridge_launch_file]:
        debug_logs.append(LogInfo(msg=f'DEBUG: File exists {path}? {os.path.isfile(path)}'))

    # 1) Gazebo TurtleBot3 world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # 2) SLAM Toolbox (async lifecycle, autostart=False)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'autostart': 'False',
            'use_sim_time': 'True'
        }.items()
    )

    # 3) Nav2 bringup (lifecycle, autostart=False)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'autostart': 'False',
            'use_sim_time': 'True'
        }.items()
    )

    # 4) rosbridge_websocket (XML launch)
    rosbridge = IncludeLaunchDescription(
        XmlLaunchDescriptionSource(rosbridge_launch_file),
        launch_arguments={
            'port': '9090'
        }.items()
    )

    # 5) Nodo Orchestrator
    orchestrator_node = Node(
        package='orchestrator',
        executable='orchestrator',
        name='ui_orchestrator',
        output='screen'
    )

    # Devolver todas las acciones: debug + componentes
    return LaunchDescription(debug_logs + [
        gazebo,
        slam_toolbox,
        nav2,
        rosbridge,
        orchestrator_node
    ])
