#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


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
        launch_arguments={
            'use_sim_time': 'True',
            'world': LaunchConfiguration('world')
        }.items()
    )

    # 2) SLAM Toolbox (async lifecycle, autostart=True) - opcional
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'autostart': 'True',
            'use_sim_time': 'True',
            'slam_params_file': LaunchConfiguration('slam_params_file')
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_slam'))
    )

    # 3) Nav2 bringup (lifecycle, autostart=True)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'autostart': 'True',
            'use_sim_time': 'True',
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file')
        }.items()
    )

    # 4) Map Server (lifecycle)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': True
        }]
    )

    # 5) rosbridge_websocket (XML launch)
    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(rosbridge_launch_file),
        launch_arguments={
            'port': '9090'
        }.items()
    )

    # 5) Nodo Orchestrator
    _orchestrator_core_node = Node(
        package='orchestrator',
        executable='orchestrator',
        name='ui_orchestrator',
        output='screen'
    )

    # Declarar argumentos de lanzamiento estándar de TurtleBot3
    declare_world_arg = DeclareLaunchArgument('world', default_value='empty', description='World file name')
    declare_map_arg = DeclareLaunchArgument('map', default_value='/root/maps/Turtle1.yaml', description='Map file name')
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            'burger.yaml'
        ),
        description='Params file'
    )
    declare_use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='False',
        description='Launch SLAM Toolbox (True) o solo usar mapa existente (False)'
    )

    declare_slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('slam_toolbox'),
            'config',
            'mapper_params_online_async.yaml'
        ),
        description='SLAM params file'
    )

    # Devolver todas las acciones: debug + componentes
    return LaunchDescription(debug_logs + [

        declare_world_arg,
        declare_map_arg,
        declare_use_slam_arg,
        declare_params_file_arg,
        declare_slam_params_file_arg,
        gazebo,
        slam_toolbox,
        nav2,
        map_server_node,
        rosbridge,
        TimerAction(period=1.0, actions=[_orchestrator_core_node])
    ])
