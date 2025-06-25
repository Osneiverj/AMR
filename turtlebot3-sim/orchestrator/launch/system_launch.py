#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ----------------------------------------
    # 1) Rutas de los paquetes y ficheros
    # ----------------------------------------
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    slam_toolbox_pkg     = get_package_share_directory('slam_toolbox')
    nav2_bringup_pkg     = get_package_share_directory('nav2_bringup')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    nav2_params_file = os.path.join(nav2_bringup_pkg, 'params', 'nav2_params.yaml')

    default_bt_xml = os.path.join(
        nav2_bringup_pkg, 'behavior_trees', 'navigate_w_replanning_cycle.xml')
    gazebo_launch_file = os.path.join(
        turtlebot3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')
    slam_launch_file = os.path.join(
        slam_toolbox_pkg, 'launch', 'online_async_launch.py')
    # Apunta al YAML por defecto que trae slam_toolbox
    slam_params_file = os.path.join(
        slam_toolbox_pkg, 'config', 'mapper_params_online_async.yaml')
    nav2_launch_file = os.path.join(
        nav2_bringup_pkg, 'launch', 'navigation_launch.py')
    nav2_params_file = os.path.join(nav2_bringup_pkg, 'params', 'nav2_params.yaml')
    default_bt_xml    = os.path.join(nav2_bringup_pkg, 'behavior_trees', 'navigate_w_replanning_cycle.xml')

    
    # ----------------------------------------
    # 2) Debug (opcional) para verificar rutas
    # ----------------------------------------
    debug_logs = [
        LogInfo(msg=f'DEBUG: SLAM launch file exists? {os.path.isfile(slam_launch_file)}'),
        LogInfo(msg=f'DEBUG: SLAM params file exists? {os.path.isfile(slam_params_file)}'),
    ]
    
    # ----------------------------------------
    # 3) Include de Gazebo, SLAM, Nav2, etc.
    # ----------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'use_sim_time': 'True'}.items()
    )
    
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            # Aquí pasamos el fichero de parámetros
            'slam_params_file': slam_params_file,
            'use_sim_time': 'True',
            'autostart': 'False'
        }.items()
    )
    
    
    debug_logs += [
        LogInfo(msg=f'DEBUG: Nav2 params file exists? {os.path.isfile(nav2_params_file)}'),
        LogInfo(msg=f'DEBUG: Default BT XML exists? {os.path.isfile(default_bt_xml)}'),
    ]

    nav2_launch_args = {
        'use_sim_time': 'True',
        'autostart':    'False',
        'params_file':  nav2_params_file,
    }
    if os.path.isfile(default_bt_xml):
        nav2_launch_args['default_bt_xml_filename'] = default_bt_xml

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments=nav2_launch_args.items()
    )
    
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}]
    )
    
    orchestrator_node = Node(
        package='orchestrator',
        executable='orchestrator',  # entry point name
        name='ui_orchestrator',
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription(debug_logs + [
        gazebo,
        slam_toolbox,
        nav2,
        rosbridge,
        orchestrator_node
    ])
