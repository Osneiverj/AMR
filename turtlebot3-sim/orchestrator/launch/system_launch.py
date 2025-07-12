#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    rosbridge_pkg = get_package_share_directory('rosbridge_server')

    tb3_launch_file = os.path.join(nav2_bringup_pkg, 'launch', 'tb3_simulation_launch.py')
    rosbridge_launch_file = os.path.join(rosbridge_pkg, 'launch', 'rosbridge_websocket_launch.xml')

    tb3_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_launch_file),
        launch_arguments={
            'use_sim_time': 'True',
            'slam': 'False',
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'world': LaunchConfiguration('world'),
            'use_rviz': 'False'
        }.items(),
    )

    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(rosbridge_launch_file),
        launch_arguments={'port': '9090'}.items(),
    )

    orchestrator = Node(
        package='orchestrator',
        executable='orchestrator',
        name='ui_orchestrator',
        output='screen'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(nav2_bringup_pkg, 'worlds', 'world_only.model'),
        description='Gazebo world file'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_bringup_pkg, 'maps', 'turtlebot3_world.yaml'),
        description='Map YAML file'
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_bringup_pkg, 'params', 'nav2_params.yaml'),
        description='Nav2 parameters file'
    )

    return LaunchDescription([
        declare_world,
        declare_map,
        declare_params,
        tb3_sim,
        rosbridge,
        TimerAction(period=1.0, actions=[orchestrator]),
    ])
