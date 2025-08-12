import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    rosbridge_pkg = get_package_share_directory('rosbridge_server')

    tb3_launch_file = os.path.join(nav2_bringup_pkg, 'launch', 'tb3_simulation_launch.py')
    rosbridge_launch_file = os.path.join(rosbridge_pkg, 'launch', 'rosbridge_websocket_launch.xml')

    # Declaraciones de argumentos
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(nav2_bringup_pkg, 'worlds', 'world_only.model'),
        description='Gazebo world file'
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join('/root/maps', 'Turtle1.yaml'),
        description='Map YAML file'
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_bringup_pkg, 'params', 'nav2_params.yaml'),
        description='Nav2 parameters file'
    )

    # Lanzamiento de la simulación del TB3 (sin Nav2 todavía)
    tb3_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_launch_file),
        launch_arguments={
            'use_sim_time': 'True',
            'slam': 'False',
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'world': LaunchConfiguration('world'),
            'use_rviz': 'False',
            'autostart': 'False'  # No iniciamos Nav2 automáticamente
        }.items(),
    )

    # Rosbridge
    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(rosbridge_launch_file),
        launch_arguments={'port': '9090'}.items(),
    )

    # Orquestador de UI
    orchestrator = Node(
        package='orchestrator',
        executable='orchestrator',
        name='ui_orchestrator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Gestor de ciclo de vida de Nav2
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        'amcl',
        'map_server'
    ]
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': lifecycle_nodes
        }]
    )

    return LaunchDescription([
        declare_world,
        declare_map,
        declare_params,
        tb3_sim,
        rosbridge,
        lifecycle_manager,
        orchestrator,
    ])
