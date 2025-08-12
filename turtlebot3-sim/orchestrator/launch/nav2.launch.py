import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Argumento para la ruta del mapa
    # Resolve a safe default params file for Nav2
    nav2_params_default = os.path.join(
        get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml'
    )
    print(f"[nav2.launch] nav2_params_default: {nav2_params_default} (exists={os.path.exists(nav2_params_default)})")
    bringup_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
    print(f"[nav2.launch] bringup_launch.py: {bringup_launch} (exists={os.path.exists(bringup_launch)})")

    # Choose a safe default map if available; otherwise, we will enable SLAM
    default_map = '/root/maps/turtle.yaml'
    if not os.path.exists(default_map):
        candidates = sorted(glob('/root/maps/*.yaml'))
        if candidates:
            default_map = candidates[0]
    print(f"[nav2.launch] default_map: {default_map} (exists={os.path.exists(default_map)})")

    map_arg = DeclareLaunchArgument('map', default_value=default_map, description='Full path to map file to load')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_default,
        description='Full path to the ROS2 parameters file to use for Nav2'
    )

    # Enable SLAM automatically if no map file exists
    slam_default = 'False' if os.path.exists(default_map) else 'True'
    print(f"[nav2.launch] slam_default: {slam_default}")

    return LaunchDescription([
        map_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                bringup_launch
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'True',
                'params_file': LaunchConfiguration('params_file'),
                'slam': slam_default,
            }.items(),
        ),
    ])
