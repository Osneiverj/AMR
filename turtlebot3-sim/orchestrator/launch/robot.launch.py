import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Use our local sim launcher to avoid empty-path issues
    orchestrator_dir = get_package_share_directory('orchestrator')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(orchestrator_dir, 'launch', 'sim.launch.py')
            )
        )
    ])
