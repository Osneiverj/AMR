import os
import shutil
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ruta al paquete orchestrator
    orchestrator_dir = get_package_share_directory('orchestrator')

    # Ruta al mapa dentro del contenedor
    map_file = os.path.join('/root', 'maps', 'turtle.yaml')
    # Diagnostics
    print(f"[system_launch] orchestrator_dir: {orchestrator_dir}")
    print(f"[system_launch] map_file: {map_file} (exists={os.path.exists(map_file)})")
    nav2_path = os.path.join(orchestrator_dir, 'launch', 'nav2.launch.py')
    robot_path = os.path.join(orchestrator_dir, 'launch', 'robot.launch.py')
    print(f"[system_launch] robot.launch.py: {robot_path} (exists={os.path.exists(robot_path)})")
    print(f"[system_launch] nav2.launch.py: {nav2_path} (exists={os.path.exists(nav2_path)})")

    # Resolve executables on PATH
    orch_exec = shutil.which('orchestrator')
    rb_exec = shutil.which('rosbridge_websocket')
    print(f"[system_launch] which(orchestrator): {orch_exec}")
    print(f"[system_launch] which(rosbridge_websocket): {rb_exec}")

    # Feature flags to isolate failures
    enable_orchestrator = os.environ.get('TB3_ENABLE_ORCHESTRATOR', 'true').lower() in ('1', 'true', 'yes', 'on')
    enable_rosbridge = os.environ.get('TB3_ENABLE_ROSBRIDGE', 'true').lower() in ('1', 'true', 'yes', 'on')
    print(f"[system_launch] enable_orchestrator: {enable_orchestrator}")
    print(f"[system_launch] enable_rosbridge: {enable_rosbridge}")

    actions = []

    enable_sim = os.environ.get('TB3_ENABLE_SIM', 'true').lower() in ('1', 'true', 'yes', 'on')
    enable_nav2 = os.environ.get('TB3_ENABLE_NAV2', 'true').lower() in ('1', 'true', 'yes', 'on')
    print(f"[system_launch] enable_sim: {enable_sim}")
    print(f"[system_launch] enable_nav2: {enable_nav2}")

    if enable_sim:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    robot_path
                )
            )
        )
    else:
        print('[system_launch] Skipping sim include (TB3_ENABLE_SIM=false)')

    if enable_nav2:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    nav2_path
                ),
                launch_arguments={'map': map_file}.items()
            )
        )
    else:
        print('[system_launch] Skipping nav2 include (TB3_ENABLE_NAV2=false)')

    if enable_orchestrator:
        actions.append(
            Node(
                package='orchestrator',
                executable='orchestrator',
                name='orchestrator',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        )
    else:
        print('[system_launch] Skipping orchestrator node (TB3_ENABLE_ORCHESTRATOR=false)')

    if enable_rosbridge:
        actions.append(
            Node(
                package='rosbridge_server',
                executable='rosbridge_websocket',
                name='rosbridge_websocket',
                output='screen'
            )
        )
    else:
        print('[system_launch] Skipping rosbridge_websocket (TB3_ENABLE_ROSBRIDGE=false)')

    return LaunchDescription(actions)