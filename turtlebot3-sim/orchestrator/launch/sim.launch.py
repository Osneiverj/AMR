import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Resolve Gazebo world path from turtlebot3_gazebo with fallbacks
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    worlds_dir = os.path.join(tb3_gazebo_share, 'worlds')
    candidate_worlds = [
        'turtlebot3_world.world',
        'turtlebot3_world.model',
        'turtlebot3.world',
    ]
    world = None
    for name in candidate_worlds:
        p = os.path.join(worlds_dir, name)
        if os.path.exists(p):
            world = p
            break
    print(f"[sim.launch] turtlebot3_gazebo share: {tb3_gazebo_share} (exists={os.path.exists(tb3_gazebo_share)})")
    print(f"[sim.launch] Worlds dir: {worlds_dir} (exists={os.path.exists(worlds_dir)})")
    print(f"[sim.launch] Selected world: {world} (exists={os.path.exists(world) if world else 'n/a'})")

    # TurtleBot3 model URDF path
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'), 'urdf', f'turtlebot3_{model}.urdf'
    )
    print(f"[sim.launch] TURTLEBOT3_MODEL: {model}")
    print(f"[sim.launch] URDF path: {urdf} (exists={os.path.exists(urdf)})")

    # Allow headless by default; override with TB3_SIM_GUI=true
    gui_default = os.environ.get('TB3_SIM_GUI', 'false').lower()
    gui_flag = 'true' if gui_default in ('1', 'true', 'yes', 'on') else 'false'
    print(f"[sim.launch] GUI flag: {gui_flag} (TB3_SIM_GUI={gui_default})")

    # Prepare Gazebo launch args with a safe world default
    gazebo_launch_args = {
        'verbose': 'true',
        'gui': gui_flag,
    }
    if world and os.path.exists(world):
        gazebo_launch_args['world'] = world
    else:
        # Fallback to Gazebo classic empty world if present
        fallback_world = '/usr/share/gazebo-11/worlds/empty.world'
        if os.path.exists(fallback_world):
            gazebo_launch_args['world'] = fallback_world
    gazebo_launch_file = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    print(f"[sim.launch] gazebo_ros launch: {gazebo_launch_file} (exists={os.path.exists(gazebo_launch_file)})")
    print(f"[sim.launch] gazebo launch args: {gazebo_launch_args}")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazebo_launch_file
        ),
        launch_arguments=gazebo_launch_args.items()
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'tb3', '-file', urdf, '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn,
    ])
