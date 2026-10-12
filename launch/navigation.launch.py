"""Launch Nav2 with an explicit world-aligned map and measured robot footprint.

Requires the localization bridge and corrected scans to be running. Commands
default to /nav/cmd_vel_preview for bringup checks before connecting the drive.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def launch_nodes(context):
    map_path = LaunchConfiguration('map').perform(context)
    footprint = LaunchConfiguration('footprint').perform(context)
    if not Path(map_path).is_file():
        raise ValueError(f'Navigation map does not exist: {map_path}')
    # Validate the polygon before passing it to Nav2's string footprint parser.
    import json
    import math
    points = json.loads(footprint)
    if (len(points) < 3 or any(len(p) != 2 for p in points)
            or not all(math.isfinite(v) for p in points for v in p)):
        raise ValueError('footprint must be a finite polygon in base-frame metres')
    params_path = Path(get_package_share_directory('lidar_outline'))/'config/nav2_isaac.yaml'
    params = RewrittenYaml(source_file=str(params_path),
                           param_rewrites={'use_sim_time': 'true', 'footprint': footprint},
                           convert_types=True)
    output = LaunchConfiguration('cmd_vel_topic').perform(context)
    definitions = [
        ('nav2_map_server', 'map_server', 'map_server'),
        ('nav2_planner', 'planner_server', 'planner_server'),
        ('nav2_controller', 'controller_server', 'controller_server'),
        ('nav2_behaviors', 'behavior_server', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
    ]
    actions = []
    for package, executable, name in definitions:
        overrides = {'use_sim_time': True}
        if name == 'map_server':
            overrides.update(yaml_filename=map_path, frame_id='world')
        # Costmap nodes read their parameters from the same rewritten file.
        actions.append(Node(package=package, executable=executable, name=name,
                            parameters=[params, overrides], output='screen',
                            remappings=[('cmd_vel', output)]))
    actions.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': [name for _, _, name in definitions]}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Verified occupancy-map YAML in world coordinates'),
        DeclareLaunchArgument('footprint', description='Measured polygon as [[x,y],...] in base metres'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/nav/cmd_vel_preview',
                              description='Set /cmd_vel only after map/footprint verification'),
        OpaqueFunction(function=launch_nodes),
    ])
