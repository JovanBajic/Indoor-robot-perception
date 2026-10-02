from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start RViz and LiDAR processing on the Isaac simulation clock."""
    return LaunchDescription([
        Node(
            package='lidar_outline',
            executable='lidar_outline',
            name='lidar_split_merge',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
