"""Define Python packaging, ROS resource installation and command entry points for

lidar_outline.
"""

from glob import glob

from setuptools import find_packages, setup

package_name = 'lidar_outline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.rviz') + glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='jovan',
    maintainer_email='jovan@todo.todo',
    description='Extract LiDAR line outlines and publish keyboard velocity commands.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_outline = lidar_outline.lidar_outline_node:main',
            'wasd_teleop = lidar_outline.wasd_teleop:main',
            'capture_camera = lidar_outline.capture_camera:main',
            'find_object = lidar_outline.find_object_node:main',
            'isaac_localization = lidar_outline.isaac_localization:main',
            'world_map = lidar_outline.world_map:main',
            'semantic_navigation = lidar_outline.semantic_navigation:main',
            'navigation_scans = lidar_outline.navigation_scans:main',
        ],
    },
)
