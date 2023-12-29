import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    mapping_path = get_package_share_directory('mapping')
    map_path = os.path.join(
        mapping_path,
        'maps',
        'map.yaml'
    )
    print('LAUNCH MAP', map_path)

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('turtlebot3_navigation2'),
                    'launch/navigation2.launch.py')),
            launch_arguments={
                'use_sim_time': 'True',
                'map': map_path,
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join( mapping_path, 'launch/world.launch.py'))
        ),
        Node(
            package='mapping',
            executable='mapping_controller',
            name='mapping_controller',
            prefix='xterm -e',
        ),
        Node(
            package='mapping',
            executable='mapping_node',
            name='mapping_node',
            prefix='xterm -e',
        ),
    ])
