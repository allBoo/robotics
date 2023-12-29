import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mapping'),
                    'launch/world.launch.py'))
        ),
        Node(
            package='turtlebot3_teleop',
            executable='teleop_keyboard',
            name='turtlebot3_teleop',
            prefix='xterm -e',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('turtlebot3_cartographer'),
                    'launch/cartographer.launch.py')),
            launch_arguments={
                'use_sim_time': 'True'
            }.items()
        ),

        # Node(
        #     package='turtlesim',
        #     executable="turtle_teleop_key",
        #     output='screen',
        #     prefix='xterm -e',
        #     name='teleop'
        # )
    ])
