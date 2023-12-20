from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node'
        ),
        Node(
            package='turtletest',
            executable='turtletest_node',
            name='turtletest_spinner'
        ),
        Node(
            package='turtletest',
            executable='turtletest_controller',
            name='turtletest_controller',
            prefix='xterm -e',
        ),
        # Node(
        #     package='turtlesim',
        #     executable="turtle_teleop_key",
        #     output='screen',
        #     prefix='xterm -e',
        #     name='teleop'
        # )
    ])
