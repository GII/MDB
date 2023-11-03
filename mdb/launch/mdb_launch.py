from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node definitions
        Node(
            package='mdb',
            executable='commander',
            name='commander',
            output='screen',
        ),
        Node(
            package='mdb',
            executable='ltm',
            name='ltm',
            output='screen',
        ),
        Node(
            package='mdb',
            executable='main_loop',
            name='main_loop',
            output='screen',
        ),
        Node(
            package='mdb',
            executable='publisher',
            name='publisher',
            output='screen',
        ),
    ])
