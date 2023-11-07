from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node definitions
        Node(
            package='core',
            executable='commander',
            name='commander',
            output='screen',
        ),
        Node(
            package='core',
            executable='ltm',
            name='ltm',
            output='screen',
        ),
        Node(
            package='core',
            executable='main_loop',
            name='main_loop',
            output='screen',
        )
    ])
