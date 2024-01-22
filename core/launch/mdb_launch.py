from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node definitions
        Node(
            package='core',
            executable='commander',
            output='screen',
        ),
        Node(
            package='core',
            executable='ltm',
            output='screen',
            arguments=['0']
        )
        ,
        Node(
            package='core',
            executable='execution_node',
            output='screen',
        ),
        Node(
            package='core',
            executable='execution_node',
            output='screen',
        )
    ])
