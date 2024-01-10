from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node definitions
        Node(
            package='core',
            executable='commander',
            # name='commander',
            output='screen',
        ),
        Node(
            package='core',
            executable='ltm',
            # name='ltm',
            output='screen',
            arguments=['0']
        ),
        Node(
            package='core',
            executable='execution_node',
            # name='ex0',
            output='screen',
            arguments=['0'],
        ),
        Node(
            package='core',
            executable='execution_node',
            # name='ex1',
            output='screen',
            arguments=['1'],
        )
    ])
