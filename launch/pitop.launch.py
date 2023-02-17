from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pitop_ros2',
            namespace='pitop',
            executable='pitop_node',
            name='pitop_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'wheel_separation': 0.163,
                'wheel_diameter': 0.065,
                'timer_period': 0.01,
            }],
        )
    ])
