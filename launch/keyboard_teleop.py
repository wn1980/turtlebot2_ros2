from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='keyboard_teleop',
            output='screen',
            prefix = 'xterm -e',
            remappings=[
                ('cmd_vel', 'velocity_smoother/input')
            ]
        )
    ])