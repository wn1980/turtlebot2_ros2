from launch import LaunchDescription
from launch_ros.actions import Node

import os
import ament_index_python.packages
import yaml

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # velocity_smoother
    share_dir = ament_index_python.packages.get_package_share_directory('turtlebot2_ros2')
    params_file = os.path.join(share_dir, 'config', 'velocity_smoother_params.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['velocity_smoother']['ros__parameters']

    # packs to the container
    container = ComposableNodeContainer(
            name='keyboard_teleop_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='velocity_smoother',
                    plugin='velocity_smoother::VelocitySmoother',
                    name='velocity_smoother_teleop',
                    remappings=[
                        ('velocity_smoother_teleop/smoothed', '/cmd_vel_mux/input/keyop'),
                        #('velocity_smoother_teleop/feedback/cmd_vel', '/mobile_base/commands/velocity'),
                        ('velocity_smoother_teleop/feedback/odometry', '/odom'),
                        ('velocity_smoother_teleop/feedback/cmd_vel', '/cmd_vel'),
                    ],
                    parameters=[params]
                )
            ],
            output='both',
    )

    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='keyboard_teleop_node',
            output='screen',
            prefix = 'xterm -T keyboard_teleop -e',
            remappings=[
                ('cmd_vel', 'velocity_smoother_teleop/input')
            ]
        ),

        container
    ])
