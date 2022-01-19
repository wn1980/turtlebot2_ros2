import os
import yaml

import ament_index_python.packages

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # package root
    share_dir = ament_index_python.packages.get_package_share_directory('turtlebot2_ros2')

    # kobuki_ros node
    params_file = os.path.join(share_dir, 'config', 'kobuki_node_params.yaml')
    
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

    kobuki_node = ComposableNode(
        package='kobuki_node',
        plugin='kobuki_node::KobukiRos',
        name='kobuki_ros_node',
        parameters=[params]
    )

    # safety_controller
    params_file = os.path.join(share_dir, 'config', 'safety_controller_params.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_safety_controller_node']['ros__parameters']

    safety_controller_node = ComposableNode(
        package='kobuki_safety_controller',
        plugin='kobuki_safety_controller::SafetyController',
        name='safety_controller_node',
        remappings=[
            ('cmd_vel', 'input/safety_controller')
        ],
        parameters=[params]
    )

    # cmd_vel_mux
    params_file = os.path.join(share_dir, 'config', 'cmd_vel_mux_params.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['cmd_vel_mux']['ros__parameters']

    cmd_vel_mux_node = ComposableNode(
        package='cmd_vel_mux',
        plugin='cmd_vel_mux::CmdVelMux',
        name='cmd_vel_mux',
        remappings=[
            ('cmd_vel', 'commands/velocity')
        ],
        parameters=[params]
    )

    # velocity_smoother
    params_file = os.path.join(share_dir, 'config', 'velocity_smoother_params.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['velocity_smoother']['ros__parameters']
    
    velocity_smoother_node = ComposableNode(
        package='velocity_smoother',
        plugin='velocity_smoother::VelocitySmoother',
        name='velocity_smoother',
        remappings=[
            ('velocity_smoother/smoothed', 'input/keyop'),
            ('velocity_smoother/feedback/cmd_vel', 'commands/velocity'),
            ('velocity_smoother/feedback/odometry', 'odom')
        ],
        parameters=[params]
    )

    # kobuki_bumper2pc
    params_file = os.path.join(share_dir, 'config', 'kobuki_bumper2pc_params.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_bumper2pc']['ros__parameters']

    kobuki_bumper2pc_node = ComposableNode(
        package='kobuki_bumper2pc',
        plugin='kobuki_bumper2pc::Bumper2PcNode',
        name='kobuki_bumper2pc_node',
        remappings=[
            ('core_sensors', 'sensors/core'),
            ('pointcloud', 'bumper_pointcloud')
        ],
        parameters=[params]
    )

    # packs to the container
    container = ComposableNodeContainer(
            name='mobile_base_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                kobuki_node,
                safety_controller_node,
                cmd_vel_mux_node,
                kobuki_bumper2pc_node,
                #velocity_smoother_node
            ],
            output='both',
    )

    # TF node
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_base_link',
        arguments=['0', '0', '0','0', '0', '0', '1','base','base_link'],
    )

    # Finally, return all nodes
    return LaunchDescription([
        container,
        tf2_node,
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '/enable', 'std_msgs/msg/Empty', '--once'],
            output='screen'
        )
    ])
