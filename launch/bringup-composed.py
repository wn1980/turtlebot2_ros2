import os
import ament_index_python.packages
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # package root
    share_dir = ament_index_python.packages.get_package_share_directory('turtlebot2_ros2')

    # kobuki node
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

    # packs to the container
    container = ComposableNodeContainer(
            name='kobuki_node_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                kobuki_node,
                safety_controller_node,
                cmd_vel_mux_node
            ],
            output='both',
    )

    # Finally, return all nodes
    return LaunchDescription([
        container,
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '/enable', 'std_msgs/msg/Empty', '--once'],
            output='screen'
        )
    ])
