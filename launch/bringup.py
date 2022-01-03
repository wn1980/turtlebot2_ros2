import os
import ament_index_python.packages
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # package root
    share_dir = ament_index_python.packages.get_package_share_directory('turtlebot2_ros2')

    # kobuki node
    params_file = os.path.join(share_dir, 'config', 'kobuki_node_params.yaml')
    
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

    kobuki_ros_node = Node(
        package='kobuki_node',
        executable='kobuki_ros_node',
        output='both',
        parameters=[params]
    )

    # velocity_smoother
    params_file = os.path.join(share_dir, 'config', 'velocity_smoother_params.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['velocity_smoother']['ros__parameters']

    velocity_smoother_node = Node(
        package='velocity_smoother',
        executable='velocity_smoother',
        output='both',
        remappings=[
            ('velocity_smoother/smoothed', 'input/keyop')
        ],
        parameters=[params]
    )

    # cmd_vel_mux
    params_file = os.path.join(share_dir, 'config', 'cmd_vel_mux_params.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['cmd_vel_mux']['ros__parameters']

    cmd_vel_mux_node = Node(
        package='cmd_vel_mux',
        executable='cmd_vel_mux_node',
        output='both',
        remappings=[
            ('cmd_vel', 'commands/velocity')
        ],
        parameters=[params]
    )

    # safety_controller
    params_file = os.path.join(share_dir, 'config', 'safety_controller_params.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_safety_controller_node']['ros__parameters']
    
    safety_controller_node = Node(
        package='kobuki_safety_controller',
        executable='kobuki_safety_controller_node',
        output='both',
        remappings=[
            ('cmd_vel', 'input/safety_controller')
        ],
        parameters=[params]
    )

    # Finally, return all nodes
    return LaunchDescription([
        kobuki_ros_node, 
        velocity_smoother_node,
        cmd_vel_mux_node,
        safety_controller_node,
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '/enable', 'std_msgs/msg/Empty', '--once'],
            output='screen'
        )
    ])
