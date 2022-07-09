# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import ament_index_python.packages
import yaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

MAP_NAME='map' #change to the name of your own map here

def generate_launch_description():
    depth_sensor = os.getenv('LINOROBOT2_DEPTH_SENSOR', '')

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('turtlebot2_ros2'), 'rviz', 'linorobot2_navigation.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('turtlebot2_ros2'), 'maps', f'{MAP_NAME}.yaml']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('turtlebot2_ros2'), 'config/nav', '_nav2_params.yaml']
    )

    laser_launch_path = PathJoinSubstitution(
        [FindPackageShare('turtlebot2_ros2'), 'launch', 'ydlidar.launch.py']
    )

    # velocity_smoother
    share_dir = ament_index_python.packages.get_package_share_directory('turtlebot2_ros2')
    params_file = os.path.join(share_dir, 'config', 'velocity_smoother_params.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['velocity_smoother']['ros__parameters']

    # packs to the container
    navigation_container = ComposableNodeContainer(
            name='navigation_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='velocity_smoother',
                    plugin='velocity_smoother::VelocitySmoother',
                    name='velocity_smoother_navi',
                    remappings=[
                        ('velocity_smoother_navi/input', 'cmd_vel'),
                        ('velocity_smoother_navi/smoothed', '/cmd_vel_mux/input/navigation'),
                        ('velocity_smoother_navi/feedback/cmd_vel', '/mobile_base/commands/velocity'),
                        ('velocity_smoother_navi/feedback/odometry', '/odom')
                    ],
                    parameters=[params]
                )
            ],
            output='both',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

       DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path,
                'remappings': ('cmd_vel', '/velocity_smoother_navi/input')
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        ),

        navigation_container,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_launch_path)
        ),

    ])

"""
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_launch_path)
        ),

        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[ydlidar_config_path]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0.02', '0', '0.13','0', '0', '0', '1','base_link','laser_frame'],
        ),
"""
        