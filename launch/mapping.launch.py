import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    slam_config_path = PathJoinSubstitution(
        [FindPackageShare('turtlebot2_ros2'), 'config/nav', '_mapper_params_online_async.yaml']
    )

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('turtlebot2_ros2'), 'config/nav', '_nav2_params.yaml']
    )

    laser_launch_path = PathJoinSubstitution(
        [FindPackageShare('turtlebot2_ros2'), 'launch', 'ydlidar.launch.py']
        #[FindPackageShare('turtlebot2_ros2'), 'launch', 'laser.launch.py']
    )

    ros_distro = EnvironmentVariable('ROS_DISTRO')
    slam_param_name = 'params_file'
    if ros_distro == 'galactic': 
        slam_param_name = 'slam_params_file'

    return LaunchDescription([

        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_launch_path)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                slam_param_name: slam_config_path
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path
            }.items()
        ),

    ])

"""
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path
            }.items()
        ),
"""