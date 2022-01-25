import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    slam_config_path = PathJoinSubstitution(
        [FindPackageShare('turtlebot2_ros2'), 'config/nav', 'slam.yaml']
    )

    laser_launch_path = PathJoinSubstitution(
        [FindPackageShare('turtlebot2_ros2'), 'launch', 'ydlidar.launch.py']
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                #'use_sim_time': LaunchConfiguration("sim"),
                'params_file': slam_config_path
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_launch_path)
        ),
    ])
