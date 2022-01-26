import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    nav2_config_path = PathJoinSubstitution(
        #[FindPackageShare('turtlebot2_ros2'), 'config/nav', 'nav2_params.yaml']
        [FindPackageShare('turtlebot2_ros2'), 'config/nav', 'navigation.yaml']
    )


    return LaunchDescription([

        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path,
                #'remappings': ('cmd_vel', 'cmd_vel_mux/input/navigation')
            }.items()
        ),


    ])

"""
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_path)
        ),
"""