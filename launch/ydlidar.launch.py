import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ydlidar_config_path = PathJoinSubstitution(
        [FindPackageShare("turtlebot2_ros2"), "config/sensors", "ydlidar_x4.yaml"]
    )

    return LaunchDescription([
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
        )
    ])
