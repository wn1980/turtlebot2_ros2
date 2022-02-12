from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("turtlebot2_ros2"), "config", "ekf.yaml"]
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[("odometry/filtered", "odom")]
        ),

    ])
