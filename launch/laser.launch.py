import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode

import lifecycle_msgs.msg

def generate_launch_description():
    share_dir = get_package_share_directory('turtlebot2_ros2')
    parameter_file = LaunchConfiguration('params_file')
 
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config/sensors', 'ydlidar_x4.yaml'
        ),
        description='FPath to the ROS2 parameters file to use.'
    )

    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/',
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.02', '0', '0.13','0', '0', '0', '1','base_link','laser_frame'],
    )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
    ])
