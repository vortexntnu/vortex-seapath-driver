import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    seapath_ros_driver_node = Node(
            package='seapath_ros_driver',
            executable='seapath_ros_driver_node',
            name='seapath_ros_driver_node',
            parameters=[os.path.join(get_package_share_directory('seapath_ros_driver'),'params','seapath_params.yaml')],
            output='screen',
        )
    return LaunchDescription([
        seapath_ros_driver_node
    ])