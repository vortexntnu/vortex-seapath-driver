import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument)
from launch_ros.actions import Node

def generate_launch_description():
    default_params_file = os.path.join(get_package_share_directory('seapath_ros_driver'),'params','seapath_params.yaml')
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
                                            default_value=str(
                                                default_params_file),
                                            description='name or path to the parameters file to use.')
    seapath_ros_driver_node = Node(
            package='seapath_ros_driver',
            executable='seapath_ros_driver_node',
            name='seapath_ros_driver_node',
            parameters=[params_file],
            output='screen',
        )
    return LaunchDescription([
        params_file_arg,
        seapath_ros_driver_node
    ])