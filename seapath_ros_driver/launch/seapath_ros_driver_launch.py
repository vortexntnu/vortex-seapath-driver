from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'seapath_ros_driver',
            executable = 'seapath_ros_driver_node',
            name = 'seapath_ros_node'
        )
    ])



