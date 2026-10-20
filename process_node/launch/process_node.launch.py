import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    return launch.LaunchDescription([
        Node(  # add by zht
            package='process_node',
            executable='process_node',
            namespace='process_node',
            name='process_node',
        ),
    ])
