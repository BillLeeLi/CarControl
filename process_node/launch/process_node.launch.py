"""启动感知与运动控制节点；回放时传 use_sim_time:=true。"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config = os.path.join(get_package_share_directory('process_node'),
                          'config', 'baseline.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('show_visualization', default_value='true'),
        Node(
            package='process_node',
            executable='process_node',
            namespace='process_node',
            name='process_node',
            output='screen',
            parameters=[LaunchConfiguration('config'), {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                               value_type=bool),
                'show_visualization': ParameterValue(
                    LaunchConfiguration('show_visualization'), value_type=bool),
            }],
        ),
    ])
