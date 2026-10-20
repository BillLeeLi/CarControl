from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('hw1'),
        'rviz',
        'config.rviz'
    )
    return LaunchDescription([
        # 启动指定节点
        Node(
            package='hw1',
            executable='hw1',
            name='hw1',
            output='screen'
        ),
        Node(
            package='hw1_answer',
            executable='answer',
            name='answer',
            output='screen'
        ),
        Node(
            package='visualization',
            executable='trajectory_visualization',
            name='trajectory_visualization',
            output='screen'
        ),
        
        # 启动 RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
        
    ])