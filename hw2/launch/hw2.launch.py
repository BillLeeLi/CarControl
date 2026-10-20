# filepath: /ros2_ws/src/my_package/launch/my_launch_file.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('hw2'),
        'rviz',
        'config.rviz'
    )
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='/hw2_data',
        description='Path to the ROS 2 bag file'
    )
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate for ros2 bag play'
    )
    return LaunchDescription([
        # 声明 bag_file 参数
        bag_file_arg,

        # 声明 rate 参数
        rate_arg,
        # 启动指定节点
        Node(
            package='visualization',
            executable='easy_voting',
            name='easy_voting',
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
        
        # 播放指定的 ROS 2 bag 文件
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_file'),'--rate', LaunchConfiguration('rate')],
            output='screen'
        )
    ])