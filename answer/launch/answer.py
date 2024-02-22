import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    homework_dir = get_package_share_directory('homework')
    answer_dir = get_package_share_directory('answer')

    homework_node = Node(
        package='homework',
        executable='homework_node',
        output='screen',
    )

    answer_node = Node(
        package='answer',
        executable='answer_node',
        output='screen',
    )

    return LaunchDescription([
        homework_node,
        answer_node,
    ])
