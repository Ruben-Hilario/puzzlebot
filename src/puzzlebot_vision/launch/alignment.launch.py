import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    video_publisher_node = Node(
        package='puzzlebot_vision',
        executable='video_publisher.py',
        name='video_publisher',
        output='screen'
    )

    alignment_node = Node(
        package='puzzlebot_vision',
        executable='aligment.py',
        name='alignment_node',
        output='screen'
    )
    return LaunchDescription([
        video_publisher_node,
        alignment_node
    ])
    
