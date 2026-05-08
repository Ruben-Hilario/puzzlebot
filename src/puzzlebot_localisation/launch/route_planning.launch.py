import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    map_node = Node(
        package='puzzlebot_localisation',
        executable='map_publisher',
        name='map_publisher',
        output='screen'
    )
    path_node = Node(
        package='puzzlebot_localisation',
        executable='route_node',
        name='route_node',
        output='screen'
    )

    return LaunchDescription([
        map_node,
        path_node
    ])