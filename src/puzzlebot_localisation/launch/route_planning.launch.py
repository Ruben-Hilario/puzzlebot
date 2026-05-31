import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

    
ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true'  , choices=['true', 'false'], description='Use sim time')
]

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_path = get_package_share_directory('puzzlebot_description') + '/rviz/path.rviz'
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d',rviz_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        *ARGUMENTS,
        map_node,
        path_node,
        rviz_node
    ])