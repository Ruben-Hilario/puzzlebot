import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true'  , choices=['true', 'false'], description='Use sim time'),
    DeclareLaunchArgument('resolution', default_value='0.05', description='Resolution of a grid cell in the published occupancy grid'),
    DeclareLaunchArgument('publish_period_sec', default_value='1.0', description='OccupancyGrid publishing period'),
    DeclareLaunchArgument('use_rviz', default_value='true', choices=['true','false'], description='Enable rviz')   
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    config_dir = get_package_share_directory('puzzlebot_localisation') +  '/config/'
    config_name = 'puzzlebot.lua'
    rviz_path = get_package_share_directory('puzzlebot_description') + '/rviz/map.rviz'

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time' : use_sim_time}],
        arguments = ['-configuration_directory', config_dir,
                '-configuration_basename', config_name]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        parameters =[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', resolution, 
            '-publish_period_sec', publish_period_sec
        ]
    )
    
    localisation_node = Node(
        package='puzzlebot_localisation',
        executable='odom_node',
        name='odom_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
                
    occupancy_grid_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), 'occupancy_grid.launch.py']),
        launch_arguments={
            'use_sim_time':use_sim_time,
            'resolution':resolution,
            'publish_period_sec':publish_period_sec,
            'use_rviz':use_rviz
        }.items()
    )
    
    return LaunchDescription([
        *ARGUMENTS, 
        localisation_node,
        cartographer_node, 
        occupancy_grid_node,
        occupancy_grid_bridge,
    ])
