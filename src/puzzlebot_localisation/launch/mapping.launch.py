import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

#from ament_indexpython import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true'  , choices=['true', 'false'], description='Use sim time'),
    DeclareLaunchArgument('resolution', default_value='0.05', description='Resolution of a grid cell in the published occupancy grid'),
    DeclareLaunchArgument('publish_period_sec', default_value='1.0', description='OccupancyGrid publishing period'),
    DeclareLaunchArgument('use_rviz', default_value='true', choices=['true','false'], description='Enable rviz'),
    DeclareLaunchArgument('use_real', default_value='true', choices=['true', 'false'], description='Use real robot'),
]


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    use_rviz = LaunchConfiguration('use_rviz')
    use_real = LaunchConfiguration('use_real')
    rviz_path = get_package_share_directory('puzzlebot_description') + '/rviz/map.rviz'
    
    #Dead reckoning
    odom_node = Node(
        package='puzzlebot_localisation',
        executable='odom_node',
        name='odom_node',
        output='screen'
    )

    occupancy_grid_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
        launch_arguments={
            'use_sim_time':use_sim_time,
            'resolution':resolution,
            'publish_period_sec':publish_period_sec,
            'use_rviz':use_rviz,
            'rviz_path':rviz_path,
            'use_real': use_real
        }.items(),
        # For simulated the bridge is already on gazebo launch file, only for real robot
        condition=IfCondition(use_real)
    )

    mcl_node = Node(
        package='puzzlebot_localisation',
        executable='mcl_node',
        name='mcl_node',
        output='screen'
    )

    return LaunchDescription([
        odom_node,
        mcl_node,
        occupancy_grid_bridge,
    ])
