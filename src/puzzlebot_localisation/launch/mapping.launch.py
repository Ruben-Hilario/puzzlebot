import os
from launch_ros.actions import Node
from launch import LaunchDescription
#from ament_indexpython import get_package_share_directory


def generate_launch_description():
    odom_node = Node(
        package='puzzlebot_localisation',
        executable='odom_node',
        name='odom_node',
        output='screen'
    )
    mcl_node = Node(
        package='puzzlebot_localisation',
        executable='mcl_node',
        name='mcl_node',
        output='screen'
    )
    route_node = Node(
        package='puzzlebot_control',
        executable='pid_node',
        name='pid_node',
        output='screen'
    )
     # MCL Python Node
    
    mcl_pynode = Node(
        package='puzzlebot_localisation',
        executable='MCL.py',
        name='mcl_pynode',
        output='screen'
    )    

    return LaunchDescription([
        odom_node,
        #mcl_node,
        mcl_pynode,
        #pid_node,
    ])
