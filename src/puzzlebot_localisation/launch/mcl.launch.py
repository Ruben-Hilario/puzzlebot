import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true'  , choices=['true', 'false'], description='Use sim time'),
    DeclareLaunchArgument('use_rviz', default_value='true', choices=['true','false'], description='Enable rviz'),
    DeclareLaunchArgument('use_real', default_value='false', choices=['true', 'false'], description='Use real robot'),
    DeclareLaunchArgument('cpp', default_value='true')
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_real = LaunchConfiguration('use_real')
    cpp = LaunchConfiguration('cpp')

    rviz_cpp = get_package_share_directory('puzzlebot_description') + '/rviz/puzzlebot.rviz'
    rviz_py = get_package_share_directory('puzzlebot_description') + '/rviz/mcl_py.rviz'
    
    map_node = Node (
        package='puzzlebot_localisation',
        executable='map_publisher',
        name='map_publisher',
        output='screen',
    )
  
    #Dead reckoning
    localisation_node = Node(
        package='puzzlebot_localisation',
        executable='odom_node',
        name='odom_node',
        output='screen'
    )

    tf_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/tf.launch.py']),
        launch_arguments={
            'use_sim_time':use_sim_time,
            'use_rviz':use_rviz,
            'use_real': use_real
        }.items(),
        # For simulated the bridge is already on gazebo launch file, only for real robot
        condition=IfCondition(use_real)
    )

    cpp_mcl_node = Node(
        package='puzzlebot_localisation',
        executable='mcl_node',
        name='mcl_node',
        output='screen',
        condition=IfCondition(cpp)
    )
    
    py_mcl_node = Node(
        package='puzzlebot_localisation',
        executable='MCL.py',
        name='mcl_node',
        output='screen',
        condition=UnlessCondition(cpp)
    )

    cpp_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d',rviz_cpp],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(cpp)
    )

    py_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d',rviz_py],
        parameters=[{'use_sim_time':use_sim_time}],
        condition=UnlessCondition(cpp)
    )

    route_node = Node(
        package='puzzlebot_localisation',
        executable='route_node',
        name='route_node',
        output='screen',
    )

    follower_node = Node(
        package='puzzlebot_localisation',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
    )

    return LaunchDescription([
        *ARGUMENTS,
        localisation_node,
        cpp_mcl_node,
        py_mcl_node,
        tf_bridge,
        route_node,
        cpp_rviz_node,
        py_rviz_node,
        follower_node,
        map_node,
    ])
