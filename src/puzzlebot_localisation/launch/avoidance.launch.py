import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true'  , choices=['true', 'false'], description='Use sim time'),
    DeclareLaunchArgument('use_real', default_value='false', choices=['true', 'false'], description='Use real robot'),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_real = LaunchConfiguration('use_real')

    rviz_cpp = get_package_share_directory('puzzlebot_description') + '/rviz/simple.rviz'

    tf_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/tf.launch.py']),
        launch_arguments={
            'use_sim_time':use_sim_time,
            'use_real': use_real
        }.items(),
        # For simulated the bridge is already on gazebo launch file, only for real robot
        condition=IfCondition(use_real)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cpp],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    obstacle_avoidance_node = Node(
        package='puzzlebot_localisation',
        executable='obstacle_avoidance',
        name='obstacle_avoidance_node',
        output='screen',
    )

    return LaunchDescription([
        *ARGUMENTS,
        rviz_node,
        obstacle_avoidance_node,
        tf_bridge
    ])
