import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_file = os.path.join(get_package_share_directory('manchester_weekly'), 'rviz', 'markers.rviz')
    puzzlebot_transform_node = Node(
        name='mini_challenge_week_1',
        package='manchester_weekly',
        executable='mini_challenge_week_1'
    )

    rviz_node = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file]
    )
    rqt_tf_tree_node = Node(
        name='rt_tf_tree',
        package='rqt_tf_tree',
        executable='rqt_tf_tree'
    )
    
    return LaunchDescription([
        puzzlebot_transform_node,
        rviz_node,
        #rqt_tf_tree_node,
    ])
