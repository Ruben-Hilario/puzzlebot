from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    test_node = Node(
        name='testing_node',
        package='manchester_weekly',
        executable='testing_node'
    )

    rviz_node = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2'
    )

    rqt_tf_tree_node = Node(
        name='rt_tf_tree',
        package='rqt_tf_tree',
        executable='rqt_tf_tree'
    )

    return LaunchDescription([
        test_node,
        rviz_node,
        #rqt_tf_tree_node
    ])
