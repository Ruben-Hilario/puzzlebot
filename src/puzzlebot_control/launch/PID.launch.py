from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pid_Node = Node(
        package='puzzlebot_control',
        executable='pid_node',
        name='pid_node',
        output='screen'
    )
    pid_py_node = Node(
        package='puzzlebot_control',
        executable='cl.py',
        name='pid_py',
        output='screen'
    )

    deadReckoning_Node = Node(
        package='puzzlebot_localisation',
        executable='odom_node',
        name='odom_node',
        output='screen'
    )

    return LaunchDescription([
        deadReckoning_Node,
        pid_Node
        #pid_py_node
    ])
