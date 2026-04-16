import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_file = os.path.join(get_package_share_directory('manchester_weekly'), 'rviz', 'markers.rviz')
    urdf_file = os.path.join(get_package_share_directory('puzzlebot_description'), 'models', 'puzzlebot', 'model.urdf')
    
    # Read the URDF file for robot_state_publisher
    with open(urdf_file, 'r') as urdf:
        robot_desc = urdf.read()
    
    # URDF frame publisher node
    urdf_publisher_node = Node(
        name='week2_urdf',
        package='manchester_weekly',
        executable='week2_urdf'
    )
    
    # Robot state publisher to handle URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    rviz_node = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file]
    )
    
    return LaunchDescription([
        urdf_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ])
