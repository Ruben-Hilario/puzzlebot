from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
ARGUMENTS=[
    DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true','false'],description='use simulation time'),
    DeclareLaunchArgument('use_rviz', default_value='true', choices=['true','false'], description='Enable rviz'),
    DeclareLaunchArgument('robot_description', default_value='')
]

#Remapping for real puzzlebot
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    robot_description = LaunchConfiguration('robot_description')

    joint_states_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time}
        ]
    )
    
    odom_node = Node(
        package='puzzlebot_description',
        executable='joint_pub',
        name='odometry_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    lidar_tf_bridge_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'puzzlebot/chassis/rplidar'],
        parameters=[{'use_sim_time': use_sim_time}]
    )


    return LaunchDescription([
    joint_states_node,
    odom_node,
    lidar_tf_bridge_node,
    ])