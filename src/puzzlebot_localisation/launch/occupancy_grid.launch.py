from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
ARGUMENTS=[
    DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true','false'],description='use simulation time'),
    DeclareLaunchArgument('publish_period_sec', default_value='0.1', description='Occupancy grid publishing period'),
    DeclareLaunchArgument('resolution', default_value='0.05', description='Occupancy grid resolution'),
    DeclareLaunchArgument('use_rviz', default_value='true', choices=['true','false'], description='Enable rviz')
]

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rviz_map = get_package_share_directory('puzzlebot_description') + '/rviz/map.rviz'

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d',rviz_map],
        parameters=[{'use_sim_time':use_sim_time}]
    )

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


    return LaunchDescription([
    rviz_node,
    joint_states_node,
    odom_node  
    ])