from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

ARGUMENTS=[
    DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true','false'],description='use simulation time'),
    DeclareLaunchArgument('use_rviz', default_value='true', choices=['true','false'], description='Enable rviz'),
    DeclareLaunchArgument('namespace',default_value='/'), 
    DeclareLaunchArgument('real', default_value='true',choices=['true','false'])
]

#Remapping for real puzzlebot
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    urdf_path = get_package_share_directory('puzzlebot_description') + '/models/puzzlebot/model.urdf'
    robot_description = Command(['cat ', urdf_path])

    namespace=LaunchConfiguration('namespace')
    real=LaunchConfiguration('real')

    joint_states_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        namespace=namespace,
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time,
             'frame_prefix': [namespace, '/']}
        ]
    )
    
    odom_node = Node(
        package='puzzlebot_description',
        executable='joint_pub',
        name='odometry_node',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Real robot: lidar_link → laser (rplidar_ros driver frame)
    lidar_tf_bridge_real = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_bridge',
        namespace=namespace,
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', [namespace, '/lidar_link'],
                   '--child-frame-id', 'laser'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(real)
    )

    # Simulation: lidar_link → puzzlebot/chassis/rplidar (Gazebo sensor frame)
    lidar_tf_bridge_sim = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_bridge',
        namespace=namespace,
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', [namespace, '/lidar_link'],
                   '--child-frame-id', 'puzzlebot/chassis/rplidar'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(real)
    )

    return LaunchDescription([
        joint_states_node,
        odom_node,
        lidar_tf_bridge_real,
        lidar_tf_bridge_sim,
    ])