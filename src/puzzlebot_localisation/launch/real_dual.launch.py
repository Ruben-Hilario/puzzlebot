import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.path_join_substitution import PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument( 'use_sim_time',default_value='false',description='Use simulation clock if true'),
    DeclareLaunchArgument('use_rviz', default_value='true', choices=['true','false'], description='Enable rviz'),
    DeclareLaunchArgument('namespace1', default_value='robot_ideal'),
    DeclareLaunchArgument('namespace2', default_value='robot_real'),
    DeclareLaunchArgument('use_sim', default_value='true', choices=['true','false'], description='Use simulation or real robot'),
]

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    namespace1=LaunchConfiguration('namespace1')
    namespace2=LaunchConfiguration('namespace2')

    urdf_path = get_package_share_directory('puzzlebot_description') + '/models/puzzlebot/model.urdf'
    rviz_path = get_package_share_directory('puzzlebot_description') + '/rviz/dual.rviz'
    robot_description = Command(['cat ', urdf_path])

    robot_params = {
        'wheel_radius':  0.05,
        'wheel_base':    0.19,
        'sampling_time': 0.05,
        'k_r': 0.1592,
        'k_l': 0.2128,
    }

    tf_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/tf.launch.py']),
        launch_arguments={
            'use_sim_time':use_sim_time,
            'use_rviz':use_rviz,
            'robot_description':robot_description,
        }.items()
    )

    static_tf_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_map',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'world', '--child-frame-id', 'map']
    )

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom']
    )

    # Robot 1
    r1_robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace1,
        output='screen',
        parameters=[{
            'use_sim_time':      use_sim_time,
            'robot_description': robot_description,
            'frame_prefix': namespace1,
        }]
    )

    r1_kinematic = Node(
        package='puzzlebot_localisation',
        executable='test.py',
        name='test_node',
        namespace=namespace1,
        output='screen',
        parameters=[robot_params],
    )
    
    r1_localisation = Node(
        package='puzzlebot_localisation',
        executable='odom_node',
        name='odom_node',
        namespace=namespace1,
        output='screen',
        parameters=[{**robot_params, 'odom_frame': 'odom'}]
    )

    r1_joint_state = Node(
        package='puzzlebot_description',
        executable='joint_pub',
        name='joint_pub',
        namespace=namespace1,
        output='screen',
    )

    # Robot 2 — Robot REAL
    # No corre kinematic simulado: los encoders vienen del hardware (Jetson)
    # El remap conecta los topics globales del robot real al namespace robot2
    r2_robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace2,
        output='screen',
        parameters=[{
            'use_sim_time':      use_sim_time,
            'robot_description': robot_description,
            'frame_prefix':     namespace2,
        }]
    )


    r2_localisation = Node(
        package='puzzlebot_localisation',
        executable='odom_node',
        name='odom_node',
        namespace=namespace2,
        output='screen',
    )

    r2_joint_state = Node(
        package='puzzlebot_description',
        executable='joint_pub',
        name='joint_pub',
        namespace=namespace2,
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
    )

    return LaunchDescription([
        *ARGUMENTS,
        static_tf_world_map,
        static_tf_map_odom,
        tf_bridge,
        r1_robot_state_pub,
        r1_kinematic,
        r1_localisation,
        r1_joint_state,
        r2_robot_state_pub,
        r2_localisation,
        r2_joint_state,
        rviz_node,
    ])