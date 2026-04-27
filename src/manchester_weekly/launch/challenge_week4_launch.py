import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Parametros ambos robots
    robot_params = {
        'wheel_radius':  0.05,
        'wheel_base':    0.19,
        'sampling_time': 0.05,
    }

    # URDF
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_path = os.path.join(
        get_package_share_directory('puzzlebot_description'),
        'models',
        'puzzlebot',
        'puzzlebot.urdf'
    )
    with open(urdf_path, 'r') as f:
        robot_desc_base = f.read()


    # TFs estáticos globales
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

    r1_robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time':      use_sim_time,
            'robot_description': robot_desc_base,
            'frame_prefix':            'robot1/',
        }]
    )

    # Robot 1
    r1_kinematic = Node(
        package='manchester_weekly',
        executable='puzzlebot_kinematic.py',
        name='puzzlebot_kinematic',
        namespace='robot1',
        output='screen',
        parameters=[robot_params]
    )

    r1_localisation = Node(
        package='manchester_weekly',
        executable='localisation.py',
        name='localisation',
        namespace='robot1',
        output='screen',
        parameters=[{**robot_params, 'odom_frame': 'odom'}]
    )

    r1_joint_state = Node(
        package='manchester_weekly',
        executable='joint_state_publisher.py',
        name='joint_state_publisher',
        namespace='robot1',
        output='screen',
        parameters=[{'sampling_time': robot_params['sampling_time']}]
    )

    r1_control = Node(
        package='manchester_weekly',
        executable='control.py',
        name='control',
        namespace='robot1',
        output='screen',
        parameters=[{
            'Kd': 0.3, 'Ktheta': 0.6,
            'threshold': 0.1, 'sampling_time': 0.05,
            'v_max': 0.2, 'w_max': 1.0,
        }]
    )

    r1_setpoint = Node(
        package='manchester_weekly',
        executable='set_poin_generator.py',
        name='set_poin_generator',
        namespace='robot1',
        output='screen',
        parameters=[{'trajectory': 'square', 'side_length': 1.0, 'loop': False}]
    )

    r2_robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot2',
        output='screen',
        parameters=[{
            'use_sim_time':      use_sim_time,
            'robot_description': robot_desc_base,
            'frame_prefix':            'robot2/',
        }]
    )

    # Robot 2
    r2_kinematic = Node(
        package='manchester_weekly',
        executable='puzzlebot_kinematic.py',
        name='puzzlebot_kinematic',
        namespace='robot2',
        output='screen',
        parameters=[{**robot_params, 'x0': 0.0, 'y0': 1.5}]
    )

    r2_localisation = Node(
        package='manchester_weekly',
        executable='localisation.py',
        name='localisation',
        namespace='robot2',
        output='screen',
        parameters=[{**robot_params, 'odom_frame': 'odom'}]
    )

    r2_joint_state = Node(
        package='manchester_weekly',
        executable='joint_state_publisher.py',
        name='joint_state_publisher',
        namespace='robot2',
        output='screen',
        parameters=[{'sampling_time': robot_params['sampling_time']}]
    )   

    r2_control = Node(
        package='manchester_weekly',
        executable='control.py',
        name='control',
        namespace='robot2',
        output='screen',
        parameters=[{
            'Kd': 0.3, 'Ktheta': 0.6,
            'threshold': 0.1, 'sampling_time': 0.05,
            'v_max': 0.2, 'w_max': 1.0,
        }]
    )

    r2_setpoint = Node(
        package='manchester_weekly',
        executable='set_poin_generator.py',
        name='set_poin_generator',
        namespace='robot2',
        output='screen',
        parameters=[{'trajectory': 'pentagon', 'side_length': 1.0, 'loop': False}]
    )

    rviz_config = os.path.join(
        get_package_share_directory('manchester_weekly'),
        'rviz',
        'two_robots.rviz'
    )

    # Visualización
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    rqt_tf_tree_node = Node(
        name='rqt_tf_tree',
        package='rqt_tf_tree',
        executable='rqt_tf_tree'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        # TFs globales
        static_tf_world_map,
        static_tf_map_odom,

        # Robot 1
        r1_robot_state_pub,
        r1_kinematic,
        r1_localisation,
        r1_joint_state,
        r1_control,
        r1_setpoint,

        # Robot 2
        r2_robot_state_pub,
        r2_kinematic,
        r2_localisation,
        r2_joint_state,
        r2_control,
        r2_setpoint,

        # Visualización
        rqt_tf_tree_node,
        rviz_node,
    ])