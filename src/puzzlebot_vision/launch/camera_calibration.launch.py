from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description(context, *args, **kwargs):

    video_path = os.path.join(get_package_share_directory('puzzlebot_vision'), 'media', 'puzzlebot3.mp4')

    video_publisher_node = Node(
        package='puzzlebot_vision',
        executable='video_publisher',
        name='video_publisher',
        output='screen',
        parameters=[
            {'video_path': video_path},
            {'fps': 30.0},
            {'topic_name': 'image_raw'},
            {'loop': True}
        ]
    )

    camera_info_node = Node(
        package='cv_utils',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        namespace='camera',
        output='screen',
        parameters=[
            {'frame_width': 2160},
            {'frame_height': 3840},
            {'camera_name': 'camera'}
        ]
    )

    camera_calibration_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='cameracalibrator',
        output='screen',
        arguments=[
            '--size', '7x5',
            '--square', '0.03'
        ],
        remappings=[
            ('image', '/image_raw'),
        ]
    )
    
    
    return LaunchDescription([
        video_publisher_node,
        camera_info_node,
        #camera_calibration_node
    ])
        
