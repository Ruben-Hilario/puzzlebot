#Launch file for doing inferenc, could be adjusted to test out with local media
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    video_path = os.path.join(get_package_share_directory('puzzlebot_vision'), 'media','video' 'puzzlebot3.mp4')

    video_publisher_node = Node(
        package='puzzlebot_vision',
        executable='video_publisher.py',
        name='video_publisher',
        output='screen',
    )

    inference_node = Node(
        package='puzzlebot_vision',
        executable='inference_node',
        name='inference_node',
        output='screen',
        parameters=[
            {'debug_mode': False},
            {'model_name': 'yoloN'},
            {'device': 'cuda'}
        ]
    )
    
    test_node = Node(
        package='puzzlebot_vision',
        executable='test.py',
        name='test_node',
        output='screen'
    )

    return LaunchDescription([
        video_publisher_node,
        test_node
    ])

