#Launch file for doing inferenc, could be adjusted to test out with local media
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description(context, *args, **kwargs):
    mode = DeclareLaunchArgument('mode',default_value='real_time',
        description='Mode of operation: "real_time" to use camera feed, "video" to use video file'
    )
    model = DeclareLaunchArgument('model', default_value='yoloN',
        description='Model to use for inference: "yoloN", "yoloDamo", "rfdetr"'
    )
    mode = LaunchConfiguration('mode').perform(context)
    model = LaunchConfiguration('model').perform(context)

    video_path = os.path.join(get_package_share_dixrectory('puzzlebot_vision'), 'media', 'puzzlebot3.mp4')

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

    return LaunchDescription([
        video_publisher_node,
    ])

