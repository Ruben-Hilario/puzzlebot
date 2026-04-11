from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_node(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    
    if mode == 'record':
        return [
            Node(
                package='puzzlebot_vision',
                executable='record_node',
                name='vision_record_node',
                output='screen',
                parameters=[
                    {'debug_mode': False},
                    {'record_rate': 20.0},
                    {'video_save_path': '/tmp/puzzlebot_vision'}
                ]
            )
        ]
    elif mode == 'qr':
        return [
            Node(
                package='puzzlebot_vision',
                executable='qr_node',
                name='vision_qr_node',
                output='screen',
                parameters=[
                    {'debug_mode': False},
                    {'min_qr_size': 50},
                    {'timer_rate': 20.0}
                ]
            )
        ]
    else:
        raise ValueError(f"Unknown mode: {mode}. Use 'record' or 'qr'")

def generate_launch_description():
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='qr',
        description='Vision mode: "record" to record video or "qr" to detect QR codes'
    )
    
    return LaunchDescription([
        mode_arg,
        OpaqueFunction(function=launch_node)
    ])
