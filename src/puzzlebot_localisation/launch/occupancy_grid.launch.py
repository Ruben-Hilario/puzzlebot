from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
ARGUMENTS=[
    DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true','false'],description='use simulation time'),
    DeclareLaunchArgument('publish_period_sec', default_value='0.1', description='Occupancy grid publishing period'),
    DeclareLaunchArgument('resolution', default_value='0.05', description='Occupancy grid resolution'),
    
]

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    occupancy_grid_node = 

    return LaunchDescription([
        

        
    ])
