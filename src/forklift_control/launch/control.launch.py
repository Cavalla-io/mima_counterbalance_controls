from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='forklift_control',
            executable='ForkliftControlNode',
            name='ForkliftControlNode',
            output='screen',
            parameters=[{'param_name': 'param_value'}],  # optional
        ),
        Node(
            package='forklift_control',
            executable='robot_status.py',
            name='robot_status_aggregator',
            output='screen',
        ),
        
    ])