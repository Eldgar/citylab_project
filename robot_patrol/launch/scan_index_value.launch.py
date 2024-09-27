from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='scan_index_value',
            name='scan_index_value_node',
            output='screen'
        )
    ])
