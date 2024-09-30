import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_burger.urdf'
    )

    # Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Launch RViz with the configuration file
    rviz_config_path = os.path.join(
        get_package_share_directory('robot_patrol'),
        'config',
        'robot_environment.rviz'
    )
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_path],
        output='screen'
    )

    direction_service = Node(
        package='robot_patrol',
        executable='direction_service',
        name='direction_service',
        output='screen'
    )
    go_to_pose_action = Node(
        package='robot_patrol',
        executable='go_to_pose_action',
        name='go_to_pose_action',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        direction_service,
        go_to_pose_action
    ])