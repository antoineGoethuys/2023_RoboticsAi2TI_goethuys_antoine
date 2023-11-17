from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='avoid_obstacle_pkg',
            executable='avoid_obstacle',
            output='screen'),
    ])