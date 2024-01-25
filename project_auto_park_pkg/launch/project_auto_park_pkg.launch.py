
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project_auto_park_pkg',
            executable='project_auto_park',
            output='screen'),
    ])