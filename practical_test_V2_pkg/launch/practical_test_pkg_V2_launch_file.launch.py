from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='practical_test_V2_pkg',
            executable='practical_test_V2',
            output='screen'),
    ])