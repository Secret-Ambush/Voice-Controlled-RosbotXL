from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbot_talks',
            executable='speech_to_text_node',
            output='screen'),
    ])