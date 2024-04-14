# Import the necessary modules from launch and launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # This function sets up the ROS 2 launch description
    return LaunchDescription([
        Node(
            package='rosbot_talks',  # The name of the package containing the node
            executable='speech_to_text_node',  # The name of the executable to run
            output='screen',  # Directs the output to the screen
            name='speech_to_text'  # Optionally, specify a custom node name (if desired)
        ),
    ])
