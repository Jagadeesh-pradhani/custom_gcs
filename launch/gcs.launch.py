from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='custom_gcs',
            executable='bridge_node',
            name='gcs_bridge',
            output='screen'
        )
    ])