from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='semantic_map',
            executable='semantic_map_node',
            name='semantic_map_node',
            output='screen'
        )
    ]) 
