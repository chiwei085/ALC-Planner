from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='alc_planner',
            executable='alc_planner_node',
            name='alc_planner',
            output='screen',
        ),
    ])
