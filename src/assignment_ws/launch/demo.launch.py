from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='assignment_ws', executable='state_subscriber',      name='state_monitor',  output='screen', emulate_tty=True),
        Node(package='assignment_ws', executable='perception_node',    name='perception',     output='screen', emulate_tty=True),
        Node(package='assignment_ws', executable='waypoint_navigator', name='navigator',      output='screen', emulate_tty=True),
    ])
