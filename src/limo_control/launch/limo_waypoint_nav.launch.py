from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='limo_control',
            executable='limo_waypoint_nav',
            name='limo_waypoint_nav',
            output='screen',
            remappings=[
                ('/cmd_vel', '/cmd_vel')  # adjust if your sim doesn't use plain /cmd_vel
            ]
        ),
    ])
