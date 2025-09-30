from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("limo_bringup"),
                         "launch", "limo_gz_bringup.launch.py")
        )
    )

    # Bridge ROS /cmd_vel <-> Gazebo /cmd_vel (your plugin uses <topic>cmd_vel</topic>)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"],
        output="screen",
    )

    # Run teleop in its own terminal so it has a real TTY
    teleop_xterm = ExecuteProcess(
        cmd=["xterm", "-fa", "Monospace", "-fs", "12",
             "-e", "ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"],
        output="screen"
    )

    return LaunchDescription([bringup, bridge, teleop_xterm])
