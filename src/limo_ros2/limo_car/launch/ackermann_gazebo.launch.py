from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Locate limo.urdf.xacro
    pkg_share = get_package_share_directory('limo_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'limo.urdf.xacro')

    # Convert xacro → urdf xml string
    robot_description_config = xacro.process_file(urdf_file).toxml()

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config,
                     'use_sim_time': True}]
    )

    # Joint state publisher (sim only)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # Gazebo (classic)
    gazebo = Node(
        package='gazebo_ros',
        executable='gazebo',
        arguments=['-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo
    ])
