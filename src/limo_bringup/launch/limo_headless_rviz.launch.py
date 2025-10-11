from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    limo_description_pkg = get_package_share_directory('limo_description')
    urdf_file = os.path.join(limo_description_pkg, 'urdf', 'limo.urdf')

    # Start Gazebo Sim headless (-r = run, --headless = no GUI)
    gzserver = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '--headless'],
        output='screen'
    )

    # Spawn robot into Gazebo Sim
    spawn = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-file',
        os.path.join(limo_description_pkg, 'urdf', 'limo.sdf'),
        '-name', 'limo'
    ],
    output='screen'
    )

    robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    arguments=[
        os.path.join(
            '/home/matt/rinse_ws/src/RINSE_Senior_Design/src/limo_ros2/limo_description/urdf',
            'limo.urdf'
        )
    ],
    output='screen'
    )

    # Launch RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        gzserver,
        spawn,
        robot_state_publisher,
        rviz
    ])

