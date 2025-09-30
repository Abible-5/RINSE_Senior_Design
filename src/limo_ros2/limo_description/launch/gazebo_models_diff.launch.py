import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_name = 'limo_description'
    robot_name_in_model = 'limo'
    rviz_config_file_path = 'rviz/urdf.rviz'
    urdf_file_path = 'urdf/limo_four_diff.xacro'
    world_file_path = 'worlds/neighborhood.world'

    # Spawn pose
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'

    # Paths
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    world_path = os.path.join(pkg_share, world_file_path)

    # Launch configs
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation clock')

    # Robot state publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', default_urdf_model_path]),
            'use_sim_time': use_sim_time
        }]
    )

    # Joint state publisher GUI
    joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
        output='screen'
    )

    # Start Gazebo Sim (gz sim)
    gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-v4', world_path],
        output='screen'
    )

    # Spawn robot into Gazebo
    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name_in_model,
            '-topic', 'robot_description',
            '-x', spawn_x_val,
            '-y', spawn_y_val,
            '-z', spawn_z_val,
            '-Y', spawn_yaw_val
        ],
        output='screen'
    )

    # Bridge ROS 2 <-> Gazebo topics
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        output='screen'
    )

    ld = LaunchDescription()

    # Declare args
    ld.add_action(declare_use_sim_time_cmd)

    # Add actions
    ld.add_action(gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_gui_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bridge_cmd)

    return ld