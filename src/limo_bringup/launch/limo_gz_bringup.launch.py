from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro

def _launch(context, *args, **kwargs):
    use_gui = LaunchConfiguration("jsp_gui").perform(context).lower() == "true"

    # 1) URDF
    share_desc = get_package_share_directory("limo_description")
    xacro_path = os.path.join(share_desc, "urdf", "limo_four_diff.xacro")
    robot_xml = xacro.process_file(xacro_path).toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_xml, "use_sim_time": True}],
        output="screen",
    )

    jsp = Node(
        package=("joint_state_publisher_gui" if use_gui else "joint_state_publisher"),
        executable=("joint_state_publisher_gui" if use_gui else "joint_state_publisher"),
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # 2) Gazebo
    gz_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            "gz_args": f"-r -v 3 {os.path.expanduser('~/snr_proj_ws/src/RINSE_Senior_Design/custom_world/worlds/room.world')}"
        }.items(),
    )

    # 3) Spawn model
    create = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "limo", "-topic", "/robot_description", "-x", "-2", "-y", "-2", "-z", "0"],
        output="screen",
    )
    spawn = TimerAction(period=3.0, actions=[create])

    # 4) ✅ Single bridge only
    spawn = TimerAction(period=3.0, actions=[create])
    bridge = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
               arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/odom@gz.msgs.Odometry@nav_msgs/msg/Odometry",
                "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
                "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V"
                ],
                output="screen",
            )
        ],
    )

    return [rsp, jsp, gz, spawn, bridge]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("jsp_gui", default_value="true"),
        OpaqueFunction(function=_launch),
    ])
