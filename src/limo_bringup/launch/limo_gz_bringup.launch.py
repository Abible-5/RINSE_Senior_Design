from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro


def _launch(context, *args, **kwargs):
    use_rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    use_gui  = LaunchConfiguration("jsp_gui").perform(context).lower() == "true"

    # 1) Hard-code the exact xacro that aggregates everything you need
    share_desc = get_package_share_directory("limo_description")
    HARD_XACRO = os.path.join(share_desc, "urdf", "limo_four_diff.xacro")  # <-- change if you want a different file
    if not os.path.exists(HARD_XACRO):
        raise RuntimeError(f"xacro file not found: {HARD_XACRO}")

    robot_xml = xacro.process_file(HARD_XACRO).toxml()
    if "<link" not in robot_xml:
        raise RuntimeError(f"xacro expanded but has no <link>: {HARD_XACRO}")

    # 2) Robot description
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_xml, "use_sim_time": True}],
        output="screen",
    )

    # 3) Joint states
    jsp = Node(
        package=("joint_state_publisher_gui" if use_gui else "joint_state_publisher"),
        executable=("joint_state_publisher_gui" if use_gui else "joint_state_publisher"),
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # 4) Start Gazebo Sim via its official launch file
    ros_gz_share = get_package_share_directory("ros_gz_sim")
    gz_launch = os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
    if not os.path.exists(gz_launch):
        raise RuntimeError(f"ros_gz_sim launch not found: {gz_launch}")

    # You can add args like: '-r empty.sdf' for headless or just 'empty.sdf' for the default viewer.
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": "empty.sdf"}.items()  # start an empty world
    )

    # 5) Spawn from /robot_description using the 'create' tool you have
    create_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "limo", "-topic", "/robot_description", "-x", "0", "-y", "0", "-z", "0"],
        output="screen",
    )
    spawn = TimerAction(period=3.0, actions=[create_node])  # small delay so the world is up

    nodes = [rsp, jsp, gz, spawn]
    if use_rviz:
        nodes.append(Node(package="rviz2", executable="rviz2", output="screen"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz2"),
        DeclareLaunchArgument("jsp_gui", default_value="true", description="Use joint_state_publisher_gui"),
        OpaqueFunction(function=_launch),
    ])