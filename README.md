# RINSE Senior Design — ROS 2 Jazzy Workspace

## run these first
sudo apt update
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui


## Build
cd ~/<name_of_your_workspace>
colcon build --symlink-install
source install/setup.bash

## Run
ros2 launch limo_bringup limo_gz_bringup.launch.py
  #this should launch rviz and gazebo, rviz is probably empty, but gazebo should have the limo

## Run remapping
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist /odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry

## Launch Waypoint
ros2 launch limo_control limo_waypoint_nav.launch.py
