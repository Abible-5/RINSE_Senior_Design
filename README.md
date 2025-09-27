# RINSE Senior Design — ROS 2 Jazzy Workspace

## Prereqs (Ubuntu 24.04)
sudo apt update
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui

## Build
cd ~/snr_proj_ws
colcon build --symlink-install
source install/setup.bash

## Run
ros2 launch limo_bringup limo_gz_bringup.launch.py
