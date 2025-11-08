# RINSE Senior Design — ROS 2 Jazzy Workspace

## Install jazzy and Gazebo, if needed
sudo apt update
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui



## Build
cd ~/<name_of_your_workspace>  

colcon build --symlink-install  

source install/setup.bash

## Run - Terminal 1
ros2 launch limo_bringup limo_gz_bringup.launch.py  

#this will launch gazebo into the built world room

## Launch Waypoint - Terminal 2
ros2 launch limo_control limo_waypoint_nav.launch.py
   

## Run remapping - ONLY need to run this if limo will not move
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist /odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry
    

The terminal in which the waypoint node is launched will have a continous log showing odometry position and heading error.
Currently after completing the alignment turn for off-axis waypoints, the odometry reading will decrease as the limo advances forwards for approximately .5 meters, then will start counting back up,
but we do not know why. We do know the odom reading is open loop, so any interference or slippage in the model is not reflected in the odom reading.
Odom also centers at spawn location. Currently limo spawns at (-2,-2), and odom shows (0,0) after spawning.
  
    
   
