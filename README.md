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


## Custom World
// Navigate to your workspace.
cd ~/<your_workspace_name>/src

// Create a custom ROS 2 package.
ros2 pkg create --build-type ament_cmake custom_world

// Navigate to the package folder.
cd ~/your_workspace_name/src/custom_world

// Create the worlds folder.
mkdir -p worlds

// Create and open the room.world file for editing.
nano ~/your_workspace_name/src/custom_world/worlds/room.world

// Paste the following code/description into room.world and save it.

<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="room_world">

    <!-- Floor -->
    <model name="floor">
      <static>true</static>
      <link name="floor_link">
        <pose>0 0 -0.05 0 0 0</pose>
        <collision name="floor_collision">
          <geometry>
            <box><size>7.62 8.53 0.1</size></box>
          </geometry>
        </collision>
        <visual name="floor_visual">
          <geometry>
            <box><size>7.62 8.53 0.1</size></box>
          </geometry>
          <material>
            <ambient>0.75 0.75 0.75 1</ambient>
            <diffuse>0.75 0.75 0.75 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Wall 1 -->
    <model name="wall1">
      <static>true</static>
      <link name="wall1_link">
        <pose>-3.81 0 1.25 0 0 0</pose>
        <collision name="wall1_collision">
          <geometry>
            <box><size>0.1 8.53 2.5</size></box>
          </geometry>
        </collision>
        <visual name="wall1_visual">
          <geometry>
            <box><size>0.1 8.53 2.5</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Wall 2 -->
    <model name="wall2">
      <static>true</static>
      <link name="wall2_link">
        <pose>3.81 0 1.25 0 0 0</pose>
        <collision name="wall2_collision">
          <geometry>
            <box><size>0.1 8.53 2.5</size></box>
          </geometry>
        </collision>
        <visual name="wall2_visual">
          <geometry>
            <box><size>0.1 8.53 2.5</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Wall 3 -->
    <model name="wall3">
      <static>true</static>
      <link name="wall3_link">
        <pose>0 -4.265 1.25 0 0 0</pose>
        <collision name="wall3_collision">
          <geometry>
            <box><size>7.62 0.1 2.5</size></box>
          </geometry>
        </collision>
        <visual name="wall3_visual">
          <geometry>
            <box><size>7.62 0.1 2.5</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Wall 4 -->
    <model name="wall4">
      <static>true</static>
      <link name="wall4_link">
        <pose>0 4.265 1.25 0 0 0</pose>
        <collision name="wall4_collision">
          <geometry>
            <box><size>7.62 0.1 2.5</size></box>
          </geometry>
        </collision>
        <visual name="wall4_visual">
          <geometry>
            <box><size>7.62 0.1 2.5</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Table (brown): 6ft x 2ft x 3ft -->
    <model name="table">
      <static>true</static>

      <!-- Tabletop -->
      <link name="table_top_link">
        <pose>0 0 0.91 0 0 0</pose>
        <collision name="table_top_collision">
          <geometry>
            <box><size>1.83 0.61 0.05</size></box>
          </geometry>
        </collision>
        <visual name="table_top_visual">
          <geometry>
            <box><size>1.83 0.61 0.05</size></box>
          </geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <!-- Table Legs (brown) -->
      <link name="leg1">
        <pose>-0.86 -0.30 0.45 0 0 0</pose>
        <collision name="leg1_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="leg1_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <link name="leg2">
        <pose>0.86 -0.30 0.45 0 0 0</pose>
        <collision name="leg2_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="leg2_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <link name="leg3">
        <pose>-0.86 0.30 0.45 0 0 0</pose>
        <collision name="leg3_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="leg3_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <link name="leg4">
        <pose>0.86 0.30 0.45 0 0 0</pose>
        <collision name="leg4_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="leg4_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

    </model>

    <!-- Square Table Extension for L-shape -->
    <model name="table2">
      <static>true</static>

    <!-- Tabletop -->
      <link name="table_top_link">
        <pose>0.62 -0.61 0.91 0 0 0</pose>
        <collision name="table_top_collision">
          <geometry>
            <box><size>0.61 0.61 0.05</size></box>
          </geometry>
        </collision>
        <visual name="table_top_visual">
          <geometry>
            <box><size>0.61 0.61 0.05</size></box>
          </geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <!-- Table Legs -->
      <link name="leg1">
        <pose>0.315 -0.915 0.45 0 0 0</pose>
        <collision name="leg1_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="leg1_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <link name="leg2">
        <pose>0.925 -0.915 0.45 0 0 0</pose>
        <collision name="leg2_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="leg2_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <link name="leg3">
        <pose>0.315 -0.305 0.45 0 0 0</pose>
        <collision name="leg3_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="leg3_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <link name="leg4">
        <pose>0.925 -0.305 0.45 0 0 0</pose>
        <collision name="leg4_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="leg4_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

    </model>


<!-------------------------------------------------------------------->

    <!-- looonnngg table -->
    <model name="table_left_triple">
      <static>true</static>

      <!-- Tabletop -->
      <link name="table_left_triple_top">
        <pose>0 3.85 0.91 0 0 0</pose>  
        <collision name="table_left_triple_top_collision">
          <geometry><box><size>5.49 0.61 0.05</size></box></geometry>  
        </collision>
        <visual name="table_left_triple_top_visual">
          <geometry><box><size>5.49 0.61 0.05</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>        
        </visual>
      </link>

      <!-- Table Legs -->
      <link name="table_left_triple_leg1">
        <pose>-2.3 4.2 0.45 0 0 0</pose> <!-- Adjusted to the left -->
        <collision name="table_left_triple_leg1_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="table_left_triple_leg1_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <link name="table_left_triple_leg2">
        <pose>-2.3 3.6 0.45 0 0 0</pose> <!-- Adjusted to the left -->
        <collision name="table_left_triple_leg2_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="table_left_triple_leg2_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>


      <link name="table_left_triple_leg3">
        <pose>2.3 4.2 0.45 0 0 0</pose> <!-- Adjusted to the left -->
        <collision name="table_left_triple_leg3_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="table_left_triple_leg3_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>
      
      <link name="table_left_triple_leg4">
        <pose>2.3 3.6 0.45 0 0 0</pose> <!-- Adjusted to the left -->
        <collision name="table_left_triple_leg4_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="table_left_triple_leg4_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>


<!-------------------------------------------------------------------->
    <!-- Chair on the left side of the table -->
    <model name="chair1">
    <!-- Chair placed on the left side of the table, rotated to face the tabl-->
      <pose>0 0.7 0 0 0 3.1416</pose>

      <static>true</static>
      <link name="chair_link">

        <!-- Seat -->
        <visual name="seat_visual">
          <pose>0 0 0.45 0 0 0</pose>
          <geometry>
            <box><size>0.46 0.46 0.05</size></box>
          </geometry>
          <material>
            <ambient>0.4 0.25 0.1 1</ambient>
            <diffuse>0.4 0.25 0.1 1</diffuse>
          </material>
        </visual>
        <collision name="seat_collision">
          <pose>0 0 0.45 0 0 0</pose>
          <geometry>
            <box><size>0.46 0.46 0.05</size></box>
          </geometry>
        </collision>

        <!-- Backrest -->
        <visual name="backrest_visual">
          <pose>0 -0.2 0.85 0 0 0</pose>
          <geometry>
            <box><size>0.46 0.05 0.8</size></box>
          </geometry>
          <material>
            <ambient>0.4 0.25 0.1 1</ambient>
            <diffuse>0.4 0.25 0.1 1</diffuse>
          </material>
        </visual>
        <collision name="backrest_collision">
          <pose>0 -0.2 0.85 0 0 0</pose>
          <geometry>
            <box><size>0.46 0.05 0.8</size></box>
          </geometry>
        </collision>


        <!-- Legs -->
        <!-- Front Left -->
        <visual name="leg_fl_visual">
          <pose>0.2 0.2 0.225 0 0 0</pose>
          <geometry>
            <box><size>0.05 0.05 0.45</size></box>
          </geometry>
          <material>
            <ambient>0.4 0.25 0.1 1</ambient>
            <diffuse>0.4 0.25 0.1 1</diffuse>
          </material>
        </visual>
        <collision name="leg_fl_collision">
          <pose>0.2 0.2 0.225 0 0 0</pose>
          <geometry>
            <box><size>0.05 0.05 0.45</size></box>
          </geometry>
        </collision>

        <!-- Front Right -->
        <visual name="leg_fr_visual">
          <pose>-0.2 0.2 0.225 0 0 0</pose>
          <geometry>
            <box><size>0.05 0.05 0.45</size></box>
          </geometry>
          <material>
            <ambient>0.4 0.25 0.1 1</ambient>
            <diffuse>0.4 0.25 0.1 1</diffuse>
          </material>
        </visual>
        <collision name="leg_fr_collision">
          <pose>-0.2 0.2 0.225 0 0 0</pose>
          <geometry>
            <box><size>0.05 0.05 0.45</size></box>
          </geometry>
        </collision>
        
        <!-- Back Left -->
        <visual name="leg_bl_visual">
          <pose>0.2 -0.2 0.225 0 0 0</pose>
          <geometry>
            <box><size>0.05 0.05 0.45</size></box>
          </geometry>
          <material>
            <ambient>0.4 0.25 0.1 1</ambient>
            <diffuse>0.4 0.25 0.1 1</diffuse>
          </material>
        </visual>
        <collision name="leg_bl_collision">
          <pose>0.2 -0.2 0.225 0 0 0</pose>
          <geometry>
            <box><size>0.05 0.05 0.45</size></box>
          </geometry>
        </collision>
        
        <!-- Back Right -->
        <visual name="leg_br_visual">
          <pose>-0.2 -0.2 0.225 0 0 0</pose>
          <geometry>
            <box><size>0.05 0.05 0.45</size></box>
          </geometry>
          <material>
            <ambient>0.4 0.25 0.1 1</ambient>
            <diffuse>0.4 0.25 0.1 1</diffuse>
          </material>
        </visual>
        <collision name="leg_br_collision">
          <pose>-0.2 -0.2 0.225 0 0 0</pose>
          <geometry>
            <box><size>0.05 0.05 0.45</size></box>
          </geometry>
        </collision>
      </link>
    </model>
<!------------------------------------------------------------------>
    <model name="table_back1">
      <static>true</static>

      <!-- Tabletop -->
      <link name="table_back1_top">
        <pose>3.22 -1.26 0.91 0 0 1.5708</pose>
        <collision name="top_collision">
          <geometry><box><size>1.83 0.61 0.05</size></box></geometry>
        </collision>
        <visual name="top_visual">
          <geometry><box><size>1.83 0.61 0.05</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>


      <!-- Table Legs -->
      <link name="leg1">
        <pose>3.525 -2.175 0.45 0 0 1.5708</pose>
        <collision name="leg1_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="leg1_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <link name="leg2">
        <pose>2.915 -2.175 0.45 0 0 1.5708</pose>
        <collision name="leg2_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="leg2_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>


      <link name="leg3">
        <pose>3.525 -0.345 0.45 0 0 1.5708</pose>
        <collision name="leg3_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="leg3_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <link name="leg4">
        <pose>2.915 -0.345 0.45 0 0 1.5708</pose>
        <collision name="leg4_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="leg4_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
<!------------------------------------------------------------------------->

    <model name="table_back2">
      <static>true</static>

      <!-- Tabletop -->
      <link name="table_back2_top">
        <pose>3.22 1.26 0.91 0 0 1.5708</pose>
        <collision name="table_back2_top_collision">
          <geometry><box><size>1.83 0.61 0.05</size></box></geometry>
        </collision>
        <visual name="table_back2_top_visual">
          <geometry><box><size>1.83 0.61 0.05</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>


      <!-- Leg 1 -->
      <link name="table_back2_leg1">
        <pose>3.525 2.175 0.45 0 0 1.5708</pose>
        <collision name="table_back2_leg1_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="table_back2_leg1_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>


      <!-- Leg 2 -->
      <link name="table_back2_leg2">
        <pose>2.915 2.175 0.45 0 0 1.5708</pose>
        <collision name="table_back2_leg2_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="table_back2_leg2_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <!-- Leg 3 -->
      <link name="table_back2_leg3">
        <pose>3.525 0.345 0.45 0 0 1.5708</pose>
        <collision name="table_back2_leg3_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="table_back2_leg3_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>

      <!-- Leg 4 -->
      <link name="table_back2_leg4">
        <pose>2.915 0.345 0.45 0 0 1.5708</pose>
        <collision name="table_back2_leg4_collision">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
        </collision>
        <visual name="table_back2_leg4_visual">
          <geometry><box><size>0.05 0.05 0.9</size></box></geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

<!-------------------------------------------------------------------------->

    <model name="computer_on_table_back1">
      <static>true</static>


      <!-- Monitor -->
      <link name="monitor2">
        <pose>3.22 1.18 1.23 0 0 1.5708</pose>
        <collision name="monitor2_collision">
          <geometry>
            <box><size>0.4 0.07 0.3</size></box>
          </geometry>
        </collision>
        <visual name="monitor2_visual">
          <geometry>
            <box><size>0.4 0.07 0.3</size></box>
          </geometry>
          <material>
            <ambient>0.1 0.1 0.1 1</ambient>
            <diffuse>0.1 0.1 0.1 1</diffuse>
          </material>
        </visual>
      </link>
      
      <!-- Mounting Pole -->
      <link name="monitor2_pole">
        <pose>3.22 1.18 1.05 0 0 0</pose>
        <collision name="monitor2_pole_collision">
          <geometry>
            <cylinder>
              <radius>0.025</radius>
              <length>0.35</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="monitor2_pole_visual">
          <geometry>
            <cylinder>
              <radius>0.025</radius>
              <length>0.35</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
          </material>
        </visual>
     </link>
     
     <!-- Square Base -->
     <link name="monitor2_base">
       <pose>3.22 1.18 0.92 0 0 0</pose> <!-- Raised slightly for visibility -->
       <collision name="monitor2_base_collision">
         <geometry>
           <box><size>0.2 0.2 0.02</size></box>
         </geometry>
       </collision>
       <visual name="monitor2_base_visual">
         <geometry>
           <box><size>0.2 0.2 0.02</size></box>
         </geometry>
         <material>
           <ambient>0.2 0.2 0.2 1</ambient>
           <diffuse>0.2 0.2 0.2 1</diffuse>
         </material>
       </visual>
     </link>
   </model>


<!------------------------------------------------------------------------->

   <model name="computer_on_table_back2">
     <static>true</static>

     <!-- Monitor -->
     <link name="monitor">
       <pose>3.22 -1.18 1.23 0 0 1.5708</pose> <!-- Moved forward along Y -->
       <collision name="monitor_collision">
         <geometry>
           <box><size>0.4 0.07 0.3</size></box>
         </geometry>
       </collision>
       <visual name="monitor_visual">
         <geometry>
           <box><size>0.4 0.07 0.3</size></box>
         </geometry>
         <material>
           <ambient>0.1 0.1 0.1 1</ambient>
           <diffuse>0.1 0.1 0.1 1</diffuse>
         </material>
       </visual>
     </link>

     <!-- Mounting Pole -->
     <link name="monitor_pole">
       <pose>3.22 -1.18 1.05 0 0 0</pose> <!-- Below the monitor -->
       <collision name="pole_collision">
         <geometry>
           <cylinder>
             <radius>0.025</radius>
             <length>0.35</length>
           </cylinder>
         </geometry>
       </collision>
       <visual name="pole_visual">
         <geometry>
           <cylinder>
             <radius>0.025</radius>
             <length>0.35</length>
           </cylinder>
         </geometry>
         <material>
           <ambient>0.02 0.02 0.02 1</ambient>
           <diffuse>0.02 0.02 0.02 1</diffuse>
         </material>
       </visual>
     </link>
     
     <!-- Square Base -->
     <link name="monitor_base">
       <pose>3.22 -1.18 0.91 0 0 0</pose> <!-- On the tabletop -->
       <collision name="base_collision">
         <geometry>
           <box><size>0.2 0.2 0.02</size></box>
         </geometry>
       </collision>
       <visual name="base_visual">
         <geometry>
           <box><size>0.2 0.2 0.02</size></box>
         </geometry>
         <material>
           <ambient>0.1 0.1 0.1 1</ambient>
           <diffuse>0.1 0.1 0.1 1</diffuse>
         </material>
       </visual>
     </link>
   </model>


  </world>
</sdf>


## Replace empty world with custom world
// Go into file.
nano ~/<your_workspace_name>/src/limo_bringup/launch/limo_gz_bringup.launch.py

// Find :"gz_args": "empty.sdf" and replace with.
"gz_args": "~/<your_workspace_name>/src/custom_world/worlds/room.world"

// Save and you should now be able to launch the file with the custom world.

