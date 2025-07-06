# localization

This package provides SLAM and AMCL launch files and configs for the RoboRacer project.  
It supports running the whole pipeline either with direct command line or with ROS 2 launch files.

**Note:** The coordinate frames now follow the standard: `f1tenth_1_odom` for odometry, `map` for the global map.

<<<<<<< HEAD
## 1. Run SLAM (Command Line)
=======
###############################
## 1. Run SLAM (Command Line)##
###############################
>>>>>>> 8711f37 (Add README)

```bash
# Build and source
colcon build --symlink-install
source install/setup.bash

# Start simulator (headless)
./AutoDRIVE\ Simulator.x86_64 -batchmode -nographics -ip 127.0.0.1 -port 4567

# Start bridge
ros2 run autodrive_f1tenth autodrive_bridge

# Run SLAM toolbox directly
ros2 run slam_toolbox sync_slam_toolbox_node \
  --ros-args \
  -p mode:=mapping \
  -p use_sim_time:=false \
  -p base_frame:=f1tenth_1 \
  -p odom_frame:=f1tenth_1_odom \
  -p map_frame:=map \
  -p throttle_scans:=2 \
  -p scan_queue_size:=50 \
  -r /scan:=/autodrive/f1tenth_1/lidar

# Open RViz2 to visualize
rviz2
<<<<<<< HEAD
```  

##  2. Run SLAM (Launch Script)

```bash
=======

#################################
##  2. Run SLAM (Launch Script)##
#################################

>>>>>>> 8711f37 (Add README)
# Same build
colcon build --symlink-install
source install/setup.bash

# Start simulator
./AutoDRIVE\ Simulator.x86_64 -batchmode -nographics -ip 127.0.0.1 -port 4567

# Start bridge
ros2 run autodrive_f1tenth autodrive_bridge

# Use the launch file to run SLAM
ros2 launch localization slam_launch.py

# Open RViz2
rviz2

# Debug TF tree if needed
ros2 run tf2_tools view_frames
<<<<<<< HEAD
```

## 3. Run AMCL (Launch Script)

```bash
=======

################################
## 3. Run AMCL (Launch Script)##
################################

>>>>>>> 8711f37 (Add README)
# Build
colcon build --symlink-install
source install/setup.bash

# Start simulator
./AutoDRIVE\ Simulator.x86_64 -batchmode -nographics -ip 127.0.0.1 -port 4567

# Start bridge
ros2 run autodrive_f1tenth autodrive_bridge

# Run AMCL
ros2 launch localization amcl_launch.py

# Open RViz2
rviz2

# If needed, check and control the lifecycle:
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle set /map_server configure
ros2 lifecycle set /amcl configure
ros2 lifecycle set /map_server activate
ros2 lifecycle set /amcl activate

# In RViz, click '2D Pose Estimate' to set the initial pose
<<<<<<< HEAD
```

## Notes
=======


-------Notes-------
>>>>>>> 8711f37 (Add README)
If you want to change the map:
Update setup.py with your new .yaml and .pgm.
Update amcl_launch.py to use the correct yaml map file.
The default TF tree is: map --> f1tenth_1_odom --> f1tenth_1

bridge publishes odom → base_link, AMCL provides map → odom.
<<<<<<< HEAD
```
=======
-------------------
>>>>>>> 8711f37 (Add README)

RoboRacer Localization | Last updated: 2025
