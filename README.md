# Autonomous drone using ORBSLAM2 on the Jetson Nano
Run ORBSLAM2 on the Jetson Nano, using recorded rosbags (e.g., EUROC) or live footage from a Bebop2 Drone.
Tested with Monocular camera in real time - https://www.youtube.com/watch?v=nSu7ru0SKbI&feature=youtu.be

The first part of the repo is based on the work of Thien Nguyen (hoangthien94)

## Prerequisite
* Start with a fresh flash for Jetson Nano.

## Installation

* Jetson Nano installation -   https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit
* Install ROS Melodic 
* After installing ROS Melodic, follow this thread and install the ORB_SLAM2_CUDA repo in your home folder https://github.com/hoangthien94/ORB_SLAM2_CUDA/issues/11#issuecomment-541394341 
* Don't forget to add the following to your .bashrc:

1. source /opt/ros/melodic/setup.bash
2. export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/your_comp/ORB_SLAM2_CUDA/Examples/ROS

## Running OrbSLAM2 on the EUROC dataset:

Add the euroc.yaml configuration file from the EUROC dataset to:

~/ORB_SLAM2_CUDA/Examples/ROS/ORB_SLAM2_CUDA/conf

Change the ros_mono.launch in the original repo to the file in this repo.

Then, type  the following commands in a terminal:

1. sudo jetson_clocks
2. roslaunch ~/ORB_SLAM2_CUDA/Examples/ROS/ORB_SLAM2_CUDA/launch/ros_mono.launch bUseViewer:=true
3. rosbag play  bag file    (the bagfile is from EUROC)

## Running OrbSLAM2 with the Bebop2 camera's video feed:
* create a ROS worksapce, bebop_ws, in your home folder according to https://bebop-autonomy.readthedocs.io/en/latest/installation.html

Copy the bebop.yaml file from this repo to:

~/ORB_SLAM2_CUDA/Examples/ROS/ORB_SLAM2_CUDA/conf

Change the bebop_ros_mono.launch in the original repo to the file in this repo.

Then, type  the following commands in a terminal:

1. sudo jetson_clocks
2. connect to bebop wifi
3. roslaunch ~/bebop_ws/src/bebop_autonomy/bebop_driver/launch/bebop_node.launch
4. roslaunch ~/ORB_SLAM2_CUDA/Examples/ROS/ORB_SLAM2_CUDA/launch/bebop_ros_mono.launch bUseViewer:=true
(now, check the slam manually by moving drone around in your hand)
    

## Close loop position control using the OrbSLAM2's pose as feedback:
Add the drone_control ros package from this repo (TBD soon) to the src directory of bebop_ws, and build.

From now on please be very cautious

1. In a terminal type: roslaunch ~/bebop_ws/src/drone_control_fb_slam/launch/drone_bebop_control.launch
this will make the drone hover in one place using it's own OF and height sensors
2. In a terminal type: rostopic pub --once /bebop/state_change std_msgs/Bool "data: true"
this will make the drone hover in one place using the SLAM's pose
3. To land the drone, 	rostopic pub --once /bebop/land std_msgs/Empty





