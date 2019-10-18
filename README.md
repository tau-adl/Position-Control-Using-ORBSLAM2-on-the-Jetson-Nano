# ORBSLAM2-for-Jetson-Nano2
Run ORBSLAM2 on the Jetson Nano, using recorded rosbags (e.g., EUROC) or live footage from a Bebop2 Drone.
Tested with Monocular camera in real time.

The first part of the repo is based on the work of Thien Nguyen (hoangthien94)

## Prerequisite
* Start with a fresh flash for Jetson Nano.

## Installation

* Jetson Nano installation -   https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit
* Install ROS Melodic 
* After installing ROS Melodic, follow this thread https://github.com/hoangthien94/ORB_SLAM2_CUDA/issues/11#issuecomment-541394341
* Don't forget to add the following to your .bashrc:

source /opt/ros/melodic/setup.bash

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/your_comp/ORB_SLAM2_CUDA/Examples/ROS

## Running OrbSLAM2 on the EUROC dataset:

Add a yaml configuration file from EUROC under:
~/ORB_SLAM2_CUDA/Examples/ROS/ORB_SLAM2_CUDA/conf

Change the ros_mono.launch in the original repo to the file in this repo.

type  the following commands in a terminal:

sudo jetson_clocks

roslaunch ~/ORB_SLAM2_CUDA/Examples/ROS/ORB_SLAM2_CUDA/launch/ros_mono.launch bUseViewer:=true

rosbag play  bag file    (the bagfile is from EUROC)

## Running OrbSLAM2 on the Bebop2 camera:
* create a ROS worksapce, bebop_ws, in your home folder according to https://bebop-autonomy.readthedocs.io/en/latest/installation.htm

    1. sudo jetson_clocks
    2. connect to bebop wifi
    3. roslaunch ~/bebop_ws/src/bebop_autonomy/bebop_driver/launch/bebop_node.launch
    4. roslaunch ~/ORB_SLAM2_CUDA/Examples/ROS/ORB_SLAM2_CUDA/launch/bebop_ros_mono.launch bUseViewer:=true
(now lock slam manually with drone)
    5. roslaunch ~/bebop_ws/src/drone_control_fb_slam/launch/drone_bebop_control.launch
    6. rostopic pub --once /bebop/state_change std_msgs/Bool "data: true"
    7. rostopic pub --once /bebop/path_change std_msgs/Bool "data: true"
    8. rostopic pub --once /bebop/land std_msgs/Empty





