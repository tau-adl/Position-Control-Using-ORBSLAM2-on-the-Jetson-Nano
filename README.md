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

Change the ros_mono.launch file to:
    <?xml version="1.0"?>
<launch>

	<?xml version="1.0"?>
<launch>
	<arg name="vocabularty_path" default="$(find ORB_SLAM2_CUDA)/../../../Vocabulary/ORBvoc.txt"/>
    <!--arg name="vocabularty_path" default="/run/user/1000/ORBvoc.txt" /-->
    <arg name="camera_setting_path" default="$(find ORB_SLAM2_CUDA)/conf/euroc.yaml" />
    <arg name="bUseViewer" default="false" />
    <arg name="bEnablePublishROSTopic" default="true" />

    <node name="ORB_SLAM2_CUDA" pkg="ORB_SLAM2_CUDA" type="Mono" output="screen" 
	args="$(arg vocabularty_path) $(arg camera_setting_path) $(arg bUseViewer) $(arg bEnablePublishROSTopic)">
        <remap from="/camera/image_raw" to="/cam0/image_raw"/>
        <!--remap from="camera" to="/bebop/image_raw"/-->

  	</node>
    <!--node name="record" pkg="rosbag" type="record" args="-o /home/nano/ORB_SLAM2_CUDA/bag_rec/bag_rec /orb_slam2_pose"/-->
</launch>

type  the following commands in a terminal:

sudo jetson_clocks

roslaunch ~/ORB_SLAM2_CUDA/Examples/ROS/ORB_SLAM2_CUDA/launch/ros_mono.launch bUseViewer:=true

rosbag play  bag file

## Running OrbSLAM2 on the Bebop2 camera:



