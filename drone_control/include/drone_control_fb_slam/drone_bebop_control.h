#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace drone_bebop_control
{

class droneBebopControl final
{

public:
  droneBebopControl(const ros::NodeHandle& nh,
                 const ros::NodeHandle& nh_private);
  droneBebopControl() : droneBebopControl(ros::NodeHandle(), ros::NodeHandle("~") ) {}

  void run();

private:

  // ROS 
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber pose_slam_sub_;
  ros::Subscriber state_change_sub_;
  ros::Subscriber path_change_sub_;
  ros::Publisher desired_velocity_pub_;
  ros::Publisher land_pub_;
  ros::Publisher take_off_pub_;

  // Callback for SLAM output
  void poseSlamCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void stateChangeCallback(const std_msgs::Bool& msg);
  void pathChangeCallback(const std_msgs::Bool& msg);
  //path_change_sub_ = nh_.subscribe("path_change", 1, &droneBebopControl::pathChangeCallback, this);
  double position_x, position_x_0;
  double position_y, position_y_0;
  double position_z, position_z_0;
  bool use_slam_;  // If True, it will use the SLAM, else will Hover
  bool fly_path_;


  // Parameters
  void loadParameters();
  double scale_true_; // Smoothers for CNN outs
  //double critical_prob_coll_;
  //double max_forward_index_;
  std::string name_;

  // Internal variables

  double desired_forward_velocity_;
  double desired_side_velocity_;
  int fly_path_state_;
  int count,round;
  double pos_command_x, pos_command_y, pos_command_z;
  geometry_msgs::Twist cmd_velocity_;
  double prev_err_x_, f_D_err_x_;
  double prev_err_y_, f_D_err_y_, Integ_y;
  double prev_err_z_, f_D_err_z_, Integ_z;
  unsigned int no_slam_counter_, no_slam_alert_;

};

/*
class HeightChange{

public:
  HeightChange(const ros::NodeHandle& nh,
                 const ros::NodeHandle& nh_private);
  HeightChange() : HeightChange(ros::NodeHandle(), ros::NodeHandle("~") ) {}

  virtual ~HeightChange();

  void run();

private:

  // ROS 
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber is_up_sub_;
  ros::Subscriber move_sub_;
  ros::Subscriber alt_c_sub_;

  ros::Publisher desired_velocity_pub_;

  // Callback for networks outputs
  void is_up(const std_msgs::Empty& msg);
  void move(const std_msgs::Empty& msg);
  void change_altitude(const std_msgs::Empty& msg);

  std::string name_;
  bool is_up_;
  bool change_altitude_, should_move_;

};*/

}
