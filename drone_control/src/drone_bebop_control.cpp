#include "drone_control_fb_slam/drone_bebop_control.h"


namespace drone_bebop_control
{

droneBebopControl::droneBebopControl(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private)
  :   nh_(nh),
      nh_private_(nh_private),
      name_(nh_private.getNamespace())
{
  //ROS_INFO("[%s]: Initializing droneBebopControl Node", name_.c_str());
  ROS_INFO("Initializing droneBebopControl Node");
  loadParameters();
  pose_slam_sub_ = nh_.subscribe("pose_SLAM_topic", 1, &droneBebopControl::poseSlamCallback, this);
  state_change_sub_ = nh_.subscribe("state_change", 1, &droneBebopControl::stateChangeCallback, this);
  path_change_sub_ = nh_.subscribe("path_change", 1, &droneBebopControl::pathChangeCallback, this);
  

  desired_velocity_pub_ = nh_.advertise < geometry_msgs::Twist > ("velocity", 1);
  land_pub_ = nh_.advertise < std_msgs::Empty > ("/bebop/land", 1);
  take_off_pub_ = nh_.advertise < std_msgs::Empty > ("/bebop/takeoff", 1);

  //steering_angle_ = 0.0;
  //probability_of_collision_ = 0.0;

  // Aggressive initialization
  desired_forward_velocity_ = 0.0;
  desired_side_velocity_ = 0.0;

  use_slam_ = false;
  fly_path_ = false;
  fly_path_state_ = 1.0; // Initial position
  count = 0;
  round = 0;
  pos_command_x = 0.0;
  pos_command_y = 0.0;
  pos_command_z = 0.5;
  prev_err_x_ = 0.0;
  prev_err_y_ = 0.0;
  prev_err_z_ = 0.0;
  f_D_err_x_ = 0.0;
  f_D_err_y_ = 0.0;
  f_D_err_z_ = 0.0; 

  no_slam_alert_ = 1;
  no_slam_counter_ = 0;

}


void droneBebopControl::run()
{

  ros::Duration(2.0).sleep();

  ros::Rate rate(30.0);

  Integ_y = 0;
  ROS_INFO("Integ_y = 0");

  while (ros::ok())
  {

    // Desired body frame velocity to world frame
    //double desired_forward_velocity_m = (1.0 -  probability_of_collision_) * max_forward_index_;
    //if (desired_forward_velocity_m <= 0.0)
    //{
      //ROS_INFO("Detected negative forward velocity! Drone will now stop!");
      //desired_forward_velocity_m  = 0;
    //}
    double Kp_pos_x = 0.025;
    double Kd_pos_x = 0.125;
    double Kp_pos_y = 0.025;
    double Kd_pos_y = 0.125;
    double Ki_pos_y = 0.000;
    double Kp_pos_z = 0.025;
    double Kd_pos_z = 0.125;
    double command_z_hover = 0;

    double D_err_x, err_x;
    double D_err_y, err_y;
    double D_err_z, err_z;



    // filtered P+D navigation:
    err_x = (pos_command_x - (position_x - position_x_0)*scale_true_);
    D_err_x = err_x - prev_err_x_;
    f_D_err_x_ = 0.5648*f_D_err_x_ + 12.75*D_err_x;
    cmd_velocity_.linear.x = Kp_pos_x*err_x + Kd_pos_x*f_D_err_x_;

    if (cmd_velocity_.linear.x >= 0.2)
      cmd_velocity_.linear.x = 0.2;
    if (cmd_velocity_.linear.x<= -0.2)
      cmd_velocity_.linear.x = -0.2;

    err_y = (pos_command_y - (position_y - position_y_0)*scale_true_);
    D_err_y = err_y - prev_err_y_;
    f_D_err_y_ = 0.5648*f_D_err_y_ + 12.75*D_err_y;    
    cmd_velocity_.linear.y = Kp_pos_y*err_y + Kd_pos_y*f_D_err_y_ ;

    if (cmd_velocity_.linear.y >= 0.2)
      cmd_velocity_.linear.y = 0.2;
    if (cmd_velocity_.linear.y <= -0.2)
      cmd_velocity_.linear.y = -0.2;

    //err_z = (pos_command_z - (position_z - position_z_0)*scale_true_);
    //D_err_z = err_z - prev_err_z_;
    //f_D_err_z_ = 0.5648*f_D_err_z_ + 12.75*D_err_z;    
    //cmd_velocity_.linear.z = Kp_pos_z*err_z + Kd_pos_z*f_D_err_z_ ;

    prev_err_x_ = err_x;
    prev_err_y_ = err_y;
    prev_err_z_ = err_z;



    // Verify that slam is still working..
    no_slam_counter_ = (no_slam_counter_ > 10) ? no_slam_counter_ : no_slam_counter_+1;
    if (no_slam_counter_ > 10)
    {
        ROS_INFO("lost SLAM tracking, holding place.");
        no_slam_alert_ = 1;
    }

    // Publish desired state if slam is still running
    if (use_slam_ && !no_slam_alert_)  
    {
        desired_velocity_pub_.publish(cmd_velocity_);
        //ROS_INFO("PUBLISHING VELOCITY");
    }
    else if (use_slam_ && no_slam_alert_)
    {
        cmd_velocity_.linear.x = 0.0;
        cmd_velocity_.linear.y = 0.0;
        cmd_velocity_.linear.z = 0.0;
        cmd_velocity_.angular.z = 0.0;
        desired_velocity_pub_.publish(cmd_velocity_);
    }
    else
    {
        cmd_velocity_.linear.x = 0.0;    
        cmd_velocity_.linear.y = 0.0;
        cmd_velocity_.linear.z = 0.0;
        cmd_velocity_.angular.z = 0.0;
        desired_velocity_pub_.publish(cmd_velocity_);
        //ROS_INFO("NOT PUBLISHING VELOCITY");
        position_x_0 = position_x;
        position_y_0 = position_y;
        position_z_0 = position_z;
    }

    //ROS_INFO("error_x.: %.3f   command_x: %.3f    error_y.: %.3f   command_y: %.3f", pos_command_x - (position_x - position_x_0), cmd_velocity_.linear.x, pos_command_y - (position_y - position_y_0), cmd_velocity_.linear.y); 
    //ROS_INFO("error_x.: %.3f   command_x: %.3f   ", pos_command_x - (position_x - position_x_0), cmd_velocity_.linear.x);
    
    //ROS_INFO("poscomm.: %.3f   pos: %.3f  vel_comm: %.3f   ", pos_command_x , position_x - position_x_0,  cmd_velocity_.linear.x);
//    ROS_INFO("POSX: %.3f - POSY: %.3f, err_y: %5.3f f_d_err_y: %5.3f, err_x: %5.3f f_d_err_x: %5.3f",
//             position_x, position_y, Kp_pos_y*err_y, Kd_pos_y*f_D_err_y_, Kp_pos_x*err_x, Kd_pos_x*f_D_err_x_);

    //ROS_INFO("POSX: %.3f - POSY: %.3f - POSZ: %5.3f",
             //position_x - position_x_0, position_y - position_y_0, position_z);


    ROS_INFO("POSX: %.3f - POSY: %.3f - POSZ: %.3f", err_x, err_y, cmd_velocity_.linear.z);
    
    if ((fly_path_) && (round<2))
    {
        bool is_inplace = false;
        //ROS_INFO("test1");
        switch (fly_path_state_)
        {
           case 1: // left back
              pos_command_x = 0.0;
              pos_command_y = -2.0; //0.5
              

              if (((err_y <= 0.15) && (err_y >= -0.15)) && ((err_x <= 0.15) && (err_x >= -0.15)))
              { 
                 is_inplace = true;
                 count = count + 1; 
                 if (count > 100)
                 {
                   ROS_INFO("DONE 10 seconds of mode LEFT BACK.");
                   ROS_INFO("DONE 10 seconds of mode LEFT BACK.");
                   ROS_INFO("DONE 10 seconds of mode LEFT BACK.");
                   fly_path_state_ = 2;
                   count = 0;
                 }
              }
              //ROS_INFO("mode LEFT BACK : %s", is_inplace ? "IN PLACE" : "ON THE WAY");
              break;

           case 2: // left front
              pos_command_x = 0.8;
              pos_command_y = -2.0; //0.5

              if (((err_y <= 0.15) && (err_y >= -0.15)) && ((err_x <= 0.15) && (err_x >= -0.15)))
              {
                 is_inplace = true;
                 count = count + 1;   
                 if (count > 100)
                 {
                   ROS_INFO("DONE 10 seconds of mode LEFT FRONT.");
                   ROS_INFO("DONE 10 seconds of mode LEFT FRONT.");
                   ROS_INFO("DONE 10 seconds of mode LEFT FRONT.");
                   fly_path_state_ = 3;
                   count = 0;
                 }
                
              }
              //ROS_INFO("mode LEFT FRONT : %s", is_inplace ? "IN PLACE" : "ON THE WAY");
              break;

           case 3:  // right front
              pos_command_x = 0.8;
              pos_command_y = 0.0;

              if (((err_y <= 0.15) && (err_y >= -0.15)) && ((err_x <= 0.15) && (err_x >= -0.15)))
              {
                 is_inplace = true;
                 count = count + 1;   
                 if (count > 100)
                 {
                   ROS_INFO("DONE 10 seconds of mode RIGHT FRONT.");
                   ROS_INFO("DONE 10 seconds of mode RIGHT FRONT.");
                   ROS_INFO("DONE 10 seconds of mode RIGHT FRONT.");
                   fly_path_state_ = 4;
                   count = 0;
                 }
              }
              ROS_INFO("mode RIGHT FRONT : %s", is_inplace ? "IN PLACE" : "ON THE WAY");
              break;

           case 4:  //  right back
              pos_command_x = 0.0;
              pos_command_y = 0.0;

              if (((err_y <= 0.15) && (err_y >= -0.15)) && ((err_x <= 0.15) && (err_x >= -0.15)))
              { 
                 is_inplace = true;
                 count = count + 1;   
                 if (count > 100)
                 {
                   ROS_INFO("DONE 10 seconds of mode RIGHT BACK.");
                   ROS_INFO("DONE 10 seconds of mode RIGHT BACK.");
                   ROS_INFO("DONE 10 seconds of mode RIGHT BACK.");
                   fly_path_state_ = 1;
                   count = 0;
                   //round++;
                 }
              }
              ROS_INFO("mode RIGHT BACK : %s", is_inplace ? "IN PLACE" : "ON THE WAY");
              break;
        }
    }

    if (round==0)
    {
        std_msgs::Empty takeoff_empty; 
        take_off_pub_.publish(takeoff_empty);
        round = round + 1;
    }
   

    
    if (round==2)
    {
      std_msgs::Empty land_empty;
      use_slam_ = 0;
      land_pub_.publish(land_empty);
    }


    rate.sleep();

    ros::spinOnce();

  }

}

void droneBebopControl::poseSlamCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

  //probability_of_collision_ = msg->collision_prob;
  //steering_angle_ = msg->steering_angle;

  position_x = msg->pose.pose.position.x;
  position_y = msg->pose.pose.position.y;
  position_z = msg->pose.pose.position.z;
  //position_z = msg.pose.pose.position.z;
  
  no_slam_counter_ = 0;
  no_slam_alert_ = 0;
 
}

void droneBebopControl::stateChangeCallback(const std_msgs::Bool& msg)
{
    //change current state
    use_slam_ = msg.data;
    ROS_INFO("change state");
}

void droneBebopControl::pathChangeCallback(const std_msgs::Bool& msg)
{
    //change current state
    fly_path_ = msg.data;
    ROS_INFO("fly path");
}

void droneBebopControl::loadParameters()
{

  ROS_INFO("Reading parameters");
  //ROS_INFO("[%s]: Reading parameters", name_.c_str()); 
  //nh_private_.param<double>("alpha_velocity", alpha_velocity_, 0.3);
  nh_private_.param<double>("scale_true", scale_true_, 1.0);
  //nh_private_.param<double>("max_forward_index", max_forward_index_, 0.2);
  //nh_private_.param<double>("critical_prob", critical_prob_coll_, 0.7);

}

} // namespace drone_bebop_control

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_bebop_control_");
  drone_bebop_control::droneBebopControl dn;

  dn.run();

  return 0;
}
