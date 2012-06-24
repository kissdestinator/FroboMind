#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include <stdio.h>
#include <fmMsgs/Joy.h>

ros::Publisher vel_pub;
ros::Subscriber joy_sub;
ros::Publisher pub_;
geometry_msgs::Twist cmdvel_;
double lin_vel;
double ang_vel;

void callback(fmMsgs::Joy joy)
{
	geometry_msgs::TwistStamped pub_msg;
		
	if(joy.buttons[7]){
		pub_msg.twist.linear.x = lin_vel ;
		pub_msg.twist.angular.z = ang_vel;
	}
	else{
		pub_msg.twist.linear.x = joy.axes[4] ;
		pub_msg.twist.angular.z = -joy.axes[3]/0.12;
	}
	
	cmdvel_.angular.z = pub_msg.twist.angular.z * -0.5;
	cmdvel_.linear.x = pub_msg.twist.linear.x * -1;
        pub_.publish(cmdvel_);
	vel_pub.publish(pub_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joyToTurt");

  ros::NodeHandle h;
  ros::NodeHandle nh("~");
  
  std::string velocity_pub_topic;
  std::string joystick_sub_topic;

  nh.param<std::string>("velocity_pub_topic", velocity_pub_topic, "/speed_from_joystick");
  nh.param<std::string>("joystick_sub_topic", joystick_sub_topic, "/fmHMI/joy");
  nh.param<double>("linear_velocity", lin_vel, 0.5);
  nh.param<double>("angular_velocity", ang_vel, 0.5);

  vel_pub = h.advertise<geometry_msgs::TwistStamped>(velocity_pub_topic, 1);
  joy_sub = h.subscribe(joystick_sub_topic, 1, callback);
  pub_ = h.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  ros::spin();
  
  return(0);
}
