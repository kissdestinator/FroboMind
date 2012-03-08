#include <ros/ros.h>
#include "fmMsgs/desired_speed.h"
#include <stdio.h>
#include <fmMsgs/Joy.h>

ros::Publisher vel_pub;
ros::Subscriber joy_sub;

void callback(fmMsgs::Joy joy)
{
	fmMsgs::desired_speed hastighed;

	if(joy.buttons[6]==1)
		hastighed.speed_right = hastighed.speed_left = -1;
	else if(joy.buttons[7]==1)
		hastighed.speed_right = hastighed.speed_left = 1;
	else
	{ 
		hastighed.speed_right = joy.axes[4];
		hastighed.speed_right = joy.axes[3];
		hastighed.speed_left = joy.axes[1];
	}	
	
	vel_pub.publish(hastighed);
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

  vel_pub = h.advertise<fmMsgs::desired_speed>(velocity_pub_topic, 1);
  joy_sub = h.subscribe(joystick_sub_topic, 1, callback);

  ros::spin();
  
  return(0);
}
