#include <ros/ros.h>
#include "fmMsgs/desired_speed.h"
#include <stdio.h>
#include <fmMsgs/Joy.h>

int start;
fmMsgs::Joy joy;

void callback(fmMsgs::Joy joyIn)
{
    joy = joyIn;
//  ROS_INFO("%f", (double)joy.axes[4]);
//  ROS_INFO("%f", (double)joy.axes[1]);
    start = 1;
    return;
}

static void mainLoop(ros::NodeHandle &h)
{

    ros::Publisher vel_pub_ = h.advertise<fmMsgs::desired_speed>("/speed_from_joystick", 1);

    while(start == 0)
    {
	ros::spinOnce();
    }
	
    ros::Rate loop_rate(50);

    while(ros::ok())
    {	
    	fmMsgs::desired_speed hastighed;

	
	if(joy.button[6]==1)
		hastighed.speed_right = hastighed.speed_left = -1;
	else if(joy.button[7]==1)
		hastighed.speed_right = hastighed.speed_left = 1;
	else
	{ 
	    	hastighed.speed_right = joy.axes[4];
	    	hastighed.speed_left = joy.axes[1];
	}	
    	
    	vel_pub_.publish(hastighed);
    	
    	ros::spinOnce(); 
	
	loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joyToTurt");

  ros::NodeHandle h;
  ros::NodeHandle nh("~");
  
  start = 0;

  ros::Subscriber sub = h.subscribe("/fmHMI/joy", 1, callback);

  mainLoop(h);
  
  return(0);
}
