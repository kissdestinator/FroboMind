#include <ros/ros.h>
#include "fmMsgs/desired_speed.h"
#include <stdio.h>
#include </opt/ros/electric/stacks/common_msgs/sensor_msgs/msg_gen/cpp/include/sensor_msgs/Joy.h>

class JoyToTurtle
{
	public:
		JoyToTurtle();
	  int tjek;
	private:
};

sensor_msgs::Joy joy;
int start;

JoyToTurtle::JoyToTurtle()
{
}



void callback(sensor_msgs::Joy joyIn)
{
    joy = joyIn;
    ROS_INFO("%f", (double)joy.axes[4]);
    ROS_INFO("%f", (double)joy.axes[1]);
    start = 1;
    return;
}

static void mainLoop()
{
	while(start == 0)
	{
		ros::spinOnce();
	}
	
    ros::NodeHandle h;
    ros::Publisher vel_pub_ = h.advertise<fmMsgs::desired_speed>("/speed_from_joystick", 1);

    ros::Rate loop_rate(50);

    while(true)
    {	
    	fmMsgs::desired_speed hastighed;
    	hastighed.speed_right = joy.axes[4];
    	hastighed.speed_left = joy.axes[1];
    	
    	vel_pub_.publish(hastighed);
    	
    	ros::spinOnce(); 

	loop_rate.sleep();
 
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joyToTurt");

  ros::NodeHandle h;

  start = 0;

  ros::Subscriber sub = h.subscribe("/fmHMI/joy", 1, callback);

  mainLoop();
  
  return(0);
}
