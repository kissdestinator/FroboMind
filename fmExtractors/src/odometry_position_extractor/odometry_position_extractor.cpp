#include <ros/ros.h>
#include <stdio.h>
#include "fmMsgs/odometry.h"
#include "fmMsgs/Vector3.h"
#include "math.h"


using namespace std;

double x,y,th,vl,vr,vx,vy,vth;
ros::Time current_time, last_time;
double lengthBetweenTwoWheels = 0.21+0.03;

void right_callback(fmMsgs::odometry odo_msg_in)
{
	vr = odo_msg_in.speed;
	return;
}

void left_callback(fmMsgs::odometry odo_msg_in)
{
	vl = odo_msg_in.speed;
	return;
}

int main(int argc, char** argv)
{
		ros::init(argc, argv, "odometry_position_extractor");
		
		ros::NodeHandle h;
		
		vl = vr = vy = vx = th = x = y = vth =0;

		ros::Subscriber sub_left = h.subscribe("/fmSensors/left_odometry", 1, left_callback);
		ros::Subscriber sub_right = h.subscribe("/fmSensors/right_odometry", 1, right_callback);
		
	    fmMsgs::Vector3 pub_msg;
	
		while(h.ok()){
			current_time = ros::Time::now();
			vx = (vl+vr)/2;
			vy = 0;
			vth = (vr-vl)/lengthBetweenTwoWheels; //angular velocity in radian per second. 
			
				//compute odometry in a typical way given the velocities of the robot
		    double dt = (current_time - last_time).toSec();
		    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		    double delta_th = vth * dt; 

		    x += delta_x;
		    y += delta_y;
		    th += delta_th;
		    
		    pub_msg.x = x;
		    pub_msg.y = y;
		    pub_msg.th = th;

		    ros::Publisher odom_pub = h.advertise<fmMsgs::Vector3>("xyz_position", 1); 
		    
		    ROS_INFO("HEJ");
		    
		    last_time = current_time;
		    
		    ros::spinOnce();
			
		}
}
